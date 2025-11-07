using Robust.Shared.NewPhysics.Sensors;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    internal record struct b2SensorQueryContext
    {
	    b2SensorTaskContext* taskContext;
	    public Sensor sensor;
        public Fixture sensorShape;
        public Transform transform;
    };

    // Sensor shapes need to
    // - detect begin and end overlap events
    // - events must be reported in deterministic order
    // - maintain an active list of overlaps for query

    // Assumption
    // - sensors don't detect shapes on the same body

    // Algorithm
    // Query all sensors for overlaps
    // Check against previous overlaps

    // Data structures
    // Each sensor has an double buffered array of overlaps
    // These overlaps use a shape reference with index and generation

    private static bool SensorQueryCallback( int proxyId, ulong userData, b2SensorQueryContext queryContext)
    {
	    int shapeId = (int)userData;

	    var sensorShape = queryContext.sensorShape;
	    int sensorShapeId = sensorShape.Id;

	    if ( shapeId == sensorShapeId )
	    {
		    return true;
	    }

	    var otherShape = _shapes[shapeId];

	    // Are sensor events enabled on the other shape?
	    if (otherShape.EnableSensorEvents == false)
	    {
		    return true;
	    }

	    // Skip shapes on the same body
	    if (otherShape.BodyId == sensorShape.BodyId)
	    {
		    return true;
	    }

	    // Check filter
        // TODO: Layer / mask
	    if (ShouldShapesCollide(sensorShape.Filter, otherShape.Filter) == false)
	    {
		    return true;
	    }

	    // Custom filter here

	    var otherTransform = GetPhysicsTransform(otherShape.Body.Owner);

	    DistanceInput input = new();
	    input.ProxyA = MakeShapeDistanceProxy( sensorShape );
	    input.ProxyB = MakeShapeDistanceProxy( otherShape );
	    input.TransformA = queryContext.transform;
	    input.TransformB = otherTransform;
	    input.UseRadii = true;
	    var cache = new SimplexCache();
	    b2DistanceOutput output = b2ShapeDistance( &input, &cache, NULL, 0 );

	    bool overlaps = output.distance < 10.0f * float.Epsilon;
	    if ( overlaps == false )
	    {
		    return true;
	    }

	    // Record the overlap
	    var sensor = queryContext.sensor;
        sensor.overlaps2.Add(new Visitor()
        {
            shapeId = shapeId,
        });

	    return true;
    }

    static int b2CompareVisitors( const void* a, const void* b )
    {
	    const b2Visitor* sa = a;
	    const b2Visitor* sb = b;

	    if ( sa->shapeId < sb->shapeId )
	    {
		    return -1;
	    }

	    return 1;
    }

    private void SensorTask( int startIndex, int endIndex, uint threadIndex)
    {
	    b2SensorTaskContext* taskContext = world->sensorTaskContexts.data + threadIndex;

	    DebugTools.Assert(startIndex < endIndex);

	    b2DynamicTree* trees = world->broadPhase.trees;
	    for ( int sensorIndex = startIndex; sensorIndex < endIndex; ++sensorIndex )
	    {
		    var sensor = _sensors[sensorIndex];
		    var sensorShape = _shapes[sensor.shapeId];

		    // Swap overlap arrays
		    (sensor.overlaps1, sensor.overlaps2) = (sensor.overlaps2, sensor.overlaps1);
            sensor.overlaps2.Clear();

		    // Append sensor hits
		    int hitCount = sensor.hits.Count;
		    for ( int i = 0; i < hitCount; ++i )
		    {
                sensor.overlaps2.Add(sensor.hits[i]);
		    }

		    // Clear the hits
		    sensor.hits.Clear();

		    var body = _bodies[sensorShape.BodyId];
		    if ( body.SetIndex == (int) SetType.DisabledSet || sensorShape.EnableSensorEvents == false )
		    {
			    if ( sensor->overlaps1.count != 0 )
			    {
				    // This sensor is dropping all overlaps because it has been disabled.
				    b2SetBit( &taskContext->eventBits, sensorIndex );
			    }
			    continue;
		    }

		    var transform = GetPhysicsTransform(body.Owner);

		    var queryContext = new b2SensorQueryContext()
            {
			    sensor = sensor,
			    sensorShape = sensorShape,
			    transform = transform,
		    };

		    DebugTools.Assert( sensorShape.SensorIndex == sensorIndex );
		    var queryBounds = sensorShape.aabb;

		    // Query all trees
		    b2DynamicTree_Query( trees + 0, queryBounds, sensorShape->filter.maskBits, SensorQueryCallback, &queryContext );
		    b2DynamicTree_Query( trees + 1, queryBounds, sensorShape->filter.maskBits, SensorQueryCallback, &queryContext );
		    b2DynamicTree_Query( trees + 2, queryBounds, sensorShape->filter.maskBits, SensorQueryCallback, &queryContext );

		    // Sort the overlaps to enable finding begin and end events.
		    qsort( sensor->overlaps2.data, sensor->overlaps2.count, sizeof( b2Visitor ), b2CompareVisitors );

		    // Remove duplicates from overlaps2 (sorted). Duplicates are possible due to the hit events appended earlier.
		    int uniqueCount = 0;
		    int overlapCount = sensor->overlaps2.count;
		    b2Visitor* overlapData = sensor->overlaps2.data;
		    for ( int i = 0; i < overlapCount; ++i )
		    {
			    if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
			    {
				    overlapData[uniqueCount] = overlapData[i];
				    uniqueCount += 1;
			    }
		    }
		    sensor->overlaps2.count = uniqueCount;

		    int count1 = sensor->overlaps1.count;
		    int count2 = sensor->overlaps2.count;
		    if ( count1 != count2 )
		    {
			    // something changed
			    b2SetBit( &taskContext->eventBits, sensorIndex );
		    }
		    else
		    {
			    for ( int i = 0; i < count1; ++i )
			    {
				    b2Visitor* s1 = sensor->overlaps1.data + i;
				    b2Visitor* s2 = sensor->overlaps2.data + i;

				    if ( s1->shapeId != s2->shapeId || s1->generation != s2->generation )
				    {
					    // something changed
					    b2SetBit( &taskContext->eventBits, sensorIndex );
					    break;
				    }
			    }
		    }
	    }

	    b2TracyCZoneEnd( sensor_task );
}

    private void OverlapSensors()
    {
        int sensorCount = _sensors.count;

        if ( sensorCount == 0 )
	    {
		    return;
	    }

	    b2TracyCZoneNC( overlap_sensors, "Sensors", b2_colorMediumPurple, true );

	    for ( int i = 0; i < world.workerCount; ++i )
	    {
		    b2SetBitCountAndClear( &world.sensorTaskContexts.data[i].eventBits, sensorCount );
	    }

	    // Parallel-for sensors overlaps
	    int minRange = 16;
	    void* userSensorTask = world.enqueueTaskFcn( &SensorTask, sensorCount, minRange, world, world.userTaskContext );
	    world.taskCount += 1;
	    if ( userSensorTask != NULL )
	    {
		    world.finishTaskFcn( userSensorTask, world.userTaskContext );
	    }

	    b2TracyCZoneNC( sensor_state, "Events", b2_colorLightSlateGray, true );

	    b2BitSet* bitSet = &world.sensorTaskContexts.data[0].eventBits;
	    for ( int i = 1; i < world.workerCount; ++i )
	    {
		    b2InPlaceUnion( bitSet, &world.sensorTaskContexts.data[i].eventBits );
	    }

	    // Iterate sensors bits and publish events
	    // Process sensor state changes. Iterate over set bits
	    uint64_t* bits = bitSet.bits;
	    uint32_t blockCount = bitSet.blockCount;

	    for ( uint32_t k = 0; k < blockCount; ++k )
	    {
		    uint64_t word = bits[k];
		    while ( word != 0 )
		    {
			    uint32_t ctz = b2CTZ64( word );
			    int sensorIndex = (int)( 64 * k + ctz );

			    b2Sensor* sensor = b2SensorArray_Get( &world.sensors, sensorIndex );
			    b2Shape* sensorShape = b2ShapeArray_Get( &world.shapes, sensor.shapeId );
			    b2ShapeId sensorId = { sensor.shapeId + 1, world.worldId, sensorShape.generation };

			    int count1 = sensor.overlaps1.count;
			    int count2 = sensor.overlaps2.count;
			    const b2Visitor* refs1 = sensor.overlaps1.data;
			    const b2Visitor* refs2 = sensor.overlaps2.data;

			    // overlaps1 can have overlaps that end
			    // overlaps2 can have overlaps that begin
			    int index1 = 0, index2 = 0;
			    while ( index1 < count1 && index2 < count2 )
			    {
				    const b2Visitor* r1 = refs1 + index1;
				    const b2Visitor* r2 = refs2 + index2;
				    if ( r1.shapeId == r2.shapeId )
				    {
					    if ( r1.generation < r2.generation )
					    {
						    // end
						    b2ShapeId visitorId = { r1.shapeId + 1, world.worldId, r1.generation };
						    b2SensorEndTouchEvent event = {
							    .sensorShapeId = sensorId,
							    .visitorShapeId = visitorId,
						    };
						    b2SensorEndTouchEventArray_Push( &world.sensorEndEvents[world.endEventArrayIndex], event );
						    index1 += 1;
					    }
					    else if ( r1.generation > r2.generation )
					    {
						    // begin
						    b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
						    b2SensorBeginTouchEvent event = { sensorId, visitorId };
						    b2SensorBeginTouchEventArray_Push( &world.sensorBeginEvents, event );
						    index2 += 1;
					    }
					    else
					    {
						    // persisted
						    index1 += 1;
						    index2 += 1;
					    }
				    }
				    else if ( r1.shapeId < r2.shapeId )
				    {
					    // end
					    b2ShapeId visitorId = { r1.shapeId + 1, world.worldId, r1.generation };
					    b2SensorEndTouchEvent event = { sensorId, visitorId };
					    b2SensorEndTouchEventArray_Push( &world.sensorEndEvents[world.endEventArrayIndex], event );
					    index1 += 1;
				    }
				    else
				    {
					    // begin
					    b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
					    b2SensorBeginTouchEvent event = { sensorId, visitorId };
					    b2SensorBeginTouchEventArray_Push( &world.sensorBeginEvents, event );
					    index2 += 1;
				    }
			    }

			    while ( index1 < count1 )
			    {
				    // end
				    const b2Visitor* r1 = refs1 + index1;
				    b2ShapeId visitorId = { r1.shapeId + 1, world->worldId, r1->generation };
				    b2SensorEndTouchEvent event = { sensorId, visitorId };
				    b2SensorEndTouchEventArray_Push( &world->sensorEndEvents[world->endEventArrayIndex], event );
				    index1 += 1;
			    }

			    while ( index2 < count2 )
			    {
				    // begin
				    const b2Visitor* r2 = refs2 + index2;
				    b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
				    b2SensorBeginTouchEvent event = { sensorId, visitorId };
				    b2SensorBeginTouchEventArray_Push( &world->sensorBeginEvents, event );
				    index2 += 1;
			    }

			    // Clear the smallest set bit
			    word = word & ( word - 1 );
		    }
	    }
    }

    private void DestroySensor(Fixture fixture)
    {
        if (fixture.Hard)
        {
            DebugTools.Assert("Tried to delete a hard fixture?");
            return;
        }

        var sensor = _sensors[fixture.SensorIndex];

        for ( int i = 0; i < sensor.overlaps2.Count; ++i )
        {
            var visitor = sensor.overlaps2[i];

            var end = new SensorEndTouchEvent()
            {

            };

            _sensorEndEvents[_endEventArrayIndex].Add(end);
        }

        // Destroy sensor
        sensor.hits.Clear();
        sensor.overlaps1.Clear();
        sensor.overlaps2.Clear();

        var movedIndex = _sensors.Count;
        var movedSensor = _sensors.RemoveSwap(fixture.SensorIndex);

        if ( movedIndex != PhysicsConstants.NullIndex )
        {
            // Fixup moved sensor
            var otherSensorShape = _shapes[movedSensor.shapeId];
            otherSensorShape.SensorIndex = fixture.SensorIndex;
        }
    }
}
