using System;
using System.Collections;
using System.Collections.Generic;
using Robust.Shared.NewPhysics.Sensors;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private record struct SensorTaskContext;

    internal record struct SensorQueryContext
    {
	    SensorTaskContext taskContext;
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

    private static bool SensorQueryCallback( int proxyId, ulong userData, SensorQueryContext queryContext)
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

    private void OverlapSensors()
    {
        int sensorCount = _sensors.Count;

        if ( sensorCount == 0 )
	    {
		    return;
	    }

        var batchSize = _sensorJob.BatchSize;
        var batchCount = (sensorCount / batchSize) + 1;

        for (var i = 0; i < _sensorJob.EventBits.Count; i++)
        {
            _sensorJob.EventBits[i].SetAll(false);
        }

        for (var i = _sensorJob.EventBits.Count; i < batchCount; i++)
        {
            _sensorJob.EventBits.Add(new BitArray(batchSize));
        }

	    // Parallel-for sensors overlaps
        _parallel.ProcessNow(_sensorJob, sensorCount);

	    // Iterate sensors bits and publish events
	    // Process sensor state changes. Iterate over set bits
        for (var i = 0; i < batchSize; i++)
        {
            var bitset = _sensorJob.EventBits[i];
            var blockCount = _sensorJob.BatchSize;

            for (var k = 0; k < blockCount; ++k)
	        {
		        var word = bitset[k];
		        while (word)
		        {
			        uint32_t ctz = b2CTZ64( word );
			        int sensorIndex = (int)( 64 * k + ctz );

			        var sensor = _sensors[sensorIndex];
			        var sensorShape = _shapes[sensor.shapeId];
			        var sensorId = { sensor.shapeId + 1, world.worldId, sensorShape.generation };

			        int count1 = sensor.overlaps1.Count;
			        int count2 = sensor.overlaps2.Count;
			        ref var refs1 = ref sensor.overlaps1;
			        ref var refs2 = ref sensor.overlaps2;

			        // overlaps1 can have overlaps that end
			        // overlaps2 can have overlaps that begin
			        int index1 = 0, index2 = 0;
			        while ( index1 < count1 && index2 < count2 )
                    {
                        ref var r1 = ref refs1[index1];
                        ref var r2 = ref refs2[index2];

				        if ( r1.shapeId == r2.shapeId )
				        {
					        if ( r1.generation < r2.generation )
					        {
						        // end
						        b2ShapeId visitorId = { r1.shapeId + 1, world.worldId, r1.generation };
                                var ev = new SensorEndTouchEvent();
                                _sensorEndEvents[_endEventArrayIndex].Add(ev);

						        index1 += 1;
					        }
					        else if ( r1.generation > r2.generation )
					        {
						        // begin
						        b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
                                var ev = new SensorBeginTouchEvent();
                                _sensorBeginEvents.Add(ev);
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
                            var ev = new SensorEndTouchEvent();
                            _sensorEndEvents[_endEventArrayIndex].Add(ev);
					        index1 += 1;
				        }
				        else
				        {
					        // begin
					        b2ShapeId visitorId = { r2.shapeId + 1, world.worldId, r2.generation };
                            var ev = new SensorBeginTouchEvent();
                            _sensorBeginEvents.Add(ev);
					        index2 += 1;
				        }
			        }

			        while ( index1 < count1 )
			        {
				        // end
				        const b2Visitor* r1 = refs1 + index1;
				        b2ShapeId visitorId = { r1.shapeId + 1, world->worldId, r1->generation };
                        var ev = new SensorEndTouchEvent();
                        _sensorEndEvents[_endEventArrayIndex].Add(ev);
				        index1 += 1;
			        }

			        while ( index2 < count2 )
			        {
				        // begin
				        const b2Visitor* r2 = refs2 + index2;
				        b2ShapeId visitorId = { r2->shapeId + 1, world->worldId, r2->generation };
                        var ev = new SensorBeginTouchEvent();
                        _sensorBeginEvents.Add(ev);
				        index2 += 1;
			        }

			        // Clear the smallest set bit
			        word = word & ( word - 1 );
		        }
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

    private sealed class SensorJob : IParallelRobustJob
    {
        public int BatchSize => 16;

        public NewPhysicsSystem Physics = default!;

        public List<BitArray> EventBits = new();

        public void Execute(int sensorIndex)
        {
            b2SensorTaskContext* taskContext = world->sensorTaskContexts.data + threadIndex;

            var batchIndex = sensorIndex / BatchSize;
            var eventBits = EventBits[batchIndex];

            // TODO: Check the other eventbit batches

	        b2DynamicTree* trees = world->broadPhase.trees;

		    var sensor = Physics._sensors[sensorIndex];
		    var sensorShape = Physics._shapes[sensor.shapeId];

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
		    if ( body.Comp.SetIndex == (int) SetType.DisabledSet || sensorShape.EnableSensorEvents == false )
		    {
			    if ( sensor.overlaps1.Count != 0 )
			    {
				    // This sensor is dropping all overlaps because it has been disabled.
				    eventBits.Set(sensorIndex, true);
			    }

                return;
            }

		    var transform = Physics.GetPhysicsTransform(body.Owner);

		    var queryContext = new SensorQueryContext()
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
		    int overlapCount = sensor.overlaps2.Count;
		    var overlapData = sensor.overlaps2;
		    for ( int i = 0; i < overlapCount; ++i )
		    {
			    if ( uniqueCount == 0 || overlapData[i].shapeId != overlapData[uniqueCount - 1].shapeId )
			    {
				    overlapData[uniqueCount] = overlapData[i];
				    uniqueCount += 1;
			    }
		    }
		    sensor.overlaps2.count = uniqueCount;

		    int count1 = sensor.overlaps1.Count;
		    int count2 = sensor.overlaps2.Count;
		    if ( count1 != count2 )
		    {
			    // something changed
			    eventBits.Set(sensorIndex, true);
		    }
		    else
		    {
			    for ( int i = 0; i < count1; ++i )
                {
                    var s1 = sensor.overlaps1[i];
                    var s2 = sensor.overlaps2[i];

				    if ( s1.shapeId != s2.shapeId || s1.generation != s2.generation )
				    {
					    // something changed
					    eventBits.Set(sensorIndex, true);
					    break;
				    }
			    }
		    }
        }
    }
}
