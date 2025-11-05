namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
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
	    void* userSensorTask = world.enqueueTaskFcn( &b2SensorTask, sensorCount, minRange, world, world.userTaskContext );
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

	    b2TracyCZoneEnd( sensor_state );
	    b2TracyCZoneEnd( overlap_sensors );
    }
}
