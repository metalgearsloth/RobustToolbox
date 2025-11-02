using System.Diagnostics;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    // Validate contact touching status.
    [Conditional("DEBUG")]
    private void ValidateContacts()
    {
        int contactCount = _contacts.Count;
        DebugTools.Assert(contactCount == _contactIdPool.Capacity);
        int allocatedContactCount = 0;

        for ( int contactIndex = 0; contactIndex < contactCount; ++contactIndex )
        {
            var contact = _contacts[contactIndex];
            if (contact.contactId == PhysicsConstants.NullIndex)
            {
                continue;
            }

            DebugTools.Assert(contact.contactId == contactIndex);

            allocatedContactCount += 1;

            bool touching = (contact.flags & ContactFlags.ContactTouchingFlag ) != 0;

            int setId = contact.setIndex;

            if ( setId == (int) SetType.AwakeSet )
            {
                if ( touching )
                {
                    DebugTools.Assert(0 <= contact.colorIndex && contact.colorIndex < PhysicsConstants.GraphColorCount);
                }
                else
                {
                    DebugTools.Assert(contact.colorIndex == PhysicsConstants.NullIndex);
                }
            }
            else if (setId >= (int) SetType.FirstSleepingSet)
            {
                // Only touching contacts allowed in a sleeping set
                DebugTools.Assert( touching == true );
            }
            else
            {
                // Sleeping and non-touching contacts belong in the disabled set
                DebugTools.Assert( touching == false && setId == (int) SetType.DisabledSet );
            }

            ref var contactSim = ref GetContactSim(contact);
            DebugTools.Assert(contactSim.contactId == contactIndex);
            DebugTools.Assert(contactSim.bodyIdA == contact.edges._00.bodyId);
            DebugTools.Assert(contactSim.bodyIdB == contact.edges._01.bodyId);

            bool simTouching = (contactSim.simFlags & ContactSimFlags.SimTouchingFlag) != 0;
            DebugTools.Assert(touching == simTouching);

            DebugTools.Assert(0 <= contactSim.manifold.pointCount && contactSim.manifold.pointCount <= 2);
        }

        int contactIdCount = _contactIdPool.Count;
        DebugTools.Assert( allocatedContactCount == contactIdCount );
    }

    // Validates solver sets, but not island connectivity
    [Conditional("DEBUG")]
    private void ValidateSolverSets()
    {
	    DebugTools.Assert(_bodyIdPool.Capacity == _bodies.Count);
	    DebugTools.Assert(_contactIdPool.Capacity == _contacts.Count);
	    DebugTools.Assert(_jointIdPool.Capacity == _joints.Count);
	    DebugTools.Assert(_islandIdPool.Capacity == _islands.Count);
	    DebugTools.Assert(_solverSetPool.Capacity == _solverSets.Length);

	    int activeSetCount = 0;
	    int totalBodyCount = 0;
	    int totalJointCount = 0;
	    int totalContactCount = 0;
	    int totalIslandCount = 0;

	    // Validate all solver sets
	    int setCount = _solverSets.Length;

	    for ( int setIndex = 0; setIndex < setCount; ++setIndex )
	    {
		    var set = _solverSets[setIndex];
		    if (set.setIndex != PhysicsConstants.NullIndex )
		    {
			    activeSetCount += 1;

			    if (setIndex == (int) SetType.StaticSet)
			    {
				    DebugTools.Assert( set.contactSims.Count == 0 );
				    DebugTools.Assert( set.islandSims.Count == 0 );
				    DebugTools.Assert( set.bodyStates.Count == 0 );
			    }
			    else if ( setIndex == (int) SetType.DisabledSet )
			    {
				    DebugTools.Assert( set.islandSims.Count == 0 );
				    DebugTools.Assert( set.bodyStates.Count == 0 );
			    }
			    else if ( setIndex == (int) SetType.AwakeSet )
			    {
				    DebugTools.Assert( set.bodySims.Count == set.bodyStates.Count );
				    DebugTools.Assert( set.jointSims.Count == 0 );
			    }
			    else
			    {
				    DebugTools.Assert( set.bodyStates.Count == 0 );
			    }

			    // Validate bodies
			    {
				    var bodies = _bodies;
				    DebugTools.Assert( set.bodySims.Count >= 0 );
				    totalBodyCount += set.bodySims.Count;
				    for ( int i = 0; i < set.bodySims.Count; ++i )
				    {
					    ref var bodySim = ref set.bodySims[i];

					    int bodyId = bodySim.bodyId;
					    DebugTools.Assert( 0 <= bodyId && bodyId < world.bodies.Count );
					    b2Body* body = bodies + bodyId;
					    DebugTools.Assert( body.setIndex == setIndex );
					    DebugTools.Assert( body.localIndex == i );

					    if ( body.type == b2_dynamicBody )
					    {
						    DebugTools.Assert( body.flags & b2_dynamicFlag );
					    }

					    if ( setIndex == b2_disabledSet )
					    {
						    DebugTools.Assert( body.headContactKey == PhysicsConstants.NullIndex );
					    }

					    // Validate body shapes
					    int prevShapeId = PhysicsConstants.NullIndex;
					    int shapeId = body.headShapeId;

					    while ( shapeId != PhysicsConstants.NullIndex )
                        {
                            var shape = _shapes[shapeId];
						    DebugTools.Assert(shape.Id == shapeId);
						    DebugTools.Assert(shape.prevShapeId == prevShapeId);

						    if ( setIndex == (int) SetType.DisabledSet )
						    {
							    DebugTools.Assert( shape.proxyKey == PhysicsConstants.NullIndex );
						    }
						    else if ( setIndex == (int) SetType.StaticSet )
						    {
							    DebugTools.Assert( B2_PROXY_TYPE( shape.proxyKey ) == b2_staticBody );
						    }
						    else
						    {
							    b2BodyType proxyType = B2_PROXY_TYPE( shape.proxyKey );
							    DebugTools.Assert( proxyType == (int) BodyType.Kinematic || proxyType == (int) BodyType.Dynamic );
						    }

						    prevShapeId = shapeId;
						    shapeId = shape.nextShapeId;
					    }

					    // Validate body contacts
					    int contactKey = body.headContactKey;
					    while ( contactKey != PhysicsConstants.NullIndex )
					    {
						    int contactId = contactKey >> 1;
						    int edgeIndex = contactKey & 1;

						    b2Contact* contact = _contacts[contactId];
						    DebugTools.Assert( contact.setIndex != b2_staticSet );
						    DebugTools.Assert( contact.edges[0].bodyId == bodyId || contact.edges[1].bodyId == bodyId );
						    contactKey = contact.edges[edgeIndex].nextKey;
					    }

					    // Validate body joints
					    int jointKey = body.headJointKey;
					    while ( jointKey != PhysicsConstants.NullIndex )
					    {
						    int jointId = jointKey >> 1;
						    int edgeIndex = jointKey & 1;

						    b2Joint* joint = b2JointArray_Get( &world.joints, jointId );

						    int otherEdgeIndex = edgeIndex ^ 1;

						    b2Body* otherBody = b2BodyArray_Get( &world.bodies, joint.edges[otherEdgeIndex].bodyId );

						    if ( setIndex == b2_disabledSet || otherBody.setIndex == b2_disabledSet )
						    {
							    DebugTools.Assert( joint.setIndex == b2_disabledSet );
						    }
						    else if ( setIndex == b2_staticSet && otherBody.setIndex == b2_staticSet )
						    {
							    DebugTools.Assert( joint.setIndex == b2_staticSet );
						    }
						    else if ( body.type != b2_dynamicBody && otherBody.type != b2_dynamicBody )
						    {
							    DebugTools.Assert( joint.setIndex == b2_staticSet );
						    }
						    else if ( setIndex == b2_awakeSet )
						    {
							    DebugTools.Assert( joint.setIndex == b2_awakeSet );
						    }
						    else if ( setIndex >= b2_firstSleepingSet )
						    {
							    DebugTools.Assert( joint.setIndex == setIndex );
						    }

						    b2JointSim* jointSim = b2GetJointSim( world, joint );
						    DebugTools.Assert( jointSim.jointId == jointId );
						    DebugTools.Assert( jointSim.bodyIdA == joint.edges[0].bodyId );
						    DebugTools.Assert( jointSim.bodyIdB == joint.edges[1].bodyId );

						    jointKey = joint.edges[edgeIndex].nextKey;
					    }
				    }
			    }

			    // Validate contacts
			    {
				    DebugTools.Assert( set.contactSims.Count >= 0 );
				    totalContactCount += set.contactSims.Count;
				    for ( int i = 0; i < set.contactSims.Count; ++i )
				    {
					    b2ContactSim* contactSim = set.contactSims.data + i;
					    b2Contact* contact = b2ContactArray_Get( &world.contacts, contactSim.contactId );
					    if ( setIndex == b2_awakeSet )
					    {
						    // contact should be non-touching if awake
						    // or it could be this contact hasn't been transferred yet
						    DebugTools.Assert( contactSim.manifold.pointCount == 0 ||
								       ( contactSim.simFlags & b2_simStartedTouching ) != 0 );
					    }
					    DebugTools.Assert( contact.setIndex == setIndex );
					    DebugTools.Assert( contact.colorIndex == PhysicsConstants.NullIndex );
					    DebugTools.Assert( contact.localIndex == i );
				    }
			    }

			    // Validate joints
			    {
				    DebugTools.Assert( set.jointSims.Count >= 0 );
				    totalJointCount += set.jointSims.Count;
				    for ( int i = 0; i < set.jointSims.Count; ++i )
				    {
					    b2JointSim* jointSim = set.jointSims.data + i;
					    b2Joint* joint = b2JointArray_Get( &world.joints, jointSim.jointId );
					    DebugTools.Assert( joint.setIndex == setIndex );
					    DebugTools.Assert( joint.colorIndex == PhysicsConstants.NullIndex );
					    DebugTools.Assert( joint.localIndex == i );
				    }
			    }

			    // Validate islands
			    {
				    DebugTools.Assert( set.islandSims.Count >= 0 );
				    totalIslandCount += set.islandSims.Count;
				    for ( int i = 0; i < set.islandSims.Count; ++i )
				    {
					    b2IslandSim* islandSim = set.islandSims.data + i;
					    b2Island* island = b2IslandArray_Get( &world.islands, islandSim.islandId );
					    DebugTools.Assert( island.setIndex == setIndex );
					    DebugTools.Assert( island.localIndex == i );
				    }
			    }
		    }
		    else
		    {
			    DebugTools.Assert( set.bodySims.Count == 0 );
			    DebugTools.Assert( set.contactSims.Count == 0 );
			    DebugTools.Assert( set.jointSims.Count == 0 );
			    DebugTools.Assert( set.islandSims.Count == 0 );
			    DebugTools.Assert( set.bodyStates.Count == 0 );
		    }
	    }

	    int setIdCount = b2GetIdCount( &world.solverSetIdPool );
	    DebugTools.Assert( activeSetCount == setIdCount );

	    int bodyIdCount = b2GetIdCount( &world.bodyIdPool );
	    DebugTools.Assert( totalBodyCount == bodyIdCount );

	    int islandIdCount = b2GetIdCount( &world.islandIdPool );
	    DebugTools.Assert( totalIslandCount == islandIdCount );

	    // Validate constraint graph
	    for ( int colorIndex = 0; colorIndex < PhysicsConstants.GraphColorCount; ++colorIndex )
	    {
		    var color = _constraintGraph.colors[colorIndex];
		    int bitCount = 0;

		    DebugTools.Assert( color.ContactSims.Count >= 0 );
		    totalContactCount += color.ContactSims.Count;
		    for ( int i = 0; i < color.ContactSims.Count; ++i )
		    {
			    var contactSim = color.ContactSims[i];
			    var contact = _contacts[contactSim.contactId];
			    // contact should be touching in the constraint graph or awaiting transfer to non-touching
			    DebugTools.Assert( contactSim.manifold.pointCount > 0 ||
					       ( contactSim.simFlags & ( b2_simStoppedTouching | b2_simDisjoint ) ) != 0 );
			    DebugTools.Assert( contact.setIndex == b2_awakeSet );
			    DebugTools.Assert( contact.colorIndex == colorIndex );
			    DebugTools.Assert( contact.localIndex == i );

			    int bodyIdA = contact.edges._00.bodyId;
			    int bodyIdB = contact.edges[1].bodyId;

			    if ( colorIndex < PhysicsConstants.OverflowIndex )
			    {
				    b2Body* bodyA = _bodies[bodyIdA];
				    b2Body* bodyB = _bodies[bodyIdB];
				    DebugTools.Assert( b2GetBit( &color.bodySet, bodyIdA ) == ( bodyA.type == b2_dynamicBody ) );
				    DebugTools.Assert( b2GetBit( &color.bodySet, bodyIdB ) == ( bodyB.type == b2_dynamicBody ) );

				    bitCount += bodyA.type == b2_dynamicBody ? 1 : 0;
				    bitCount += bodyB.type == b2_dynamicBody ? 1 : 0;
			    }
		    }

		    DebugTools.Assert( color.jointSims.Count >= 0 );
		    totalJointCount += color.jointSims.Count;
		    for ( int i = 0; i < color.jointSims.Count; ++i )
		    {
			    b2JointSim* jointSim = color.jointSims.data + i;
			    b2Joint* joint = b2JointArray_Get( &world.joints, jointSim.jointId );
			    DebugTools.Assert( joint.setIndex == b2_awakeSet );
			    DebugTools.Assert( joint.colorIndex == colorIndex );
			    DebugTools.Assert( joint.localIndex == i );

			    int bodyIdA = joint.edges[0].bodyId;
			    int bodyIdB = joint.edges[1].bodyId;

			    if ( colorIndex < B2_OVERFLOW_INDEX )
			    {
				    b2Body* bodyA = b2BodyArray_Get( &world.bodies, bodyIdA );
				    b2Body* bodyB = b2BodyArray_Get( &world.bodies, bodyIdB );
				    DebugTools.Assert( b2GetBit( &color.bodySet, bodyIdA ) == ( bodyA.type == b2_dynamicBody ) );
				    DebugTools.Assert( b2GetBit( &color.bodySet, bodyIdB ) == ( bodyB.type == b2_dynamicBody ) );

				    bitCount += bodyA.type == b2_dynamicBody ? 1 : 0;
				    bitCount += bodyB.type == b2_dynamicBody ? 1 : 0;
			    }
		    }

		    // Validate the bit population for this graph color
		    DebugTools.Assert( bitCount == b2CountSetBits( &color.bodySet ) );
	    }

	    int contactIdCount = b2GetIdCount( &world.contactIdPool );
	    DebugTools.Assert( totalContactCount == contactIdCount );
	    DebugTools.Assert( totalContactCount == (int)world.broadPhase.pairSet.Count );

	    int jointIdCount = b2GetIdCount( &world.jointIdPool );
	    DebugTools.Assert( totalJointCount == jointIdCount );

    // Validate shapes
    // This is very slow on compounds
    #if 0
	    int shapeCapacity = b2Array(world.shapeArray).Count;
	    for (int shapeIndex = 0; shapeIndex < shapeCapacity; shapeIndex += 1)
	    {
		    b2Shape* shape = world.shapeArray + shapeIndex;
		    if (shape.id != shapeIndex)
		    {
			    continue;
		    }

		    DebugTools.Assert(0 <= shape.bodyId && shape.bodyId < b2Array(world.bodyArray).Count);

		    b2Body* body = world.bodyArray + shape.bodyId;
		    DebugTools.Assert(0 <= body.setIndex && body.setIndex < b2Array(world.solverSetArray).Count);

		    b2SolverSet* set = world.solverSetArray + body.setIndex;
		    DebugTools.Assert(0 <= body.localIndex && body.localIndex < set.sims.Count);

		    b2BodySim* bodySim = set.sims.data + body.localIndex;
		    DebugTools.Assert(bodySim.bodyId == shape.bodyId);

		    bool found = false;
		    int shapeCount = 0;
		    int index = body.headShapeId;
		    while (index != PhysicsConstants.NullIndex)
		    {
			    b2CheckId(world.shapeArray, index);
			    b2Shape* s = world.shapeArray + index;
			    if (index == shapeIndex)
			    {
				    found = true;
			    }

			    index = s.nextShapeId;
			    shapeCount += 1;
		    }

		    DebugTools.Assert(found);
		    DebugTools.Assert(shapeCount == body.shapeCount);
	    }
    #endif
    }void
}
