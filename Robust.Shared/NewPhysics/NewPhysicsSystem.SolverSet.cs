using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    // Wake a solver set. Does not merge islands.
    // Contacts can be in several places:
    // 1. non-touching contacts in the disabled set
    // 2. non-touching contacts already in the awake set
    // 3. touching contacts in the sleeping set
    // This handles contact types 1 and 3. Type 2 doesn't need any action.
    internal void WakeSolverSet(int setIndex)
    {
	    DebugTools.Assert(setIndex >= (int) SetType.FirstSleepingSet);
	    var set = _solverSets[setIndex];
	    var awakeSet = _solverSets[(int) SetType.AwakeSet];
	    var disabledSet = _solverSets[(int) SetType.DisabledSet];

	    var bodies = world->bodies.data;

	    int bodyCount = set.bodySims.Count;

	    for ( int i = 0; i < bodyCount; ++i )
	    {
		    b2BodySim* simSrc = set->bodySims.data + i;

		    b2Body* body = bodies + simSrc->bodyId;
		    DebugTools.Assert( body->setIndex == setIndex );
		    body->setIndex = b2_awakeSet;
		    body->localIndex = awakeSet->bodySims.count;

		    // Reset sleep timer
		    body->sleepTime = 0.0f;

		    b2BodySim* simDst = b2BodySimArray_Add( &awakeSet->bodySims );
		    memcpy( simDst, simSrc, sizeof( b2BodySim ) );

		    b2BodyState* state = b2BodyStateArray_Add( &awakeSet->bodyStates );
		    *state = b2_identityBodyState;
		    state->flags = body->flags;

		    // move non-touching contacts from disabled set to awake set
		    int contactKey = body->headContactKey;
		    while ( contactKey != B2_NULL_INDEX )
		    {
			    int edgeIndex = contactKey & 1;
			    int contactId = contactKey >> 1;

			    b2Contact* contact = b2ContactArray_Get( &world->contacts, contactId );

			    contactKey = contact->edges[edgeIndex].nextKey;

			    if ( contact->setIndex != b2_disabledSet )
			    {
				    DebugTools.Assert( contact->setIndex == b2_awakeSet || contact->setIndex == setIndex );
				    continue;
			    }

			    int localIndex = contact->localIndex;
			    b2ContactSim* contactSim = b2ContactSimArray_Get( &disabledSet->contactSims, localIndex );

			    DebugTools.Assert( ( contact->flags & b2_contactTouchingFlag ) == 0 && contactSim->manifold.pointCount == 0 );

			    contact->setIndex = b2_awakeSet;
			    contact->localIndex = awakeSet->contactSims.count;
			    b2ContactSim* awakeContactSim = b2ContactSimArray_Add( &awakeSet->contactSims );
			    memcpy( awakeContactSim, contactSim, sizeof( b2ContactSim ) );

			    int movedLocalIndex = b2ContactSimArray_RemoveSwap( &disabledSet->contactSims, localIndex );
			    if ( movedLocalIndex != B2_NULL_INDEX )
			    {
				    // fix moved element
				    b2ContactSim* movedContactSim = disabledSet->contactSims.data + localIndex;
				    b2Contact* movedContact = b2ContactArray_Get( &world->contacts, movedContactSim->contactId );
				    DebugTools.Assert( movedContact->localIndex == movedLocalIndex );
				    movedContact->localIndex = localIndex;
			    }
		    }
	    }

	    // transfer touching contacts from sleeping set to contact graph
	    {
		    int contactCount = set->contactSims.count;
		    for ( int i = 0; i < contactCount; ++i )
		    {
			    b2ContactSim* contactSim = set->contactSims.data + i;
			    b2Contact* contact = b2ContactArray_Get( &world->contacts, contactSim->contactId );
			    DebugTools.Assert( contact->flags & b2_contactTouchingFlag );
			    DebugTools.Assert( contactSim->simFlags & b2_simTouchingFlag );
			    DebugTools.Assert( contactSim->manifold.pointCount > 0 );
			    DebugTools.Assert( contact->setIndex == setIndex );
			    b2AddContactToGraph( world, contactSim, contact );
			    contact->setIndex = b2_awakeSet;
		    }
	    }

	    // transfer joints from sleeping set to awake set
	    {
		    int jointCount = set->jointSims.count;
		    for ( int i = 0; i < jointCount; ++i )
		    {
			    b2JointSim* jointSim = set->jointSims.data + i;
			    b2Joint* joint = b2JointArray_Get( &world->joints, jointSim->jointId );
			    DebugTools.Assert( joint->setIndex == setIndex );
			    b2AddJointToGraph( world, jointSim, joint );
			    joint->setIndex = b2_awakeSet;
		    }
	    }

	    // transfer island from sleeping set to awake set
	    // Usually a sleeping set has only one island, but it is possible
	    // that joints are created between sleeping islands and they
	    // are moved to the same sleeping set.
	    {
		    int islandCount = set->islandSims.count;
		    for ( int i = 0; i < islandCount; ++i )
		    {
			    b2IslandSim* islandSrc = set->islandSims.data + i;
			    b2Island* island = b2IslandArray_Get( &world->islands, islandSrc->islandId );
			    island->setIndex = b2_awakeSet;
			    island->localIndex = awakeSet->islandSims.count;
			    b2IslandSim* islandDst = b2IslandSimArray_Add( &awakeSet->islandSims );
			    memcpy( islandDst, islandSrc, sizeof( b2IslandSim ) );
		    }
	    }

	    // destroy the sleeping set
	    b2DestroySolverSet( world, setIndex );
    }
}
