using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Islands;
using Robust.Shared.Physics;
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
    private void WakeSolverSet(int setIndex)
    {
	    DebugTools.Assert(setIndex >= (int) SetType.FirstSleepingSet);
	    var set = _solverSets[setIndex];
	    var awakeSet = _solverSets[(int) SetType.AwakeSet];
	    var disabledSet = _solverSets[(int) SetType.DisabledSet];

        int bodyCount = set.bodySims.Count;

	    for ( int i = 0; i < bodyCount; ++i )
	    {
		    ref var simSrc = ref set.bodySims[i];

		    var body = _bodies[simSrc.bodyId].Comp;
		    DebugTools.Assert( body.SetIndex == setIndex );
		    body.SetIndex = (int) SetType.AwakeSet;
		    body.LocalIndex = awakeSet.bodySims.Count;

		    // Reset sleep timer
		    body.SleepTime = 0.0f;

		    var simDst = new BodySim();
            awakeSet.bodySims.Add(simDst);

		    var state = new BodyState();
		    state = BodyState.Identity;
		    state.flags = body.Flags;
            awakeSet.bodyStates.Add(state);

		    // move non-touching contacts from disabled set to awake set
		    int contactKey = body.headContactKey;
		    while ( contactKey != PhysicsConstants.NullIndex )
		    {
			    int edgeIndex = contactKey & 1;
			    int contactId = contactKey >> 1;

			    var contact = _contacts[contactId];

			    contactKey = contact.edges[edgeIndex].nextKey;

			    if ( contact.setIndex != (int) SetType.DisabledSet )
			    {
				    DebugTools.Assert( contact.setIndex == (int) SetType.AwakeSet || contact.setIndex == setIndex );
				    continue;
			    }

			    int localIndex = contact.localIndex;
			    ref var contactSim = ref disabledSet.contactSims[localIndex];

			    DebugTools.Assert( ( contact.flags & ContactFlags.ContactTouchingFlag ) == 0 && contactSim.manifold.pointCount == 0 );

			    contact.setIndex = (int) SetType.AwakeSet;
			    contact.localIndex = awakeSet.contactSims.Count;
                var awakeContactSim = new ContactSim();
                awakeSet.contactSims.Add(awakeContactSim);

			    var movedContactSim = disabledSet.contactSims.RemoveSwap(localIndex);
                var movedLocalIndex = disabledSet.contactSims.Count;

			    if ( movedLocalIndex != PhysicsConstants.NullIndex )
			    {
				    // fix moved element
				    var movedContact = _contacts[movedContactSim.contactId];
				    DebugTools.Assert( movedContact.localIndex == movedLocalIndex );
				    movedContact.localIndex = localIndex;
			    }
		    }
	    }

	    // transfer touching contacts from sleeping set to contact graph
	    {
		    int contactCount = set.contactSims.Count;
		    for ( int i = 0; i < contactCount; ++i )
		    {
			    ref var contactSim = ref set.contactSims[i];
			    var contact = _contacts[contactSim.contactId];
			    DebugTools.Assert((contact.flags & ContactFlags.ContactTouchingFlag) != 0x0);
			    DebugTools.Assert((contactSim.simFlags & ContactSimFlags.SimTouchingFlag) != 0x0 );
			    DebugTools.Assert(contactSim.manifold.pointCount > 0 );
			    DebugTools.Assert(contact.setIndex == setIndex );
			    AddContactToGraph(ref contactSim, contact);
			    contact.setIndex = (int) SetType.AwakeSet;
		    }
	    }

	    // transfer joints from sleeping set to awake set
	    {
		    int jointCount = set.jointSims.Count;
		    for ( int i = 0; i < jointCount; ++i )
		    {
			    ref var jointSim = ref set.jointSims[i];
			    var joint = _joints[jointSim.jointId];
			    DebugTools.Assert( joint.SetIndex == setIndex );
			    AddJointToGraph(in jointSim, joint);
			    joint.SetIndex = (int) SetType.AwakeSet;
		    }
	    }

	    // transfer island from sleeping set to awake set
	    // Usually a sleeping set has only one island, but it is possible
	    // that joints are created between sleeping islands and they
	    // are moved to the same sleeping set.
	    {
		    int islandCount = set.islandSims.Count;
		    for ( int i = 0; i < islandCount; ++i )
		    {
			    ref var islandSrc = ref set.islandSims[i];
			    var island = _islands[islandSrc.IslandId];
			    island.SetIndex = (int) SetType.AwakeSet;
			    island.LocalIndex = awakeSet.islandSims.Count;
                var islandDst = new IslandSim();
                awakeSet.islandSims.Add(islandDst);
		    }
	    }

	    // destroy the sleeping set
	    DestroySolverSet( setIndex );
    }

    private void DestroySolverSet(int setIndex)
    {
        var set = _solverSets[setIndex];

        set.bodySims.Clear();
        set.bodyStates.Clear();
        set.contactSims.Clear();
        set.jointSims.Clear();
        set.islandSims.Clear();

        set.setIndex = PhysicsConstants.NullIndex;

        _solverSetPool.FreeId(setIndex);
    }
}
