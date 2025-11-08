using System;
using Robust.Shared.NewPhysics.Islands;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private sealed class SplitIslandJob : IRobustJob
    {
        public NewPhysicsSystem System = default!;

        public void Execute()
        {
            DebugTools.Assert(System._splitIslandId != PhysicsConstants.NullIndex);
            System.SplitIsland(System._splitIslandId);
        }
    }

    private Island CreateIsland(int setIndex)
    {
        DebugTools.Assert(setIndex is (int) SetType.AwakeSet or >= (int) SetType.FirstSleepingSet);

        int islandId = _islandIdPool.AllocId();

        if (islandId == _islands.Count)
        {
            var emptyIsland = new Island();
            _islands.Add(emptyIsland);
        }
        else
        {
            DebugTools.Assert(_islands[islandId].SetIndex == PhysicsConstants.NullIndex);
        }

        var set = _solverSets[setIndex];

        var island = _islands[islandId];
        island.SetIndex = setIndex;
        island.LocalIndex = set.islandSims.Count;
        island.islandId = islandId;
        island.headBody = PhysicsConstants.NullIndex;
        island.tailBody = PhysicsConstants.NullIndex;
        island.bodyCount = 0;
        island.headContact = PhysicsConstants.NullIndex;
        island.tailContact = PhysicsConstants.NullIndex;
        island.contactCount = 0;
        island.headJoint = PhysicsConstants.NullIndex;
        island.tailJoint = PhysicsConstants.NullIndex;
        island.jointCount = 0;
        island.constraintRemoveCount = 0;

        var islandSim = new IslandSim();
        islandSim.IslandId = islandId;
        set.islandSims.Add(islandSim);

        return island;
    }

    private void AddContactToIsland( int islandId, b2Contact contact )
    {
        DebugTools.Assert( contact.islandId == PhysicsConstants.NullIndex );
        DebugTools.Assert( contact.islandPrev == PhysicsConstants.NullIndex );
        DebugTools.Assert( contact.islandNext == PhysicsConstants.NullIndex );

        var island = _islands[islandId];

        if ( island.headContact != PhysicsConstants.NullIndex )
        {
            contact.islandNext = island.headContact;
            var headContact = _contacts[island.headContact];
            headContact.islandPrev = contact.contactId;
        }

        island.headContact = contact.contactId;
        if ( island.tailContact == PhysicsConstants.NullIndex )
        {
            island.tailContact = island.headContact;
        }

        island.contactCount += 1;
        contact.islandId = islandId;

        ValidateIsland(islandId);
    }

    private void DestroyIsland(int islandId)
    {
        if ( _splitIslandId == islandId )
        {
            _splitIslandId = PhysicsConstants.NullIndex;
        }

        // assume island is empty
        var island = _islands[islandId];
        var set = _solverSets[island.SetIndex];
        var movedElement = set.islandSims.RemoveSwap(island.LocalIndex);
        var movedIndex = set.islandSims.Count;

        if ( movedIndex != PhysicsConstants.NullIndex )
        {
            // Fix index on moved element
            int movedId = movedElement.IslandId;
            var movedIsland = _islands[movedId];
            DebugTools.Assert( movedIsland.LocalIndex == movedIndex );
            movedIsland.LocalIndex = island.LocalIndex;
        }

        // Free island and id (preserve island revision)
        island.islandId = PhysicsConstants.NullIndex;
        island.SetIndex = PhysicsConstants.NullIndex;
        island.LocalIndex = PhysicsConstants.NullIndex;
        _islandIdPool.FreeId(islandId);
    }

    private int MergeIslands( int islandIdA, int islandIdB )
    {
	    if ( islandIdA == islandIdB )
	    {
		    return islandIdA;
	    }

	    if ( islandIdA == PhysicsConstants.NullIndex )
	    {
		    DebugTools.Assert( islandIdB != PhysicsConstants.NullIndex );
		    return islandIdB;
	    }

	    if ( islandIdB == PhysicsConstants.NullIndex )
	    {
		    DebugTools.Assert( islandIdA != PhysicsConstants.NullIndex );
		    return islandIdA;
	    }

	    var islandA = _islands[islandIdA];
	    var islandB = _islands[islandIdB];

	    // Keep the biggest island to reduce cache misses
	    Island big;
	    Island small;

	    if ( islandA.bodyCount >= islandB.bodyCount )
	    {
		    big = islandA;
		    small = islandB;
	    }
	    else
	    {
		    big = islandB;
		    small = islandA;
	    }

	    int bigId = big.islandId;

	    // remap island indices (cache misses)
	    int bodyId = small.headBody;
	    while ( bodyId != PhysicsConstants.NullIndex )
	    {
		    var ent = _bodies[bodyId];
            var body = ent.Comp;
            body.IslandId = bigId;
            bodyId = body.IslandNext;
	    }

	    int contactId = small.headContact;
	    while ( contactId != PhysicsConstants.NullIndex )
	    {
		    var contact = _contacts[contactId];
		    contact.IslandId = bigId;
		    contactId = contact.IslandNext;
	    }

	    int jointId = small.headJoint;
	    while ( jointId != PhysicsConstants.NullIndex )
	    {
		    var joint = _joints[jointId];
		    joint.IslandId = bigId;
		    jointId = joint.IslandNext;
	    }

	    // connect body lists
	    DebugTools.Assert( big.tailBody != PhysicsConstants.NullIndex );
	    var tailBody = _bodies[big.tailBody].Comp;
	    DebugTools.Assert( tailBody.islandNext == PhysicsConstants.NullIndex );
	    tailBody.islandNext = small.headBody;

	    DebugTools.Assert( small.headBody != PhysicsConstants.NullIndex );
	    var headBody = _bodies[small.headBody];
	    DebugTools.Assert( headBody.islandPrev == PhysicsConstants.NullIndex );
	    headBody.islandPrev = big.tailBody;

	    big.tailBody = small.tailBody;
	    big.bodyCount += small.bodyCount;

	    // connect contact lists
	    if ( big.headContact == PhysicsConstants.NullIndex )
	    {
		    // Big island has no contacts
		    DebugTools.Assert( big.tailContact == PhysicsConstants.NullIndex && big.contactCount == 0 );
		    big.headContact = small.headContact;
		    big.tailContact = small.tailContact;
		    big.contactCount = small.contactCount;
	    }
	    else if ( small.headContact != PhysicsConstants.NullIndex )
	    {
		    // Both islands have contacts
		    DebugTools.Assert( small.tailContact != PhysicsConstants.NullIndex && small.contactCount > 0 );
		    DebugTools.Assert( big.tailContact != PhysicsConstants.NullIndex && big.contactCount > 0 );

		    var tailContact = _contacts[big.tailContact];
		    DebugTools.Assert( tailContact.islandNext == PhysicsConstants.NullIndex );
		    tailContact.islandNext = small.headContact;

		    var headContact = _contacts[small.headContact];
		    DebugTools.Assert( headContact.islandPrev == PhysicsConstants.NullIndex );
		    headContact.islandPrev = big.tailContact;

		    big.tailContact = small.tailContact;
		    big.contactCount += small.contactCount;
	    }

	    if ( big.headJoint == PhysicsConstants.NullIndex )
	    {
		    // Root island has no joints
		    DebugTools.Assert( big.tailJoint == PhysicsConstants.NullIndex && big.jointCount == 0 );
		    big.headJoint = small.headJoint;
		    big.tailJoint = small.tailJoint;
		    big.jointCount = small.jointCount;
	    }
	    else if ( small.headJoint != PhysicsConstants.NullIndex )
	    {
		    // Both islands have joints
		    DebugTools.Assert( small.tailJoint != PhysicsConstants.NullIndex && small.jointCount > 0 );
		    DebugTools.Assert( big.tailJoint != PhysicsConstants.NullIndex && big.jointCount > 0 );

		    var tailJoint = _joints[big.tailJoint];
		    DebugTools.Assert( tailJoint.IslandNext == PhysicsConstants.NullIndex );
		    tailJoint.IslandNext = small.headJoint;

		    var headJoint = _joints[small.headJoint];
		    DebugTools.Assert( headJoint.IslandPrev == PhysicsConstants.NullIndex );
		    headJoint.IslandPrev = big.tailJoint;

		    big.tailJoint = small.tailJoint;
		    big.jointCount += small.jointCount;
	    }

	    // Track removed constraints
	    big.constraintRemoveCount += small.constraintRemoveCount;

	    small.bodyCount = 0;
	    small.contactCount = 0;
	    small.jointCount = 0;
	    small.headBody = PhysicsConstants.NullIndex;
	    small.headContact = PhysicsConstants.NullIndex;
	    small.headJoint = PhysicsConstants.NullIndex;
	    small.tailBody = PhysicsConstants.NullIndex;
	    small.tailContact = PhysicsConstants.NullIndex;
	    small.tailJoint = PhysicsConstants.NullIndex;
	    small.constraintRemoveCount = 0;

	    DestroyIsland(small.islandId);

	    ValidateIsland(bigId);

	    return bigId;
    }

    private void UnlinkJoint(BaseJoint joint)
    {
        if (joint.IslandId == PhysicsConstants.NullIndex)
        {
            return;
        }

        // remove from island
        int islandId = joint.IslandId;
        var island = _islands[islandId];

        if (joint.IslandPrev != PhysicsConstants.NullIndex)
        {
            var prevJoint = _joints[joint.IslandPrev];
            DebugTools.Assert( prevJoint.IslandNext == joint.JointId );
            prevJoint.IslandNext = joint.IslandNext;
        }

        if (joint.IslandNext != PhysicsConstants.NullIndex)
        {
            var nextJoint = _joints[joint.IslandNext];
            DebugTools.Assert( nextJoint.IslandPrev == joint.JointId );
            nextJoint.IslandPrev = joint.IslandPrev;
        }

        if (island.headJoint == joint.JointId)
        {
            island.headJoint = joint.IslandNext;
        }

        if (island.tailJoint == joint.JointId)
        {
            island.tailJoint = joint.IslandPrev;
        }

        DebugTools.Assert(island.jointCount > 0);
        island.jointCount -= 1;
        island.constraintRemoveCount += 1;

        joint.IslandId = PhysicsConstants.NullIndex;
        joint.IslandPrev = PhysicsConstants.NullIndex;
        joint.IslandNext = PhysicsConstants.NullIndex;

        ValidateIsland(islandId);
    }

    // Possible optimizations:
    // 2. start from the sleepy bodies and stop processing if a sleep body is connected to a non-sleepy body
    // 3. use a sleepy flag on bodies to avoid velocity access
    private void SplitIsland(int baseId )
    {
	    var baseIsland = _islands[baseId];
	    int setIndex = baseIsland.SetIndex;

	    if ( setIndex != (int) SetType.AwakeSet )
	    {
		    // can only split awake island
		    return;
	    }

	    if ( baseIsland.constraintRemoveCount == 0 )
	    {
		    // this island doesn't need to be split
		    return;
	    }

	    ValidateIsland(baseId);

	    int bodyCount = baseIsland.bodyCount;

        // No lock is needed because I ensure the allocator is not used while this task is active.
	    Span<int> stack = stackalloc int[bodyCount];
        Span<int> bodyIds = stackalloc int[bodyCount];

	    // Build array containing all body indices from base island. These
	    // serve as seed bodies for the depth first search (DFS).
	    int index = 0;
	    int nextBody = baseIsland.headBody;
	    while ( nextBody != PhysicsConstants.NullIndex )
	    {
		    bodyIds[index++] = nextBody;
		    var bodyEnt = _bodies[nextBody];

		    nextBody = bodyEnt.Comp.islandNext;
	    }

	    DebugTools.Assert( index == bodyCount );

	    // Each island is found as a depth first search starting from a seed body
	    for ( int i = 0; i < bodyCount; ++i )
	    {
		    int seedIndex = bodyIds[i];
		    var seed = _bodies[seedIndex].Comp;
		    DebugTools.Assert( seed.SetIndex == setIndex );

		    if ( seed.IslandId != baseId )
		    {
			    // The body has already been visited
			    continue;
		    }

		    int stackCount = 0;
		    stack[stackCount++] = seedIndex;

		    // Create new island
		    // No lock needed because only a single island can split per time step. No islands are being used during the constraint
		    // solve. However, islands are touched during body finalization.
		    var island = CreateIsland(setIndex);

		    int islandId = island.islandId;
		    seed.islandId = islandId;

		    // Perform a depth first search (DFS) on the constraint graph.
		    while ( stackCount > 0 )
		    {
			    // Grab the next body off the stack and add it to the island.
			    int bodyId = stack[--stackCount];
			    var body = _bodies[bodyId].Comp;
			    DebugTools.Assert( body.SetIndex == (int) SetType.AwakeSet) );
			    DebugTools.Assert( body.IslandId == islandId );

			    // Add body to island
			    if ( island.tailBody != PhysicsConstants.NullIndex )
			    {
				    _bodies[island.tailBody].islandNext = bodyId;
			    }
			    body.islandPrev = island.tailBody;
			    body.islandNext = PhysicsConstants.NullIndex;
			    island.tailBody = bodyId;

			    if ( island.headBody == PhysicsConstants.NullIndex )
			    {
				    island.headBody = bodyId;
			    }

			    island.bodyCount += 1;

			    // Search all contacts connected to this body.
			    int contactKey = body.headContactKey;
			    while ( contactKey != PhysicsConstants.NullIndex )
			    {
				    int contactId = contactKey >> 1;
				    int edgeIndex = contactKey & 1;

				    var contact = _contacts[contactId];
				    DebugTools.Assert( contact.contactId == contactId );
                    var edgeSpan = contact.edges.AsSpan;

				    // Next key
				    contactKey = edgeSpan[edgeIndex].nextKey;

				    // Has this contact already been added to this island?
				    if ( contact.islandId == islandId )
				    {
					    continue;
				    }

				    // Is this contact enabled and touching?
				    if ( ( contact.flags & ContactFlags.ContactTouchingFlag ) == 0 )
				    {
					    continue;
				    }

				    int otherEdgeIndex = edgeIndex ^ 1;
				    int otherBodyId = edgeSpan[otherEdgeIndex].bodyId;
				    var otherBody = _bodies[otherBodyId];

				    // Maybe add other body to stack
				    if ( otherBody.IslandId != islandId && otherBody.SetIndex != (int) SetType.StaticSet )
				    {
					    DebugTools.Assert( stackCount < bodyCount );
					    stack[stackCount++] = otherBodyId;

					    // Need to update the body's island id immediately so it is not traversed again
					    otherBody.IslandId = islandId;
				    }

				    // Add contact to island
				    contact.islandId = islandId;
				    if ( island.tailContact != PhysicsConstants.NullIndex )
				    {
					    var tailContact = _contacts[island.tailContact];
					    tailContact.islandNext = contactId;
				    }
				    contact.islandPrev = island.tailContact;
				    contact.islandNext = PhysicsConstants.NullIndex;
				    island.tailContact = contactId;

				    if ( island.headContact == PhysicsConstants.NullIndex )
				    {
					    island.headContact = contactId;
				    }

				    island.contactCount += 1;
			    }

			    // Search all joints connect to this body.
			    int jointKey = body.HeadJointKey;
			    while ( jointKey != PhysicsConstants.NullIndex )
			    {
				    int jointId = jointKey >> 1;
				    int edgeIndex = jointKey & 1;

				    var joint = _joints[jointId];
				    DebugTools.Assert( joint.JointId == jointId );

				    // Next key
                    var jointEdges = joint.Edges.AsSpan;

				    jointKey = jointEdges[edgeIndex].nextKey;

				    // Has this joint already been added to this island?
				    if (joint.IslandId == islandId)
				    {
					    continue;
				    }

				    // todo redundant with test below?
				    if ( joint.SetIndex == (int) SetType.DisabledSet )
				    {
					    continue;
				    }

				    int otherEdgeIndex = edgeIndex ^ 1;
				    int otherBodyId = jointEdges[otherEdgeIndex].bodyId;
				    var otherent = _bodies[otherBodyId];
                    var otherBody = otherent.Comp;

				    // Don't simulate joints connected to disabled bodies.
				    if ( otherBody.SetIndex == (int) SetType.DisabledSet )
				    {
					    continue;
				    }

				    // At least one body must be dynamic
				    if ( body.Comp.BodyType != BodyType.Dynamic && otherBody.BodyType != BodyType.Dynamic )
				    {
					    continue;
				    }

				    // Maybe add other body to stack
				    if ( otherBody.IslandId != islandId && otherBody.SetIndex == (int) SetType.AwakeSet )
				    {
					    DebugTools.Assert( stackCount < bodyCount );
					    stack[stackCount++] = otherBodyId;

					    // Need to update the body's island id immediately so it is not traversed again
					    otherBody.islandId = islandId;
				    }

				    // Add joint to island
				    joint.islandId = islandId;
				    if ( island.tailJoint != PhysicsConstants.NullIndex )
				    {
					    var tailJoint = _joints[island.tailJoint];
					    tailJoint.islandNext = jointId;
				    }
				    joint.islandPrev = island.tailJoint;
				    joint.islandNext = PhysicsConstants.NullIndex;
				    island.tailJoint = jointId;

				    if ( island.headJoint == PhysicsConstants.NullIndex )
				    {
					    island.headJoint = jointId;
				    }

				    island.jointCount += 1;
			    }
		    }

		    ValidateIsland(islandId);
	    }

	    // Done with the base split island. This is delayed because the baseId is used as a marker and it
	    // should not be recycled in while splitting.
	    DestroyIsland(baseId);

	    b2FreeArenaItem( alloc, bodyIds );
	    b2FreeArenaItem( alloc, stack );
    }
}
