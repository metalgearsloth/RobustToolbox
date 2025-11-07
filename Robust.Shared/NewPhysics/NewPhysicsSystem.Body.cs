using System.Diagnostics.Contracts;
using Robust.Shared.GameObjects;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private bool IsValid(in BodyId id)
    {
        if ( id.index1 < 1 || _bodies.Count < id.index1 )
        {
            // invalid index
            return false;
        }

        var ent = _bodies[id.index1 - 1];
        var body = ent.Comp;

        if ( body.SetIndex == PhysicsConstants.NullIndex )
        {
            // this was freed
            return false;
        }

        DebugTools.Assert(body.LocalIndex != PhysicsConstants.NullIndex);

        if (ent.Owner != id.Uid)
        {
            // this id is orphaned
            return false;
        }

        return true;
    }

    private Entity<PhysicsComponent> GetBodyFullId(BodyId bodyId)
    {
        DebugTools.Assert(IsValid(bodyId));
        return _bodies[bodyId.index1 - 1];
    }

    private BodyId MakeBodyId(int bodyId)
    {
        var body = _bodies[bodyId];
        return new BodyId()
        {
            index1 = bodyId + 1,
            Uid = body.Owner,
        };
    }

    private ref BodySim GetBodySim(PhysicsComponent body)
    {
        // TODO: Dump storing bodysim / bodystate indefinitely and just derive it from physicscomponent.l

        var set = _solverSets[body.SetIndex];
        ref var sims = ref set.bodySims;
        return ref sims[body.LocalIndex];
    }

    [Pure]
    private BodyState? GetBodyState(PhysicsComponent body)
    {
        if (body.SetIndex == (int)SetType.AwakeSet)
        {
            var set = _solverSets[body.SetIndex];
            ref var states = ref set.bodyStates;
            return states[body.LocalIndex];
        }

        return null;
    }

    private void CreateIslandForBody(int setIndex, PhysicsComponent body)
    {
        DebugTools.Assert( body.IslandId == PhysicsConstants.NullIndex );
        DebugTools.Assert( body.islandPrev == PhysicsConstants.NullIndex );
        DebugTools.Assert( body.islandNext == PhysicsConstants.NullIndex );
        DebugTools.Assert( setIndex != (int) SetType.DisabledSet );

        var island = CreateIsland( setIndex );

        body.IslandId = island.islandId;
        island.headBody = body.Id;
        island.tailBody = body.Id;
        island.bodyCount = 1;
    }

    private void RemoveBodyFromIsland(Entity<PhysicsComponent> ent)
    {
        var body = ent.Comp;

        if ( body.IslandId == PhysicsConstants.NullIndex )
        {
            DebugTools.Assert( body.islandPrev == PhysicsConstants.NullIndex );
            DebugTools.Assert( body.islandNext == PhysicsConstants.NullIndex );
            return;
        }

        int islandId = body.IslandId;
        var island = _islands[islandId];

        // Fix the island's linked list of sims
        if ( body.islandPrev != PhysicsConstants.NullIndex )
        {
            var prevBody = _bodies[body.islandPrev].Comp;
            prevBody.islandNext = body.islandNext;
        }

        if ( body.islandNext != PhysicsConstants.NullIndex )
        {
            var nextBody = _bodies[body.islandNext].Comp;
            nextBody.islandPrev = body.islandPrev;
        }

        DebugTools.Assert( island.bodyCount > 0 );
        island.bodyCount -= 1;
        bool islandDestroyed = false;

        if ( island.headBody == body.Id )
        {
            island.headBody = body.islandNext;

            if ( island.headBody == PhysicsConstants.NullIndex )
            {
                // Destroy empty island
                DebugTools.Assert( island.tailBody == body.Id );
                DebugTools.Assert( island.bodyCount == 0 );
                DebugTools.Assert( island.contactCount == 0 );
                DebugTools.Assert( island.jointCount == 0 );

                // Free the island
                DestroyIsland( island.islandId );
                islandDestroyed = true;
            }
        }
        else if ( island.tailBody == body.Id )
        {
            island.tailBody = body.islandPrev;
        }

        if ( islandDestroyed == false )
        {
            ValidateIsland( islandId );
        }

        body.IslandId = PhysicsConstants.NullIndex;
        body.islandPrev = PhysicsConstants.NullIndex;
        body.islandNext = PhysicsConstants.NullIndex;
    }

    public void DestroyBodyContacts(PhysicsComponent body, bool wakeBodies)
    {
        var edgeKey = body.headContactKey;

        while (edgeKey != PhysicsConstants.NullIndex)
        {
            int contactId = edgeKey >> 1;
            int edgeIndex = edgeKey & 1;

            var contact = _contacts[contactId];
            edgeKey = contact.edges.AsSpan[edgeIndex].nextKey;
            DestroyContact( contact, wakeBodies );
        }

        ValidateSolverSets();
    }

    private BodyId CreateBody()
    {
        // TODO: Here's the crux because they have a def that can be used as a datafield to load shit in.

    }

    public bool WakeBody(Entity<PhysicsComponent> entity)
    {
        var body = entity.Comp;

        if ( body.SetIndex >= (int) SetType.FirstSleepingSet )
        {
            WakeSolverSet(body.SetIndex);
            ValidateSolverSets();
            return true;
        }

        return false;
    }
}
