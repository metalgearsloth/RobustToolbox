using Robust.Shared.GameObjects;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
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
}
