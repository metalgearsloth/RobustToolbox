using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

public partial class SharedPhysicsSystem
{
    private PhysicsWorld _defaultWorld = default!;

    private readonly List<PhysicsWorld> _worlds = new();

    private void InitializeWorlds()
    {
        _defaultWorld = CreateWorld();
    }

    private void ShutdownWorlds()
    {
        DestroyWorld(_defaultWorld);
    }

    public PhysicsWorld CreateWorld()
    {
        var world = new PhysicsWorld()
        {
            AutoClearForces = _cfg.GetCVar(CVars.AutoClearForces)
        };

        _worlds.Add(world);
        return world;
    }

    public void DestroyWorld(PhysicsWorld world)
    {
        _worlds.Remove(world);
    }

    #region AddRemove

    internal void AddAwakeBody(Entity<PhysicsComponent> entity)
    {
        if (!entity.Comp.CanCollide)
        {
            Log.Error($"Tried to add non-colliding {ToPrettyString(entity)} as an awake body to map!");
            DebugTools.Assert(false);
            return;
        }

        if (entity.Comp.BodyType == BodyType.Static)
        {
            Log.Error($"Tried to add static body {ToPrettyString(entity)} as an awake body!");
            DebugTools.Assert(false);
            return;
        }

        DebugTools.Assert(entity.Comp.Awake);
        DefaultWorld.AwakeBodies.Add(entity);
    }

    internal void RemoveSleepBody(Entity<PhysicsComponent> entity)
    {
        DefaultWorld.AwakeBodies.Remove(entity);
    }

    #endregion
}
