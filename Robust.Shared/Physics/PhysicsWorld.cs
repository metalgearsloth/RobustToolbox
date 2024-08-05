using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Physics;

public sealed class PhysicsWorld
{
    [Dependency] private readonly IEntityManager _entManager = default!;

    public bool AutoClearForces;

    /// <summary>
    /// When substepping the client needs to know about the first position to use for lerping.
    /// </summary>
    public readonly Dictionary<EntityUid, (EntityUid ParentUid, Vector2 LocalPosition, Angle LocalRotation)>
        LerpData = new();

    /// <summary>
    /// Keep a buffer of everything that moved in a tick. This will be used to check for physics contacts.
    /// </summary>
    [ViewVariables]
    internal readonly Dictionary<FixtureProxy, Box2> MoveBuffer = new();

    /// <summary>
    ///     All awake bodies on this map.
    /// </summary>
    [ViewVariables]
    internal readonly HashSet<Entity<PhysicsComponent>> AwakeBodies = new();

    /// <summary>
    ///     Store last tick's invDT
    /// </summary>
    internal float _invDt0;

    // TODO: Make Broadphase its own thing that stores references to all the map / grid broadphases.
    // Kill IBroadPhase
    // Make B2DynamicTree not a child of thing
    // Make the IEnumerables on broadphasecomponent ref structs probably

    public PhysicsBroadphase BroadPhase = new();

    public void AddAwakeEntity(Entity<PhysicsComponent> entity)
    {
        if (!entity.Comp.CanCollide)
        {
            DebugTools.Assert(false);
            return;
        }

        if (entity.Comp.BodyType == BodyType.Static)
        {
            DebugTools.Assert(false);
            return;
        }

        AwakeBodies.Add(entity);
    }

    public bool RemoveAwakeEntity(Entity<PhysicsComponent> entity)
    {
        return AwakeBodies.Remove(entity);
    }
}
