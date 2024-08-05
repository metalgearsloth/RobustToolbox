using System.Collections.Generic;
using Robust.Shared.GameObjects;

namespace Robust.Shared.Physics;

/// <summary>
/// Stores all of the physics broadphase information for a world.
/// This can comprise multiple maps.
/// </summary>
public sealed class PhysicsBroadphase
{
    public List<Entity<BroadphaseComponent>> Subs = new();

    public void AddSub(Entity<BroadphaseComponent> entity)
    {
        Subs.Add(entity);
    }

    public bool RemoveSub(Entity<BroadphaseComponent> entity)
    {
        return Subs.Remove(entity);
    }
}
