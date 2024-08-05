using Robust.Shared.GameObjects;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Systems;

public partial class SharedPhysicsSystem
{
    #region AddRemove

    internal void AddAwakeBody(EntityUid uid, PhysicsComponent body, EntityUid mapUid, PhysicsMapComponent? map = null)
    {
        PhysMapQuery.Resolve(mapUid, ref map, false);
        AddAwakeBody(uid, body, map);
    }

    internal void RemoveSleepBody(EntityUid uid, PhysicsComponent body, PhysicsMapComponent? map = null)
    {
        map?.AwakeBodies.Remove(body);
    }

    internal void RemoveSleepBody(EntityUid uid, PhysicsComponent body, EntityUid mapUid, PhysicsMapComponent? map = null)
    {
        PhysMapQuery.Resolve(mapUid, ref map, false);
        RemoveSleepBody(uid, body, map);
    }

    #endregion
}
