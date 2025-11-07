using Robust.Shared.GameObjects;
using Robust.Shared.Physics;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    public Transform GetPhysicsTransform(EntityUid uid, TransformComponent? xform = null)
    {
        if (!EntityManager.TransformQuery.Resolve(uid, ref xform))
            return Physics.Transform.Empty;

        var (worldPos, worldRot) = XformSystem.GetWorldPositionRotation(xform);

        return new Transform(worldPos, worldRot);
    }
}
