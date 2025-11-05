using System.Numerics;
using Robust.Shared.Physics;

namespace Robust.Shared.NewPhysics.Bodies;

internal record struct BodyState
{
    public static readonly BodyState Identity = new()
    {
        linearVelocity = Vector2.Zero,
        angularVelocity = 0f,
        flags = 0,
        deltaPosition = Vector2.Zero,
        deltaRotation = new(1f, 0f),
    };

    public Vector2 linearVelocity;
    public float angularVelocity;

    // b2BodyFlags
    // Important flags: locking, dynamic
    public uint flags;

    // Using delta position reduces round-off error far from the origin
    public Vector2 deltaPosition;

    // Using delta rotation because I cannot access the full rotation on static bodies in
    // the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
    public Quaternion2D deltaRotation;
}
