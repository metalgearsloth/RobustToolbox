using System.Numerics;
using System.Runtime.InteropServices;
using Robust.Shared.Physics;

namespace Robust.Shared.NewPhysics.Bodies;

[StructLayout(LayoutKind.Explicit)]
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

    [FieldOffset(sizeof(float) * 0)]
    public Vector2 linearVelocity;

    [FieldOffset(sizeof(float) * 2)]
    public float angularVelocity;

    // b2BodyFlags
    // Important flags: locking, dynamic
    // I know it's a uint but 32 bits is 32 bits.
    [FieldOffset(sizeof(float) * 3)]
    public uint flags;

    // Using delta position reduces round-off error far from the origin
    [FieldOffset(sizeof(float) * 4)]
    public Vector2 deltaPosition;

    // Using delta rotation because I cannot access the full rotation on static bodies in
    // the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
    [FieldOffset(sizeof(float) * 6)]
    public Quaternion2D deltaRotation;
}
