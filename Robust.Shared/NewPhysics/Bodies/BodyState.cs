using System.Numerics;

namespace Robust.Shared.NewPhysics.Bodies;

public record struct BodyState
{
    Vector2 linearVelocity;
    float angularVelocity;

    // b2BodyFlags
    // Important flags: locking, dynamic
    uint flags;

    // Using delta position reduces round-off error far from the origin
    Vector2 deltaPosition;

    // Using delta rotation because I cannot access the full rotation on static bodies in
    // the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
    Quaternion deltaRotation;
}
