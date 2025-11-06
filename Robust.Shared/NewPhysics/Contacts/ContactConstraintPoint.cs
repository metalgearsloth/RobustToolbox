using System.Numerics;

namespace Robust.Shared.NewPhysics.Joints;

public record struct ContactConstraintPoint
{
    internal Vector2 anchorA;
    internal Vector2 anchorB;
    internal float baseSeparation;
    internal float relativeVelocity;
    internal float normalImpulse;
    internal float tangentImpulse;
    internal float totalNormalImpulse;
    internal float normalMass;
    internal float tangentMass;
}
