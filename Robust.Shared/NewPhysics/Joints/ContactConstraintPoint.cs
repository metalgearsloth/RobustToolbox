using System.Numerics;

namespace Robust.Shared.NewPhysics.Joints;

public record struct ContactConstraintPoint
{
    Vector2 anchorA, anchorB;
    float baseSeparation;
    float relativeVelocity;
    float normalImpulse;
    float tangentImpulse;
    float totalNormalImpulse;
    float normalMass;
    float tangentMass;
}
