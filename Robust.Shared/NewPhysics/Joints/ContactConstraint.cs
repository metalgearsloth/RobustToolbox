using System.Numerics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Joints;

internal sealed class ContactConstraint
{
    int indexA;
    int indexB;
    FixedArray2<ContactConstraintPoint> points;
    Vector2 normal;
    float invMassA, invMassB;
    float invIA, invIB;
    float friction;
    float restitution;
    float tangentSpeed;
    float rollingResistance;
    float rollingMass;
    float rollingImpulse;
    Softness softness;
    int pointCount;
}
