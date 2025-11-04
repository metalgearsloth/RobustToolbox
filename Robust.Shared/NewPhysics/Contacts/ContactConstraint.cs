using System.Numerics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics.Joints;

internal record struct ContactConstraint
{
    public int indexA;
    public int indexB;
    public FixedArray2<ContactConstraintPoint> points;
    public Vector2 normal;
    public float invMassA, invMassB;
    public float invIA, invIB;
    public float friction;
    public float restitution;
    public float tangentSpeed;
    public float rollingResistance;
    public float rollingMass;
    public float rollingImpulse;
    public Softness softness;
    public int pointCount;
}
