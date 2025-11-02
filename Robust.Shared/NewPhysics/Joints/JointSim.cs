using Robust.Shared.Physics;
using Robust.Shared.Physics.Dynamics.Joints;

namespace Robust.Shared.NewPhysics.Joints;

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
internal record struct JointSim
{
    public int jointId;

    public int bodyIdA;
    public int bodyIdB;

    public JointType type;

    public Transform localFrameA;
    public Transform localFrameB;

    public float invMassA, invMassB;
    public float invIA, invIB;

    public float constraintHertz;
    public float constraintDampingRatio;

    public Softness constraintSoftness;

    public float forceThreshold;
    public float torqueThreshold;
}
