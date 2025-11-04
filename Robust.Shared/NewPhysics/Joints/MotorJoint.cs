using System.Numerics;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// A motor joint is used to control the relative velocity and or transform between two bodies.
/// With a velocity of zero this acts like top-down friction.
/// @ingroup motor_joint
public sealed partial class MotorJoint : BaseJoint
{
    /// The desired linear velocity
    [DataField]
    public Vector2 linearVelocity;

    /// The maximum motor force in newtons
    [DataField]
    public float maxVelocityForce;

    /// The desired angular velocity
    [DataField]
    public float angularVelocity;

    /// The maximum motor torque in newton-meters
    [DataField]
    public float maxVelocityTorque;

    /// Linear spring hertz for position control
    [DataField]
    public float linearHertz;

    /// Linear spring damping ratio
    [DataField]
    public float linearDampingRatio;

    /// Maximum spring force in newtons
    [DataField]
    public float maxSpringForce;

    /// Angular spring hertz for position control
    [DataField]
    public float angularHertz;

    /// Angular spring damping ratio
    [DataField]
    public float angularDampingRatio;

    /// Maximum spring torque in newton-meters
    [DataField]
    public float maxSpringTorque;
}
