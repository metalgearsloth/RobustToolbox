using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Revolute joint definition
/// A point on body B is fixed to a point on body A. Allows relative rotation.
/// @ingroup revolute_joint
public sealed partial class RevoluteJoint : BaseJoint
{
    /// The target angle for the joint in radians. The spring-damper will drive
    /// to this angle.
    [DataField]
    public float targetAngle;

    /// Enable a rotational spring on the revolute hinge axis
    [DataField]
    public bool enableSpring;

    /// The spring stiffness Hertz, cycles per second
    [DataField]
    public float hertz;

    /// The spring damping ratio, non-dimensional
    [DataField]
    public float dampingRatio;

    /// A flag to enable joint limits
    [DataField]
    public bool enableLimit;

    /// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
    [DataField]
    public float lowerAngle;

    /// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
    [DataField]
    public float upperAngle;

    /// A flag to enable the joint motor
    [DataField]
    public bool enableMotor;

    /// The maximum motor torque, typically in newton-meters
    [DataField]
    public float maxMotorTorque;

    /// The desired motor speed in radians per second
    [DataField]
    public float motorSpeed;
}
