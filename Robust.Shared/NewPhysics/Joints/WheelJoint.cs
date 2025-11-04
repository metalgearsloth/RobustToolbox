using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Wheel joint definition
/// Body B is a wheel that may rotate freely and slide along the local x-axis in frame A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup wheel_joint
public sealed partial class WheelJoint : BaseJoint
{
    /// Enable a linear spring along the local axis
    [DataField]
    public bool enableSpring;

    /// Spring stiffness in Hertz
    [DataField]
    public float hertz;

    /// Spring damping ratio, non-dimensional
    [DataField]
    public float dampingRatio;

    /// Enable/disable the joint linear limit
    [DataField]
    public bool enableLimit;

    /// The lower translation limit
    [DataField]
    public float lowerTranslation;

    /// The upper translation limit
    [DataField]
    public float upperTranslation;

    /// Enable/disable the joint rotational motor
    [DataField]
    public bool enableMotor;

    /// The maximum motor torque, typically in newton-meters
    [DataField]
    public float maxMotorTorque;

    /// The desired motor speed in radians per second
    [DataField]
    public float motorSpeed;
}
