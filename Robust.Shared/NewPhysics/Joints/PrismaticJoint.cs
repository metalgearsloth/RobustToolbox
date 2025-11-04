using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Prismatic joint definition
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup prismatic_joint
public sealed class PrismaticJoint
{
    /// Enable a linear spring along the prismatic joint axis
    [DataField]
    public bool enableSpring;

    /// The spring stiffness Hertz, cycles per second
    [DataField]
    public float hertz;

    /// The spring damping ratio, non-dimensional
    [DataField]
    public float dampingRatio;

    /// The target translation for the joint in meters. The spring-damper will drive
    /// to this translation.
    [DataField]
    public float targetTranslation;

    /// Enable/disable the joint limit
    [DataField]
    public bool enableLimit;

    /// The lower translation limit
    [DataField]
    public float lowerTranslation;

    /// The upper translation limit
    [DataField]
    public float upperTranslation;

    /// Enable/disable the joint motor
    [DataField]
    public bool enableMotor;

    /// The maximum motor force, typically in newtons
    [DataField]
    public float maxMotorForce;

    /// The desired motor speed, typically in meters per second
    [DataField]
    public float motorSpeed;
}
