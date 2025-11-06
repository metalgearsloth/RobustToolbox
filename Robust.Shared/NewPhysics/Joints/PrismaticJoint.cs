using System.Numerics;
using Robust.Shared.Physics;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.NewPhysics.Joints;

/// Prismatic joint definition
/// Body B may slide along the x-axis in local frame A. Body B cannot rotate relative to body A.
/// The joint translation is zero when the local frame origins coincide in world space.
/// @ingroup prismatic_joint
public sealed partial class PrismaticJoint : BaseJoint
{
    /// Enable a linear spring along the prismatic joint axis
    [DataField]
    public bool enableSpring { get; internal set; }

    /// The spring stiffness Hertz, cycles per second
    [DataField]
    public float hertz { get; internal set; }

    /// The spring damping ratio, non-dimensional
    [DataField]
    public float dampingRatio { get; internal set; }

    /// The target translation for the joint in meters. The spring-damper will drive
    /// to this translation.
    [DataField]
    public float targetTranslation { get; internal set; }

    /// Enable/disable the joint limit
    [DataField]
    public bool enableLimit { get; internal set; }

    /// The lower translation limit
    [DataField]
    public float lowerTranslation { get; internal set; }

    /// The upper translation limit
    [DataField]
    public float upperTranslation { get; internal set; }

    /// Enable/disable the joint motor
    [DataField]
    public bool enableMotor { get; internal set; }

    /// The maximum motor force, typically in newtons
    [DataField]
    public float maxMotorForce { get; internal set; }

    /// The desired motor speed, typically in meters per second
    [DataField]
    public float motorSpeed { get; internal set; }

    internal Vector2 impulse;
    internal float springImpulse;
    internal float motorImpulse;
    internal float lowerImpulse;
    internal float upperImpulse;

    internal int IndexA;
    internal int IndexB;
    internal Transform frameA;
    internal Transform frameB;
    internal Vector2 deltaCenter;
    internal Softness springSoftness;
}
