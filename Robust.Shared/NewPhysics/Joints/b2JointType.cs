namespace Robust.Shared.NewPhysics.Joints;

/// Joint type enumeration
///
/// This is useful because all joint types use b2JointId and sometimes you
/// want to get the type of a joint.
/// @ingroup joint
public enum b2JointType : byte
{
    b2_distanceJoint,
    b2_filterJoint,
    b2_motorJoint,
    b2_prismaticJoint,
    b2_revoluteJoint,
    b2_weldJoint,
    b2_wheelJoint,
}
