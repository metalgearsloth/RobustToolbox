using System;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    #region Common

    private void PrepareJoint(ref JointSim joint)
    {
        // Clamp joint hertz based on the time step to reduce jitter.
        float hertz = MathF.Min(joint.constraintHertz, 0.25f * _invH);
        joint.constraintSoftness = MakeSoft(hertz, joint.constraintDampingRatio, _h);

        switch (joint.type)
        {
            case b2JointType.b2_distanceJoint:
                PrepareDistanceJoint(ref joint);
                break;

            case b2JointType.b2_motorJoint:
                PrepareMotorJoint(ref joint);
                break;

            case b2JointType.b2_filterJoint:
                break;

            case b2JointType.b2_prismaticJoint:
                PreparePrismaticJoint(ref joint);
                break;

            case b2JointType.b2_revoluteJoint:
                PrepareRevoluteJoint(ref joint);
                break;

            case b2JointType.b2_weldJoint:
                PrepareWeldJoint(ref joint);
                break;

            case b2JointType.b2_wheelJoint:
                PrepareWheelJoint(ref joint);
                break;

            default:
                DebugTools.Assert(false);
                break;
        }
    }

    private void WarmStartJoint(ref JointSim joint)
    {
        switch (joint.type)
        {
            case b2JointType.b2_distanceJoint:
                WarmStartDistanceJoint(ref joint);
                break;

            case b2JointType.b2_motorJoint:
                WarmStartMotorJoint(ref joint);
                break;

            case b2JointType.b2_filterJoint:
                break;

            case b2JointType.b2_prismaticJoint:
                WarmStartPrismaticJoint(ref joint);
                break;

            case b2JointType.b2_revoluteJoint:
                WarmStartRevoluteJoint(ref joint);
                break;

            case b2JointType.b2_weldJoint:
                WarmStartWeldJoint(ref joint);
                break;

            case b2JointType.b2_wheelJoint:
                WarmStartWheelJoint(ref joint);
                break;

            default:
                DebugTools.Assert(false);
                break;
        }
    }

    private void SolveJoint(ref JointSim joint, bool useBias )
    {
        switch (joint.type)
        {
            case b2JointType.b2_distanceJoint:
                SolveDistanceJoint(ref joint, useBias);
                break;

            case b2JointType.b2_motorJoint:
                SolveMotorJoint(ref joint);
                break;

            case b2JointType.b2_filterJoint:
                break;

            case b2JointType.b2_prismaticJoint:
                SolvePrismaticJoint(ref joint, useBias);
                break;

            case b2JointType.b2_revoluteJoint:
                SolveRevoluteJoint(ref joint, useBias);
                break;

            case b2JointType.b2_weldJoint:
                SolveWeldJoint(ref joint, useBias);
                break;

            case b2JointType.b2_wheelJoint:
                SolveWheelJoint(ref joint, useBias);
                break;

            default:
                DebugTools.Assert(false);
                break;
        }
    }

    #endregion

    #region Tasks

    private void PrepareJointsTask(int startIndex, int endIndex)
    {
        for ( int i = startIndex; i < endIndex; ++i )
        {
            ref var joint = ref _contextJoints[i];
            PrepareJoint(ref joint);
        }
    }

    private void WarmStartJointsTask( int startIndex, int endIndex, int colorIndex )
    {
        var color = _constraintGraph.colors[colorIndex];
        ref var joints = ref color.JointSims;
        DebugTools.Assert(0 <= startIndex && startIndex < joints.Count);
        DebugTools.Assert(startIndex <= endIndex && endIndex <= joints.Count);

        for ( int i = startIndex; i < endIndex; ++i )
        {
            ref var joint = ref joints[i];
            WarmStartJoint(ref joint);
        }
    }

    private void SolveJointsTask( int startIndex, int endIndex, int colorIndex, bool useBias,
        int workerIndex )
    {
        var color = _constraintGraph.colors[colorIndex];
        ref var joints = ref color.JointSims;
        DebugTools.Assert(0 <= startIndex && startIndex < color.JointSims.Count);
        DebugTools.Assert(startIndex <= endIndex && endIndex <= color.JointSims.Count);

        // TODO: Need solver context or smth for these
        // TODO: States by ref, probably store them in valuelist.
        var jointStateBitSet = &context.world.taskContexts.data[workerIndex].jointStateBitSet;

        for ( int i = startIndex; i < endIndex; ++i )
        {
            ref var joint = joints[i];
            SolveJoint(ref joint, useBias);

            if (useBias && (joint.forceThreshold < float.MaxValue || joint.torqueThreshold < float.MaxValue) &&
                 b2GetBit( jointStateBitSet, joint.jointId ) == false)
            {
                float force, torque;
                b2GetJointReaction( joint, context.inv_h, &force, &torque );

                // Check thresholds. A zero threshold means all awake joints get reported.
                if ( force >= joint.forceThreshold || torque >= joint.torqueThreshold )
                {
                    // Flag this joint for processing.
                    b2SetBit( jointStateBitSet, joint.jointId );
                }
            }
        }

        b2TracyCZoneEnd( solve_joints );
    }

    #endregion

    #region Overflow

    private void PrepareOverflowJoints()
    {
        var graph = _constraintGraph;
        var joints = graph.colors[PhysicsConstants.OverflowIndex].JointSims;
        var jointCount = graph.colors[PhysicsConstants.OverflowIndex].JointSims.Count;

        for ( int i = 0; i < jointCount; ++i )
        {
            ref var joint = ref joints[i];
            PrepareJoint(ref joint);
        }
    }

    private void WarmStartOverflowJoints()
    {
	    var graph = _constraintGraph;
	    var joints = graph.colors[PhysicsConstants.OverflowIndex].JointSims;
	    int jointCount = graph.colors[PhysicsConstants.OverflowIndex].JointSims.Count;

	    for ( int i = 0; i < jointCount; ++i )
	    {
		    ref var joint = ref joints[i];
		    WarmStartJoint(ref joint);
	    }
    }

    private void SolveOverflowJoints( bool useBias )
    {
	    b2TracyCZoneNC( solve_joints, "SolveJoints", b2_colorLemonChiffon, true );

	    b2ConstraintGraph* graph = context.graph;
	    b2JointSim* joints = graph.colors[PhysicsConstants.OverflowIndex].jointSims.data;
	    int jointCount = graph.colors[PhysicsConstants.OverflowIndex].jointSims.count;

	    for ( int i = 0; i < jointCount; ++i )
	    {
		    b2JointSim* joint = joints + i;
		    SolveJoint( joint, context, useBias );
	    }

	    b2TracyCZoneEnd( solve_joints );
    }

    #endregion
}
