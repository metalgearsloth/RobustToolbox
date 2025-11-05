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
                PrepareMotorJoint(joint);
                break;

            case b2JointType.b2_filterJoint:
                break;

            case b2JointType.b2_prismaticJoint:
                PreparePrismaticJoint(joint);
                break;

            case b2JointType.b2_revoluteJoint:
                PrepareRevoluteJoint(joint);
                break;

            case b2JointType.b2_weldJoint:
                PrepareWeldJoint(joint);
                break;

            case b2JointType.b2_wheelJoint:
                PrepareWheelJoint(joint);
                break;

            default:
                DebugTools.Assert( false );
                break;
        }
    }

    private void WarmStartJoint(ref JointSim joint)
    {
        switch (joint.type)
        {
            case b2JointType.b2_distanceJoint:
                WarmStartDistanceJoint(joint);
                break;

            case b2JointType.b2_motorJoint:
                b2WarmStartMotorJoint(joint);
                break;

            case b2JointType.b2_filterJoint:
                break;

            case b2JointType.b2_prismaticJoint:
                b2WarmStartPrismaticJoint( joint, context );
                break;

            case b2JointType.b2_revoluteJoint:
                WarmStartRevoluteJoint( joint, context );
                break;

            case b2JointType.b2_weldJoint:
                b2WarmStartWeldJoint( joint, context );
                break;

            case b2JointType.b2_wheelJoint:
                b2WarmStartWheelJoint( joint, context );
                break;

            default:
                DebugTools.Assert(false);
                break;
        }
    }

    private void b2SolveJoint( b2JointSim* joint, bool useBias )
    {
        switch ( joint.type )
        {
            case b2_distanceJoint:
                b2SolveDistanceJoint( joint, context, useBias );
                break;

            case b2_motorJoint:
                b2SolveMotorJoint( joint, context );
                break;

            case b2_filterJoint:
                break;

            case b2_prismaticJoint:
                b2SolvePrismaticJoint( joint, context, useBias );
                break;

            case b2_revoluteJoint:
                b2SolveRevoluteJoint( joint, context, useBias );
                break;

            case b2_weldJoint:
                b2SolveWeldJoint( joint, context, useBias );
                break;

            case b2_wheelJoint:
                b2SolveWheelJoint( joint, context, useBias );
                break;

            default:
                DebugTools.Assert( false );
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
        b2TracyCZoneNC( warm_joints, "WarmJoints", b2_colorGold, true );

        b2GraphColor* color = context.graph.colors + colorIndex;
        b2JointSim* joints = color.jointSims.data;
        DebugTools.Assert( 0 <= startIndex && startIndex < color.jointSims.count );
        DebugTools.Assert( startIndex <= endIndex && endIndex <= color.jointSims.count );

        for ( int i = startIndex; i < endIndex; ++i )
        {
            b2JointSim* joint = joints + i;
            WarmStartJoint( joint, context );
        }

        b2TracyCZoneEnd( warm_joints );
    }

    private void SolveJointsTask( int startIndex, int endIndex, int colorIndex, bool useBias,
        int workerIndex )
    {
        b2TracyCZoneNC( solve_joints, "SolveJoints", b2_colorLemonChiffon, true );

        b2GraphColor* color = context.graph.colors + colorIndex;
        b2JointSim* joints = color.jointSims.data;
        DebugTools.Assert( 0 <= startIndex && startIndex < color.jointSims.count );
        DebugTools.Assert( startIndex <= endIndex && endIndex <= color.jointSims.count );

        b2BitSet* jointStateBitSet = &context.world.taskContexts.data[workerIndex].jointStateBitSet;

        for ( int i = startIndex; i < endIndex; ++i )
        {
            b2JointSim* joint = joints + i;
            b2SolveJoint( joint, context, useBias );

            if ( useBias && ( joint.forceThreshold < FLT_MAX || joint.torqueThreshold < FLT_MAX ) &&
                 b2GetBit( jointStateBitSet, joint.jointId ) == false )
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
		    b2SolveJoint( joint, context, useBias );
	    }

	    b2TracyCZoneEnd( solve_joints );
    }

    #endregion
}
