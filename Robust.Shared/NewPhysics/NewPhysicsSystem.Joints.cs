using System;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private void PrepareJointsTask(int startIndex, int endIndex, ref StepContext context)
    {
        var joints = context.Joints;

        for ( int i = startIndex; i < endIndex; ++i )
        {
            ref var joint = ref joints[i];
            PrepareJoint(ref joint);
        }
    }

    private void PrepareJoint(ref JointSim joint)
    {
        // Clamp joint hertz based on the time step to reduce jitter.
        float hertz = MathF.Min( joint.constraintHertz, 0.25f * Context.inv_h );
        joint.constraintSoftness = MakeSoft( hertz, joint.constraintDampingRatio, Context.h );

        switch (joint.type)
        {
            case b2_distanceJoint:
                b2PrepareDistanceJoint( joint, context );
                break;

            case b2_motorJoint:
                b2PrepareMotorJoint( joint, context );
                break;

            case b2_filterJoint:
                break;

            case b2_prismaticJoint:
                b2PreparePrismaticJoint( joint, context );
                break;

            case b2_revoluteJoint:
                b2PrepareRevoluteJoint( joint, context );
                break;

            case b2_weldJoint:
                b2PrepareWeldJoint( joint, context );
                break;

            case b2_wheelJoint:
                b2PrepareWheelJoint( joint, context );
                break;

            default:
                DebugTools.Assert( false );
                break;
        }
    }

    void b2WarmStartJoint( b2JointSim* joint, b2StepContext* context )
    {
	    switch ( joint->type )
	    {
		    case b2_distanceJoint:
			    b2WarmStartDistanceJoint( joint, context );
			    break;

		    case b2_motorJoint:
			    b2WarmStartMotorJoint( joint, context );
			    break;

		    case b2_filterJoint:
			    break;

		    case b2_prismaticJoint:
			    b2WarmStartPrismaticJoint( joint, context );
			    break;

		    case b2_revoluteJoint:
			    b2WarmStartRevoluteJoint( joint, context );
			    break;

		    case b2_weldJoint:
			    b2WarmStartWeldJoint( joint, context );
			    break;

		    case b2_wheelJoint:
			    b2WarmStartWheelJoint( joint, context );
			    break;

		    default:
			    B2_ASSERT( false );
	    }
    }

    void b2SolveJoint( b2JointSim* joint, b2StepContext* context, bool useBias )
    {
	    switch ( joint->type )
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
			    B2_ASSERT( false );
	    }
    }

    void b2PrepareOverflowJoints( b2StepContext* context )
    {
	    b2TracyCZoneNC( prepare_joints, "PrepJoints", b2_colorOldLace, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2JointSim* joints = graph->colors[B2_OVERFLOW_INDEX].jointSims.data;
	    int jointCount = graph->colors[B2_OVERFLOW_INDEX].jointSims.count;

	    for ( int i = 0; i < jointCount; ++i )
	    {
		    b2JointSim* joint = joints + i;
		    b2PrepareJoint( joint, context );
	    }

	    b2TracyCZoneEnd( prepare_joints );
    }

    void b2WarmStartOverflowJoints( b2StepContext* context )
    {
	    b2TracyCZoneNC( prepare_joints, "PrepJoints", b2_colorOldLace, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2JointSim* joints = graph->colors[B2_OVERFLOW_INDEX].jointSims.data;
	    int jointCount = graph->colors[B2_OVERFLOW_INDEX].jointSims.count;

	    for ( int i = 0; i < jointCount; ++i )
	    {
		    b2JointSim* joint = joints + i;
		    b2WarmStartJoint( joint, context );
	    }

	    b2TracyCZoneEnd( prepare_joints );
    }

    void b2SolveOverflowJoints( b2StepContext* context, bool useBias )
    {
	    b2TracyCZoneNC( solve_joints, "SolveJoints", b2_colorLemonChiffon, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2JointSim* joints = graph->colors[B2_OVERFLOW_INDEX].jointSims.data;
	    int jointCount = graph->colors[B2_OVERFLOW_INDEX].jointSims.count;

	    for ( int i = 0; i < jointCount; ++i )
	    {
		    b2JointSim* joint = joints + i;
		    b2SolveJoint( joint, context, useBias );
	    }

	    b2TracyCZoneEnd( solve_joints );
    }
}
