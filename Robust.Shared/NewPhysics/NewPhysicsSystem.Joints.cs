using System;
using System.Collections;
using System.Numerics;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private void DestroyJointInternal(BaseJoint joint, bool wakeBodies)
    {
	    int jointId = joint.JointId;

	    ref var edgeA = ref joint.Edges._00;
	    ref var edgeB = ref joint.Edges._01;

	    int idA = edgeA.bodyId;
	    int idB = edgeB.bodyId;
	    var bodyA = _bodies[idA];
	    var bodyB = _bodies[idB];

	    // Remove from body A
	    if (edgeA.prevKey != PhysicsConstants.NullIndex)
	    {
		    var prevJoint = _joints[edgeA.prevKey >> 1];
		    ref var prevEdge = ref prevJoint.Edges.AsSpan[edgeA.prevKey & 1];
		    prevEdge.nextKey = edgeA.nextKey;
	    }

	    if (edgeA.nextKey != PhysicsConstants.NullIndex)
	    {
		    var nextJoint = _joints[edgeA.nextKey >> 1];
		    ref var nextEdge = ref nextJoint.Edges.AsSpan[edgeA.nextKey & 1];
		    nextEdge.prevKey = edgeA.prevKey;
	    }

	    int edgeKeyA = ( jointId << 1 ) | 0;
	    if (bodyA.Comp.headJointKey == edgeKeyA)
	    {
		    bodyA.Comp.headJointKey = edgeA.nextKey;
	    }

	    bodyA.Comp.JointCount -= 1;

	    // Remove from body B
	    if (edgeB.prevKey != PhysicsConstants.NullIndex)
	    {
		    var prevJoint = _joints[edgeB.prevKey >> 1];
		    var prevEdge = prevJoint.Edges.AsSpan[edgeB.prevKey & 1];
		    prevEdge.nextKey = edgeB.nextKey;
	    }

	    if ( edgeB.nextKey != PhysicsConstants.NullIndex )
	    {
		    var nextJoint = _joints[edgeB.nextKey >> 1];
		    var nextEdge = nextJoint.Edges.AsSpan[edgeB.nextKey & 1];
		    nextEdge.prevKey = edgeB.prevKey;
	    }

	    int edgeKeyB = ( jointId << 1 ) | 1;
	    if ( bodyB.Comp.headJointKey == edgeKeyB )
	    {
		    bodyB.Comp.headJointKey = edgeB.nextKey;
	    }

	    bodyB.Comp.JointCount -= 1;

	    if ( joint.IslandId != PhysicsConstants.NullIndex )
	    {
		    DebugTools.Assert( joint.SetIndex > (int) SetType.DisabledSet );
		    UnlinkJoint(joint);
	    }
	    else
	    {
		    DebugTools.Assert(joint.SetIndex <= (int) SetType.DisabledSet);
	    }

	    // Remove joint from solver set that owns it
	    int setIndex = joint.SetIndex;
	    int localIndex = joint.LocalIndex;

	    if ( setIndex == (int) SetType.AwakeSet)
	    {
		    RemoveJointFromGraph(joint.Edges._00.bodyId, joint.Edges._01.bodyId, joint.ColorIndex, localIndex );
	    }
	    else
	    {
		    var set = _solverSets[setIndex];
		    int movedIndex = b2JointSimArray_RemoveSwap( &set.jointSims, localIndex );
		    if ( movedIndex != B2_NULL_INDEX )
		    {
			    // Fix moved joint
			    b2JointSim* movedJointSim = set.jointSims.data + localIndex;
			    int movedId = movedJointSim.jointId;
			    b2Joint* movedJoint = b2JointArray_Get( &world.joints, movedId );
			    B2_ASSERT( movedJoint.localIndex == movedIndex );
			    movedJoint.localIndex = localIndex;
		    }
	    }

	    // Free joint and id (preserve joint generation)
	    joint.setIndex = B2_NULL_INDEX;
	    joint.localIndex = B2_NULL_INDEX;
	    joint.colorIndex = B2_NULL_INDEX;
	    joint.jointId = B2_NULL_INDEX;
	    b2FreeId( &world.jointIdPool, jointId );

	    if ( wakeBodies )
	    {
		    WakeBody(bodyA);
		    WakeBody(bodyB);
	    }

	    ValidateSolverSets();
    }

    public void DestroyJoint(BaseJoint joint, bool wakeAttached)
    {
	    DebugTools.Assert(!_locked);

	    if (_locked)
	    {
		    return;
	    }

	    DestroyJointInternal(joint, wakeAttached);
    }

    b2Joint* b2GetJointFullId( b2World* world, b2JointId jointId )
    {
        int id = jointId.index1 - 1;
        b2Joint* joint = b2JointArray_Get( &world.joints, id );
        B2_ASSERT( joint.jointId == id && joint.generation == jointId.generation );
        return joint;
    }

    b2JointSim* b2GetJointSim( b2World* world, b2Joint* joint )
    {
        if ( joint.setIndex == b2_awakeSet )
        {
            B2_ASSERT( 0 <= joint.colorIndex && joint.colorIndex < B2_GRAPH_COLOR_COUNT );
            b2GraphColor* color = world.constraintGraph.colors + joint.colorIndex;
            return b2JointSimArray_Get( &color.jointSims, joint.localIndex );
        }

        b2SolverSet* set = b2SolverSetArray_Get( &world.solverSets, joint.setIndex );
        return b2JointSimArray_Get( &set.jointSims, joint.localIndex );
    }

    private void GetJointReaction(ref JointSim sim, float invTimeStep, ref float force, ref float torque)
    {
	    float linearImpulse = 0.0f;
	    float angularImpulse = 0.0f;

	    switch ( sim.type )
	    {
		    case b2JointType.b2_distanceJoint:
		    {
			    var joint = (DistanceJoint) _joints[sim.jointId];
			    linearImpulse = MathF.Abs(joint.Impulse + joint.LowerImpulse - joint.UpperImpulse + joint.MotorImpulse);
		    }
		    break;

		    case b2JointType.b2_motorJoint:
		    {
                var joint = (MotorJoint) _joints[sim.jointId];
			    linearImpulse = (joint.linearVelocityImpulse + joint.linearSpringImpulse).Length();
			    angularImpulse = MathF.Abs(joint.angularVelocityImpulse + joint.angularSpringImpulse);
		    }
		    break;

		    case b2JointType.b2_prismaticJoint:
		    {
                var joint = (PrismaticJoint) _joints[sim.jointId];
			    float perpImpulse = joint.impulse.X;
			    float axialImpulse = joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;
			    linearImpulse = MathF.Sqrt( perpImpulse * perpImpulse + axialImpulse * axialImpulse );
			    angularImpulse = MathF.Abs( joint.impulse.Y );
		    }
		    break;

		    case b2JointType.b2_revoluteJoint:
		    {
                var joint = (RevoluteJoint) _joints[sim.jointId];

			    linearImpulse = joint.linearImpulse.Length();
			    angularImpulse = MathF.Abs( joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse );
		    }
		    break;

		    case b2JointType.b2_weldJoint:
		    {
                var joint = (WeldJoint) _joints[sim.jointId];
			    linearImpulse = joint.linearImpulse.Length();
			    angularImpulse = MathF.Abs( joint.angularImpulse );
		    }
		    break;

		    case b2JointType.b2_wheelJoint:
		    {
                var joint = (WheelJoint) _joints[sim.jointId];
			    float perpImpulse = joint.perpImpulse;
			    float axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;
			    linearImpulse = MathF.Sqrt( perpImpulse * perpImpulse + axialImpulse * axialImpulse );
			    angularImpulse = MathF.Abs(joint.motorImpulse);
		    }
		    break;

		    default:
			    break;
	    }

	    force = linearImpulse * invTimeStep;
	    torque = angularImpulse * invTimeStep;
    }

    private static Vector2 GetJointConstraintForce(BaseJoint joint)
    {
	    b2JointSim* base = b2GetJointSim( world, joint );

	    switch ( joint.type )
	    {
		    case b2_distanceJoint:
			    return b2GetDistanceJointForce( world, base );

		    case b2_motorJoint:
			    return b2GetMotorJointForce( world, base );

		    case b2_filterJoint:
			    return b2Vec2_zero;

		    case b2_prismaticJoint:
			    return b2GetPrismaticJointForce( world, base );

		    case b2_revoluteJoint:
			    return b2GetRevoluteJointForce( world, base );

		    case b2_weldJoint:
			    return b2GetWeldJointForce( world, base );

		    case b2_wheelJoint:
			    return b2GetWheelJointForce( world, base );

		    default:
			    B2_ASSERT( false );
			    return b2Vec2_zero;
	    }
    }

    static float b2GetJointConstraintTorque( b2World* world, b2Joint* joint )
    {
	    b2JointSim* base = b2GetJointSim( world, joint );

	    switch ( joint.type )
	    {
		    case b2_distanceJoint:
			    return 0.0f;

		    case b2_motorJoint:
			    return b2GetMotorJointTorque( world, base );

		    case b2_filterJoint:
			    return 0.0f;

		    case b2_prismaticJoint:
			    return b2GetPrismaticJointTorque( world, base );

		    case b2_revoluteJoint:
			    return b2GetRevoluteJointTorque( world, base );

		    case b2_weldJoint:
			    return b2GetWeldJointTorque( world, base );

		    case b2_wheelJoint:
			    return b2GetWheelJointTorque( world, base );

		    default:
			    B2_ASSERT( false );
			    return 0.0f;
	    }
    }

    private Vector2 b2Joint_GetConstraintForce(JointId jointId)
    {
	    var joint = _joints[jointId.index1];
	    return GetJointConstraintForce( joint );
    }

    float b2Joint_GetConstraintTorque( b2JointId jointId )
    {
	    b2World* world = b2GetWorld( jointId.world0 );
	    b2Joint* joint = b2GetJointFullId( world, jointId );
	    return b2GetJointConstraintTorque( world, joint );
    }

    float b2Joint_GetLinearSeparation( b2JointId jointId )
    {
	    b2World* world = b2GetWorld( jointId.world0 );
	    b2Joint* joint = b2GetJointFullId( world, jointId );
	    b2JointSim* base = b2GetJointSim( world, joint );

	    b2Transform xfA = b2GetBodyTransform( world, joint.edges[0].bodyId );
	    b2Transform xfB = b2GetBodyTransform( world, joint.edges[1].bodyId );

	    b2Vec2 pA = b2TransformPoint( xfA, base.localFrameA.p );
	    b2Vec2 pB = b2TransformPoint( xfB, base.localFrameB.p );
	    b2Vec2 dp = b2Sub( pB, pA );

	    switch ( joint.type )
	    {
		    case b2_distanceJoint:
		    {
			    b2DistanceJoint* distanceJoint = &base.distanceJoint;
			    float length = b2Length( dp );
			    if ( distanceJoint.enableSpring )
			    {
				    if ( distanceJoint.enableLimit )
				    {
					    if ( length < distanceJoint.minLength )
					    {
						    return distanceJoint.minLength - length;
					    }

					    if ( length > distanceJoint.maxLength )
					    {
						    return length - distanceJoint.maxLength;
					    }

					    return 0.0f;
				    }

				    return 0.0f;
			    }

			    return b2AbsFloat( length - distanceJoint.length );
		    }

		    case b2_motorJoint:
			    return 0.0f;

		    case b2_filterJoint:
			    return 0.0f;

		    case b2_prismaticJoint:
		    {
			    b2PrismaticJoint* prismaticJoint = &base.prismaticJoint;
			    b2Vec2 axisA = b2RotateVector( xfA.q, (b2Vec2){ 1.0f, 0.0f } );
			    b2Vec2 perpA = b2LeftPerp( axisA );
			    float perpendicularSeparation = b2AbsFloat( b2Dot( perpA, dp ) );
			    float limitSeparation = 0.0f;

			    if ( prismaticJoint.enableLimit )
			    {
				    float translation = b2Dot( axisA, dp );
				    if ( translation < prismaticJoint.lowerTranslation )
				    {
					    limitSeparation = prismaticJoint.lowerTranslation - translation;
				    }

				    if ( prismaticJoint.upperTranslation < translation )
				    {
					    limitSeparation = translation - prismaticJoint.upperTranslation;
				    }
			    }

			    return sqrtf( perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation );
		    }

		    case b2_revoluteJoint:
			    return b2Length( dp );

		    case b2_weldJoint:
		    {
			    b2WeldJoint* weldJoint = &base.weldJoint;
			    if ( weldJoint.linearHertz == 0.0f )
			    {
				    return b2Length( dp );
			    }

			    return 0.0f;
		    }

		    case b2_wheelJoint:
		    {
			    b2WheelJoint* wheelJoint = &base.wheelJoint;
			    b2Vec2 axisA = b2RotateVector( xfA.q, (b2Vec2){ 1.0f, 0.0f } );
			    b2Vec2 perpA = b2LeftPerp( axisA );
			    float perpendicularSeparation = b2AbsFloat( b2Dot( perpA, dp ) );
			    float limitSeparation = 0.0f;

			    if ( wheelJoint.enableLimit )
			    {
				    float translation = b2Dot( axisA, dp );
				    if ( translation < wheelJoint.lowerTranslation )
				    {
					    limitSeparation = wheelJoint.lowerTranslation - translation;
				    }

				    if ( wheelJoint.upperTranslation < translation )
				    {
					    limitSeparation = translation - wheelJoint.upperTranslation;
				    }
			    }

			    return sqrtf( perpendicularSeparation * perpendicularSeparation + limitSeparation * limitSeparation );
		    }

		    default:
			    B2_ASSERT( false );
			    return 0.0f;
	    }
    }

    float b2Joint_GetAngularSeparation( b2JointId jointId )
    {
	    b2World* world = b2GetWorld( jointId.world0 );
	    b2Joint* joint = b2GetJointFullId( world, jointId );
	    b2JointSim* base = b2GetJointSim( world, joint );

	    b2Transform xfA = b2GetBodyTransform( world, joint.edges[0].bodyId );
	    b2Transform xfB = b2GetBodyTransform( world, joint.edges[1].bodyId );
	    float relativeAngle = b2RelativeAngle( xfA.q, xfB.q );

	    switch ( joint.type )
	    {
		    case b2_distanceJoint:
			    return 0.0f;

		    case b2_motorJoint:
			    return 0.0f;

		    case b2_filterJoint:
			    return 0.0f;

		    case b2_prismaticJoint:
		    {
			    return relativeAngle;
		    }

		    case b2_revoluteJoint:
		    {
			    b2RevoluteJoint* revoluteJoint = &base.revoluteJoint;
			    if ( revoluteJoint.enableLimit )
			    {
				    float angle = relativeAngle;
				    if ( angle < revoluteJoint.lowerAngle )
				    {
					    return revoluteJoint.lowerAngle - angle;
				    }

				    if ( revoluteJoint.upperAngle < angle )
				    {
					    return angle - revoluteJoint.upperAngle;
				    }
			    }

			    return 0.0f;
		    }

		    case b2_weldJoint:
		    {
			    b2WeldJoint* weldJoint = &base.weldJoint;
			    if ( weldJoint.angularHertz == 0.0f )
			    {
				    return relativeAngle;
			    }

			    return 0.0f;
		    }

		    case b2_wheelJoint:
			    return 0.0f;

		    default:
			    B2_ASSERT( false );
			    return 0.0f;
	    }
    }

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

    private void SolveJointsTask( int startIndex, int endIndex, int colorIndex, bool useBias, BitArray jointStateBitSet )
    {
        var color = _constraintGraph.colors[colorIndex];
        ref var joints = ref color.JointSims;
        DebugTools.Assert(0 <= startIndex && startIndex < color.JointSims.Count);
        DebugTools.Assert(startIndex <= endIndex && endIndex <= color.JointSims.Count);

        // TODO: States by ref, probably store them in valuelist.

        for ( int i = startIndex; i < endIndex; ++i )
        {
            ref var joint = ref joints[i];
            SolveJoint(ref joint, useBias);

            if (useBias && (joint.forceThreshold < float.MaxValue || joint.torqueThreshold < float.MaxValue) &&
                jointStateBitSet.Get(joint.jointId) == false)
            {
                float force = 0f, torque = 0f;
                GetJointReaction(ref joint, _invH, ref force, ref torque);

                // Check thresholds. A zero threshold means all awake joints get reported.
                if (force >= joint.forceThreshold || torque >= joint.torqueThreshold)
                {
                    // Flag this joint for processing.
                    jointStateBitSet.Set(joint.jointId, true);
                }
            }
        }
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
        var graph = _constraintGraph;
	    ref var joints = ref graph.colors[PhysicsConstants.OverflowIndex].JointSims;
	    int jointCount = graph.colors[PhysicsConstants.OverflowIndex].JointSims.Count;

	    for ( int i = 0; i < jointCount; ++i )
	    {
		    var joint = joints[i];
		    SolveJoint(ref joint, useBias);
	    }
    }

    #endregion
}
