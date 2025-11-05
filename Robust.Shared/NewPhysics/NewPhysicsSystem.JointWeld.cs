namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    // Point-to-point constraint
    // C = p2 - p1
    // Cdot = v2 - v1
    //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    // J = [-E -r1_skew E r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Angle constraint
    // C = angle2 - angle1 - referenceAngle
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    // 3x3 Block
    // K = [J1] * invM * [J1T J2T]
    //     [J2]
    //   = [J1] * [invM * J1T invM * J2T]
    //     [J2]
    //   = [J1 * invM * J1T J1 * invM * J2T]
    //     [J2 * invM * J1T J2 * invM * J2T]

    void b2PrepareWeldJoint( b2JointSim* base, b2StepContext* context )
    {
	    DebugTools.Assert( base.type == b2_weldJoint );

	    // chase body id to the solver set where the body lives
	    int idA = base.bodyIdA;
	    int idB = base.bodyIdB;

	    b2World* world = context.world;

	    b2Body* bodyA = b2BodyArray_Get( &world.bodies, idA );
	    b2Body* bodyB = b2BodyArray_Get( &world.bodies, idB );

	    DebugTools.Assert( bodyA.setIndex == (int) SetType.AwakeSet || bodyB.setIndex == (int) SetType.AwakeSet );
	    b2SolverSet* setA = b2SolverSetArray_Get( &world.solverSets, bodyA.setIndex );
	    b2SolverSet* setB = b2SolverSetArray_Get( &world.solverSets, bodyB.setIndex );

	    int localIndexA = bodyA.localIndex;
	    int localIndexB = bodyB.localIndex;

	    b2BodySim* bodySimA = b2BodySimArray_Get( &setA.bodySims, localIndexA );
	    b2BodySim* bodySimB = b2BodySimArray_Get( &setB.bodySims, localIndexB );

	    float mA = bodySimA.invMass;
	    float iA = bodySimA.invInertia;
	    float mB = bodySimB.invMass;
	    float iB = bodySimB.invInertia;

	    base.invMassA = mA;
	    base.invMassB = mB;
	    base.invIA = iA;
	    base.invIB = iB;

	    b2WeldJoint* joint = &base.weldJoint;
	    joint.indexA = bodyA.setIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.indexB = bodyB.setIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // Compute joint anchor frames with world space rotation, relative to center of mass
	    joint.frameA.q = b2MulRot( bodySimA.transform.q, base.localFrameA.q );
	    joint.frameA.p = Quaternion2D.RotateVector( bodySimA.transform.q, b2Sub( base.localFrameA.p, bodySimA.localCenter ) );
	    joint.frameB.q = b2MulRot( bodySimB.transform.q, base.localFrameB.q );
	    joint.frameB.p = Quaternion2D.RotateVector( bodySimB.transform.q, b2Sub( base.localFrameB.p, bodySimB.localCenter ) );

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint.deltaCenter = b2Sub( bodySimB.center, bodySimA.center );

	    float ka = iA + iB;
	    joint.axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	    if ( joint.linearHertz == 0.0f )
	    {
		    joint.linearSpring = base.constraintSoftness;
	    }
	    else
	    {
		    joint->linearSpring = b2MakeSoft( joint->linearHertz, joint->linearDampingRatio, context->h );
	    }

	    if ( joint->angularHertz == 0.0f )
	    {
		    joint->angularSpring = base->constraintSoftness;
	    }
	    else
	    {
		    joint->angularSpring = b2MakeSoft( joint->angularHertz, joint->angularDampingRatio, context->h );
	    }

	    if ( context->enableWarmStarting == false )
	    {
		    joint->linearImpulse = Vector2.Zero;
		    joint->angularImpulse = 0.0f;
	    }
    }

    void b2WarmStartWeldJoint( b2JointSim* base, b2StepContext* context )
    {
	    float mA = base->invMassA;
	    float mB = base->invMassB;
	    float iA = base->invIA;
	    float iB = base->invIB;

	    // dummy state for static bodies
	    b2BodyState dummyState = bodyState.Identity;

	    b2WeldJoint* joint = &base->weldJoint;

	    b2BodyState* stateA = joint->indexA == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexA;
	    b2BodyState* stateB = joint->indexB == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexB;

	    b2Vec2 rA = Quaternion2D.RotateVector( stateA->deltaRotation, joint->frameA.p );
	    b2Vec2 rB = Quaternion2D.RotateVector( stateB->deltaRotation, joint->frameB.p );

	    if ( stateA->flags & b2_dynamicFlag )
	    {
		    stateA->linearVelocity = Vector2Helpers.MulSub( stateA->linearVelocity, mA, joint->linearImpulse );
		    stateA->angularVelocity -= iA * ( Vector2Helpers.Cross( rA, joint->linearImpulse ) + joint->angularImpulse );
	    }

	    if ( stateB->flags & b2_dynamicFlag )
	    {
		    stateB->linearVelocity = Vector2Helpers.MulAdd( stateB->linearVelocity, mB, joint->linearImpulse );
		    stateB->angularVelocity += iB * ( Vector2Helpers.Cross( rB, joint->linearImpulse ) + joint->angularImpulse );
	    }
    }

    void b2SolveWeldJoint( b2JointSim* base, b2StepContext* context, bool useBias )
    {
	    DebugTools.Assert( base->type == b2_weldJoint );

	    float mA = base->invMassA;
	    float mB = base->invMassB;
	    float iA = base->invIA;
	    float iB = base->invIB;

	    // dummy state for static bodies
	    b2BodyState dummyState = bodyState.Identity;

	    b2WeldJoint* joint = &base->weldJoint;

	    b2BodyState* stateA = joint->indexA == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexA;
	    b2BodyState* stateB = joint->indexB == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexB;

	    b2Vec2 vA = stateA->linearVelocity;
	    float wA = stateA->angularVelocity;
	    b2Vec2 vB = stateB->linearVelocity;
	    float wB = stateB->angularVelocity;

	    // Block solve doesn't work correctly with mixed stiffness values
    #if B2_WELD_BLOCK_SOLVE
	    // J = [-I -r1_skew I r2_skew]
	    //     [ 0       -1 0       1]
	    // r_skew = [-ry; rx]

	    // Matlab
	    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
	    b2Vec2 rA = Quaternion2D.RotateVector( stateA->deltaRotation, joint->frameA.p );
	    b2Vec2 rB = Quaternion2D.RotateVector( stateB->deltaRotation, joint->frameB.p );

	    b2Mat33 K;
	    K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	    K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	    K.cz.x = -rA.y * iA - rB.y * iB;
	    K.cx.y = K.cy.x;
	    K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	    K.cz.y = rA.x * iA + rB.x * iB;
	    K.cx.z = K.cz.x;
	    K.cy.z = K.cz.y;
	    K.cz.z = iA + iB;

	    b2Vec3 bias = {0.0f, 0.0f, 0.0f};
	    float linearMassScale = 1.0f;
	    float linearImpulseScale = 0.0f;
	    if ( useBias || joint->linearHertz > 0.0f )
	    {
		    // linear
		    b2Vec2 dcA = stateA->deltaPosition;
		    b2Vec2 dcB = stateB->deltaPosition;
		    b2Vec2 jointTranslation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

		    bias.x = joint->linearSpring.biasRate * jointTranslation.x;
		    bias.y = joint->linearSpring.biasRate * jointTranslation.y;

		    linearMassScale = joint->linearSpring.massScale;
		    linearImpulseScale = joint->linearSpring.impulseScale;
	    }

	    float angularMassScale = 1.0f;
	    float angularImpulseScale = 0.0f;
	    if ( useBias || joint->angularHertz > 0.0f )
	    {
		    // angular
		    b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		    b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		    b2Rot relQ = b2InvMulRot( qA, qB );
		    float jointAngle = b2Rot_GetAngle( relQ );

		    bias.z = joint->angularSpring.biasRate * jointAngle;

		    angularMassScale = joint->angularSpring.massScale;
		    angularImpulseScale = joint->angularSpring.impulseScale;
	    }

	    b2Vec2 Cdot1 = b2Sub( b2Add( vB, Vector2Helpers.Cross( wB, rB ) ), b2Add( vA, Vector2Helpers.Cross( wA, rA ) ) );
	    float Cdot2 = wB - wA;

	    b2Vec3 Cdot = {Cdot1.x + bias.x, Cdot1.y + bias.y, Cdot2 + bias.z};

	    b2Vec3 b = b2Solve33( &K, Cdot );

	    b2Vec2 linearImpulse = {
		    -linearMassScale * b.x - linearImpulseScale * joint->linearImpulse.x,
		    -linearMassScale * b.y - linearImpulseScale * joint->linearImpulse.y,
	    };
	    joint->linearImpulse = b2Add( joint->linearImpulse, linearImpulse );

	    float angularImpulse = -angularMassScale * b.z - angularImpulseScale * joint->angularImpulse;
	    joint->angularImpulse += angularImpulse;

	    vA = Vector2Helpers.MulSub( vA, mA, linearImpulse );
	    wA -= iA * (Vector2Helpers.Cross( rA, linearImpulse ) + angularImpulse);
	    vB = Vector2Helpers.MulAdd( vB, mB, linearImpulse );
	    wB += iB * (Vector2Helpers.Cross( rB, linearImpulse ) + angularImpulse);

	    // todo debugging
	    Cdot1 = b2Sub( b2Add( vB, Vector2Helpers.Cross( wB, rB ) ), b2Add( vA, Vector2Helpers.Cross( wA, rA ) ) );
	    Cdot2 = wB - wA;

	    if ( useBias == false && b2Length(Cdot1) > 0.0001f )
	    {
		    Cdot1.x += 0.0f;
	    }

	    if ( useBias == false && b2AbsFloat( Cdot2 ) > 0.0001f )
	    {
		    Cdot2 += 0.0f;
	    }

    #else

	    // angular constraint
	    {
		    b2Rot qA = b2MulRot( stateA->deltaRotation, joint->frameA.q );
		    b2Rot qB = b2MulRot( stateB->deltaRotation, joint->frameB.q );
		    b2Rot relQ = b2InvMulRot( qA, qB );
		    float jointAngle = b2Rot_GetAngle( relQ );

		    float bias = 0.0f;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias || joint->angularHertz > 0.0f )
		    {
			    float C = jointAngle;
			    bias = joint->angularSpring.biasRate * C;
			    massScale = joint->angularSpring.massScale;
			    impulseScale = joint->angularSpring.impulseScale;
		    }

		    float Cdot = wB - wA;
		    float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->angularImpulse;
		    joint->angularImpulse += impulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    // linear constraint
	    {
		    b2Vec2 rA = Quaternion2D.RotateVector( stateA->deltaRotation, joint->frameA.p );
		    b2Vec2 rB = Quaternion2D.RotateVector( stateB->deltaRotation, joint->frameB.p );

		    b2Vec2 bias = Vector2.Zero;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias || joint->linearHertz > 0.0f )
		    {
			    b2Vec2 dcA = stateA->deltaPosition;
			    b2Vec2 dcB = stateB->deltaPosition;
			    b2Vec2 C = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint->deltaCenter );

			    bias = b2MulSV( joint->linearSpring.biasRate, C );
			    massScale = joint->linearSpring.massScale;
			    impulseScale = joint->linearSpring.impulseScale;
		    }

		    b2Vec2 Cdot = b2Sub( b2Add( vB, Vector2Helpers.Cross( wB, rB ) ), b2Add( vA, Vector2Helpers.Cross( wA, rA ) ) );

		    b2Mat22 K;
		    K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		    K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		    K.cx.y = K.cy.x;
		    K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		    b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

		    b2Vec2 impulse = {
			    -massScale * b.x - impulseScale * joint->linearImpulse.x,
			    -massScale * b.y - impulseScale * joint->linearImpulse.y,
		    };

		    joint->linearImpulse = b2Add( joint->linearImpulse, impulse );

		    vA = Vector2Helpers.MulSub( vA, mA, impulse );
		    wA -= iA * Vector2Helpers.Cross( rA, impulse );
		    vB = Vector2Helpers.MulAdd( vB, mB, impulse );
		    wB += iB * Vector2Helpers.Cross( rB, impulse );
	    }
    #endif

	    DebugTools.Assert( b2IsValidVec2( vA ) );
	    DebugTools.Assert( b2IsValidFloat( wA ) );
	    DebugTools.Assert( b2IsValidVec2( vB ) );
	    DebugTools.Assert( b2IsValidFloat( wB ) );

	    if ( stateA->flags & b2_dynamicFlag )
	    {
		    stateA->linearVelocity = vA;
		    stateA->angularVelocity = wA;
	    }

	    if ( stateB->flags & b2_dynamicFlag )
	    {
		    stateB->linearVelocity = vB;
		    stateB->angularVelocity = wB;
	    }
    }
}
