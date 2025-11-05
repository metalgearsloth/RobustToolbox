namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    // Linear constraint (point-to-line)
    // d = pB - pA = xB + rB - xA - rA
    // C = dot(ay, d)
    // Cdot = dot(d, cross(wA, ay)) + dot(ay, vB + cross(wB, rB) - vA - cross(wA, rA))
    //      = -dot(ay, vA) - dot(cross(d + rA, ay), wA) + dot(ay, vB) + dot(cross(rB, ay), vB)
    // J = [-ay, -cross(d + rA, ay), ay, cross(rB, ay)]

    // Spring linear constraint
    // C = dot(ax, d)
    // Cdot = = -dot(ax, vA) - dot(cross(d + rA, ax), wA) + dot(ax, vB) + dot(cross(rB, ax), vB)
    // J = [-ax -cross(d+rA, ax) ax cross(rB, ax)]

    // Motor rotational constraint
    // Cdot = wB - wA
    // J = [0 0 -1 0 0 1]

    void b2PrepareWheelJoint( b2JointSim* base, b2StepContext* context )
    {
	    DebugTools.Assert( base.type == b2_wheelJoint );

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

	    b2WheelJoint* joint = &base.wheelJoint;

	    joint.indexA = bodyA.setIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.indexB = bodyB.setIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // Compute joint anchor frames with world space rotation, relative to center of mass
	    joint.frameA.q = b2MulRot( bodySimA.transform.q, base.localFrameA.q );
	    joint.frameA.p = Quaternion2D.RotateVector( bodySimA.transform.q, b2Sub( base.localFrameA.p, bodySimA.localCenter ) );
	    joint.frameB.q = b2MulRot( bodySimB.transform.q, base.localFrameB.q );
	    joint->frameB.p = Quaternion2D.RotateVector( bodySimB->transform.q, b2Sub( base->localFrameB.p, bodySimB->localCenter ) );

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint->deltaCenter = b2Sub( bodySimB->center, bodySimA->center );

	    b2Vec2 rA = joint->frameA.p;
	    b2Vec2 rB = joint->frameB.p;

	    b2Vec2 d = b2Add( joint->deltaCenter, b2Sub( rB, rA ) );
	    b2Vec2 axisA = Quaternion2D.RotateVector( joint->frameA.q, (b2Vec2){ 1.0f, 0.0f } );
	    b2Vec2 perpA = b2LeftPerp( axisA );

	    // perpendicular constraint (keep wheel on line)
	    float s1 = Vector2Helpers.Cross( b2Add( d, rA ), perpA );
	    float s2 = Vector2Helpers.Cross( rB, perpA );

	    float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
	    joint->perpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

	    // spring constraint
	    float a1 = Vector2Helpers.Cross( b2Add( d, rA ), axisA );
	    float a2 = Vector2Helpers.Cross( rB, axisA );

	    float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	    joint->axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	    joint->springSoftness = b2MakeSoft( joint->hertz, joint->dampingRatio, context->h );

	    float km = iA + iB;
	    joint->motorMass = km > 0.0f ? 1.0f / km : 0.0f;

	    if ( context->enableWarmStarting == false )
	    {
		    joint->perpImpulse = 0.0f;
		    joint->springImpulse = 0.0f;
		    joint->motorImpulse = 0.0f;
		    joint->lowerImpulse = 0.0f;
		    joint->upperImpulse = 0.0f;
	    }
    }

    void b2WarmStartWheelJoint( b2JointSim* base, b2StepContext* context )
    {
	    DebugTools.Assert( base->type == b2_wheelJoint );

	    float mA = base->invMassA;
	    float mB = base->invMassB;
	    float iA = base->invIA;
	    float iB = base->invIB;

	    // dummy state for static bodies
	    b2BodyState dummyState = bodyState.Identity;

	    b2WheelJoint* joint = &base->wheelJoint;

	    b2BodyState* stateA = joint->indexA == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexA;
	    b2BodyState* stateB = joint->indexB == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexB;

	    b2Vec2 rA = Quaternion2D.RotateVector( stateA->deltaRotation, joint->frameA.p );
	    b2Vec2 rB = Quaternion2D.RotateVector( stateB->deltaRotation, joint->frameB.p );

	    b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );
	    b2Vec2 axisA = Quaternion2D.RotateVector( joint->frameA.q, (b2Vec2){ 1.0f, 0.0f } );
	    axisA = Quaternion2D.RotateVector( stateA->deltaRotation, axisA );
	    b2Vec2 perpA = b2LeftPerp( axisA );

	    float a1 = Vector2Helpers.Cross( b2Add( d, rA ), axisA );
	    float a2 = Vector2Helpers.Cross( rB, axisA );
	    float s1 = Vector2Helpers.Cross( b2Add( d, rA ), perpA );
	    float s2 = Vector2Helpers.Cross( rB, perpA );

	    float axialImpulse = joint->springImpulse + joint->lowerImpulse - joint->upperImpulse;

	    b2Vec2 P = b2Add( b2MulSV( axialImpulse, axisA ), b2MulSV( joint->perpImpulse, perpA ) );
	    float LA = axialImpulse * a1 + joint->perpImpulse * s1 + joint->motorImpulse;
	    float LB = axialImpulse * a2 + joint->perpImpulse * s2 + joint->motorImpulse;

	    if ( stateA->flags & b2_dynamicFlag )
	    {
		    stateA->linearVelocity = Vector2Helpers.MulSub( stateA->linearVelocity, mA, P );
		    stateA->angularVelocity -= iA * LA;
	    }

	    if ( stateB->flags & b2_dynamicFlag )
	    {
		    stateB->linearVelocity = Vector2Helpers.MulAdd( stateB->linearVelocity, mB, P );
		    stateB->angularVelocity += iB * LB;
	    }
    }

    void b2SolveWheelJoint( b2JointSim* base, b2StepContext* context, bool useBias )
    {
	    DebugTools.Assert( base->type == b2_wheelJoint );

	    float mA = base->invMassA;
	    float mB = base->invMassB;
	    float iA = base->invIA;
	    float iB = base->invIB;

	    // dummy state for static bodies
	    b2BodyState dummyState = bodyState.Identity;

	    b2WheelJoint* joint = &base->wheelJoint;

	    b2BodyState* stateA = joint->indexA == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexA;
	    b2BodyState* stateB = joint->indexB == PhysicsConstants.NullIndex ? &dummyState : context->states + joint->indexB;

	    b2Vec2 vA = stateA->linearVelocity;
	    float wA = stateA->angularVelocity;
	    b2Vec2 vB = stateB->linearVelocity;
	    float wB = stateB->angularVelocity;

	    bool fixedRotation = ( iA + iB == 0.0f );

	    // current anchors
	    b2Vec2 rA = Quaternion2D.RotateVector( stateA->deltaRotation, joint->frameA.p );
	    b2Vec2 rB = Quaternion2D.RotateVector( stateB->deltaRotation, joint->frameB.p );

	    b2Vec2 d = b2Add( b2Add( b2Sub( stateB->deltaPosition, stateA->deltaPosition ), joint->deltaCenter ), b2Sub( rB, rA ) );
	    b2Vec2 axisA = Quaternion2D.RotateVector( joint->frameA.q, (b2Vec2){ 1.0f, 0.0f } );
	    axisA = Quaternion2D.RotateVector( stateA->deltaRotation, axisA );
	    float translation = Vector2.Dot( axisA, d );

	    float a1 = Vector2Helpers.Cross( b2Add( d, rA ), axisA );
	    float a2 = Vector2Helpers.Cross( rB, axisA );

	    // motor constraint
	    if ( joint->enableMotor && fixedRotation == false )
	    {
		    float Cdot = wB - wA - joint->motorSpeed;
		    float impulse = -joint->motorMass * Cdot;
		    float oldImpulse = joint->motorImpulse;
		    float maxImpulse = context->h * joint->maxMotorTorque;
		    joint->motorImpulse = Math.Clamp( joint->motorImpulse + impulse, -maxImpulse, maxImpulse );
		    impulse = joint->motorImpulse - oldImpulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    // spring constraint
	    if ( joint->enableSpring )
	    {
		    // This is a real spring and should be applied even during relax
		    float C = translation;
		    float bias = joint->springSoftness.biasRate * C;
		    float massScale = joint->springSoftness.massScale;
		    float impulseScale = joint->springSoftness.impulseScale;

		    float Cdot = Vector2.Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
		    float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->springImpulse;
		    joint->springImpulse += impulse;

		    b2Vec2 P = b2MulSV( impulse, axisA );
		    float LA = impulse * a1;
		    float LB = impulse * a2;

		    vA = Vector2Helpers.MulSub( vA, mA, P );
		    wA -= iA * LA;
		    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    wB += iB * LB;
	    }

	    if ( joint->enableLimit )
	    {
		    // Lower limit
		    {
			    float C = translation - joint->lowerTranslation;
			    float bias = 0.0f;
			    float massScale = 1.0f;
			    float impulseScale = 0.0f;

			    if ( C > 0.0f )
			    {
				    // speculation
				    bias = C * context->inv_h;
			    }
			    else if ( useBias )
			    {
				    bias = base->constraintSoftness.biasRate * C;
				    massScale = base->constraintSoftness.massScale;
				    impulseScale = base->constraintSoftness.impulseScale;
			    }

			    float Cdot = Vector2.Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
			    float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->lowerImpulse;
			    float oldImpulse = joint->lowerImpulse;
			    joint->lowerImpulse = MathF.Max( oldImpulse + impulse, 0.0f );
			    impulse = joint->lowerImpulse - oldImpulse;

			    b2Vec2 P = b2MulSV( impulse, axisA );
			    float LA = impulse * a1;
			    float LB = impulse * a2;

			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * LA;
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * LB;
		    }

		    // Upper limit
		    // Note: signs are flipped to keep C positive when the constraint is satisfied.
		    // This also keeps the impulse positive when the limit is active.
		    {
			    // sign flipped
			    float C = joint->upperTranslation - translation;
			    float bias = 0.0f;
			    float massScale = 1.0f;
			    float impulseScale = 0.0f;

			    if ( C > 0.0f )
			    {
				    // speculation
				    bias = C * context->inv_h;
			    }
			    else if ( useBias )
			    {
				    bias = base->constraintSoftness.biasRate * C;
				    massScale = base->constraintSoftness.massScale;
				    impulseScale = base->constraintSoftness.impulseScale;
			    }

			    // sign flipped on Cdot
			    float Cdot = Vector2.Dot( axisA, b2Sub( vA, vB ) ) + a1 * wA - a2 * wB;
			    float impulse = -massScale * joint->axialMass * ( Cdot + bias ) - impulseScale * joint->upperImpulse;
			    float oldImpulse = joint->upperImpulse;
			    joint->upperImpulse = MathF.Max( oldImpulse + impulse, 0.0f );
			    impulse = joint->upperImpulse - oldImpulse;

			    b2Vec2 P = b2MulSV( impulse, axisA );
			    float LA = impulse * a1;
			    float LB = impulse * a2;

			    // sign flipped on applied impulse
			    vA = Vector2Helpers.MulAdd( vA, mA, P );
			    wA += iA * LA;
			    vB = Vector2Helpers.MulSub( vB, mB, P );
			    wB -= iB * LB;
		    }
	    }

	    // point to line constraint
	    {
		    b2Vec2 perpA = b2LeftPerp( axisA );

		    float bias = 0.0f;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias )
		    {
			    float C = Vector2.Dot( perpA, d );
			    bias = base->constraintSoftness.biasRate * C;
			    massScale = base->constraintSoftness.massScale;
			    impulseScale = base->constraintSoftness.impulseScale;
		    }

		    float s1 = Vector2Helpers.Cross( b2Add( d, rA ), perpA );
		    float s2 = Vector2Helpers.Cross( rB, perpA );
		    float Cdot = Vector2.Dot( perpA, b2Sub( vB, vA ) ) + s2 * wB - s1 * wA;

		    float impulse = -massScale * joint->perpMass * ( Cdot + bias ) - impulseScale * joint->perpImpulse;
		    joint->perpImpulse += impulse;

		    b2Vec2 P = b2MulSV( impulse, perpA );
		    float LA = impulse * s1;
		    float LB = impulse * s2;

		    vA = Vector2Helpers.MulSub( vA, mA, P );
		    wA -= iA * LA;
		    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    wB += iB * LB;
	    }

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
