using System;
using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private void PrepareRevoluteJoint(ref JointSim sim)
    {
	    DebugTools.Assert( sim.type == b2JointType.b2_revoluteJoint );

	    // chase body id to the solver set where the body lives
	    int idA = sim.bodyIdA;
	    int idB = sim.bodyIdB;

	    var bodyA = _bodies[idA].Comp;
	    var bodyB = _bodies[idB].Comp;

	    DebugTools.Assert( bodyA.SetIndex == (int) SetType.AwakeSet || bodyB.SetIndex == (int) SetType.AwakeSet );
	    var setA = _solverSets[bodyA.SetIndex];
	    var setB = _solverSets[bodyB.SetIndex];

	    int localIndexA = bodyA.LocalIndex;
	    int localIndexB = bodyB.LocalIndex;

	    ref var bodySimA = ref setA.bodySims[localIndexA];
	    ref var bodySimB = ref setB.bodySims[localIndexB];

	    float mA = bodySimA.invMass;
	    float iA = bodySimA.invInertia;
	    float mB = bodySimB.invMass;
	    float iB = bodySimB.invInertia;

	    sim.invMassA = mA;
	    sim.invMassB = mB;
	    sim.invIA = iA;
	    sim.invIB = iB;

	    var joint = (RevoluteJoint) _joints[sim.jointId];

	    joint.IndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.IndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // Compute joint anchor frames with world space rotation, relative to center of mass.
	    // Avoid round-off here as much as possible.
	    // b2Vec2 pf = (xf.p - c) + rot(xf.q, f.p)
	    // pf = xf.p - (xf.p + rot(xf.q, lc)) + rot(xf.q, f.p)
	    // pf = rot(xf.q, f.p - lc)
	    joint.frameA.q = b2MulRot( bodySimA.transform.q, sim.localFrameA.q );
	    joint.frameA.p = Quaternion2D.RotateVector( bodySimA.transform.q, b2Sub( sim.localFrameA.p, bodySimA.localCenter ) );
	    joint.frameB.q = b2MulRot( bodySimB.transform.q, sim.localFrameB.q );
	    joint.frameB.p = Quaternion2D.RotateVector( bodySimB.transform.q, b2Sub( sim.localFrameB.p, bodySimB.localCenter ) );

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint.deltaCenter = b2Sub( bodySimB.center, bodySimA.center );

	    float k = iA + iB;
	    joint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	    joint.springSoftness = b2MakeSoft( joint.hertz, joint.dampingRatio, _h );

	    if ( context.enableWarmStarting == false )
	    {
		    joint.linearImpulse = Vector2.Zero;
		    joint.springImpulse = 0.0f;
		    joint.motorImpulse = 0.0f;
		    joint.lowerImpulse = 0.0f;
		    joint.upperImpulse = 0.0f;
	    }
    }

    private void WarmStartRevoluteJoint(ref JointSim sim)
    {
	    DebugTools.Assert( sim.type == b2_revoluteJoint );

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    b2BodyState dummyState = bodyState.Identity;

	    b2RevoluteJoint* joint = &sim.revoluteJoint;
	    b2BodyState* stateA = joint.indexA == PhysicsConstants.NullIndex ? &dummyState : context.states + joint.indexA;
	    b2BodyState* stateB = joint.indexB == PhysicsConstants.NullIndex ? &dummyState : context.states + joint.indexB;

	    b2Vec2 rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.p );
	    b2Vec2 rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.p );

	    float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	    if ( stateA.flags & b2_dynamicFlag )
	    {
		    stateA.linearVelocity = Vector2Helpers.MulSub( stateA.linearVelocity, mA, joint.linearImpulse );
		    stateA.angularVelocity -= iA * ( Vector2Helpers.Cross( rA, joint.linearImpulse ) + axialImpulse );
	    }

	    if ( stateB.flags & b2_dynamicFlag )
	    {
		    stateB.linearVelocity = Vector2Helpers.MulAdd( stateB.linearVelocity, mB, joint.linearImpulse );
		    stateB.angularVelocity += iB * ( Vector2Helpers.Cross( rB, joint.linearImpulse ) + axialImpulse );
	    }
    }

    void b2SolveRevoluteJoint( b2JointSim* base, b2StepContext* context, bool useBias )
    {
	    DebugTools.Assert( sim.type == b2_revoluteJoint );

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    b2BodyState dummyState = bodyState.Identity;

	    b2RevoluteJoint* joint = &sim.revoluteJoint;

	    b2BodyState* stateA = joint.indexA == PhysicsConstants.NullIndex ? &dummyState : context.states + joint.indexA;
	    b2BodyState* stateB = joint.indexB == PhysicsConstants.NullIndex ? &dummyState : context.states + joint.indexB;

	    b2Vec2 vA = stateA.linearVelocity;
	    float wA = stateA.angularVelocity;
	    b2Vec2 vB = stateB.linearVelocity;
	    float wB = stateB.angularVelocity;

	    b2Rot qA = b2MulRot( stateA.deltaRotation, joint.frameA.q );
	    b2Rot qB = b2MulRot( stateB.deltaRotation, joint.frameB.q );
	    b2Rot relQ = b2InvMulRot( qA, qB );

	    bool fixedRotation = ( iA + iB == 0.0f );

	    // Solve spring.
	    if ( joint.enableSpring && fixedRotation == false )
	    {
		    float jointAngle = b2Rot_GetAngle( relQ );
		    float jointAngleDelta = b2UnwindAngle( jointAngle - joint.targetAngle );

		    float C = jointAngleDelta;
		    float bias = joint.springSoftness.biasRate * C;
		    float massScale = joint.springSoftness.massScale;
		    float impulseScale = joint.springSoftness.impulseScale;

		    float Cdot = wB - wA;
		    float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
		    joint.springImpulse += impulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    // Solve motor constraint.
	    if ( joint.enableMotor && fixedRotation == false )
	    {
		    float Cdot = wB - wA - joint.motorSpeed;
		    float impulse = -joint.axialMass * Cdot;
		    float oldImpulse = joint.motorImpulse;
		    float maxImpulse = context.h * joint.maxMotorTorque;
		    joint.motorImpulse = Math.Clamp( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
		    impulse = joint.motorImpulse - oldImpulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    if ( joint.enableLimit && fixedRotation == false )
	    {
		    float jointAngle = b2Rot_GetAngle( relQ );

		    // Lower limit
		    {
			    float C = jointAngle - joint.lowerAngle;
			    float bias = 0.0f;
			    float massScale = 1.0f;
			    float impulseScale = 0.0f;
			    if ( C > 0.0f )
			    {
				    // speculation
				    bias = C * context.inv_h;
			    }
			    else if ( useBias )
			    {
				    bias = sim.constraintSoftness.biasRate * C;
				    massScale = sim.constraintSoftness.massScale;
				    impulseScale = sim.constraintSoftness.impulseScale;
			    }

			    float Cdot = wB - wA;
			    float oldImpulse = joint.lowerImpulse;
			    float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * oldImpulse;
			    joint.lowerImpulse = MathF.Max( oldImpulse + impulse, 0.0f );
			    impulse = joint.lowerImpulse - oldImpulse;

			    wA -= iA * impulse;
			    wB += iB * impulse;
		    }

		    // Upper limit
		    // Note: signs are flipped to keep C positive when the constraint is satisfied.
		    // This also keeps the impulse positive when the limit is active.
		    {
			    float C = joint.upperAngle - jointAngle;
			    float bias = 0.0f;
			    float massScale = 1.0f;
			    float impulseScale = 0.0f;
			    if ( C > 0.0f )
			    {
				    // speculation
				    bias = C * context.inv_h;
			    }
			    else if ( useBias )
			    {
				    bias = sim.constraintSoftness.biasRate * C;
				    massScale = sim.constraintSoftness.massScale;
				    impulseScale = sim.constraintSoftness.impulseScale;
			    }

			    // sign flipped on Cdot
			    float Cdot = wA - wB;
			    float oldImpulse = joint.upperImpulse;
			    float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * oldImpulse;
			    joint.upperImpulse = MathF.Max( oldImpulse + impulse, 0.0f );
			    impulse = joint.upperImpulse - oldImpulse;

			    // sign flipped on applied impulse
			    wA += iA * impulse;
			    wB -= iB * impulse;
		    }
	    }

	    // Solve point-to-point constraint
	    {
		    // J = [-I -r1_skew I r2_skew]
		    // r_skew = [-ry; rx]
		    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x]
		    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB]

		    // current anchors
		    b2Vec2 rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.p );
		    b2Vec2 rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.p );

		    b2Vec2 Cdot = b2Sub( b2Add( vB, Vector2Helpers.Cross( wB, rB ) ), b2Add( vA, Vector2Helpers.Cross( wA, rA ) ) );

		    b2Vec2 bias = Vector2.Zero;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias )
		    {
			    b2Vec2 dcA = stateA.deltaPosition;
			    b2Vec2 dcB = stateB.deltaPosition;

			    b2Vec2 separation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint.deltaCenter );
			    bias = b2MulSV( sim.constraintSoftness.biasRate, separation );
			    massScale = sim.constraintSoftness.massScale;
			    impulseScale = sim.constraintSoftness.impulseScale;
		    }

		    b2Mat22 K;
		    K.cx.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		    K.cy.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		    K.cx.y = K.cy.x;
		    K.cy.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		    b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );

		    b2Vec2 impulse;
		    impulse.x = -massScale * b.x - impulseScale * joint.linearImpulse.x;
		    impulse.y = -massScale * b.y - impulseScale * joint.linearImpulse.y;
		    joint.linearImpulse.x += impulse.x;
		    joint.linearImpulse.y += impulse.y;

		    vA = Vector2Helpers.MulSub( vA, mA, impulse );
		    wA -= iA * Vector2Helpers.Cross( rA, impulse );
		    vB = Vector2Helpers.MulAdd( vB, mB, impulse );
		    wB += iB * Vector2Helpers.Cross( rB, impulse );
	    }

	    if ( stateA.flags & b2_dynamicFlag )
	    {
		    stateA.linearVelocity = vA;
		    stateA.angularVelocity = wA;
	    }

	    if ( stateB.flags & b2_dynamicFlag )
	    {
		    stateB.linearVelocity = vB;
		    stateB.angularVelocity = wB;
	    }
    }
}
