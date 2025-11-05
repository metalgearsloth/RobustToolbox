using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    // Linear constraint (point-to-line)
// d = pB - pA = xB + rB - xA - rA
// C = dot(perp, d)
// Cdot = dot(d, cross(wA, perp)) + dot(perp, vB + cross(wB, rB) - vA - cross(wA, rA))
//      = -dot(perp, vA) - dot(cross(rA + d, perp), wA) + dot(perp, vB) + dot(cross(rB, perp), vB)
// J = [-perp, -cross(rA + d, perp), perp, cross(rB, perp)]
//
// Angular constraint
// C = aB - aA + a_initial
// Cdot = wB - wA
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -sA a sB]
//     [0  -1  0  1]
// a = perp
// sA = cross(rA + d, a) = cross(pB - xA, a)
// sB = cross(rB, a) = cross(pB - xB, a)

// Motor/Limit linear constraint
// C = dot(axA, d)
// Cdot = -dot(axA, vA) - dot(cross(rA + d, axA), wA) + dot(axA, vB) + dot(cross(rB, axA), vB)
// J = [-axA -cross(rA + d, axA) axA cross(rB, ax1)]

// Predictive limit is applied even when the limit is not active.
// Prevents a constraint speed that can lead to a constraint error in one time step.
// Want C2 = C1 + h * Cdot >= 0
// Or:
// Cdot + C1/h >= 0
// I do not apply a negative constraint error because that is handled in position correction.
// So:
// Cdot + max(C1, 0)/h >= 0

// Block Solver
// We develop a block solver that includes the angular and linear constraints. This makes the limit stiffer.
//
// The Jacobian has 2 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//
// u = perp
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

private void PreparePrismaticJoint(ref JointSim sim)
{
	DebugTools.Assert( sim.type == b2JointType.b2_prismaticJoint );

	// chase body id to the solver set where the body lives
	int idA = sim.bodyIdA;
	int idB = sim.bodyIdB;

	var bodyA = _bodies[idA].Comp;
	var bodyB = _bodies[idB].Comp;

	B2_ASSERT( bodyA.setIndex == b2_awakeSet || bodyB.setIndex == b2_awakeSet );
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

	sim.invMassA = mA;
	sim.invMassB = mB;
	sim.invIA = iA;
	sim.invIB = iB;

	b2PrismaticJoint* joint = &sim.prismaticJoint;
	joint.indexA = bodyA.setIndex == b2_awakeSet ? localIndexA : B2_NULL_INDEX;
	joint.indexB = bodyB.setIndex == b2_awakeSet ? localIndexB : B2_NULL_INDEX;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.q = b2MulRot( bodySimA.transform.q, sim.localFrameA.q );
	joint.frameA.p = b2RotateVector( bodySimA.transform.q, b2Sub( sim.localFrameA.p, bodySimA.localCenter ) );
	joint.frameB.q = b2MulRot( bodySimB.transform.q, sim.localFrameB.q );
	joint.frameB.p = b2RotateVector( bodySimB.transform.q, b2Sub( sim.localFrameB.p, bodySimB.localCenter ) );

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = b2Sub( bodySimB.center, bodySimA.center );

	joint.springSoftness = b2MakeSoft( joint.hertz, joint.dampingRatio, context.h );

	if ( context.enableWarmStarting == false )
	{
		joint.impulse = b2Vec2_zero;
		joint.springImpulse = 0.0f;
		joint.motorImpulse = 0.0f;
		joint.lowerImpulse = 0.0f;
		joint.upperImpulse = 0.0f;
	}
}

private void WarmStartPrismaticJoint(ref JointSim sim)
{
	B2_ASSERT( sim.type == b2_prismaticJoint );

	float mA = sim.invMassA;
	float mB = sim.invMassB;
	float iA = sim.invIA;
	float iB = sim.invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2PrismaticJoint* joint = &sim.prismaticJoint;

	b2BodyState* stateA = joint.indexA == B2_NULL_INDEX ? &dummyState : context.states + joint.indexA;
	b2BodyState* stateB = joint.indexB == B2_NULL_INDEX ? &dummyState : context.states + joint.indexB;

	b2Vec2 rA = b2RotateVector( stateA.deltaRotation, joint.frameA.p );
	b2Vec2 rB = b2RotateVector( stateB.deltaRotation, joint.frameB.p );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB.deltaPosition, stateA.deltaPosition ), joint.deltaCenter ), b2Sub( rB, rA ) );

	b2Vec2 axisA = b2RotateVector( joint.frameA.q, (b2Vec2){ 1.0f, 0.0f } );
	axisA = b2RotateVector( stateA.deltaRotation, axisA );

	// impulse is applied at anchor point on body B
	float a1 = b2Cross( b2Add( rA, d ), axisA );
	float a2 = b2Cross( rB, axisA );
	float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	// perpendicular constraint
	b2Vec2 perpA = b2LeftPerp( axisA );
	float s1 = b2Cross( b2Add( rA, d ), perpA );
	float s2 = b2Cross( rB, perpA );
	float perpImpulse = joint.impulse.x;
	float angleImpulse = joint.impulse.y;

	b2Vec2 P = b2Add( b2MulSV( axialImpulse, axisA ), b2MulSV( perpImpulse, perpA ) );
	float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
	float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

	if ( stateA.flags & b2_dynamicFlag )
	{
		stateA.linearVelocity = b2MulSub( stateA.linearVelocity, mA, P );
		stateA.angularVelocity -= iA * LA;
	}

	if ( stateB.flags & b2_dynamicFlag )
	{
		stateB.linearVelocity = b2MulAdd( stateB.linearVelocity, mB, P );
		stateB.angularVelocity += iB * LB;
	}
}

private void SolvePrismaticJoint(ref JointSim sim, bool useBias )
{
	B2_ASSERT( sim.type == b2_prismaticJoint );

	float mA = sim.invMassA;
	float mB = sim.invMassB;
	float iA = sim.invIA;
	float iB = sim.invIB;

	// dummy state for static bodies
	b2BodyState dummyState = b2_identityBodyState;

	b2PrismaticJoint* joint = &sim.prismaticJoint;

	b2BodyState* stateA = joint.indexA == B2_NULL_INDEX ? &dummyState : context.states + joint.indexA;
	b2BodyState* stateB = joint.indexB == B2_NULL_INDEX ? &dummyState : context.states + joint.indexB;

	b2Vec2 vA = stateA.linearVelocity;
	float wA = stateA.angularVelocity;
	b2Vec2 vB = stateB.linearVelocity;
	float wB = stateB.angularVelocity;

	b2Rot qA = b2MulRot( stateA.deltaRotation, joint.frameA.q );
	b2Rot qB = b2MulRot( stateB.deltaRotation, joint.frameB.q );
	b2Rot relQ = b2InvMulRot( qA, qB );

	// current anchors
	b2Vec2 rA = b2RotateVector( stateA.deltaRotation, joint.frameA.p );
	b2Vec2 rB = b2RotateVector( stateB.deltaRotation, joint.frameB.p );

	b2Vec2 d = b2Add( b2Add( b2Sub( stateB.deltaPosition, stateA.deltaPosition ), joint.deltaCenter ), b2Sub( rB, rA ) );

	b2Vec2 axisA = b2RotateVector( joint.frameA.q, (b2Vec2){ 1.0f, 0.0f } );
	axisA = b2RotateVector( stateA.deltaRotation, axisA );
	float translation = b2Dot( axisA, d );

	// These scalars are for torques generated by axial forces
	float a1 = b2Cross( b2Add( rA, d ), axisA );
	float a2 = b2Cross( rB, axisA );

	float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	float axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	b2Softness softness = sim.constraintSoftness;

	// spring constraint
	if ( joint.enableSpring )
	{
		// This is a real spring and should be applied even during relax
		float C = translation - joint.targetTranslation;
		float bias = joint.springSoftness.biasRate * C;
		float massScale = joint.springSoftness.massScale;
		float impulseScale = joint.springSoftness.impulseScale;

		float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
		float deltaImpulse = -massScale * axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
		joint.springImpulse += deltaImpulse;

		b2Vec2 P = b2MulSV( deltaImpulse, axisA );
		float LA = deltaImpulse * a1;
		float LB = deltaImpulse * a2;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	// Solve motor constraint
	if ( joint.enableMotor )
	{
		float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
		float impulse = axialMass * ( joint.motorSpeed - Cdot );
		float oldImpulse = joint.motorImpulse;
		float maxImpulse = context.h * joint.maxMotorForce;
		joint.motorImpulse = b2ClampFloat( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
		impulse = joint.motorImpulse - oldImpulse;

		b2Vec2 P = b2MulSV( impulse, axisA );
		float LA = impulse * a1;
		float LB = impulse * a2;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	if ( joint.enableLimit )
	{
		// Clamp the speculative distance to a reasonable value
		float speculativeDistance = 0.25f * ( joint.upperTranslation - joint.lowerTranslation );

		// Lower limit
		{
			float C = translation - joint.lowerTranslation;

			if ( C < speculativeDistance )
			{
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;

				if ( C > 0.0f )
				{
					// speculation
					float safe = b2_lengthUnitsPerMeter;
					bias = b2MinFloat( C, safe ) * context.inv_h;
				}
				else if ( useBias )
				{
					bias = softness.biasRate * C;
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				float oldImpulse = joint.lowerImpulse;
				float Cdot = b2Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
				float deltaImpulse = -axialMass * massScale * ( Cdot + bias ) - impulseScale * oldImpulse;
				joint.lowerImpulse = b2MaxFloat( oldImpulse + deltaImpulse, 0.0f );
				deltaImpulse = joint.lowerImpulse - oldImpulse;

				b2Vec2 P = b2MulSV( deltaImpulse, axisA );
				float LA = deltaImpulse * a1;
				float LB = deltaImpulse * a2;

				vA = b2MulSub( vA, mA, P );
				wA -= iA * LA;
				vB = b2MulAdd( vB, mB, P );
				wB += iB * LB;
			}
			else
			{
				joint.lowerImpulse = 0.0f;
			}
		}

		// Upper limit
		// Note: signs are flipped to keep C positive when the constraint is satisfied.
		// This also keeps the impulse positive when the limit is active.
		{
			// sign flipped
			float C = joint.upperTranslation - translation;

			if ( C < speculativeDistance )
			{
				float bias = 0.0f;
				float massScale = 1.0f;
				float impulseScale = 0.0f;

				if ( C > 0.0f )
				{
					// speculation
					float safe = b2_lengthUnitsPerMeter;
					bias = b2MinFloat( C, safe ) * context.inv_h;
				}
				else if ( useBias )
				{
					bias = softness.biasRate * C;
					massScale = softness.massScale;
					impulseScale = softness.impulseScale;
				}

				float oldImpulse = joint.upperImpulse;

				// sign flipped
				float Cdot = b2Dot( axisA, b2Sub( vA, vB ) ) + a1 * wA - a2 * wB;
				float deltaImpulse = -axialMass * massScale * ( Cdot + bias ) - impulseScale * oldImpulse;
				joint.upperImpulse = b2MaxFloat( oldImpulse + deltaImpulse, 0.0f );
				deltaImpulse = joint.upperImpulse - oldImpulse;

				b2Vec2 P = b2MulSV( deltaImpulse, axisA );
				float LA = deltaImpulse * a1;
				float LB = deltaImpulse * a2;

				// sign flipped
				vA = b2MulAdd( vA, mA, P );
				wA += iA * LA;
				vB = b2MulSub( vB, mB, P );
				wB -= iB * LB;
			}
			else
			{
				joint.upperImpulse = 0.0f;
			}
		}
	}

	// Solve the prismatic constraint in block form
	{
		b2Vec2 perpA = b2LeftPerp( axisA );

		// These scalars are for torques generated by the perpendicular constraint force
		float s1 = b2Cross( b2Add( d, rA ), perpA );
		float s2 = b2Cross( rB, perpA );

		b2Vec2 Cdot;
		Cdot.x = b2Dot( perpA, b2Sub( vB, vA ) ) + s2 * wB - s1 * wA;
		Cdot.y = wB - wA;

		b2Vec2 bias = b2Vec2_zero;
		float massScale = 1.0f;
		float impulseScale = 0.0f;
		if ( useBias )
		{
			b2Vec2 C;
			C.x = b2Dot( perpA, d );
			C.y = b2Rot_GetAngle( relQ );

			bias = b2MulSV( softness.biasRate, C );
			massScale = softness.massScale;
			impulseScale = softness.impulseScale;
		}

		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k22 = iA + iB;
		if ( k22 == 0.0f )
		{
			// For bodies with fixed rotation.
			k22 = 1.0f;
		}

		b2Mat22 K = { { k11, k12 }, { k12, k22 } };

		b2Vec2 b = b2Solve22( K, b2Add( Cdot, bias ) );
		b2Vec2 deltaImpulse;
		deltaImpulse.x = -massScale * b.x - impulseScale * joint.impulse.x;
		deltaImpulse.y = -massScale * b.y - impulseScale * joint.impulse.y;

		joint.impulse.x += deltaImpulse.x;
		joint.impulse.y += deltaImpulse.y;

		b2Vec2 P = b2MulSV( deltaImpulse.x, perpA );
		float LA = deltaImpulse.x * s1 + deltaImpulse.y;
		float LB = deltaImpulse.x * s2 + deltaImpulse.y;

		vA = b2MulSub( vA, mA, P );
		wA -= iA * LA;
		vB = b2MulAdd( vB, mB, P );
		wB += iB * LB;
	}

	B2_ASSERT( b2IsValidVec2( vA ) );
	B2_ASSERT( b2IsValidFloat( wA ) );
	B2_ASSERT( b2IsValidVec2( vB ) );
	B2_ASSERT( b2IsValidFloat( wB ) );

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
