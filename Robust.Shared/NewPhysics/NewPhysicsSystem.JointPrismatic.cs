using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
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

	DebugTools.Assert(bodyA.SetIndex == (int) SetType.AwakeSet || bodyB.SetIndex == (int) SetType.AwakeSet);
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

	var joint = (PrismaticJoint) _joints[sim.jointId];
	joint.IndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	joint.IndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	// Compute joint anchor frames with world space rotation, relative to center of mass
	joint.frameA.Quaternion2D = bodySimA.transform.Quaternion2D * sim.localFrameA.Quaternion2D;
	joint.frameA.Position = Quaternion2D.RotateVector(bodySimA.transform.Quaternion2D, sim.localFrameA.Position - bodySimA.localCenter);
    joint.frameB.Quaternion2D = bodySimB.transform.Quaternion2D * sim.localFrameB.Quaternion2D;
	joint.frameB.Position = Quaternion2D.RotateVector(bodySimB.transform.Quaternion2D, sim.localFrameB. Position - bodySimB.localCenter);

	// Compute the initial center delta. Incremental position updates are relative to this.
	joint.deltaCenter = bodySimB.center - bodySimA.center;

	joint.springSoftness = MakeSoft( joint.hertz, joint.dampingRatio, _h);

	if (_enableWarmStarting == false)
	{
		joint.impulse = Vector2.Zero;
		joint.springImpulse = 0.0f;
		joint.motorImpulse = 0.0f;
		joint.lowerImpulse = 0.0f;
		joint.upperImpulse = 0.0f;
	}
}

private void WarmStartPrismaticJoint(ref JointSim sim)
{
	DebugTools.Assert(sim.type == b2JointType.b2_prismaticJoint);

	float mA = sim.invMassA;
	float mB = sim.invMassB;
	float iA = sim.invIA;
	float iB = sim.invIB;

	// dummy state for static bodies
	var dummyState = BodyState.Identity;

	var joint = (PrismaticJoint) _joints[sim.jointId];

	var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

    var d = stateB.deltaPosition - stateA.deltaPosition + joint.deltaCenter + rB - rA;

    var axisA = Quaternion2D.RotateVector(joint.frameA.Quaternion2D, Vector2.UnitX);
	axisA = Quaternion2D.RotateVector(stateA.deltaRotation, axisA);

	// impulse is applied at anchor point on body B
	float a1 = Vector2Helpers.Cross( rA + d, axisA );
	float a2 = Vector2Helpers.Cross( rB, axisA );
	float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	// perpendicular constraint
	var perpA = axisA.LeftPerp();
	float s1 = Vector2Helpers.Cross( rA + d, perpA );
	float s2 = Vector2Helpers.Cross( rB, perpA );
	float perpImpulse = joint.impulse.X;
	float angleImpulse = joint.impulse.Y;

    var P = axialImpulse * axisA + perpImpulse * perpA;
	float LA = axialImpulse * a1 + perpImpulse * s1 + angleImpulse;
	float LB = axialImpulse * a2 + perpImpulse * s2 + angleImpulse;

	if ((stateA.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	{
		stateA.linearVelocity = Vector2Helpers.MulSub( stateA.linearVelocity, mA, P );
		stateA.angularVelocity -= iA * LA;
	}

	if ((stateB.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0 )
	{
		stateB.linearVelocity = Vector2Helpers.MulAdd( stateB.linearVelocity, mB, P );
		stateB.angularVelocity += iB * LB;
	}
}

private void SolvePrismaticJoint(ref JointSim sim, bool useBias )
{
	DebugTools.Assert(sim.type == b2JointType.b2_prismaticJoint);

	float mA = sim.invMassA;
	float mB = sim.invMassB;
	float iA = sim.invIA;
	float iB = sim.invIB;

	// dummy state for static bodies
	var dummyState = BodyState.Identity;

	var joint = (PrismaticJoint) _joints[sim.jointId];

    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	var vA = stateA.linearVelocity;
	float wA = stateA.angularVelocity;
    var vB = stateB.linearVelocity;
	float wB = stateB.angularVelocity;

    var qA = stateA.deltaRotation * joint.frameA.Quaternion2D;
    var qB = stateB.deltaRotation * joint.frameB.Quaternion2D;
    var relQ = Quaternion2D.InvMulRot( qA, qB );

	// current anchors
    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
	var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

    var d = stateB.deltaPosition - stateA.deltaPosition + joint.deltaCenter + rB - rA;

    var axisA = Quaternion2D.RotateVector(joint.frameA.Quaternion2D, Vector2.UnitX);
	axisA = Quaternion2D.RotateVector( stateA.deltaRotation, axisA );
	float translation = Vector2.Dot( axisA, d );

	// These scalars are for torques generated by axial forces
	float a1 = Vector2Helpers.Cross(rA + d, axisA );
	float a2 = Vector2Helpers.Cross( rB, axisA );

	float k = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	float axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	var softness = sim.constraintSoftness;

	// spring constraint
	if ( joint.enableSpring )
	{
		// This is a real spring and should be applied even during relax
		float C = translation - joint.targetTranslation;
		float bias = joint.springSoftness.biasRate * C;
		float massScale = joint.springSoftness.massScale;
		float impulseScale = joint.springSoftness.impulseScale;

		float Cdot = Vector2.Dot( axisA, b2Sub( vB, vA ) ) + a2 * wB - a1 * wA;
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
		float s1 = Vector2Helpers.Cross( b2Add( d, rA ), perpA );
		float s2 = Vector2Helpers.Cross( rB, perpA );

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

	DebugTools.Assert( b2IsValidVec2( vA ) );
    DebugTools.Assert( b2IsValidFloat( wA ) );
    DebugTools.Assert( b2IsValidVec2( vB ) );
    DebugTools.Assert( b2IsValidFloat( wB ) );

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
