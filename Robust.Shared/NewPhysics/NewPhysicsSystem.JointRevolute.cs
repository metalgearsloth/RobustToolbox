using System;
using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Bodies;
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
	    joint.frameA.Quaternion2D = bodySimA.transform.Quaternion2D * sim.localFrameA.Quaternion2D;
	    joint.frameA.Position = Quaternion2D.RotateVector( bodySimA.transform.Quaternion2D, sim.localFrameA.Position - bodySimA.localCenter);
	    joint.frameB.Quaternion2D = bodySimB.transform.Quaternion2D * sim.localFrameB.Quaternion2D;
	    joint.frameB.Position = Quaternion2D.RotateVector(bodySimB.transform.Quaternion2D, sim.localFrameB.Position - bodySimB.localCenter);

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint.deltaCenter = bodySimB.center - bodySimA.center;

	    float k = iA + iB;
	    joint.axialMass = k > 0.0f ? 1.0f / k : 0.0f;

	    joint.springSoftness = MakeSoft(joint.hertz, joint.dampingRatio, _h);

	    if (_enableWarmStarting == false)
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
	    DebugTools.Assert(sim.type == b2JointType.b2_revoluteJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (RevoluteJoint) _joints[sim.jointId];
	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
	    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

	    float axialImpulse = joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse;

	    if ((stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.linearVelocity = Vector2Helpers.MulSub( stateA.linearVelocity, mA, joint.linearImpulse );
		    stateA.angularVelocity -= iA * ( Vector2Helpers.Cross( rA, joint.linearImpulse ) + axialImpulse );
	    }

	    if ((stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.linearVelocity = Vector2Helpers.MulAdd( stateB.linearVelocity, mB, joint.linearImpulse );
		    stateB.angularVelocity += iB * ( Vector2Helpers.Cross( rB, joint.linearImpulse ) + axialImpulse );
	    }
    }

    private void SolveRevoluteJoint(ref JointSim sim, bool useBias )
    {
	    DebugTools.Assert( sim.type == b2JointType.b2_revoluteJoint );

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (RevoluteJoint) _joints[sim.jointId];

	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
        var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var vA = stateA.linearVelocity;
	    float wA = stateA.angularVelocity;
	    var vB = stateB.linearVelocity;
	    float wB = stateB.angularVelocity;

	    var qA = stateA.deltaRotation * joint.frameA.Quaternion2D;
	    var qB = stateB.deltaRotation * joint.frameB.Quaternion2D;
	    var relQ = Quaternion2D.InvMulRot(qA, qB);

	    bool fixedRotation = ( iA + iB == 0.0f );

	    // Solve spring.
	    if ( joint.enableSpring && fixedRotation == false )
	    {
		    float jointAngle = relQ.Angle;
		    float jointAngleDelta = ( jointAngle - joint.targetAngle );

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
		    float maxImpulse = _h * joint.maxMotorTorque;
		    joint.motorImpulse = Math.Clamp( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
		    impulse = joint.motorImpulse - oldImpulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    if ( joint.enableLimit && fixedRotation == false )
	    {
		    float jointAngle = relQ.Angle;

		    // Lower limit
		    {
			    float C = jointAngle - joint.lowerAngle;
			    float bias = 0.0f;
			    float massScale = 1.0f;
			    float impulseScale = 0.0f;
			    if ( C > 0.0f )
			    {
				    // speculation
				    bias = C * _invH;
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
				    bias = C * _invH;
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
		    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
		    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

		    var Cdot = vB + Vector2Helpers.Cross(wB, rB) - vA + Vector2Helpers.Cross(wA, rA);

		    var bias = Vector2.Zero;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if (useBias)
		    {
			    var dcA = stateA.deltaPosition;
                var dcB = stateB.deltaPosition;

                var separation = dcB - dcA + rB - rA + joint.deltaCenter;
			    bias = sim.constraintSoftness.biasRate * separation;
			    massScale = sim.constraintSoftness.massScale;
			    impulseScale = sim.constraintSoftness.impulseScale;
		    }

		    Matrix22 K;
		    K.EX.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
		    K.EY.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
		    K.EX.Y = K.EY.X;
		    K.EY.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
		    var b = K.Solve(Cdot + bias);

		    Vector2 impulse;
		    impulse.X = -massScale * b.X - impulseScale * joint.linearImpulse.X;
		    impulse.Y = -massScale * b.Y - impulseScale * joint.linearImpulse.Y;
		    joint.linearImpulse.X += impulse.X;
		    joint.linearImpulse.Y += impulse.Y;

		    vA = Vector2Helpers.MulSub( vA, mA, impulse );
		    wA -= iA * Vector2Helpers.Cross( rA, impulse );
		    vB = Vector2Helpers.MulAdd( vB, mB, impulse );
		    wB += iB * Vector2Helpers.Cross( rB, impulse );
	    }

	    if ((stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.linearVelocity = vA;
		    stateA.angularVelocity = wA;
	    }

	    if ((stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.linearVelocity = vB;
		    stateB.angularVelocity = wB;
	    }
    }
}
