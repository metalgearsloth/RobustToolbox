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
    // Point-to-point constraint
    // C = p2 - p1
    // Cdot = v2 - v1
    //      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
    // J = [-I -r1_skew I r2_skew ]
    // Identity used:
    // w k % (rx i + ry j) = w * (-ry i + rx j)

    // Angle constraint
    // C = angle2 - angle1 - referenceAngle
    // Cdot = w2 - w1
    // J = [0 0 -1 0 0 1]
    // K = invI1 + invI2

    private void PrepareMotorJoint(ref JointSim sim)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_motorJoint);

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

	    var joint = (MotorJoint) _joints[sim.jointId];
	    joint.IndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.IndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.Quaternion2D = bodySimA.transform.Quaternion2D * sim.localFrameA.Quaternion2D;
	    joint.frameA.Position = Quaternion2D.RotateVector(bodySimA.transform.Quaternion2D, sim.localFrameA.Position - bodySimA.localCenter);
	    joint.frameB.Quaternion2D = bodySimB.transform.Quaternion2D * sim.localFrameB.Quaternion2D;
	    joint.frameB.Position = Quaternion2D.RotateVector(bodySimB.transform.Quaternion2D, sim.localFrameB.Position - bodySimB.localCenter);

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint.deltaCenter = bodySimB.center - bodySimA.center;

	    var rA = joint.frameA.Position;
	    var rB = joint.frameB.Position;

	    joint.linearSpring = MakeSoft(joint.linearHertz, joint.linearDampingRatio, _h);
	    joint.angularSpring = MakeSoft(joint.angularHertz, joint.angularDampingRatio, _h);

	    Matrix22 kl;
	    kl.EX.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
	    kl.EX.Y = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
	    kl.EY.X = kl.EX.Y;
	    kl.EY.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
	    joint.linearMass = kl.GetInverse();

	    float ka = iA + iB;
	    joint.angularMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	    if (_enableWarmStarting == false )
	    {
		    joint.linearVelocityImpulse = Vector2.Zero;
		    joint.angularVelocityImpulse = 0.0f;
		    joint.linearSpringImpulse = Vector2.Zero;
		    joint.angularSpringImpulse = 0.0f;
	    }
    }

    private void WarmStartMotorJoint(ref JointSim sim)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_motorJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

        var joint = (MotorJoint)_joints[sim.jointId];

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var rA = Quaternion2D.RotateVector(stateA.deltaRotation, joint.frameA.Position);
	    var rB = Quaternion2D.RotateVector(stateB.deltaRotation, joint.frameB.Position);

	    var linearImpulse = joint.linearVelocityImpulse + joint.linearSpringImpulse;
	    float angularImpulse = joint.angularVelocityImpulse + joint.angularSpringImpulse;

	    if ((stateA.flags & (ushort) BodyFlags.b2_dynamicFlag ) != 0x0)
	    {
		    stateA.linearVelocity = Vector2Helpers.MulSub( stateA.linearVelocity, mA, linearImpulse );
		    stateA.angularVelocity -= iA * ( Vector2Helpers.Cross( rA, linearImpulse ) + angularImpulse );
	    }

	    if ((stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.linearVelocity = Vector2Helpers.MulAdd( stateB.linearVelocity, mB, linearImpulse );
		    stateB.angularVelocity += iB * ( Vector2Helpers.Cross( rB, linearImpulse ) + angularImpulse );
	    }
    }

    private void SolveMotorJoint(ref JointSim sim)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_motorJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (MotorJoint) _joints[sim.jointId];
	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var vA = stateA.linearVelocity;
	    float wA = stateA.angularVelocity;
	    var vB = stateB.linearVelocity;
	    float wB = stateB.angularVelocity;

	    // angular spring
	    if ( joint.maxSpringTorque > 0.0f && joint.angularHertz > 0.0f )
	    {
		    var qA = stateA.deltaRotation * joint.frameA.Quaternion2D;
		    var qB = stateB.deltaRotation * joint.frameB.Quaternion2D;
		    var relQ = Quaternion2D.InvMulRot(qA, qB);

		    float c = relQ.Angle;
		    float bias = joint.angularSpring.biasRate * c;
		    float massScale = joint.angularSpring.massScale;
		    float impulseScale = joint.angularSpring.impulseScale;

		    float cdot = wB - wA;

		    float maxImpulse = _h * joint.maxSpringTorque;
		    float oldImpulse = joint.angularSpringImpulse;
		    float impulse = -massScale * joint.angularMass * ( cdot + bias ) - impulseScale * oldImpulse;
		    joint.angularSpringImpulse = Math.Clamp( oldImpulse + impulse, -maxImpulse, maxImpulse );
		    impulse = joint.angularSpringImpulse - oldImpulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    // angular velocity
	    if ( joint.maxVelocityTorque > 0.0f )
	    {
		    float cdot = wB - wA - joint.angularVelocity;
		    float impulse = -joint.angularMass * cdot;

		    float maxImpulse = _h * joint.maxVelocityTorque;
		    float oldImpulse = joint.angularVelocityImpulse;
		    joint.angularVelocityImpulse = Math.Clamp( oldImpulse + impulse, -maxImpulse, maxImpulse );
		    impulse = joint.angularVelocityImpulse - oldImpulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
	    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

	    // linear spring
	    if ( joint.maxSpringForce > 0.0f && joint.linearHertz > 0.0f )
	    {
		    var dcA = stateA.deltaPosition;
		    var dcB = stateB.deltaPosition;
		    var c = dcB - dcA + rB - rA + joint.deltaCenter;

		    var bias = joint.linearSpring.biasRate * c;
		    float massScale = joint.linearSpring.massScale;
		    float impulseScale = joint.linearSpring.impulseScale;

		    var cdot =  vB + Vector2Helpers.Cross( wB, rB ) - vA + Vector2Helpers.Cross( wA, rA );
		    cdot += bias;

		    // Updating the effective mass here may be overkill
		    Matrix22 kl;
		    kl.EX.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
		    kl.EX.Y = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
		    kl.EY.X = kl.EX.Y;
		    kl.EY.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
		    joint.linearMass = kl.GetInverse();

		    var b = joint.linearMass * cdot;

		    var oldImpulse = joint.linearSpringImpulse;
		    var impulse = new Vector2(
			    -massScale * b.X - impulseScale * oldImpulse.X,
			    -massScale * b.Y - impulseScale * oldImpulse.Y);

		    float maxImpulse = _h * joint.maxSpringForce;
		    joint.linearSpringImpulse += impulse;

		    if ( joint.linearSpringImpulse.LengthSquared() > maxImpulse * maxImpulse )
		    {
			    joint.linearSpringImpulse = joint.linearSpringImpulse.Normalized();
			    joint.linearSpringImpulse.X *= maxImpulse;
			    joint.linearSpringImpulse.Y *= maxImpulse;
		    }

		    impulse = joint.linearSpringImpulse - oldImpulse;

		    vA = Vector2Helpers.MulSub( vA, mA, impulse );
		    wA -= iA * Vector2Helpers.Cross( rA, impulse );
		    vB = Vector2Helpers.MulAdd( vB, mB, impulse );
		    wB += iB * Vector2Helpers.Cross( rB, impulse );
	    }

	    // linear velocity
	    if ( joint.maxVelocityForce > 0.0f )
	    {
		    var cdot = vB + Vector2Helpers.Cross(wB, rB) - vA + Vector2Helpers.Cross(wA, rA);
		    cdot = cdot - joint.linearVelocity;
		    var b = joint.linearMass * cdot;
		    var impulse = new Vector2(-b.X, -b.Y);

		    var oldImpulse = joint.linearVelocityImpulse;
		    float maxImpulse = _h * joint.maxVelocityForce;
		    joint.linearVelocityImpulse += impulse;

		    if (joint.linearVelocityImpulse.LengthSquared() > maxImpulse * maxImpulse )
		    {
			    joint.linearVelocityImpulse = joint.linearVelocityImpulse.Normalized();
			    joint.linearVelocityImpulse.X *= maxImpulse;
			    joint.linearVelocityImpulse.Y *= maxImpulse;
		    }

		    impulse = joint.linearVelocityImpulse - oldImpulse;

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
