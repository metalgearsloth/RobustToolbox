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

    private void PrepareWheelJoint(ref JointSim sim)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_wheelJoint);

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

	    float mA = bodySimA.InvMass;
	    float iA = bodySimA.InvInertia;
	    float mB = bodySimB.InvMass;
	    float iB = bodySimB.InvInertia;

	    sim.invMassA = mA;
	    sim.invMassB = mB;
	    sim.invIA = iA;
	    sim.invIB = iB;

	    var joint = (WheelJoint) _joints[sim.jointId];

	    joint.IndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.IndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // Compute joint anchor frames with world space rotation, relative to center of mass
	    joint.frameA.Quaternion2D = bodySimA.Transform.Quaternion2D * sim.localFrameA.Quaternion2D;
	    joint.frameA.Position = Quaternion2D.RotateVector( bodySimA.Transform.Quaternion2D, sim.localFrameA.Position - bodySimA.LocalCenter);
	    joint.frameB.Quaternion2D = bodySimB.Transform.Quaternion2D * sim.localFrameB.Quaternion2D;
	    joint.frameB.Position = Quaternion2D.RotateVector( bodySimB.Transform.Quaternion2D, sim.localFrameB.Position - bodySimB.LocalCenter);

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint.deltaCenter = bodySimB.Center - bodySimA.Center;

	    var rA = joint.frameA.Position;
        var rB = joint.frameB.Position;

        var d = joint.deltaCenter + rB - rA;
        var axisA = Quaternion2D.RotateVector( joint.frameA.Quaternion2D, Vector2.UnitX);
        var perpA = axisA.LeftPerp();

	    // perpendicular constraint (keep wheel on line)
	    float s1 = Vector2Helpers.Cross(d + rA, perpA);
	    float s2 = Vector2Helpers.Cross(rB, perpA);

	    float kp = mA + mB + iA * s1 * s1 + iB * s2 * s2;
	    joint.perpMass = kp > 0.0f ? 1.0f / kp : 0.0f;

	    // spring constraint
	    float a1 = Vector2Helpers.Cross(d + rA, axisA);
	    float a2 = Vector2Helpers.Cross(rB, axisA);

	    float ka = mA + mB + iA * a1 * a1 + iB * a2 * a2;
	    joint.axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	    joint.springSoftness = MakeSoft(joint.hertz, joint.dampingRatio, _h);

	    float km = iA + iB;
	    joint.motorMass = km > 0.0f ? 1.0f / km : 0.0f;

	    if (_enableWarmStarting == false)
	    {
		    joint.perpImpulse = 0.0f;
		    joint.springImpulse = 0.0f;
		    joint.motorImpulse = 0.0f;
		    joint.lowerImpulse = 0.0f;
		    joint.upperImpulse = 0.0f;
	    }
    }

    private void WarmStartWheelJoint(ref JointSim sim)
    {
	    DebugTools.Assert( sim.type == b2JointType.b2_wheelJoint );

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (WheelJoint) _joints[sim.jointId];

	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
        var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
        var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

        var d = stateB.deltaPosition - stateA.deltaPosition + joint.deltaCenter + rB - rA;
        var axisA = Quaternion2D.RotateVector( joint.frameA.Quaternion2D, Vector2.UnitX);
	    axisA = Quaternion2D.RotateVector( stateA.deltaRotation, axisA );
        var perpA = axisA.LeftPerp();

	    float a1 = Vector2Helpers.Cross(d + rA, axisA);
	    float a2 = Vector2Helpers.Cross(rB, axisA);
	    float s1 = Vector2Helpers.Cross(d + rA, perpA);
	    float s2 = Vector2Helpers.Cross(rB, perpA);

	    float axialImpulse = joint.springImpulse + joint.lowerImpulse - joint.upperImpulse;

	    var P = axialImpulse * axisA + joint.perpImpulse * perpA;
	    float LA = axialImpulse * a1 + joint.perpImpulse * s1 + joint.motorImpulse;
	    float LB = axialImpulse * a2 + joint.perpImpulse * s2 + joint.motorImpulse;

	    if ((stateA.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.LinearVelocity = Vector2Helpers.MulSub( stateA.LinearVelocity, mA, P );
		    stateA.AngularVelocity -= iA * LA;
	    }

	    if ((stateB.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.LinearVelocity = Vector2Helpers.MulAdd( stateB.LinearVelocity, mB, P );
		    stateB.AngularVelocity += iB * LB;
	    }
    }

    private void SolveWheelJoint(ref JointSim sim, bool useBias)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_wheelJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (WheelJoint) _joints[sim.jointId];

	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
        var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

        var vA = stateA.LinearVelocity;
	    float wA = stateA.AngularVelocity;
        var vB = stateB.LinearVelocity;
	    float wB = stateB.AngularVelocity;

	    bool fixedRotation = ( iA + iB == 0.0f );

	    // current anchors
        var rA = Quaternion2D.RotateVector(stateA.deltaRotation, joint.frameA.Position);
        var rB = Quaternion2D.RotateVector(stateB.deltaRotation, joint.frameB.Position);

        var d = stateB.deltaPosition - stateA.deltaPosition + joint.deltaCenter + rB - rA;
        var axisA = Quaternion2D.RotateVector(joint.frameA.Quaternion2D, Vector2.UnitX);
	    axisA = Quaternion2D.RotateVector(stateA.deltaRotation, axisA);
	    float translation = Vector2.Dot(axisA, d);

	    float a1 = Vector2Helpers.Cross(d + rA, axisA);
	    float a2 = Vector2Helpers.Cross(rB, axisA);

	    // motor constraint
	    if ( joint.enableMotor && fixedRotation == false )
	    {
		    float Cdot = wB - wA - joint.motorSpeed;
		    float impulse = -joint.motorMass * Cdot;
		    float oldImpulse = joint.motorImpulse;
		    float maxImpulse = _h * joint.maxMotorTorque;
		    joint.motorImpulse = Math.Clamp( joint.motorImpulse + impulse, -maxImpulse, maxImpulse );
		    impulse = joint.motorImpulse - oldImpulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    // spring constraint
	    if ( joint.enableSpring )
	    {
		    // This is a real spring and should be applied even during relax
		    float C = translation;
		    float bias = joint.springSoftness.biasRate * C;
		    float massScale = joint.springSoftness.massScale;
		    float impulseScale = joint.springSoftness.impulseScale;

		    float Cdot = Vector2.Dot(axisA, vB - vA) + a2 * wB - a1 * wA;
		    float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.springImpulse;
		    joint.springImpulse += impulse;

		    var P = impulse * axisA;
		    float LA = impulse * a1;
		    float LB = impulse * a2;

		    vA = Vector2Helpers.MulSub( vA, mA, P );
		    wA -= iA * LA;
		    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    wB += iB * LB;
	    }

	    if ( joint.enableLimit )
	    {
		    // Lower limit
		    {
			    float C = translation - joint.lowerTranslation;
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

			    float Cdot = Vector2.Dot( axisA, vB - vA) + a2 * wB - a1 * wA;
			    float impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.lowerImpulse;
			    float oldImpulse = joint.lowerImpulse;
			    joint.lowerImpulse = MathF.Max( oldImpulse + impulse, 0.0f );
			    impulse = joint.lowerImpulse - oldImpulse;

                var P = impulse * axisA;
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
			    float C = joint.upperTranslation - translation;
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
			    float Cdot = Vector2.Dot(axisA, vA - vB) + a1 * wA - a2 * wB;
			    float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.upperImpulse;
			    float oldImpulse = joint.upperImpulse;
			    joint.upperImpulse = MathF.Max( oldImpulse + impulse, 0.0f );
			    impulse = joint.upperImpulse - oldImpulse;

                var P = impulse * axisA;
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
            var perpA = axisA.LeftPerp();

		    float bias = 0.0f;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias )
		    {
			    float C = Vector2.Dot( perpA, d );
			    bias = sim.constraintSoftness.biasRate * C;
			    massScale = sim.constraintSoftness.massScale;
			    impulseScale = sim.constraintSoftness.impulseScale;
		    }

		    float s1 = Vector2Helpers.Cross(d + rA, perpA);
		    float s2 = Vector2Helpers.Cross(rB, perpA);
		    float Cdot = Vector2.Dot( perpA, vB - vA) + s2 * wB - s1 * wA;

		    float impulse = -massScale * joint.perpMass * ( Cdot + bias ) - impulseScale * joint.perpImpulse;
		    joint.perpImpulse += impulse;

            var P = impulse * perpA;
		    float LA = impulse * s1;
		    float LB = impulse * s2;

		    vA = Vector2Helpers.MulSub( vA, mA, P );
		    wA -= iA * LA;
		    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    wB += iB * LB;
	    }

	    if ((stateA.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.LinearVelocity = vA;
		    stateA.AngularVelocity = wA;
	    }

	    if ((stateB.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.LinearVelocity = vB;
		    stateB.AngularVelocity = wB;
	    }
    }
}
