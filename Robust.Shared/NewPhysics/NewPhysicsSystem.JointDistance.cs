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
    // 1-D constrained system
    // m (v2 - v1) = lambda
    // v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
    // x2 = x1 + h * v2

    // 1-D mass-damper-spring system
    // m (v2 - v1) + h * d * v2 + h * k *

    // C = norm(p2 - p1) - L
    // u = (p2 - p1) / norm(p2 - p1)
    // Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
    // J = [-u -cross(r1, u) u cross(r2, u)]
    // K = J * invM * JT
    //   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

    private void PrepareDistanceJoint(ref JointSim sim)
    {
        DebugTools.Assert(sim.type == b2JointType.b2_distanceJoint);

        // chase body id to the solver set where the body lives
        int idA = sim.bodyIdA;
        int idB = sim.bodyIdB;

        var bodyA = _bodies[idA].Comp;
        var bodyB = _bodies[idB].Comp;

        DebugTools.Assert(bodyA.SetIndex == (int)SetType.AwakeSet || bodyB.SetIndex == (int)SetType.AwakeSet);

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

	    var joint = (DistanceJoint) _joints[sim.jointId];

	    joint.IndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.IndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // initial anchors in world space
	    joint.AnchorA = Quaternion2D.RotateVector( bodySimA.Transform.Quaternion2D, sim.localFrameA.Position - bodySimA.LocalCenter );
	    joint.AnchorB = Quaternion2D.RotateVector( bodySimB.Transform.Quaternion2D, sim.localFrameB.Position - bodySimB.LocalCenter );
	    joint.DeltaCenter = bodySimB.Center - bodySimA.Center;

	    var rA = joint.AnchorA;
	    var rB = joint.AnchorB;
	    var separation = rB - rA + joint.DeltaCenter;
	    var axis = separation.Normalized();

	    // compute effective mass
	    float crA = Vector2Helpers.Cross( rA, axis );
	    float crB = Vector2Helpers.Cross( rB, axis );
	    float k = mA + mB + iA * crA * crA + iB * crB * crB;
	    joint.AxialMass = k > 0.0f ? 1.0f / k : 0.0f;

	    joint.DistanceSoftness = MakeSoft(joint.hertz, joint.dampingRatio, _h);

	    if (_enableWarmStarting == false)
	    {
		    joint.Impulse = 0.0f;
		    joint.LowerImpulse = 0.0f;
		    joint.UpperImpulse = 0.0f;
		    joint.MotorImpulse = 0.0f;
	    }
    }

    private void WarmStartDistanceJoint(ref JointSim sim)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_distanceJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

        var joint = (DistanceJoint) _joints[sim.jointId];
	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.AnchorA );
	    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.AnchorB );

	    var ds = stateB.deltaPosition - stateA.deltaPosition + rB - rA;
	    var separation = joint.DeltaCenter + ds;
	    var axis = separation.Normalized();

	    float axialImpulse = joint.Impulse + joint.LowerImpulse - joint.UpperImpulse + joint.MotorImpulse;
	    var P = axialImpulse * axis;

	    if ((stateA.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.LinearVelocity = Vector2Helpers.MulSub( stateA.LinearVelocity, mA, P );
		    stateA.AngularVelocity -= iA * Vector2Helpers.Cross( rA, P );
	    }

	    if ((stateB.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.LinearVelocity = Vector2Helpers.MulAdd(stateB.LinearVelocity, mB, P);
		    stateB.AngularVelocity += iB * Vector2Helpers.Cross(rB, P);
	    }
    }

    private void SolveDistanceJoint(ref JointSim sim, bool useBias)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_distanceJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (DistanceJoint) _joints[sim.jointId];
	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var vA = stateA.LinearVelocity;
	    float wA = stateA.AngularVelocity;
	    var vB = stateB.LinearVelocity;
	    float wB = stateB.AngularVelocity;

	    // current anchors
	    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.AnchorA );
	    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.AnchorB );

	    // current separation
	    var ds = stateB.deltaPosition - stateA.deltaPosition + rB - rA;
	    var separation = joint.DeltaCenter + ds;

	    float length = separation.Length();
	    var axis = separation.Normalized();

	    // joint is soft if
	    // - spring is enabled
	    // - and (joint limit is disabled or limits are not equal)
	    if ( joint.enableSpring && ( joint.minLength < joint.maxLength || joint.enableLimit == false ) )
	    {
		    // spring
		    if ( joint.hertz > 0.0f )
		    {
			    // Cdot = dot(u, v + cross(w, r))
			    var vr = vB - vA + Vector2Helpers.Cross( wB, rB ) - Vector2Helpers.Cross( wA, rA );
			    float Cdot = Vector2.Dot( axis, vr );
			    float C = length - joint.length;
			    float bias = joint.DistanceSoftness.biasRate * C;

			    float m = joint.DistanceSoftness.massScale * joint.AxialMass;
			    float oldImpulse = joint.Impulse;
			    float impulse = -m * ( Cdot + bias ) - joint.DistanceSoftness.impulseScale * oldImpulse;

			    float h = _h;
			    joint.Impulse = Math.Clamp( joint.Impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h );
			    impulse = joint.Impulse - oldImpulse;

			    var P = Vector2Helpers.Cross( impulse, axis );
			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * Vector2Helpers.Cross( rA, P );
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
		    }

		    if ( joint.enableLimit )
		    {
			    // lower limit
			    {
				    var vr = vB - vA + Vector2Helpers.Cross( wB, rB ) - Vector2Helpers.Cross( wA, rA );
				    float Cdot = Vector2.Dot( axis, vr );

				    float C = length - joint.minLength;

				    float bias = 0.0f;
				    float massCoeff = 1.0f;
				    float impulseCoeff = 0.0f;
				    if ( C > 0.0f )
				    {
					    // speculative
					    bias = C * _invH;
				    }
				    else if ( useBias )
				    {
					    bias = sim.constraintSoftness.biasRate * C;
					    massCoeff = sim.constraintSoftness.massScale;
					    impulseCoeff = sim.constraintSoftness.impulseScale;
				    }

				    float impulse = -massCoeff * joint.AxialMass * ( Cdot + bias ) - impulseCoeff * joint.LowerImpulse;
				    float newImpulse = MathF.Max( 0.0f, joint.LowerImpulse + impulse );
				    impulse = newImpulse - joint.LowerImpulse;
				    joint.LowerImpulse = newImpulse;

				    var P = impulse * axis;
				    vA = Vector2Helpers.MulSub( vA, mA, P );
				    wA -= iA * Vector2Helpers.Cross( rA, P );
				    vB = Vector2Helpers.MulAdd( vB, mB, P );
				    wB += iB * Vector2Helpers.Cross( rB, P );
			    }

			    // upper
			    {
				    var vr = vA - vB  + Vector2Helpers.Cross( wA, rA ) - Vector2Helpers.Cross( wB, rB );
				    float Cdot = Vector2.Dot( axis, vr );

				    float C = joint.maxLength - length;

				    float bias = 0.0f;
				    float massScale = 1.0f;
				    float impulseScale = 0.0f;
				    if ( C > 0.0f )
				    {
					    // speculative
					    bias = C * _invH;
				    }
				    else if ( useBias )
				    {
					    bias = sim.constraintSoftness.biasRate * C;
					    massScale = sim.constraintSoftness.massScale;
					    impulseScale = sim.constraintSoftness.impulseScale;
				    }

				    float impulse = -massScale * joint.AxialMass * ( Cdot + bias ) - impulseScale * joint.UpperImpulse;
				    float newImpulse = MathF.Max( 0.0f, joint.UpperImpulse + impulse );
				    impulse = newImpulse - joint.UpperImpulse;
				    joint.UpperImpulse = newImpulse;

				    var P = -impulse * axis;
				    vA = Vector2Helpers.MulSub( vA, mA, P );
				    wA -= iA * Vector2Helpers.Cross( rA, P );
				    vB = Vector2Helpers.MulAdd( vB, mB, P );
				    wB += iB * Vector2Helpers.Cross( rB, P );
			    }
		    }

		    if ( joint.enableMotor )
		    {
			    var vr = vB - vA + Vector2Helpers.Cross( wB, rB ) - Vector2Helpers.Cross(wA, rA);
			    float Cdot = Vector2.Dot( axis, vr );
			    float impulse = joint.AxialMass * ( joint.motorSpeed - Cdot );
			    float oldImpulse = joint.MotorImpulse;
			    float maxImpulse = _h * joint.maxMotorForce;
			    joint.MotorImpulse = Math.Clamp( joint.MotorImpulse + impulse, -maxImpulse, maxImpulse );
			    impulse = joint.MotorImpulse - oldImpulse;

			    var P = impulse * axis;
			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * Vector2Helpers.Cross( rA, P );
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
		    }
	    }
	    else
	    {
		    // rigid constraint
		    var vr = vB - vA + Vector2Helpers.Cross(wB, rB) - Vector2Helpers.Cross(wA, rA);
		    float Cdot = Vector2.Dot( axis, vr );

		    float C = length - joint.length;

		    float bias = 0.0f;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias )
		    {
			    bias = sim.constraintSoftness.biasRate * C;
			    massScale = sim.constraintSoftness.massScale;
			    impulseScale = sim.constraintSoftness.impulseScale;
		    }

		    float impulse = -massScale * joint.AxialMass * ( Cdot + bias ) - impulseScale * joint.Impulse;
		    joint.Impulse += impulse;

		    var P = impulse * axis;
		    vA = Vector2Helpers.MulSub( vA, mA, P );
		    wA -= iA * Vector2Helpers.Cross( rA, P );
		    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    wB += iB * Vector2Helpers.Cross( rB, P );
	    }

	    if ((stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.LinearVelocity = vA;
		    stateA.AngularVelocity = wA;
	    }

	    if ((stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.LinearVelocity = vB;
		    stateB.AngularVelocity = wB;
	    }
    }
}
