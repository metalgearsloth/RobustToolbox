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

    private void PrepareWeldJoint(ref JointSim sim)
    {
	    DebugTools.Assert( sim.type == b2JointType.b2_weldJoint );

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

	    var joint = (WeldJoint) _joints[sim.jointId];
	    joint.IndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? localIndexA : PhysicsConstants.NullIndex;
	    joint.IndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? localIndexB : PhysicsConstants.NullIndex;

	    // Compute joint anchor frames with world space rotation, relative to center of mass
	    joint.frameA.Quaternion2D = bodySimA.transform.Quaternion2D * sim.localFrameA.Quaternion2D;
	    joint.frameA.Position = Quaternion2D.RotateVector( bodySimA.transform.Quaternion2D, sim.localFrameA.Position - bodySimA.localCenter);
	    joint.frameB.Quaternion2D = bodySimB.transform.Quaternion2D * sim.localFrameB.Quaternion2D;
	    joint.frameB.Position = Quaternion2D.RotateVector( bodySimB.transform.Quaternion2D, sim.localFrameB. Position - bodySimB.localCenter);

	    // Compute the initial center delta. Incremental position updates are relative to this.
	    joint.deltaCenter = bodySimB.center - bodySimA.center;

	    float ka = iA + iB;
	    joint.axialMass = ka > 0.0f ? 1.0f / ka : 0.0f;

	    if ( joint.linearHertz == 0.0f )
	    {
		    joint.linearSpring = sim.constraintSoftness;
	    }
	    else
	    {
		    joint.linearSpring = MakeSoft(joint.linearHertz, joint.linearDampingRatio, _h);
	    }

	    if ( joint.angularHertz == 0.0f )
	    {
		    joint.angularSpring = sim.constraintSoftness;
	    }
	    else
	    {
		    joint.angularSpring = MakeSoft( joint.angularHertz, joint.angularDampingRatio, _h);
	    }

	    if (_enableWarmStarting == false )
	    {
		    joint.linearImpulse = Vector2.Zero;
		    joint.angularImpulse = 0.0f;
	    }
    }

    private void WarmStartWeldJoint(ref JointSim sim)
    {
	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (WeldJoint) _joints[sim.jointId];

	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
	    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

	    if ( (stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
	    {
		    stateA.linearVelocity = Vector2Helpers.MulSub( stateA.linearVelocity, mA, joint.linearImpulse );
		    stateA.angularVelocity -= iA * ( Vector2Helpers.Cross( rA, joint.linearImpulse ) + joint.angularImpulse );
	    }

	    if ( (stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
	    {
		    stateB.linearVelocity = Vector2Helpers.MulAdd( stateB.linearVelocity, mB, joint.linearImpulse );
		    stateB.angularVelocity += iB * ( Vector2Helpers.Cross( rB, joint.linearImpulse ) + joint.angularImpulse );
	    }
    }

    private void SolveWeldJoint(ref JointSim sim, bool useBias)
    {
	    DebugTools.Assert(sim.type == b2JointType.b2_weldJoint);

	    float mA = sim.invMassA;
	    float mB = sim.invMassB;
	    float iA = sim.invIA;
	    float iB = sim.invIB;

	    // dummy state for static bodies
	    var dummyState = BodyState.Identity;

	    var joint = (WeldJoint) _joints[sim.jointId];

	    var stateA = joint.IndexA == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexA];
	    var stateB = joint.IndexB == PhysicsConstants.NullIndex ? dummyState : _states[joint.IndexB];

	    var vA = stateA.linearVelocity;
	    float wA = stateA.angularVelocity;
	    var vB = stateB.linearVelocity;
	    float wB = stateB.angularVelocity;

	    // Block solve doesn't work correctly with mixed stiffness values
    #if B2_WELD_BLOCK_SOLVE
	    // J = [-I -r1_skew I r2_skew]
	    //     [ 0       -1 0       1]
	    // r_skew = [-ry; rx]

	    // Matlab
	    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]
	    b2Vec2 rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.p );
	    b2Vec2 rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.p );

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
	    if ( useBias || joint.linearHertz > 0.0f )
	    {
		    // linear
		    b2Vec2 dcA = stateA.deltaPosition;
		    b2Vec2 dcB = stateB.deltaPosition;
		    b2Vec2 jointTranslation = b2Add( b2Add( b2Sub( dcB, dcA ), b2Sub( rB, rA ) ), joint.deltaCenter );

		    bias.x = joint.linearSpring.biasRate * jointTranslation.x;
		    bias.y = joint.linearSpring.biasRate * jointTranslation.y;

		    linearMassScale = joint.linearSpring.massScale;
		    linearImpulseScale = joint.linearSpring.impulseScale;
	    }

	    float angularMassScale = 1.0f;
	    float angularImpulseScale = 0.0f;
	    if ( useBias || joint.angularHertz > 0.0f )
	    {
		    // angular
		    b2Rot qA = b2MulRot( stateA.deltaRotation, joint.frameA.q );
		    b2Rot qB = b2MulRot( stateB.deltaRotation, joint.frameB.q );
		    b2Rot relQ = b2InvMulRot( qA, qB );
		    float jointAngle = b2Rot_GetAngle( relQ );

		    bias.z = joint.angularSpring.biasRate * jointAngle;

		    angularMassScale = joint.angularSpring.massScale;
		    angularImpulseScale = joint.angularSpring.impulseScale;
	    }

	    b2Vec2 Cdot1 = b2Sub( b2Add( vB, Vector2Helpers.Cross( wB, rB ) ), b2Add( vA, Vector2Helpers.Cross( wA, rA ) ) );
	    float Cdot2 = wB - wA;

	    b2Vec3 Cdot = {Cdot1.x + bias.x, Cdot1.y + bias.y, Cdot2 + bias.z};

	    b2Vec3 b = b2Solve33( &K, Cdot );

	    b2Vec2 linearImpulse = {
		    -linearMassScale * b.x - linearImpulseScale * joint.linearImpulse.x,
		    -linearMassScale * b.y - linearImpulseScale * joint.linearImpulse.y,
	    };
	    joint.linearImpulse = b2Add( joint.linearImpulse, linearImpulse );

	    float angularImpulse = -angularMassScale * b.z - angularImpulseScale * joint.angularImpulse;
	    joint.angularImpulse += angularImpulse;

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
		    var qA = stateA.deltaRotation * joint.frameA.Quaternion2D;
		    var qB = stateB.deltaRotation * joint.frameB.Quaternion2D;
		    var relQ = Quaternion2D.InvMulRot( qA, qB );
		    float jointAngle = relQ.Angle;

		    float bias = 0.0f;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias || joint.angularHertz > 0.0f )
		    {
			    float C = jointAngle;
			    bias = joint.angularSpring.biasRate * C;
			    massScale = joint.angularSpring.massScale;
			    impulseScale = joint.angularSpring.impulseScale;
		    }

		    float Cdot = wB - wA;
		    float impulse = -massScale * joint.axialMass * ( Cdot + bias ) - impulseScale * joint.angularImpulse;
		    joint.angularImpulse += impulse;

		    wA -= iA * impulse;
		    wB += iB * impulse;
	    }

	    // linear constraint
	    {
		    var rA = Quaternion2D.RotateVector( stateA.deltaRotation, joint.frameA.Position );
		    var rB = Quaternion2D.RotateVector( stateB.deltaRotation, joint.frameB.Position );

		    var bias = Vector2.Zero;
		    float massScale = 1.0f;
		    float impulseScale = 0.0f;
		    if ( useBias || joint.linearHertz > 0.0f )
		    {
			    var dcA = stateA.deltaPosition;
                var dcB = stateB.deltaPosition;
                var C = dcB - dcA + rB - rA + joint.deltaCenter;

			    bias = joint.linearSpring.biasRate * C;
			    massScale = joint.linearSpring.massScale;
			    impulseScale = joint.linearSpring.impulseScale;
		    }

            var Cdot = vB + Vector2Helpers.Cross(wB, rB) - vA + Vector2Helpers.Cross(wA, rA);

		    Matrix22 K;
		    K.EX.X = mA + mB + rA.Y * rA.Y * iA + rB.Y * rB.Y * iB;
		    K.EY.X = -rA.Y * rA.X * iA - rB.Y * rB.X * iB;
		    K.EX.Y = K.EY.X;
		    K.EY.Y = mA + mB + rA.X * rA.X * iA + rB.X * rB.X * iB;
            var b = K.Solve(Cdot + bias);

		    var impulse = new Vector2(
			    -massScale * b.X - impulseScale * joint.linearImpulse.X,
			    -massScale * b.Y - impulseScale * joint.linearImpulse.Y
		    );

		    joint.linearImpulse += impulse;

		    vA = Vector2Helpers.MulSub( vA, mA, impulse );
		    wA -= iA * Vector2Helpers.Cross( rA, impulse );
		    vB = Vector2Helpers.MulAdd( vB, mB, impulse );
		    wB += iB * Vector2Helpers.Cross( rB, impulse );
	    }
    #endif

	    DebugTools.Assert(vA.IsValid());
	    DebugTools.Assert(wA.IsValid());
	    DebugTools.Assert(vB.IsValid());
	    DebugTools.Assert(wB.IsValid());

	    if ((stateA.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateA.linearVelocity = vA;
		    stateA.angularVelocity = wA;
	    }

	    if ((stateB.flags & (uint) BodyFlags.b2_dynamicFlag) != 0x0)
	    {
		    stateB.linearVelocity = vB;
		    stateB.angularVelocity = wB;
	    }
    }
}
