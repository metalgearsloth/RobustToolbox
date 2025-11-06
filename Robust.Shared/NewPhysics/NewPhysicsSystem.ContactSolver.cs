using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Runtime.Intrinsics.X86;
using Robust.Shared.Collections;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Contacts;
using Robust.Shared.Physics;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private static byte _MM_SHUFFLE(int fp3, int fp2, int fp1, int fp0)
    {
        return (byte) ((fp3 << 6) | (fp2 << 4) | (fp1 << 2) | fp0);
    }

    // This is a load and transpose
    private unsafe BodyStateWide GatherBodies(Span<BodyState> states, ReadOnlySpan<int> indices)
    {
        if (Avx.IsSupported)
        {
            var identityValues = new float[8];
            identityValues[0] = 0f;
            identityValues[1] = 0f;
            identityValues[2] = 0f;
            identityValues[3] = 0f;
            identityValues[4] = 0f;
            identityValues[5] = 0f;
            identityValues[6] = 1f;
            identityValues[7] = 0f;

            var identity = Vector256.Create(identityValues);

            ref var state0 = ref states[indices[0]];
            ref var floatRef0 = ref Unsafe.As<BodyState, float>(ref state0);

            ref var state1 = ref states[indices[1]];
            ref var floatRef1 = ref Unsafe.As<BodyState, float>(ref state1);

            ref var state2 = ref states[indices[2]];
            ref var floatRef2 = ref Unsafe.As<BodyState, float>(ref state2);

            ref var state3 = ref states[indices[3]];
            ref var floatRef3 = ref Unsafe.As<BodyState, float>(ref state3);

            ref var state4 = ref states[indices[4]];
            ref var floatRef4 = ref Unsafe.As<BodyState, float>(ref state4);

            ref var state5 = ref states[indices[5]];
            ref var floatRef5 = ref Unsafe.As<BodyState, float>(ref state5);

            ref var state6 = ref states[indices[6]];
            ref var floatRef6 = ref Unsafe.As<BodyState, float>(ref state6);

            ref var state7 = ref states[indices[7]];
            ref var floatRef7 = ref Unsafe.As<BodyState, float>(ref state7);

            fixed (float* ptr0 = &floatRef0)
            fixed (float* ptr1 = &floatRef1)
            fixed (float* ptr2 = &floatRef2)
            fixed (float* ptr3 = &floatRef3)
            fixed (float* ptr4 = &floatRef4)
            fixed (float* ptr5 = &floatRef5)
            fixed (float* ptr6 = &floatRef6)
            fixed (float* ptr7 = &floatRef7)
            {
                var b0 = indices[0] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr0);
                var b1 = indices[1] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr1);
                var b2 = indices[2] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr2);
                var b3 = indices[3] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr3);
                var b4 = indices[4] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr4);
                var b5 = indices[5] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr5);
                var b6 = indices[6] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr6);
                var b7 = indices[7] == PhysicsConstants.NullIndex ? identity : Avx.LoadAlignedVector256(ptr7);

                var t0 = Avx.UnpackLow(b0, b1);
                var t1 = Avx.UnpackHigh(b0, b1);
                var t2 = Avx.UnpackLow( b2, b3 );
                var t3 = Avx.UnpackHigh( b2, b3 );
                var t4 = Avx.UnpackLow( b4, b5 );
                var t5 = Avx.UnpackHigh( b4, b5 );
                var t6 = Avx.UnpackLow( b6, b7 );
                var t7 = Avx.UnpackHigh( b6, b7 );
                var tt0 = Avx.Shuffle( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt1 = Avx.Shuffle( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
                var tt2 = Avx.Shuffle( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt3 = Avx.Shuffle( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
                var tt4 = Avx.Shuffle( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt5 = Avx.Shuffle( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
                var tt6 = Avx.Shuffle( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt7 = Avx.Shuffle( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

                var simdBody = new BodyStateWide();
                var x = Avx.Permute2x128(tt0, tt4, 0x20);
                var y = Avx.Permute2x128(tt1, tt5, 0x20);
                var w = Avx.Permute2x128(tt2, tt6, 0x20);
                var flags = Avx.Permute2x128(tt3, tt7, 0x20);
                var dpX = Avx.Permute2x128(tt0, tt4, 0x31);
                var dpY = Avx.Permute2x128(tt1, tt5, 0x31);
                var dqC = Avx.Permute2x128(tt2, tt6, 0x31);
                var dqS = Avx.Permute2x128(tt3, tt7, 0x31);

                ref var bvX = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.vX);
                ref var bvY = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.vY);
                ref var bW = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.w);
                ref var bFlags = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.flags);
                ref var bDpX = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dpX);
                ref var bDpY = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dpY);
                ref var bDqC = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dqC);
                ref var bDqS = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dqS);

                fixed (float* avX = &bvX)
                fixed (float* avY = &bvY)
                fixed (float* aW = &bW)
                fixed (float* aFlags = &bFlags)
                fixed (float* aDpX = &bDpX)
                fixed (float* aDpY = &bDpY)
                fixed (float* aDqC = &bDqC)
                fixed (float* aDqS = &bDqS)
                {
                    Avx.Store(avX, x);
                    Avx.Store(avY, y);
                    Avx.Store(aW, w);
                    Avx.Store(aFlags, flags);
                    Avx.Store(aDpX, dpX);
                    Avx.Store(aDpY, dpY);
                    Avx.Store(aDqC, dqC);
                    Avx.Store(aDqS, dqS);
                }

                return simdBody;
            }
        }
        // TODO: SSE version but eh
        else
        {
            var sim = new BodyStateWide();

            // Manually load it
            for (var i = 0; i < indices.Length; i++)
            {
                ref var state = ref states[i];
                sim.vX.AsSpan[i] = state.linearVelocity.X;
                sim.vY.AsSpan[i] = state.linearVelocity.Y;
                sim.w.AsSpan[i] = state.angularVelocity;
                sim.flags.AsSpan[i] = state.flags;
                sim.dpX.AsSpan[i] = state.deltaPosition.X;
                sim.dpY.AsSpan[i] = state.deltaPosition.Y;
                sim.dqC.AsSpan[i] = state.deltaRotation.C;
                sim.dqS.AsSpan[i] = state.deltaRotation.S;
            }

            return sim;
        }
    }

    // Writes everything back to the solver bodies but only the velocities change
    private unsafe void ScatterBodies(Span<BodyState> states, ReadOnlySpan<int> indices, ref BodyStateWide simdBody)
    {
        if (Avx.IsSupported)
        {
            ref var vxRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.vX);
            ref var vyRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.vX);
            ref var wRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.w);
            ref var flagsRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.flags);
            ref var dpXRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dpX);
            ref var dpYRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dpY);
            ref var dqCRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dqC);
            ref var dqSRef = ref Unsafe.As<FixedArray8<float>, float>(ref simdBody.dqS);

            fixed (float* ptr0 = &vxRef)
            fixed (float* ptr1 = &vyRef)
            fixed (float* ptr2 = &wRef)
            fixed (float* ptr3 = &flagsRef)
            fixed (float* ptr4 = &dpXRef)
            fixed (float* ptr5 = &dpYRef)
            fixed (float* ptr6 = &dqCRef)
            fixed (float* ptr7 = &dqSRef)
            {
                var bvX = Avx.LoadAlignedVector256(ptr0);
                var bvY = Avx.LoadAlignedVector256(ptr1);
                var bW = Avx.LoadAlignedVector256(ptr2);
                var bFlags = Avx.LoadAlignedVector256(ptr3);
                var bDpX = Avx.LoadAlignedVector256(ptr4);
                var bDpY = Avx.LoadAlignedVector256(ptr5);
                var bDqC = Avx.LoadAlignedVector256(ptr6);
                var bDqS = Avx.LoadAlignedVector256(ptr7);

                var t0 = Avx.UnpackLow(bvX, bvY);
	            var t1 = Avx.UnpackHigh( bvX, bvY );
	            var t2 = Avx.UnpackLow( bW, bFlags );
                var t3 = Avx.UnpackHigh( bW, bFlags );
                var t4 = Avx.UnpackLow( bDpX, bDpY );
                var t5 = Avx.UnpackHigh( bDpX, bDpY );
                var t6 = Avx.UnpackLow( bDqC, bDqS );
                var t7 = Avx.UnpackHigh( bDqC, bDqS );

                var tt0 = Avx.Shuffle( t0, t2, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt1 = Avx.Shuffle( t0, t2, _MM_SHUFFLE( 3, 2, 3, 2 ) );
                var tt2 = Avx.Shuffle( t1, t3, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt3 = Avx.Shuffle( t1, t3, _MM_SHUFFLE( 3, 2, 3, 2 ) );
                var tt4 = Avx.Shuffle( t4, t6, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt5 = Avx.Shuffle( t4, t6, _MM_SHUFFLE( 3, 2, 3, 2 ) );
                var tt6 = Avx.Shuffle( t5, t7, _MM_SHUFFLE( 1, 0, 1, 0 ) );
                var tt7 = Avx.Shuffle( t5, t7, _MM_SHUFFLE( 3, 2, 3, 2 ) );

	            // I don't use any dummy body in the body array because this will lead to multithreaded sharing and the
	            // associated cache flushing.
	            // todo_erin could add a check for kinematic bodies here

                if (indices[0] != PhysicsConstants.NullIndex &&
                    (states[indices[0]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[0]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt0, tt4, 0x20));
                    }
                }

                if (indices[1] != PhysicsConstants.NullIndex &&
                    (states[indices[1]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[1]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt1, tt5, 0x20));
                    }
                }

                if (indices[2] != PhysicsConstants.NullIndex &&
                    (states[indices[2]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[2]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt2, tt6, 0x20));
                    }
                }

                if (indices[3] != PhysicsConstants.NullIndex &&
                    (states[indices[3]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[3]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt3, tt7, 0x20));
                    }
                }

                if (indices[4] != PhysicsConstants.NullIndex &&
                    (states[indices[4]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[4]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt0, tt4, 0x31));
                    }
                }

                if (indices[5] != PhysicsConstants.NullIndex &&
                    (states[indices[5]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[5]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt1, tt5, 0x31));
                    }
                }

                if (indices[6] != PhysicsConstants.NullIndex &&
                    (states[indices[6]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[6]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt2, tt6, 0x31));
                    }
                }

                if (indices[7] != PhysicsConstants.NullIndex &&
                    (states[indices[7]].flags & (uint)BodyFlags.b2_dynamicFlag) != 0)
                {
                    ref var state = ref states[indices[7]];
                    ref var floatRef = ref Unsafe.As<BodyState, float>(ref state);

                    fixed (float* statePtr = &floatRef)
                    {
                        Avx.Store(statePtr, Avx.Permute2x128(tt3, tt7, 0x31));
                    }
                }
            }
        }
        // TODO: SSE
        else
        {
            for (var i = 0; i < indices.Length; i++)
            {
                ref var state = ref states[indices[i]];
                state.linearVelocity.X = simdBody.vX.AsSpan[i];
                state.linearVelocity.Y = simdBody.vY.AsSpan[i];
                state.angularVelocity = simdBody.w.AsSpan[i];
                state.flags = (uint) simdBody.flags.AsSpan[i];
                state.deltaPosition.X = simdBody.dpX.AsSpan[i];
                state.deltaPosition.Y = simdBody.dpY.AsSpan[i];
                state.deltaRotation.C = simdBody.dqC.AsSpan[i];
                state.deltaRotation.S = simdBody.dqS.AsSpan[i];
            }
        }
    }

    private void PrepareContactsTask(int startIndex, int endIndex)
    {
	    ref var contacts = ref _contextContacts;
	    ref var constraints = ref _contextSimdContactConstraints;
	    ref var awakeStates = ref _contextBodyStates;
    #if DEBUG
	    var bodies = _bodies;
    #endif

	    // Stiffer for static contacts to avoid bodies getting pushed through the ground
	    var contactSoftness = _contactSoftness;
	    var staticSoftness = _staticSoftness;
	    bool enableSoftening = _enableContactSoftening;

	    float warmStartScale = _enableWarmStarting ? 1.0f : 0.0f;

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    ref var constraint = ref constraints[i];
            var constraintIndicesA = constraint.indexA.AsSpan;
            var constraintIndicesB = constraint.indexB.AsSpan;
            var constraintNormals = constraint.normal;
            var constraintInvMassA = constraint.invMassA.AsSpan;
            var constraintInvMassB = constraint.invMassB.AsSpan;
            var constraintIA = constraint.invIA.AsSpan;
            var constraintIB = constraint.invIB.AsSpan;
            var constraintRollingMass = constraint.rollingMass.AsSpan;

		    for ( int j = 0; j < _simdWidth; ++j )
		    {
			    ref var contactSim = ref contacts[_simdWidth * i + j];

			    if ( contactSim != null )
			    {
				    ref var manifold = ref contactSim.manifold;

				    int indexA = contactSim.bodySimIndexA;
				    int indexB = contactSim.bodySimIndexB;

    #if DEBUG
				    var bodyA = bodies[contactSim.bodyIdA].Comp;
				    int validIndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? bodyA.LocalIndex : PhysicsConstants.NullIndex;
				    var bodyB = bodies[contactSim.bodyIdB].Comp;
				    int validIndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? bodyB.LocalIndex : PhysicsConstants.NullIndex;

				    DebugTools.Assert( indexA == validIndexA );
				    DebugTools.Assert( indexB == validIndexB );
    #endif
				    constraintIndicesA[j] = indexA;
				    constraintIndicesB[j] = indexB;

				    var vA = Vector2.Zero;
				    float wA = 0.0f;
				    float mA = contactSim.invMassA;
				    float iA = contactSim.invIA;
				    if ( indexA != PhysicsConstants.NullIndex )
				    {
					    ref var stateA = ref awakeStates[indexA];
					    vA = stateA.linearVelocity;
					    wA = stateA.angularVelocity;
				    }

				    var vB = Vector2.Zero;
				    float wB = 0.0f;
				    float mB = contactSim.invMassB;
				    float iB = contactSim.invIB;
				    if ( indexB != PhysicsConstants.NullIndex )
				    {
					    ref var stateB = ref awakeStates[indexB];
					    vB = stateB.linearVelocity;
					    wB = stateB.angularVelocity;
				    }

                    constraintInvMassA[j] = mA;
                    constraintInvMassB[j] = mB;
                    constraintIA[j] = iA;
                    constraintIB[j] = iB;

				    {
					    float k = iA + iB;
                        constraintRollingMass[j] = k > 0.0f ? 1.0f / k : 0.0f;
				    }

				    var soft = contactSoftness;
				    if (indexA == PhysicsConstants.NullIndex || indexB == PhysicsConstants.NullIndex)
				    {
					    soft = staticSoftness;
				    }
				    else if (enableSoftening)
				    {
					    // todo experimental feature
					    float contactHertz = MathF.Min(_contactHertz, 0.125f * _invH);
					    float ratio = 1.0f;
					    if ( mA < mB )
					    {
						    ratio = MathF.Max( 0.5f, mA / mB );
					    }
					    else if ( mB < mA )
					    {
						    ratio = MathF.Max( 0.5f, mB / mA );
					    }
					    soft = MakeSoft( ratio * contactHertz, ratio * _contactDampingRatio, _h );
				    }

				    var normal = manifold.normal;

                    // TODO: Cast as spans above.

                    constraintNormals.X.AsSpan[j] = normal.X;
                    constraintNormals.Y.AsSpan[j] = normal.Y;
                    constraint.friction.AsSpan[j] = contactSim.friction;
                    constraint.tangentSpeed.AsSpan[j] = contactSim.tangentSpeed;
                    constraint.restitution.AsSpan[j] = contactSim.restitution;
                    constraint.rollingResistance.AsSpan[j] = contactSim.restitution;
                    constraint.rollingImpulse.AsSpan[j] = warmStartScale * manifold.rollingImpulse;

                    constraint.biasRate.AsSpan[j] = soft.biasRate;
                    constraint.massScale.AsSpan[j] = soft.massScale;
                    constraint.impulseScale.AsSpan[j] = soft.impulseScale;

				    var tangent = normal.RightPerp();

				    {
					    ref var mp = ref manifold.points._00;

					    var rA = mp.anchorA;
                        var rB = mp.anchorB;

					    constraint.anchorA1.X.AsSpan[j] = rA.X;
					    constraint.anchorA1.Y.AsSpan[j] = rA.Y;
					    constraint.anchorB1.X.AsSpan[j] = rB.X;
					    constraint.anchorB1.Y.AsSpan[j] = rB.Y;

					    constraint.baseSeparation1.AsSpan[j] = mp.separation - Vector2.Dot( rB - rA, normal );

					    constraint.normalImpulse1.AsSpan[j] = warmStartScale * mp.normalImpulse;
					    constraint.tangentImpulse1.AsSpan[j] = warmStartScale * mp.tangentImpulse;
					    constraint.totalNormalImpulse1.AsSpan[j] = 0.0f;

					    float rnA = Vector2Helpers.Cross( rA, normal );
					    float rnB = Vector2Helpers.Cross( rB, normal );
					    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					    constraint.normalMass1.AsSpan[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					    float rtA = Vector2Helpers.Cross( rA, tangent );
					    float rtB = Vector2Helpers.Cross( rB, tangent );
					    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					    constraint.tangentMass1.AsSpan[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					    // relative velocity for restitution
					    var vrA = vA + Vector2Helpers.Cross( wA, rA );
                        var vrB = vB + Vector2Helpers.Cross( wB, rB );
					    constraint.relativeVelocity1.AsSpan[j] = Vector2.Dot(normal, vrB - vrA);
				    }

				    int pointCount = manifold.pointCount;
				    DebugTools.Assert( 0 < pointCount && pointCount <= 2 );

				    if ( pointCount == 2 )
				    {
					    ref var mp = ref manifold.points._01;

					    var rA = mp.anchorA;
					    var rB = mp.anchorB;

					    constraint.anchorA2.X.AsSpan[j] = rA.X;
					    constraint.anchorA2.Y.AsSpan[j] = rA.Y;
					    constraint.anchorB2.X.AsSpan[j] = rB.X;
					    constraint.anchorB2.Y.AsSpan[j] = rB.Y;

					    constraint.baseSeparation2.AsSpan[j] = mp.separation - Vector2.Dot(rB - rA, normal);

					    constraint.normalImpulse2.AsSpan[j] = warmStartScale * mp.normalImpulse;
					    constraint.tangentImpulse2.AsSpan[j] = warmStartScale * mp.tangentImpulse;
					    constraint.totalNormalImpulse2.AsSpan[j] = 0.0f;

					    float rnA = Vector2Helpers.Cross( rA, normal );
					    float rnB = Vector2Helpers.Cross( rB, normal );
					    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					    constraint.normalMass2.AsSpan[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					    float rtA = Vector2Helpers.Cross( rA, tangent );
					    float rtB = Vector2Helpers.Cross( rB, tangent );
					    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					    constraint.tangentMass2.AsSpan[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					    // relative velocity for restitution
					    var vrA = vA + Vector2Helpers.Cross( wA, rA );
					    var vrB = vB + Vector2Helpers.Cross( wB, rB );
					    constraint.relativeVelocity2.AsSpan[j] = Vector2.Dot(normal, vrB - vrA);
				    }
				    else
				    {
					    // dummy data that has no effect
					    constraint.baseSeparation2.AsSpan[j] = 0.0f;
					    constraint.normalImpulse2.AsSpan[j] = 0.0f;
					    constraint.tangentImpulse2.AsSpan[j] = 0.0f;
					    constraint.totalNormalImpulse2.AsSpan[j] = 0.0f;
					    constraint.anchorA2.X.AsSpan[j] = 0.0f;
					    constraint.anchorA2.Y.AsSpan[j] = 0.0f;
					    constraint.anchorB2.X.AsSpan[j] = 0.0f;
					    constraint.anchorB2.Y.AsSpan[j] = 0.0f;
					    constraint.normalMass2.AsSpan[j] = 0.0f;
					    constraint.tangentMass2.AsSpan[j] = 0.0f;
					    constraint.relativeVelocity2.AsSpan[j] = 0.0f;
				    }
			    }
			    else
			    {
				    // SIMD remainder
				    constraint.indexA.AsSpan[j] = PhysicsConstants.NullIndex;
				    constraint.indexB.AsSpan[j] = PhysicsConstants.NullIndex;

				    constraint.invMassA.AsSpan[j] = 0.0f;
				    constraint.invMassB.AsSpan[j] = 0.0f;
				    constraint.invIA.AsSpan[j] = 0.0f;
				    constraint.invIB.AsSpan[j] = 0.0f;

				    constraint.normal.X.AsSpan[j] = 0.0f;
				    constraint.normal.Y.AsSpan[j] = 0.0f;
				    constraint.friction.AsSpan[j] = 0.0f;
				    constraint.tangentSpeed.AsSpan[j] = 0.0f;
				    constraint.rollingResistance.AsSpan[j] = 0.0f;
				    constraint.rollingMass.AsSpan[j] = 0.0f;
				    constraint.rollingImpulse.AsSpan[j] = 0.0f;
				    constraint.biasRate.AsSpan[j] = 0.0f;
				    constraint.massScale.AsSpan[j] = 0.0f;
				    constraint.impulseScale.AsSpan[j] = 0.0f;

				    constraint.anchorA1.X.AsSpan[j] = 0.0f;
				    constraint.anchorA1.Y.AsSpan[j] = 0.0f;
				    constraint.anchorB1.X.AsSpan[j] = 0.0f;
				    constraint.anchorB1.Y.AsSpan[j] = 0.0f;
				    constraint.baseSeparation1.AsSpan[j] = 0.0f;
				    constraint.normalImpulse1.AsSpan[j] = 0.0f;
				    constraint.tangentImpulse1.AsSpan[j] = 0.0f;
				    constraint.totalNormalImpulse1.AsSpan[j] = 0.0f;
				    constraint.normalMass1.AsSpan[j] = 0.0f;
				    constraint.tangentMass1.AsSpan[j] = 0.0f;

				    constraint.anchorA2.X.AsSpan[j] = 0.0f;
				    constraint.anchorA2.Y.AsSpan[j] = 0.0f;
				    constraint.anchorB2.X.AsSpan[j] = 0.0f;
				    constraint.anchorB2.Y.AsSpan[j] = 0.0f;
				    constraint.baseSeparation2.AsSpan[j] = 0.0f;
				    constraint.normalImpulse2.AsSpan[j] = 0.0f;
				    constraint.tangentImpulse2.AsSpan[j] = 0.0f;
				    constraint.totalNormalImpulse2.AsSpan[j] = 0.0f;
				    constraint.normalMass2.AsSpan[j] = 0.0f;
				    constraint.tangentMass2.AsSpan[j] = 0.0f;

				    constraint.restitution.AsSpan[j] = 0.0f;
				    constraint.relativeVelocity1.AsSpan[j] = 0.0f;
				    constraint.relativeVelocity2.AsSpan[j] = 0.0f;
			    }
		    }
	    }
    }

    // Integrate velocities and apply damping
    private void IntegrateVelocitiesTask( int startIndex, int endIndex)
    {
	    ref var states = ref _contextBodyStates;
	    ref var sims = ref _contextSims;

        // TODO:
	    var gravity = Vector2.Zero;
	    float h = _h;
	    float maxLinearSpeed = _maxLinearVelocity;
	    float maxAngularSpeed = PhysicsConstants.MaxRotation * _invDt;
	    float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	    float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    ref var sim = ref sims[i];
		    ref var state = ref states[i];

		    var v = state.linearVelocity;
		    float w = state.angularVelocity;

		    // Apply forces, torque, gravity, and damping
		    // Apply damping.
		    // Differential equation: dv/dt + c * v = 0
		    // Solution: v(t) = v0 * exp(-c * t)
		    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		    // v2 = exp(-c * dt) * v1
		    // Pade approximation:
		    // v2 = v1 * 1 / (1 + c * dt)
		    float linearDamping = 1.0f / ( 1.0f + h * sim.linearDamping );
		    float angularDamping = 1.0f / ( 1.0f + h * sim.angularDamping );

		    // Gravity scale will be zero for kinematic bodies
		    float gravityScale = sim.invMass > 0.0f ? sim.gravityScale : 0.0f;

		    // lvd = h * im * f + h * g
		    var linearVelocityDelta = h * sim.invMass * sim.force + (h * gravityScale) * gravity;
		    float angularVelocityDelta = h * sim.invInertia * sim.torque;

		    v = Vector2Helpers.MulAdd( linearVelocityDelta, linearDamping, v );
		    w = angularVelocityDelta + angularDamping * w;

		    // Clamp to max linear speed
		    if ( Vector2.Dot( v, v ) > maxLinearSpeedSquared )
		    {
			    float ratio = maxLinearSpeed / v.Length();
			    v = ratio * v;
			    sim.flags |= (ushort) BodyFlags.b2_isSpeedCapped;
		    }

		    // Clamp to max angular speed
		    if ( w * w > maxAngularSpeedSquared && ( sim.flags & (ushort) BodyFlags.b2_allowFastRotation ) == 0 )
		    {
			    float ratio = maxAngularSpeed / MathF.Abs( w );
			    w *= ratio;
			    sim.flags |= (ushort) BodyFlags.b2_isSpeedCapped;
		    }

		    if ((state.flags & (ushort) BodyFlags.b2_lockLinearX) != 0x0)
		    {
			    v.X = 0.0f;
		    }

		    if ((state.flags & (ushort) BodyFlags.b2_lockLinearY) != 0x0)
		    {
			    v.Y = 0.0f;
		    }

		    if ((state.flags & (ushort) BodyFlags.b2_lockAngularZ) != 0x0)
		    {
			    w = 0.0f;
		    }

		    state.linearVelocity = v;
		    state.angularVelocity = w;
	    }
    }

    private void WarmStartContactsTask( int startIndex, int endIndex, int colorIndex )
    {
        var color = _constraintGraph.colors[colorIndex];
        var constraints = _contextSimdContactConstraints.Span.Slice(color.SimdConstraintIndex, color.SimdConstraintCount);

        var stateSpan = _contextBodyStates.Span;

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    ref var c = ref constraints[i];
		    var bA = GatherBodies(stateSpan, c.indexA.AsSpan);
		    var bB = GatherBodies(stateSpan, c.indexB.AsSpan);

		    var tangentX = c.normal.Y;

            var tangentY = SimdSub(b2ContactConstraintSIMD.Zero.AsSpan, c.normal.X.AsSpan);

		    {
			    // fixed anchors
			    var rA = c.anchorA1;
			    var rB = c.anchorB1;

			    Vector2Wide P;

			    P.X = SimdAdd( SimdMul( c.normalImpulse1.AsSpan, c.normal.X.AsSpan ).AsSpan, SimdMul( c.tangentImpulse1.AsSpan, tangentX.AsSpan ).AsSpan );
			    P.Y = SimdAdd( SimdMul( c.normalImpulse1.AsSpan, c.normal.Y.AsSpan ).AsSpan, SimdMul( c.tangentImpulse1.AsSpan, tangentY.AsSpan ).AsSpan );
			    bA.w = SimdMulSub( bA.w.AsSpan, c.invIA.AsSpan, SimdCross( rA, P ).AsSpan );
			    bA.vX = SimdMulSub( bA.vX.AsSpan, c.invMassA.AsSpan, P.X.AsSpan );
			    bA.vY = SimdMulSub( bA.vY.AsSpan, c.invMassA.AsSpan, P.Y.AsSpan );
			    bB.w = SimdMulAdd( bB.w.AsSpan, c.invIB.AsSpan, SimdCross( rB, P ).AsSpan );
			    bB.vX = SimdMulAdd( bB.vX.AsSpan, c.invMassB.AsSpan, P.X.AsSpan );
			    bB.vY = SimdMulAdd( bB.vY.AsSpan, c.invMassB.AsSpan, P.Y.AsSpan );
		    }

		    {
			    // fixed anchors
			    var rA = c.anchorA2;
                var rB = c.anchorB2;

			    Vector2Wide P;
			    P.X = SimdAdd( SimdMul( c.normalImpulse2.AsSpan, c.normal.X.AsSpan ).AsSpan, SimdMul( c.tangentImpulse2.AsSpan, tangentX.AsSpan ).AsSpan );
			    P.Y = SimdAdd( SimdMul( c.normalImpulse2.AsSpan, c.normal.Y.AsSpan ).AsSpan, SimdMul( c.tangentImpulse2.AsSpan, tangentY.AsSpan ).AsSpan );
			    bA.w = SimdMulSub( bA.w.AsSpan, c.invIA.AsSpan, SimdCross( rA, P ).AsSpan );
			    bA.vX = SimdMulSub( bA.vX.AsSpan, c.invMassA.AsSpan, P.X.AsSpan );
			    bA.vY = SimdMulSub( bA.vY.AsSpan, c.invMassA.AsSpan, P.Y.AsSpan );
			    bB.w = SimdMulAdd( bB.w.AsSpan, c.invIB.AsSpan, SimdCross( rB, P ).AsSpan );
			    bB.vX = SimdMulAdd( bB.vX.AsSpan, c.invMassB.AsSpan, P.X.AsSpan );
			    bB.vY = SimdMulAdd( bB.vY.AsSpan, c.invMassB.AsSpan, P.Y.AsSpan );
		    }

		    bA.w = SimdMulSub( bA.w.AsSpan, c.invIA.AsSpan, c.rollingImpulse.AsSpan );
		    bB.w = SimdMulAdd( bB.w.AsSpan, c.invIB.AsSpan, c.rollingImpulse.AsSpan );

		    ScatterBodies( _contextBodyStates.Span, c.indexA.AsSpan, ref bA );
		    ScatterBodies( _contextBodyStates.Span, c.indexB.AsSpan, ref bB );
	    }
    }

    private void SolveContactsTask( int startIndex, int endIndex, int colorIndex, bool useBias )
    {
	    ref var states = ref _contextBodyStates;
        var constraints = _contextSimdContactConstraints.Span.Slice(_constraintGraph.colors[colorIndex].SimdConstraintIndex, _constraintGraph.colors[colorIndex].SimdConstraintCount);

	    var inv_h = SimdSplat( _invH );
	    var contactSpeed = SimdSplat( -_contactSpeed );
	    var oneW = SimdSplat( 1.0f );
        var zero = new FixedArray8<float>();

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    ref var c = ref constraints[i];

		    var bA = GatherBodies(states.Span, c.indexA.AsSpan );
            var bB = GatherBodies(states.Span, c.indexA.AsSpan );

		    FixedArray8<float> biasRate, massScale, impulseScale;
		    if ( useBias )
		    {
			    biasRate = SimdMul( c.massScale.AsSpan, c.biasRate.AsSpan );
			    massScale = c.massScale;
			    impulseScale = c.impulseScale;
		    }
		    else
		    {
			    biasRate = new FixedArray8<float>();
			    massScale = oneW;
			    impulseScale = new FixedArray8<float>();
		    }

		    var totalNormalImpulse = new FixedArray8<float>();

		    var dp = new Vector2Wide()
            {
                X = SimdSub( bB.dpX.AsSpan, bA.dpX.AsSpan ),
                Y = SimdSub( bB.dpY.AsSpan, bA.dpY.AsSpan ),
            };

		    // point1 non-penetration constraint
		    {
			    // Fixed anchors for impulses
			    var rA = c.anchorA1;
                var rB = c.anchorB1;

			    // Moving anchors for current separation
                var rsA = SimdRotateVector( bA.dqC.AsSpan, bA.dqS.AsSpan, rA.X.AsSpan, rA.Y.AsSpan );
                var rsB = SimdRotateVector( bB.dqC.AsSpan, bB.dqS.AsSpan, rB.X.AsSpan, rB.Y.AsSpan );

			    // compute current separation
			    // this is subject to round-off error if the anchor is far from the body center of mass
			    var ds = new Vector2Wide(){ X = SimdAdd( dp.X.AsSpan, SimdSub( rsB.X.AsSpan, rsA.X.AsSpan ).AsSpan ), Y = SimdAdd( dp.Y.AsSpan, SimdSub( rsB.Y.AsSpan, rsA.Y.AsSpan ).AsSpan ) };
			    var s = SimdAdd( SimdDot( c.normal, ds ).AsSpan, c.baseSeparation1.AsSpan );

			    // Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
			    // The contactSpeed is meant to limit stiffness, not increase it.
			    b2FloatW mask = SimdGreaterThan( s, zero );
			    b2FloatW specBias = SimdMul( s, inv_h );
			    b2FloatW softBias = SimdMax( SimdMul( biasRate, s ), contactSpeed );

			    // todo try b2MaxW(softBias, specBias);
			    b2FloatW bias = SimdBlend( softBias, specBias, mask );

			    b2FloatW pointMassScale = SimdBlend( massScale, oneW, mask );
			    b2FloatW pointImpulseScale = SimdBlend( impulseScale, zero, mask );

			    // Relative velocity at contact
			    b2FloatW dvx = SimdSub( SimdSub( bB.vX, SimdMul( bB.w, rB.Y ) ), SimdSub( bA.vX, SimdMul( bA.w, rA.Y ) ) );
			    b2FloatW dvy = SimdSub( SimdAdd( bB.vY, SimdMul( bB.w, rB.X ) ), SimdAdd( bA.vY, SimdMul( bA.w, rA.X ) ) );
			    b2FloatW vn = SimdAdd( SimdMul( dvx, c.normal.X ), SimdMul( dvy, c.normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = SimdAdd( SimdMul( c.normalMass1, SimdAdd( SimdMul( pointMassScale, vn ), bias ) ),
										      SimdMul( pointImpulseScale, c.normalImpulse1 ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = SimdMax( SimdSub( c.normalImpulse1, negImpulse ), zero );
			    b2FloatW impulse = SimdSub( newImpulse, c.normalImpulse1 );
			    c.normalImpulse1 = newImpulse;
			    c.totalNormalImpulse1 = SimdAdd( c.totalNormalImpulse1, newImpulse );

			    totalNormalImpulse = SimdAdd( totalNormalImpulse, newImpulse );

			    // Apply contact impulse
			    b2FloatW Px = SimdMul( impulse, c.normal.X );
			    b2FloatW Py = SimdMul( impulse, c.normal.Y );

			    bA.vX = SimdMulSub( bA.vX, c.invMassA, Px );
			    bA.vY = SimdMulSub( bA.vY, c.invMassA, Py );
			    bA.w = SimdMulSub( bA.w, c.invIA, SimdSub( SimdMul( rA.X, Py ), SimdMul( rA.Y, Px ) ) );

			    bB.vX = SimdMulAdd( bB.vX, c.invMassB, Px );
			    bB.vY = SimdMulAdd( bB.vY, c.invMassB, Py );
			    bB.w = SimdMulAdd( bB.w, c.invIB, SimdSub( SimdMul( rB.X, Py ), SimdMul( rB.Y, Px ) ) );
		    }

		    // second point non-penetration constraint
		    {
			    // moving anchors for current separation
			    b2Vec2W rsA = SimdRotateVector( bA.dqC, bA.dqS, c.anchorA2 );
			    b2Vec2W rsB = SimdRotateVector( bB.dqC, bB.dqS, c.anchorB2 );

			    // compute current separation
			    b2Vec2W ds = { SimdAdd( dp.X, SimdSub( rsB.X, rsA.X ) ), SimdAdd( dp.Y, SimdSub( rsB.Y, rsA.Y ) ) };
			    b2FloatW s = SimdAdd( SimdDot( c.normal, ds ), c.baseSeparation2 );

			    b2FloatW mask = SimdGreaterThan( s, zero.AsSpan );
			    b2FloatW specBias = SimdMul( s, inv_h );
			    b2FloatW softBias = SimdMax( SimdMul( biasRate, s ), contactSpeed );
			    b2FloatW bias = SimdBlend( softBias, specBias, mask );

			    b2FloatW pointMassScale = SimdBlend( massScale, oneW, mask );
			    b2FloatW pointImpulseScale = SimdBlend( impulseScale, b2ZeroW(), mask );

			    // fixed anchors for Jacobians
			    b2Vec2W rA = c.anchorA2;
			    b2Vec2W rB = c.anchorB2;

			    // Relative velocity at contact
			    b2FloatW dvx = SimdSub( SimdSub( bB.vX, SimdMul( bB.w, rB.Y ) ), SimdSub( bA.vX, SimdMul( bA.w, rA.Y ) ) );
			    b2FloatW dvy = SimdSub( SimdAdd( bB.vY, SimdMul( bB.w, rB.X ) ), SimdAdd( bA.vY, SimdMul( bA.w, rA.X ) ) );
			    b2FloatW vn = SimdAdd( SimdMul( dvx, c.normal.X ), SimdMul( dvy, c.normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = SimdAdd( SimdMul( c.normalMass2, SimdAdd( SimdMul( pointMassScale, vn ), bias ) ),
										      SimdMul( pointImpulseScale, c.normalImpulse2 ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = b2MaxW( SimdSub( c.normalImpulse2, negImpulse ), b2ZeroW() );
			    b2FloatW impulse = SimdSub( newImpulse, c.normalImpulse2 );
			    c.normalImpulse2 = newImpulse;
			    c.totalNormalImpulse2 = SimdAdd( c.totalNormalImpulse2, newImpulse );

			    totalNormalImpulse = SimdAdd( totalNormalImpulse, newImpulse );

			    // Apply contact impulse
			    b2FloatW Px = SimdMul( impulse, c.normal.X );
			    b2FloatW Py = SimdMul( impulse, c.normal.Y );

			    bA.v.X = SimdMulSub( bA.v.X, c.invMassA, Px );
			    bA.v.Y = SimdMulSub( bA.v.Y, c.invMassA, Py );
			    bA.w = SimdMulSub( bA.w, c.invIA, SimdSub( SimdMul( rA.X, Py ), SimdMul( rA.Y, Px ) ) );

			    bB.v.X = SimdMulAdd( bB.v.X, c.invMassB, Px );
			    bB.v.Y = SimdMulAdd( bB.v.Y, c.invMassB, Py );
			    bB.w = SimdMulAdd( bB.w, c.invIB, SimdSub( SimdMul( rB.X, Py ), SimdMul( rB.Y, Px ) ) );
		    }

		    b2FloatW tangentX = c.normal.Y;
		    b2FloatW tangentY = SimdSub( b2ZeroW(), c.normal.X );

		    // point 1 friction constraint
		    {
			    // fixed anchors for Jacobians
			    b2Vec2W rA = c.anchorA1;
			    b2Vec2W rB = c.anchorB1;

			    // Relative velocity at contact
			    b2FloatW dvx = SimdSub( SimdSub( bB.v.X, SimdMul( bB.w, rB.Y ) ), SimdSub( bA.v.X, SimdMul( bA.w, rA.Y ) ) );
			    b2FloatW dvy = SimdSub( SimdAdd( bB.v.Y, SimdMul( bB.w, rB.X ) ), SimdAdd( bA.v.Y, SimdMul( bA.w, rA.X ) ) );
			    b2FloatW vt = SimdAdd( SimdMul( dvx, tangentX ), SimdMul( dvy, tangentY ) );

			    // Tangent speed (conveyor belt)
			    vt = SimdSub( vt, c.tangentSpeed );

			    // Compute tangent force
			    b2FloatW negImpulse = SimdMul( c.tangentMass1, vt );

			    // Clamp the accumulated force
			    b2FloatW maxFriction = SimdMul( c.friction, c.normalImpulse1 );
			    b2FloatW newImpulse = SimdSub( c.tangentImpulse1, negImpulse );
			    newImpulse = b2MaxW( SimdSub( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			    b2FloatW impulse = SimdSub( newImpulse, c.tangentImpulse1 );
			    c.tangentImpulse1 = newImpulse;

			    // Apply contact impulse
			    b2FloatW Px = SimdMul( impulse, tangentX );
			    b2FloatW Py = SimdMul( impulse, tangentY );

			    bA.vX = SimdMulSub( bA.vX, c.invMassA, Px );
			    bA.vY = SimdMulSub( bA.vY, c.invMassA, Py );
			    bA.w = SimdMulSub( bA.w, c.invIA, SimdSub( SimdMul( rA.X, Py ), SimdMul( rA.Y, Px ) ) );

			    bB.vX = SimdMulAdd( bB.vX, c.invMassB, Px );
			    bB.vY = SimdMulAdd( bB.vY, c.invMassB, Py );
			    bB.w = SimdMulAdd( bB.w, c.invIB, SimdSub( SimdMul( rB.X, Py ), SimdMul( rB.Y, Px ) ) );
		    }

		    // second point friction constraint
		    {
			    // fixed anchors for Jacobians
			    b2Vec2W rA = c.anchorA2;
			    b2Vec2W rB = c.anchorB2;

			    // Relative velocity at contact
			    b2FloatW dvx = SimdSub( SimdSub( bB.v.X, SimdMul( bB.w, rB.Y ) ), SimdSub( bA.v.X, SimdMul( bA.w, rA.Y ) ) );
			    b2FloatW dvy = SimdSub( SimdAdd( bB.v.Y, SimdMul( bB.w, rB.X ) ), SimdAdd( bA.v.Y, SimdMul( bA.w, rA.X ) ) );
			    b2FloatW vt = SimdAdd( SimdMul( dvx, tangentX ), SimdMul( dvy, tangentY ) );

			    // Tangent speed (conveyor belt)
			    vt = SimdSub( vt, c.tangentSpeed );

			    // Compute tangent force
			    b2FloatW negImpulse = SimdMul( c.tangentMass2, vt );

			    // Clamp the accumulated force
			    b2FloatW maxFriction = SimdMul( c.friction, c.normalImpulse2 );
			    b2FloatW newImpulse = SimdSub( c.tangentImpulse2, negImpulse );
			    newImpulse = b2MaxW( SimdSub( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			    var impulse = SimdSub( newImpulse, c.tangentImpulse2.AsSpan );
			    c.tangentImpulse2 = newImpulse;

			    // Apply contact impulse
			    var Px = SimdMul( impulse, tangentX );
                var Py = SimdMul( impulse, tangentY );

			    bA.vX = SimdMulSub( bA.vX, c.invMassA, Px );
			    bA.vY = SimdMulSub( bA.vY, c.invMassA, Py );
			    bA.w = SimdMulSub( bA.w, c.invIA, SimdSub( SimdMul( rA.X, Py ), SimdMul( rA.Y, Px ) ) );

			    bB.vX = SimdMulAdd( bB.vX, c.invMassB, Px );
			    bB.vY = SimdMulAdd( bB.vY, c.invMassB, Py );
			    bB.w = SimdMulAdd( bB.w, c.invIB, SimdSub( SimdMul( rB.X, Py ), SimdMul( rB.Y, Px ) ) );
		    }

		    // Rolling resistance
		    {
			    b2FloatW deltaLambda = SimdMul( c.rollingMass, SimdSub( bA.w, bB.w ) );
			    b2FloatW lambda = c.rollingImpulse;
			    b2FloatW maxLambda = SimdMul( c.rollingResistance, totalNormalImpulse );
			    c.rollingImpulse = b2SymClampW( SimdAdd( lambda, deltaLambda ), maxLambda );
			    deltaLambda = SimdSub( c.rollingImpulse, lambda );

			    bA.w = SimdMulSub( bA.w, c.invIA, deltaLambda );
			    bB.w = SimdMulAdd( bB.w, c.invIB, deltaLambda );
		    }

		    ScatterBodies(states.Span, c.indexA.AsSpan, ref bA );
		    ScatterBodies(states.Span, c.indexB.AsSpan, ref bB );
	    }

	    b2TracyCZoneEnd( solve_contact );
    }

    private void ApplyRestitutionTask( int startIndex, int endIndex, int colorIndex )
    {
	    ref var states = ref _contextBodyStates;
        var constraints = _contextSimdContactConstraints.Span.Slice(
            _constraintGraph.colors[colorIndex].SimdConstraintIndex,
            _constraintGraph.colors[colorIndex].SimdConstraintCount);
	    var threshold = SimdSplat(_restitutionThreshold);
	    var zero = new FixedArray8<float>();

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    ref var c = ref constraints[i];

		    if ( b2AllZeroW( c.restitution ) )
		    {
			    // No lanes have restitution. Common case.
			    continue;
		    }

		    // Create a mask based on restitution so that lanes with no restitution are not affected
		    // by the calculations below.
		    b2FloatW restitutionMask = b2EqualsW( c.restitution, zero );

		    b2BodyStateW bA = b2GatherBodies( states, c.indexA );
		    b2BodyStateW bB = b2GatherBodies( states, c.indexB );

		    // first point non-penetration constraint
		    {
			    // Set effective mass to zero if restitution should not be applied
			    b2FloatW mask1 = b2GreaterThanW( SimdAdd( c.relativeVelocity1, threshold ), zero );
			    b2FloatW mask2 = b2EqualsW( c.totalNormalImpulse1, zero );
			    b2FloatW mask = b2OrW( b2OrW( mask1, mask2 ), restitutionMask );
			    b2FloatW mass = SimdBlend( c.normalMass1, zero, mask );

			    // fixed anchors for Jacobians
			    b2Vec2W rA = c.anchorA1;
			    b2Vec2W rB = c.anchorB1;

			    // Relative velocity at contact
			    b2FloatW dvx = SimdSub( SimdSub( bB.v.X, SimdMul( bB.w, rB.Y ) ), SimdSub( bA.v.X, SimdMul( bA.w, rA.Y ) ) );
			    b2FloatW dvy = SimdSub( SimdAdd( bB.v.Y, SimdMul( bB.w, rB.X ) ), SimdAdd( bA.v.Y, SimdMul( bA.w, rA.X ) ) );
			    b2FloatW vn = SimdAdd( SimdMul( dvx, c.normal.X ), SimdMul( dvy, c.normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = SimdMul( mass, SimdAdd( vn, SimdMul( c.restitution, c.relativeVelocity1 ) ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = b2MaxW( SimdSub( c.normalImpulse1, negImpulse ), b2ZeroW() );
			    b2FloatW deltaImpulse = SimdSub( newImpulse, c.normalImpulse1 );
			    c.normalImpulse1 = newImpulse;

			    // Add the incremental impulse rather than the full impulse because this is not a sub-step
			    c.totalNormalImpulse1 = SimdAdd( c.totalNormalImpulse1, deltaImpulse );

			    // Apply contact impulse
			    b2FloatW Px = SimdMul( deltaImpulse, c.normal.X );
			    b2FloatW Py = SimdMul( deltaImpulse, c.normal.Y );

			    bA.v.X = SimdMulSub( bA.v.X, c.invMassA, Px );
			    bA.v.Y = SimdMulSub( bA.v.Y, c.invMassA, Py );
			    bA.w = SimdMulSub( bA.w, c.invIA, SimdSub( SimdMul( rA.X, Py ), SimdMul( rA.Y, Px ) ) );

			    bB.v.X = SimdMulAdd( bB.v.X, c.invMassB, Px );
			    bB.v.Y = SimdMulAdd( bB.v.Y, c.invMassB, Py );
			    bB.w = SimdMulAdd( bB.w, c.invIB, SimdSub( SimdMul( rB.X, Py ), SimdMul( rB.Y, Px ) ) );
		    }

		    // second point non-penetration constraint
		    {
			    // Set effective mass to zero if restitution should not be applied
                var mask1 = SimdGreaterThan( SimdAdd( c.relativeVelocity2, threshold ), zero );
                var mask2 = SimdEquals( c.totalNormalImpulse2, zero );
                var mask = SimdOr( SimdOr( mask1, mask2 ), restitutionMask );
			    var mass = SimdBlend( c.normalMass2, zero, mask );

			    // fixed anchors for Jacobians
			    b2Vec2W rA = c.anchorA2;
			    b2Vec2W rB = c.anchorB2;

			    // Relative velocity at contact
                var dvx = SimdSub( SimdSub( bB.v.X, SimdMul( bB.w, rB.Y ) ), SimdSub( bA.v.X, SimdMul( bA.w, rA.Y ) ) );
                var dvy = SimdSub( SimdAdd( bB.v.Y, SimdMul( bB.w, rB.X ) ), SimdAdd( bA.v.Y, SimdMul( bA.w, rA.X ) ) );
                var vn = SimdAdd( SimdMul( dvx, c.normal.X ), SimdMul( dvy, c.normal.Y ) );

			    // Compute normal impulse
                var negImpulse = SimdMul( mass, SimdAdd( vn, SimdMul( c.restitution, c.relativeVelocity2 ) ) );

			    // Clamp the accumulated impulse
			    var newImpulse = SimdMax( SimdSub( c.normalImpulse2, negImpulse ), zero.AsSpan );
                var deltaImpulse = SimdSub( newImpulse, c.normalImpulse2 );
			    c.normalImpulse2 = newImpulse;

			    // Add the incremental impulse rather than the full impulse because this is not a sub-step
			    c.totalNormalImpulse2 = SimdAdd( c.totalNormalImpulse2, deltaImpulse );

			    // Apply contact impulse
                var Px = SimdMul( deltaImpulse, c.normal.X );
                var Py = SimdMul( deltaImpulse, c.normal.Y );

			    bA.v.X = SimdMulSub( bA.v.X, c.invMassA, Px );
			    bA.v.Y = SimdMulSub( bA.v.Y, c.invMassA, Py );
			    bA.w = SimdMulSub( bA.w, c.invIA, SimdSub( SimdMul( rA.X, Py ), SimdMul( rA.Y, Px ) ) );

			    bB.v.X = SimdMulAdd( bB.v.X, c.invMassB, Px );
			    bB.v.Y = SimdMulAdd( bB.v.Y, c.invMassB, Py );
			    bB.w = SimdMulAdd( bB.w, c.invIB, SimdSub( SimdMul( rB.X, Py ), SimdMul( rB.Y, Px ) ) );
		    }

		    ScatterBodies( states.Span, c.indexA.AsSpan, bA );
		    ScatterBodies( states.Span, c.indexB.AsSpan, bB );
	    }
    }

    private void IntegratePositionsTask( int startIndex, int endIndex)
    {
        ref var states = ref _contextBodyStates;
        float h = _h;

        DebugTools.Assert( startIndex <= endIndex );

        for ( int i = startIndex; i < endIndex; ++i )
        {
            ref var state = ref states[i];

            if ((state.flags & (ushort) BodyFlags.b2_lockLinearX) != 0x0)
            {
                state.linearVelocity.X = 0.0f;
            }

            if ((state.flags & (ushort) BodyFlags.b2_lockLinearY) != 0x0)
            {
                state.linearVelocity.Y = 0.0f;
            }

            if ((state.flags & (ushort) BodyFlags.b2_lockAngularZ) != 0x0)
            {
                state.angularVelocity = 0.0f;
            }

            state.deltaPosition = Vector2Helpers.MulAdd( state.deltaPosition, h, state.linearVelocity );
            state.deltaRotation = Quaternion2D.IntegrateRotation( state.deltaRotation, h * state.angularVelocity );
        }
    }

    private void StoreImpulsesTask( int startIndex, int endIndex )
    {
	    ref var contacts = ref _contextContacts;
	    ref var constraints = ref _contextSimdContactConstraints;

        b2Manifold dummy = new();

	    for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	    {
		    ref var c = ref constraints[constraintIndex];

		    var rollingImpulse = c.rollingImpulse.AsSpan;
		    var normalImpulse1 = c.normalImpulse1.AsSpan;
		    var normalImpulse2 = c.normalImpulse2.AsSpan;
		    var tangentImpulse1 = c.tangentImpulse1.AsSpan;
		    var tangentImpulse2 = c.tangentImpulse2.AsSpan;
		    var totalNormalImpulse1 = c.totalNormalImpulse1.AsSpan;
		    var totalNormalImpulse2 = c.totalNormalImpulse2.AsSpan;
		    var normalVelocity1 = c.relativeVelocity1.AsSpan;
		    var normalVelocity2 = c.relativeVelocity2.AsSpan;

		    int baseIndex = _simdWidth * constraintIndex;

		    for ( int laneIndex = 0; laneIndex < _simdWidth; ++laneIndex )
		    {
			    var m = contacts[baseIndex + laneIndex] == null ? dummy : contacts[baseIndex + laneIndex]!.manifold;
			    m.rollingImpulse = rollingImpulse[laneIndex];

			    m.points._00.normalImpulse = normalImpulse1[laneIndex];
			    m.points._00.tangentImpulse = tangentImpulse1[laneIndex];
			    m.points._00.totalNormalImpulse = totalNormalImpulse1[laneIndex];
			    m.points._00.normalVelocity = normalVelocity1[laneIndex];

			    m.points._01.normalImpulse = normalImpulse2[laneIndex];
			    m.points._01.tangentImpulse = tangentImpulse2[laneIndex];
			    m.points._01.totalNormalImpulse = totalNormalImpulse2[laneIndex];
			    m.points._01.normalVelocity = normalVelocity2[laneIndex];
                contacts[baseIndex + laneIndex]!.manifold = m;
            }
	    }
    }

    #region Overflow

    // contact separation for sub-stepping
    // s = s0 + dot(cB + rB - cA - rA, normal)
    // normal is held constant
    // body positions c can translation and anchors r can rotate
    // s(t) = s0 + dot(cB(t) + rB(t) - cA(t) - rA(t), normal)
    // s(t) = s0 + dot(cB0 + dpB + rot(dqB, rB0) - cA0 - dpA - rot(dqA, rA0), normal)
    // s(t) = s0 + dot(cB0 - cA0, normal) + dot(dpB - dpA + rot(dqB, rB0) - rot(dqA, rA0), normal)
    // s_base = s0 + dot(cB0 - cA0, normal)

    private void PrepareOverflowContacts()
    {
	    var graph = _constraintGraph;
	    var color = graph.colors[PhysicsConstants.OverflowIndex];
	    ref var constraints = ref color.OverflowConstraints;
	    int contactCount = color.ContactSims.Count;
	    ref var contacts = ref color.ContactSims;
	    ref var awakeStates = ref _contextBodyStates;

    #if B2_VALIDATE
	    b2Body* bodies = world.bodies.data;
    #endif

	    // Stiffer for static contacts to avoid bodies getting pushed through the ground
	    var contactSoftness = _contactSoftness;
	    var staticSoftness = _staticSoftness;

	    float warmStartScale = _enableWarmStarting ? 1.0f : 0.0f;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    ref var contactSim = ref contacts[i];

		    ref var manifold = ref contactSim.manifold;
		    int pointCount = manifold.pointCount;

		    DebugTools.Assert( 0 < pointCount && pointCount <= 2 );

		    int indexA = contactSim.bodySimIndexA;
		    int indexB = contactSim.bodySimIndexB;

    #if B2_VALIDATE
		    b2Body* bodyA = bodies + contactSim.bodyIdA;
		    int validIndexA = bodyA.setIndex == (int) SetType.AwakeSet ? bodyA.localIndex : PhysicsConstants.NullIndex;
		    DebugTools.Assert( indexA == validIndexA );

		    b2Body* bodyB = bodies + contactSim.bodyIdB;
		    int validIndexB = bodyB.setIndex == (int) SetType.AwakeSet ? bodyB.localIndex : PhysicsConstants.NullIndex;
		    DebugTools.Assert( indexB == validIndexB );
    #endif

		    ref var constraint = ref constraints[i];
		    constraint.indexA = indexA;
		    constraint.indexB = indexB;
		    constraint.normal = manifold.normal;
		    constraint.friction = contactSim.friction;
		    constraint.restitution = contactSim.restitution;
		    constraint.rollingResistance = contactSim.rollingResistance;
		    constraint.rollingImpulse = warmStartScale * manifold.rollingImpulse;
		    constraint.tangentSpeed = contactSim.tangentSpeed;
		    constraint.pointCount = pointCount;

		    var vA = Vector2.Zero;
		    float wA = 0.0f;
		    float mA = contactSim.invMassA;
		    float iA = contactSim.invIA;
		    if ( indexA != PhysicsConstants.NullIndex )
		    {
			    ref var stateA = ref awakeStates[indexA];
			    vA = stateA.linearVelocity;
			    wA = stateA.angularVelocity;
		    }

            var vB = Vector2.Zero;
		    float wB = 0.0f;
		    float mB = contactSim.invMassB;
		    float iB = contactSim.invIB;
		    if ( indexB != PhysicsConstants.NullIndex )
		    {
			    ref var stateB = ref awakeStates[indexB];
			    vB = stateB.linearVelocity;
			    wB = stateB.angularVelocity;
		    }

		    if ( indexA == PhysicsConstants.NullIndex || indexB == PhysicsConstants.NullIndex )
		    {
			    constraint.softness = staticSoftness;
		    }
		    else
		    {
			    constraint.softness = contactSoftness;
		    }

		    // copy mass into constraint to avoid cache misses during sub-stepping
		    constraint.invMassA = mA;
		    constraint.invIA = iA;
		    constraint.invMassB = mB;
		    constraint.invIB = iB;

		    {
			    float k = iA + iB;
			    constraint.rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
		    }

            var normal = constraint.normal;
            var tangent = constraint.normal.RightPerp();

		    for ( int j = 0; j < pointCount; ++j )
		    {
			    ref var mp = ref manifold.points.AsSpan[j];
			    ref var cp = ref constraint.points.AsSpan[j];

			    cp.normalImpulse = warmStartScale * mp.normalImpulse;
			    cp.tangentImpulse = warmStartScale * mp.tangentImpulse;
			    cp.totalNormalImpulse = 0.0f;

                var rA = mp.anchorA;
                var rB = mp.anchorB;

			    cp.anchorA = rA;
			    cp.anchorB = rB;
			    cp.baseSeparation = mp.separation - Vector2.Dot( rB - rA, normal );

			    float rnA = Vector2Helpers.Cross( rA, normal );
			    float rnB = Vector2Helpers.Cross( rB, normal );
			    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			    cp.normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			    float rtA = Vector2Helpers.Cross( rA, tangent );
			    float rtB = Vector2Helpers.Cross( rB, tangent );
			    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			    cp.tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			    // Save relative velocity for restitution
                var vrA = vA + Vector2Helpers.Cross( wA, rA );
                var vrB = vB + Vector2Helpers.Cross( wB, rB );
			    cp.relativeVelocity = Vector2.Dot(normal, vrB - vrA);
		    }
	    }
    }

    private void WarmStartOverflowContacts()
    {
	    ref var graph = ref _constraintGraph;
	    ref var color = ref graph.colors[PhysicsConstants.OverflowIndex];
	    ref var constraints = ref color.OverflowConstraints;
	    int contactCount = color.ContactSims.Count;
	    ref var awakeSet = ref _solverSets[(int) SetType.AwakeSet];
	    ref var states = ref awakeSet.bodyStates;

	    // This is a dummy state to represent a static body because static bodies don't have a solver body.
	    var dummyState = BodyState.Identity;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    ref var constraint = ref constraints[i];

		    int indexA = constraint.indexA;
		    int indexB = constraint.indexB;

		    var stateA = indexA == PhysicsConstants.NullIndex ? dummyState : states[indexA];
            var stateB = indexB == PhysicsConstants.NullIndex ? dummyState : states[indexB];

            var vA = stateA.linearVelocity;
		    float wA = stateA.angularVelocity;
            var vB = stateB.linearVelocity;
		    float wB = stateB.angularVelocity;

		    float mA = constraint.invMassA;
		    float iA = constraint.invIA;
		    float mB = constraint.invMassB;
		    float iB = constraint.invIB;

		    // Stiffer for static contacts to avoid bodies getting pushed through the ground
            var normal = constraint.normal;
            var tangent = constraint.normal.RightPerp();
		    int pointCount = constraint.pointCount;

		    for ( int j = 0; j < pointCount; ++j )
		    {
			    ref var cp = ref constraint.points.AsSpan[j];

			    // fixed anchors
			    var rA = cp.anchorA;
                var rB = cp.anchorB;

                var P = cp.normalImpulse * normal + cp.tangentImpulse * tangent;
			    wA -= iA * Vector2Helpers.Cross( rA, P );
			    vA = Vector2Helpers.MulAdd( vA, -mA, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    }

		    wA -= iA * constraint.rollingImpulse;
		    wB += iB * constraint.rollingImpulse;

		    if ( (stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
		    {
			    stateA.linearVelocity = vA;
			    stateA.angularVelocity = wA;
                states[indexA] = stateA;
		    }

		    if ( (stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
		    {
			    stateB.linearVelocity = vB;
			    stateB.angularVelocity = wB;
                states[indexB] = stateB;
		    }
	    }
    }

    private void SolveOverflowContacts( bool useBias )
    {
	    ref var graph = ref _constraintGraph;
	    ref var color = ref graph.colors[PhysicsConstants.OverflowIndex];
	    ref var constraints = ref color.OverflowConstraints;
	    int contactCount = color.ContactSims.Count;
	    ref var awakeSet = ref _solverSets[(int) SetType.AwakeSet];
	    ref var states = ref awakeSet.bodyStates;

	    float inv_h = _invH;
	    var contactSpeed = _contactSpeed;

	    // This is a dummy body to represent a static body since static bodies don't have a solver body.
	    var dummyState = BodyState.Identity;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    ref var constraint = ref constraints[i];
		    float mA = constraint.invMassA;
		    float iA = constraint.invIA;
		    float mB = constraint.invMassB;
		    float iB = constraint.invIB;

		    var stateA = constraint.indexA == PhysicsConstants.NullIndex ? dummyState : states[constraint.indexA];
            var vA = stateA.linearVelocity;
		    float wA = stateA.angularVelocity;
            var dqA = stateA.deltaRotation;

		    var stateB = constraint.indexB == PhysicsConstants.NullIndex ? dummyState : states[constraint.indexB];
            var vB = stateB.linearVelocity;
		    float wB = stateB.angularVelocity;
            var dqB = stateB.deltaRotation;

            var dp = stateB.deltaPosition - stateA.deltaPosition;

            var normal = constraint.normal;
            var tangent = normal.RightPerp();
		    float friction = constraint.friction;
            var softness = constraint.softness;

		    int pointCount = constraint.pointCount;
		    float totalNormalImpulse = 0.0f;

		    // Non-penetration
		    for ( int j = 0; j < pointCount; ++j )
		    {
			    ref var cp = ref constraint.points.AsSpan[j];

			    // fixed anchor points
			    var rA = cp.anchorA;
			    var rB = cp.anchorB;

			    // compute current separation
			    // this is subject to round-off error if the anchor is far from the body center of mass
                var ds =  dp + Quaternion2D.RotateVector( dqB, rB ) - Quaternion2D.RotateVector( dqA, rA );
			    float s = cp.baseSeparation + Vector2.Dot( ds, normal );

			    float velocityBias = 0.0f;
			    float massScale = 1.0f;
			    float impulseScale = 0.0f;
			    if ( s > 0.0f )
			    {
				    // speculative bias
				    velocityBias = s * inv_h;
			    }
			    else if ( useBias )
			    {
				    velocityBias = MathF.Max( softness.massScale * softness.biasRate * s, -contactSpeed );
				    massScale = softness.massScale;
				    impulseScale = softness.impulseScale;
			    }

			    // relative normal velocity at contact
                var vrA = vA + Vector2Helpers.Cross( wA, rA );
                var vrB = vB + Vector2Helpers.Cross( wB, rB );
			    float vn = Vector2.Dot( vrB - vrA, normal );

			    // incremental normal impulse
			    float impulse = -cp.normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp.normalImpulse;

			    // clamp the accumulated impulse
			    float newImpulse = MathF.Max( cp.normalImpulse + impulse, 0.0f );
			    impulse = newImpulse - cp.normalImpulse;
			    cp.normalImpulse = newImpulse;
			    cp.totalNormalImpulse += newImpulse;

			    totalNormalImpulse += newImpulse;

			    // apply normal impulse
			    var P = impulse * normal;
			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * Vector2Helpers.Cross( rA, P );

			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
		    }

		    // Friction
		    for ( int j = 0; j < pointCount; ++j )
		    {
			    ref var cp = ref constraint.points.AsSpan[j];

			    // fixed anchor points
                var rA = cp.anchorA;
                var rB = cp.anchorB;

			    // relative tangent velocity at contact
			    var vrB = vB + Vector2Helpers.Cross( wB, rB );
                var vrA = vA + Vector2Helpers.Cross( wA, rA );

			    // vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			    //    = dot(vrB - vrA, tangent) - (sA + sB)

			    float vt = Vector2.Dot( vrB - vrA, tangent ) - constraint.tangentSpeed;

			    // incremental tangent impulse
			    float impulse = cp.tangentMass * ( -vt );

			    // clamp the accumulated force
			    float maxFriction = friction * cp.normalImpulse;
			    float newImpulse = Math.Clamp( cp.tangentImpulse + impulse, -maxFriction, maxFriction );
			    impulse = newImpulse - cp.tangentImpulse;
			    cp.tangentImpulse = newImpulse;

			    // apply tangent impulse
                var P = impulse * tangent;
			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * Vector2Helpers.Cross( rA, P );
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
		    }

		    // Rolling resistance
		    {
			    float deltaLambda = -constraint.rollingMass * ( wB - wA );
			    float lambda = constraint.rollingImpulse;
			    float maxLambda = constraint.rollingResistance * totalNormalImpulse;
			    constraint.rollingImpulse = Math.Clamp( lambda + deltaLambda, -maxLambda, maxLambda );
			    deltaLambda = constraint.rollingImpulse - lambda;

			    wA -= iA * deltaLambda;
			    wB += iB * deltaLambda;
		    }

		    if ( (stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
		    {
			    stateA.linearVelocity = vA;
			    stateA.angularVelocity = wA;
                states[constraint.indexA] = stateA;
            }

		    if ( (stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
		    {
			    stateB.linearVelocity = vB;
			    stateB.angularVelocity = wB;
                states[constraint.indexB] = stateB;
            }
	    }
    }

    private void ApplyOverflowRestitution()
    {
	    var graph = _constraintGraph;
	    ref var color = ref graph.colors[PhysicsConstants.OverflowIndex];
	    ref var constraints = ref color.OverflowConstraints;
	    int contactCount = color.ContactSims.Count;
	    ref var awakeSet = ref _solverSets[(int) SetType.AwakeSet];
	    ref var states = ref awakeSet.bodyStates;

	    float threshold = _restitutionThreshold;

	    // dummy state to represent a static body
	    var dummyState = BodyState.Identity;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    ref var constraint = ref constraints[i];

		    float restitution = constraint.restitution;
		    if ( restitution == 0.0f )
		    {
			    continue;
		    }

		    float mA = constraint.invMassA;
		    float iA = constraint.invIA;
		    float mB = constraint.invMassB;
		    float iB = constraint.invIB;

		    var stateA = constraint.indexA == PhysicsConstants.NullIndex ? dummyState : states[constraint.indexA];
            var vA = stateA.linearVelocity;
		    float wA = stateA.angularVelocity;

		    var stateB = constraint.indexB == PhysicsConstants.NullIndex ? dummyState : states[constraint.indexB];
		    var vB = stateB.linearVelocity;
		    float wB = stateB.angularVelocity;

            var normal = constraint.normal;
		    int pointCount = constraint.pointCount;

		    // it is possible to get more accurate restitution by iterating
		    // this only makes a difference if there are two contact points
		    // for (int iter = 0; iter < 10; ++iter)
		    {
			    for ( int j = 0; j < pointCount; ++j )
			    {
				    ref var cp = ref constraint.points.AsSpan[j];

				    // if the normal impulse is zero then there was no collision
				    // this skips speculative contact points that didn't generate an impulse
				    // The max normal impulse is used in case there was a collision that moved away within the sub-step process
				    if ( cp.relativeVelocity > -threshold || cp.totalNormalImpulse == 0.0f )
				    {
					    continue;
				    }

				    // fixed anchor points
                    var rA = cp.anchorA;
                    var rB = cp.anchorB;

				    // relative normal velocity at contact
                    var vrB = vB + Vector2Helpers.Cross( wB, rB );
                    var vrA = vA + Vector2Helpers.Cross( wA, rA );
				    float vn = Vector2.Dot( vrB - vrA, normal );

				    // compute normal impulse
				    float impulse = -cp.normalMass * ( vn + restitution * cp.relativeVelocity );

				    // clamp the accumulated impulse
				    // todo should this be stored?
				    float newImpulse = MathF.Max( cp.normalImpulse + impulse, 0.0f );
				    impulse = newImpulse - cp.normalImpulse;
				    cp.normalImpulse = newImpulse;

				    // Add the incremental impulse rather than the full impulse because this is not a sub-step
				    cp.totalNormalImpulse += impulse;

				    // apply contact impulse
				    var P = impulse * normal;
				    vA = Vector2Helpers.MulSub( vA, mA, P );
				    wA -= iA * Vector2Helpers.Cross( rA, P );
				    vB = Vector2Helpers.MulAdd( vB, mB, P );
				    wB += iB * Vector2Helpers.Cross( rB, P );
			    }
		    }

		    if ( (stateA.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
		    {
			    stateA.linearVelocity = vA;
			    stateA.angularVelocity = wA;
                states[constraint.indexA] = stateA;
		    }

		    if ( (stateB.flags & (ushort) BodyFlags.b2_dynamicFlag) != 0x0 )
		    {
			    stateB.linearVelocity = vB;
			    stateB.angularVelocity = wB;
                states[constraint.indexB] = stateB;
		    }
	    }
    }

    private void StoreOverflowImpulses()
    {
	    var graph = _constraintGraph;
	    ref var color = ref graph.colors[PhysicsConstants.OverflowIndex];
	    ref var constraints = ref color.OverflowConstraints;
	    ref var contacts = ref color.ContactSims;
	    int contactCount = color.ContactSims.Count;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    ref var constraint = ref constraints[i];
		    ref var contact = ref contacts[i];
		    ref var manifold = ref contact.manifold;
		    int pointCount = manifold.pointCount;

		    for ( int j = 0; j < pointCount; ++j )
		    {
			    manifold.points.AsSpan[j].normalImpulse = constraint.points.AsSpan[j].normalImpulse;
			    manifold.points.AsSpan[j].tangentImpulse = constraint.points.AsSpan[j].tangentImpulse;
			    manifold.points.AsSpan[j].totalNormalImpulse = constraint.points.AsSpan[j].totalNormalImpulse;
			    manifold.points.AsSpan[j].normalVelocity = constraint.points.AsSpan[j].relativeVelocity;
		    }

		    manifold.rollingImpulse = constraint.rollingImpulse;
	    }
    }

    #endregion

    #region SIMD helpers

    private unsafe FixedArray8<float> SimdMul(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        if (!Avx.IsSupported)
        {
            return new FixedArray8<float>(
                a[0] * b[0],
                a[1] * b[1],
                a[2] * b[2],
                a[3] * b[3],
                a[4] * b[4],
                a[5] * b[5],
                a[6] * b[6],
                a[7] * b[7]);
        }

        ref var floatRefA = ref Unsafe.As<ReadOnlySpan<float>, float>(ref a);
        ref var floatRefB = ref Unsafe.As<ReadOnlySpan<float>, float>(ref b);
        FixedArray8<float> returned = default;

        fixed (float* ptrA = &floatRefA)
        fixed (float* ptrB = &floatRefB)
        {
            var aData = Avx.LoadAlignedVector256(ptrA);
            var bData = Avx.LoadAlignedVector256(ptrB);
            var result = Avx.Multiply(aData, bData);

            Unsafe.WriteUnaligned(ref Unsafe.As<FixedArray8<float>, byte>(ref returned), result);

            return returned;
        }
    }

    private unsafe FixedArray8<float> SimdMulAdd(ReadOnlySpan<float> a, ReadOnlySpan<float> b, ReadOnlySpan<float> c)
    {
        if (!Avx.IsSupported)
        {
            return new FixedArray8<float>(
                a[0] + b[0] * c[0],
                a[1] + b[1] * c[1],
                a[2] + b[2] * c[2],
                a[3] + b[3] * c[3],
                a[4] + b[4] * c[4],
                a[5] + b[5] * c[5],
                a[6] + b[6] * c[6],
                a[7] + b[7] * c[7]);
        }

        ref var floatRefA = ref Unsafe.As<ReadOnlySpan<float>, float>(ref a);
        ref var floatRefB = ref Unsafe.As<ReadOnlySpan<float>, float>(ref b);
        ref var floatRefC = ref Unsafe.As<ReadOnlySpan<float>, float>(ref c);
        FixedArray8<float> returned = default;

        fixed (float* ptrA = &floatRefA)
        fixed (float* ptrB = &floatRefB)
        fixed (float* ptrC = &floatRefC)
        {
            var aData = Avx.LoadAlignedVector256(ptrA);
            var bData = Avx.LoadAlignedVector256(ptrB);
            var cData = Avx.LoadAlignedVector256(ptrC);

            var result = Avx.Add(aData, Avx.Multiply(bData, cData));

            Unsafe.WriteUnaligned(ref Unsafe.As<FixedArray8<float>, byte>(ref returned), result);

            return returned;
        }
    }

    private unsafe FixedArray8<float> SimdMulSub(ReadOnlySpan<float> a, ReadOnlySpan<float> b, ReadOnlySpan<float> c)
    {
        if (!Avx.IsSupported)
        {
            return new FixedArray8<float>(
                a[0] + b[0] * c[0],
                a[1] + b[1] * c[1],
                a[2] + b[2] * c[2],
                a[3] + b[3] * c[3],
                a[4] + b[4] * c[4],
                a[5] + b[5] * c[5],
                a[6] + b[6] * c[6],
                a[7] + b[7] * c[7]);
        }

        ref var floatRefA = ref Unsafe.As<ReadOnlySpan<float>, float>(ref a);
        ref var floatRefB = ref Unsafe.As<ReadOnlySpan<float>, float>(ref b);
        ref var floatRefC = ref Unsafe.As<ReadOnlySpan<float>, float>(ref c);
        FixedArray8<float> returned = default;

        fixed (float* ptrA = &floatRefA)
        fixed (float* ptrB = &floatRefB)
        fixed (float* ptrC = &floatRefC)
        {
            var aData = Avx.LoadAlignedVector256(ptrA);
            var bData = Avx.LoadAlignedVector256(ptrB);
            var cData = Avx.LoadAlignedVector256(ptrC);

            var result = Avx.Subtract(aData, Avx.Multiply(bData, cData));

            Unsafe.WriteUnaligned(ref Unsafe.As<FixedArray8<float>, byte>(ref returned), result);

            return returned;
        }
    }

    private unsafe FixedArray8<float> SimdAdd(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        if (!Avx.IsSupported)
        {
            return new FixedArray8<float>(
                a[0] * b[0],
                a[1] * b[1],
                a[2] * b[2],
                a[3] * b[3],
                a[4] * b[4],
                a[5] * b[5],
                a[6] * b[6],
                a[7] * b[7]);
        }

        ref var floatRefA = ref Unsafe.As<ReadOnlySpan<float>, float>(ref a);
        ref var floatRefB = ref Unsafe.As<ReadOnlySpan<float>, float>(ref b);
        FixedArray8<float> returned = default;

        fixed (float* ptrA = &floatRefA)
        fixed (float* ptrB = &floatRefB)
        {
            var aData = Avx.LoadAlignedVector256(ptrA);
            var bData = Avx.LoadAlignedVector256(ptrB);
            var result = Avx.Add(aData, bData);

            Unsafe.WriteUnaligned(ref Unsafe.As<FixedArray8<float>, byte>(ref returned), result);

            return returned;
        }
    }

    private unsafe FixedArray8<float> SimdSub(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        if (!Avx.IsSupported)
        {
            return new FixedArray8<float>(
                a[0] * b[0],
                a[1] * b[1],
                a[2] * b[2],
                a[3] * b[3],
                a[4] * b[4],
                a[5] * b[5],
                a[6] * b[6],
                a[7] * b[7]);
        }

        ref var floatRefA = ref Unsafe.As<ReadOnlySpan<float>, float>(ref a);
        ref var floatRefB = ref Unsafe.As<ReadOnlySpan<float>, float>(ref b);
        FixedArray8<float> returned = default;

        fixed (float* ptrA = &floatRefA)
        fixed (float* ptrB = &floatRefB)
        {
            var aData = Avx.LoadAlignedVector256(ptrA);
            var bData = Avx.LoadAlignedVector256(ptrB);
            var result = Avx.Subtract(aData, bData);

            Unsafe.WriteUnaligned(ref Unsafe.As<FixedArray8<float>, byte>(ref returned), result);

            return returned;
        }
    }

    private FixedArray8<float> SimdSplat(float scalar)
    {
        if (!Avx.IsSupported)
        {
            return new FixedArray8<float>(
                scalar,
                scalar,
                scalar,
                scalar,
                scalar,
                scalar,
                scalar,
                scalar);
        }

        FixedArray8<float> returned = default;

        var result = Vector256.Create(scalar);

        Unsafe.WriteUnaligned(ref Unsafe.As<FixedArray8<float>, byte>(ref returned), result);
        return returned;
    }

    private FixedArray8<float> SimdDot(Vector2Wide a, Vector2Wide b)
    {
        return SimdAdd( SimdMul( a.X.AsSpan, b.X.AsSpan ).AsSpan, SimdMul( a.Y.AsSpan, b.Y.AsSpan ).AsSpan );
    }

    private FixedArray8<float> SimdCross(Vector2Wide a, Vector2Wide b)
    {
        return SimdSub(SimdMul(a.X.AsSpan, b.Y.AsSpan).AsSpan, SimdMul(a.Y.AsSpan, b.X.AsSpan).AsSpan);
    }

    private Vector2Wide SimdRotateVector(ReadOnlySpan<float> qC, ReadOnlySpan<float> qS, ReadOnlySpan<float> vX, ReadOnlySpan<float> vY)
    {
        return new Vector2Wide()
        {
            X = SimdSub( SimdMul( qC, vX ).AsSpan, SimdMul( qS, vY ).AsSpan ),
            Y = SimdAdd( SimdMul( qS, vX ).AsSpan, SimdMul( qC, vY ).AsSpan ),
        };
    }

    #endregion
}
