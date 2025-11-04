using Robust.Shared.NewPhysics.Contacts;
using Robust.Shared.Threading;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private void PrepareContactsTask(int startIndex, int endIndex, in StepContext context)
    {
	    var contacts = context.contacts;
	    var constraints = context.simdContactConstraints;
	    var awakeStates = context.states;
    #if DEBUG
	    var bodies = _bodies;
    #endif

	    // Stiffer for static contacts to avoid bodies getting pushed through the ground
	    var contactSoftness = context.contactSoftness;
	    var staticSoftness = context.staticSoftness;
	    bool enableSoftening = _enableContactSoftening;

	    float warmStartScale = _enableWarmStarting ? 1.0f : 0.0f;

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    var constraint = constraints[i];

		    for ( int j = 0; j < B2_SIMD_WIDTH; ++j )
		    {
			    b2ContactSim* contactSim = contacts[B2_SIMD_WIDTH * i + j];

			    if ( contactSim != NULL )
			    {
				    const b2Manifold* manifold = &contactSim->manifold;

				    int indexA = contactSim->bodySimIndexA;
				    int indexB = contactSim->bodySimIndexB;

    #if DEBUG
				    b2Body* bodyA = bodies + contactSim->bodyIdA;
				    int validIndexA = bodyA->setIndex == b2_awakeSet ? bodyA->localIndex : B2_NULL_INDEX;
				    b2Body* bodyB = bodies + contactSim->bodyIdB;
				    int validIndexB = bodyB->setIndex == b2_awakeSet ? bodyB->localIndex : B2_NULL_INDEX;

				    B2_ASSERT( indexA == validIndexA );
				    B2_ASSERT( indexB == validIndexB );
    #endif
				    constraint->indexA[j] = indexA;
				    constraint->indexB[j] = indexB;

				    b2Vec2 vA = b2Vec2_zero;
				    float wA = 0.0f;
				    float mA = contactSim->invMassA;
				    float iA = contactSim->invIA;
				    if ( indexA != B2_NULL_INDEX )
				    {
					    b2BodyState* stateA = awakeStates + indexA;
					    vA = stateA->linearVelocity;
					    wA = stateA->angularVelocity;
				    }

				    b2Vec2 vB = b2Vec2_zero;
				    float wB = 0.0f;
				    float mB = contactSim->invMassB;
				    float iB = contactSim->invIB;
				    if ( indexB != B2_NULL_INDEX )
				    {
					    b2BodyState* stateB = awakeStates + indexB;
					    vB = stateB->linearVelocity;
					    wB = stateB->angularVelocity;
				    }

				    ( (float*)&constraint->invMassA )[j] = mA;
				    ( (float*)&constraint->invMassB )[j] = mB;
				    ( (float*)&constraint->invIA )[j] = iA;
				    ( (float*)&constraint->invIB )[j] = iB;

				    {
					    float k = iA + iB;
					    ( (float*)&constraint->rollingMass )[j] = k > 0.0f ? 1.0f / k : 0.0f;
				    }

				    b2Softness soft = contactSoftness;
				    if (indexA == B2_NULL_INDEX || indexB == B2_NULL_INDEX)
				    {
					    soft = staticSoftness;
				    }
				    else if (enableSoftening)
				    {
					    // todo experimental feature
					    float contactHertz = b2MinFloat( world->contactHertz, 0.125f * context->inv_h );
					    float ratio = 1.0f;
					    if ( mA < mB )
					    {
						    ratio = b2MaxFloat( 0.5f, mA / mB );
					    }
					    else if ( mB < mA )
					    {
						    ratio = b2MaxFloat( 0.5f, mB / mA );
					    }
					    soft = b2MakeSoft( ratio * contactHertz, ratio * world->contactDampingRatio, context->h );
				    }

				    b2Vec2 normal = manifold->normal;
				    ( (float*)&constraint->normal.X )[j] = normal.x;
				    ( (float*)&constraint->normal.Y )[j] = normal.y;

				    ( (float*)&constraint->friction )[j] = contactSim->friction;
				    ( (float*)&constraint->tangentSpeed )[j] = contactSim->tangentSpeed;
				    ( (float*)&constraint->restitution )[j] = contactSim->restitution;
				    ( (float*)&constraint->rollingResistance )[j] = contactSim->rollingResistance;
				    ( (float*)&constraint->rollingImpulse )[j] = warmStartScale * manifold->rollingImpulse;

				    ( (float*)&constraint->biasRate )[j] = soft.biasRate;
				    ( (float*)&constraint->massScale )[j] = soft.massScale;
				    ( (float*)&constraint->impulseScale )[j] = soft.impulseScale;

				    b2Vec2 tangent = b2RightPerp( normal );

				    {
					    const b2ManifoldPoint* mp = manifold->points + 0;

					    b2Vec2 rA = mp->anchorA;
					    b2Vec2 rB = mp->anchorB;

					    ( (float*)&constraint->anchorA1.X )[j] = rA.x;
					    ( (float*)&constraint->anchorA1.Y )[j] = rA.y;
					    ( (float*)&constraint->anchorB1.X )[j] = rB.x;
					    ( (float*)&constraint->anchorB1.Y )[j] = rB.y;

					    ( (float*)&constraint->baseSeparation1 )[j] = mp->separation - b2Dot( b2Sub( rB, rA ), normal );

					    ( (float*)&constraint->normalImpulse1 )[j] = warmStartScale * mp->normalImpulse;
					    ( (float*)&constraint->tangentImpulse1 )[j] = warmStartScale * mp->tangentImpulse;
					    ( (float*)&constraint->totalNormalImpulse1 )[j] = 0.0f;

					    float rnA = b2Cross( rA, normal );
					    float rnB = b2Cross( rB, normal );
					    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					    ( (float*)&constraint->normalMass1 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					    float rtA = b2Cross( rA, tangent );
					    float rtB = b2Cross( rB, tangent );
					    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					    ( (float*)&constraint->tangentMass1 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					    // relative velocity for restitution
					    b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
					    b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
					    ( (float*)&constraint->relativeVelocity1 )[j] = b2Dot( normal, b2Sub( vrB, vrA ) );
				    }

				    int pointCount = manifold->pointCount;
				    B2_ASSERT( 0 < pointCount && pointCount <= 2 );

				    if ( pointCount == 2 )
				    {
					    const b2ManifoldPoint* mp = manifold->points + 1;

					    b2Vec2 rA = mp->anchorA;
					    b2Vec2 rB = mp->anchorB;

					    ( (float*)&constraint->anchorA2.X )[j] = rA.x;
					    ( (float*)&constraint->anchorA2.Y )[j] = rA.y;
					    ( (float*)&constraint->anchorB2.X )[j] = rB.x;
					    ( (float*)&constraint->anchorB2.Y )[j] = rB.y;

					    ( (float*)&constraint->baseSeparation2 )[j] = mp->separation - b2Dot( b2Sub( rB, rA ), normal );

					    ( (float*)&constraint->normalImpulse2 )[j] = warmStartScale * mp->normalImpulse;
					    ( (float*)&constraint->tangentImpulse2 )[j] = warmStartScale * mp->tangentImpulse;
					    ( (float*)&constraint->totalNormalImpulse2 )[j] = 0.0f;

					    float rnA = b2Cross( rA, normal );
					    float rnB = b2Cross( rB, normal );
					    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					    ( (float*)&constraint->normalMass2 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					    float rtA = b2Cross( rA, tangent );
					    float rtB = b2Cross( rB, tangent );
					    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					    ( (float*)&constraint->tangentMass2 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					    // relative velocity for restitution
					    b2Vec2 vrA = b2Add( vA, b2CrossSV( wA, rA ) );
					    b2Vec2 vrB = b2Add( vB, b2CrossSV( wB, rB ) );
					    ( (float*)&constraint->relativeVelocity2 )[j] = b2Dot( normal, b2Sub( vrB, vrA ) );
				    }
				    else
				    {
					    // dummy data that has no effect
					    ( (float*)&constraint->baseSeparation2 )[j] = 0.0f;
					    ( (float*)&constraint->normalImpulse2 )[j] = 0.0f;
					    ( (float*)&constraint->tangentImpulse2 )[j] = 0.0f;
					    ( (float*)&constraint->totalNormalImpulse2 )[j] = 0.0f;
					    ( (float*)&constraint->anchorA2.X )[j] = 0.0f;
					    ( (float*)&constraint->anchorA2.Y )[j] = 0.0f;
					    ( (float*)&constraint->anchorB2.X )[j] = 0.0f;
					    ( (float*)&constraint->anchorB2.Y )[j] = 0.0f;
					    ( (float*)&constraint->normalMass2 )[j] = 0.0f;
					    ( (float*)&constraint->tangentMass2 )[j] = 0.0f;
					    ( (float*)&constraint->relativeVelocity2 )[j] = 0.0f;
				    }
			    }
			    else
			    {
				    // SIMD remainder
				    constraint->indexA[j] = B2_NULL_INDEX;
				    constraint->indexB[j] = B2_NULL_INDEX;

				    ( (float*)&constraint->invMassA )[j] = 0.0f;
				    ( (float*)&constraint->invMassB )[j] = 0.0f;
				    ( (float*)&constraint->invIA )[j] = 0.0f;
				    ( (float*)&constraint->invIB )[j] = 0.0f;

				    ( (float*)&constraint->normal.X )[j] = 0.0f;
				    ( (float*)&constraint->normal.Y )[j] = 0.0f;
				    ( (float*)&constraint->friction )[j] = 0.0f;
				    ( (float*)&constraint->tangentSpeed )[j] = 0.0f;
				    ( (float*)&constraint->rollingResistance )[j] = 0.0f;
				    ( (float*)&constraint->rollingMass )[j] = 0.0f;
				    ( (float*)&constraint->rollingImpulse )[j] = 0.0f;
				    ( (float*)&constraint->biasRate )[j] = 0.0f;
				    ( (float*)&constraint->massScale )[j] = 0.0f;
				    ( (float*)&constraint->impulseScale )[j] = 0.0f;

				    ( (float*)&constraint->anchorA1.X )[j] = 0.0f;
				    ( (float*)&constraint->anchorA1.Y )[j] = 0.0f;
				    ( (float*)&constraint->anchorB1.X )[j] = 0.0f;
				    ( (float*)&constraint->anchorB1.Y )[j] = 0.0f;
				    ( (float*)&constraint->baseSeparation1 )[j] = 0.0f;
				    ( (float*)&constraint->normalImpulse1 )[j] = 0.0f;
				    ( (float*)&constraint->tangentImpulse1 )[j] = 0.0f;
				    ( (float*)&constraint->totalNormalImpulse1 )[j] = 0.0f;
				    ( (float*)&constraint->normalMass1 )[j] = 0.0f;
				    ( (float*)&constraint->tangentMass1 )[j] = 0.0f;

				    ( (float*)&constraint->anchorA2.X )[j] = 0.0f;
				    ( (float*)&constraint->anchorA2.Y )[j] = 0.0f;
				    ( (float*)&constraint->anchorB2.X )[j] = 0.0f;
				    ( (float*)&constraint->anchorB2.Y )[j] = 0.0f;
				    ( (float*)&constraint->baseSeparation2 )[j] = 0.0f;
				    ( (float*)&constraint->normalImpulse2 )[j] = 0.0f;
				    ( (float*)&constraint->tangentImpulse2 )[j] = 0.0f;
				    ( (float*)&constraint->totalNormalImpulse2 )[j] = 0.0f;
				    ( (float*)&constraint->normalMass2 )[j] = 0.0f;
				    ( (float*)&constraint->tangentMass2 )[j] = 0.0f;

				    ( (float*)&constraint->restitution )[j] = 0.0f;
				    ( (float*)&constraint->relativeVelocity1 )[j] = 0.0f;
				    ( (float*)&constraint->relativeVelocity2 )[j] = 0.0f;
			    }
		    }
	    }

	    b2TracyCZoneEnd( prepare_contact );
    }

    void b2WarmStartContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex )
    {
	    b2TracyCZoneNC( warm_start_contact, "Warm Start", b2_colorGreen, true );

	    b2BodyState* states = context->states;
	    b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints;

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    b2ContactConstraintSIMD* c = constraints + i;
		    b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		    b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		    b2FloatW tangentX = c->normal.Y;
		    b2FloatW tangentY = b2SubW( b2ZeroW(), c->normal.X );

		    {
			    // fixed anchors
			    b2Vec2W rA = c->anchorA1;
			    b2Vec2W rB = c->anchorB1;

			    b2Vec2W P;
			    P.X = b2AddW( b2MulW( c->normalImpulse1, c->normal.X ), b2MulW( c->tangentImpulse1, tangentX ) );
			    P.Y = b2AddW( b2MulW( c->normalImpulse1, c->normal.Y ), b2MulW( c->tangentImpulse1, tangentY ) );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, P.X );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, P.Y );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, P.X );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, P.Y );
		    }

		    {
			    // fixed anchors
			    b2Vec2W rA = c->anchorA2;
			    b2Vec2W rB = c->anchorB2;

			    b2Vec2W P;
			    P.X = b2AddW( b2MulW( c->normalImpulse2, c->normal.X ), b2MulW( c->tangentImpulse2, tangentX ) );
			    P.Y = b2AddW( b2MulW( c->normalImpulse2, c->normal.Y ), b2MulW( c->tangentImpulse2, tangentY ) );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, P.X );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, P.Y );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, P.X );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, P.Y );
		    }

		    bA.w = b2MulSubW( bA.w, c->invIA, c->rollingImpulse );
		    bB.w = b2MulAddW( bB.w, c->invIB, c->rollingImpulse );

		    b2ScatterBodies( states, c->indexA, &bA );
		    b2ScatterBodies( states, c->indexB, &bB );
	    }

	    b2TracyCZoneEnd( warm_start_contact );
    }

    void b2SolveContactsTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex, bool useBias )
    {
	    b2TracyCZoneNC( solve_contact, "Solve Contact", b2_colorAliceBlue, true );

	    b2BodyState* states = context->states;
	    b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints;
	    b2FloatW inv_h = b2SplatW( context->inv_h );
	    b2FloatW contactSpeed = b2SplatW( -context->world->contactSpeed );
	    b2FloatW oneW = b2SplatW( 1.0f );

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    b2ContactConstraintSIMD* c = constraints + i;

		    b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		    b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		    b2FloatW biasRate, massScale, impulseScale;
		    if ( useBias )
		    {
			    biasRate = b2MulW( c->massScale, c->biasRate );
			    massScale = c->massScale;
			    impulseScale = c->impulseScale;
		    }
		    else
		    {
			    biasRate = b2ZeroW();
			    massScale = oneW;
			    impulseScale = b2ZeroW();
		    }

		    b2FloatW totalNormalImpulse = b2ZeroW();

		    b2Vec2W dp = { b2SubW( bB.dp.X, bA.dp.X ), b2SubW( bB.dp.Y, bA.dp.Y ) };

		    // point1 non-penetration constraint
		    {
			    // Fixed anchors for impulses
			    b2Vec2W rA = c->anchorA1;
			    b2Vec2W rB = c->anchorB1;

			    // Moving anchors for current separation
			    b2Vec2W rsA = b2RotateVectorW( bA.dq, rA );
			    b2Vec2W rsB = b2RotateVectorW( bB.dq, rB );

			    // compute current separation
			    // this is subject to round-off error if the anchor is far from the body center of mass
			    b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			    b2FloatW s = b2AddW( b2DotW( c->normal, ds ), c->baseSeparation1 );

			    // Apply speculative bias if separation is greater than zero, otherwise apply soft constraint bias
			    // The contactSpeed is meant to limit stiffness, not increase it.
			    b2FloatW mask = b2GreaterThanW( s, b2ZeroW() );
			    b2FloatW specBias = b2MulW( s, inv_h );
			    b2FloatW softBias = b2MaxW( b2MulW( biasRate, s ), contactSpeed );

			    // todo try b2MaxW(softBias, specBias);
			    b2FloatW bias = b2BlendW( softBias, specBias, mask );

			    b2FloatW pointMassScale = b2BlendW( massScale, oneW, mask );
			    b2FloatW pointImpulseScale = b2BlendW( impulseScale, b2ZeroW(), mask );

			    // Relative velocity at contact
			    b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			    b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			    b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = b2AddW( b2MulW( c->normalMass1, b2AddW( b2MulW( pointMassScale, vn ), bias ) ),
										      b2MulW( pointImpulseScale, c->normalImpulse1 ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse1, negImpulse ), b2ZeroW() );
			    b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse1 );
			    c->normalImpulse1 = newImpulse;
			    c->totalNormalImpulse1 = b2AddW( c->totalNormalImpulse1, newImpulse );

			    totalNormalImpulse = b2AddW( totalNormalImpulse, newImpulse );

			    // Apply contact impulse
			    b2FloatW Px = b2MulW( impulse, c->normal.X );
			    b2FloatW Py = b2MulW( impulse, c->normal.Y );

			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    // second point non-penetration constraint
		    {
			    // moving anchors for current separation
			    b2Vec2W rsA = b2RotateVectorW( bA.dq, c->anchorA2 );
			    b2Vec2W rsB = b2RotateVectorW( bB.dq, c->anchorB2 );

			    // compute current separation
			    b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			    b2FloatW s = b2AddW( b2DotW( c->normal, ds ), c->baseSeparation2 );

			    b2FloatW mask = b2GreaterThanW( s, b2ZeroW() );
			    b2FloatW specBias = b2MulW( s, inv_h );
			    b2FloatW softBias = b2MaxW( b2MulW( biasRate, s ), contactSpeed );
			    b2FloatW bias = b2BlendW( softBias, specBias, mask );

			    b2FloatW pointMassScale = b2BlendW( massScale, oneW, mask );
			    b2FloatW pointImpulseScale = b2BlendW( impulseScale, b2ZeroW(), mask );

			    // fixed anchors for Jacobians
			    b2Vec2W rA = c->anchorA2;
			    b2Vec2W rB = c->anchorB2;

			    // Relative velocity at contact
			    b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			    b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			    b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = b2AddW( b2MulW( c->normalMass2, b2AddW( b2MulW( pointMassScale, vn ), bias ) ),
										      b2MulW( pointImpulseScale, c->normalImpulse2 ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse2, negImpulse ), b2ZeroW() );
			    b2FloatW impulse = b2SubW( newImpulse, c->normalImpulse2 );
			    c->normalImpulse2 = newImpulse;
			    c->totalNormalImpulse2 = b2AddW( c->totalNormalImpulse2, newImpulse );

			    totalNormalImpulse = b2AddW( totalNormalImpulse, newImpulse );

			    // Apply contact impulse
			    b2FloatW Px = b2MulW( impulse, c->normal.X );
			    b2FloatW Py = b2MulW( impulse, c->normal.Y );

			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    b2FloatW tangentX = c->normal.Y;
		    b2FloatW tangentY = b2SubW( b2ZeroW(), c->normal.X );

		    // point 1 friction constraint
		    {
			    // fixed anchors for Jacobians
			    b2Vec2W rA = c->anchorA1;
			    b2Vec2W rB = c->anchorB1;

			    // Relative velocity at contact
			    b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			    b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			    b2FloatW vt = b2AddW( b2MulW( dvx, tangentX ), b2MulW( dvy, tangentY ) );

			    // Tangent speed (conveyor belt)
			    vt = b2SubW( vt, c->tangentSpeed );

			    // Compute tangent force
			    b2FloatW negImpulse = b2MulW( c->tangentMass1, vt );

			    // Clamp the accumulated force
			    b2FloatW maxFriction = b2MulW( c->friction, c->normalImpulse1 );
			    b2FloatW newImpulse = b2SubW( c->tangentImpulse1, negImpulse );
			    newImpulse = b2MaxW( b2SubW( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			    b2FloatW impulse = b2SubW( newImpulse, c->tangentImpulse1 );
			    c->tangentImpulse1 = newImpulse;

			    // Apply contact impulse
			    b2FloatW Px = b2MulW( impulse, tangentX );
			    b2FloatW Py = b2MulW( impulse, tangentY );

			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    // second point friction constraint
		    {
			    // fixed anchors for Jacobians
			    b2Vec2W rA = c->anchorA2;
			    b2Vec2W rB = c->anchorB2;

			    // Relative velocity at contact
			    b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			    b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			    b2FloatW vt = b2AddW( b2MulW( dvx, tangentX ), b2MulW( dvy, tangentY ) );

			    // Tangent speed (conveyor belt)
			    vt = b2SubW( vt, c->tangentSpeed );

			    // Compute tangent force
			    b2FloatW negImpulse = b2MulW( c->tangentMass2, vt );

			    // Clamp the accumulated force
			    b2FloatW maxFriction = b2MulW( c->friction, c->normalImpulse2 );
			    b2FloatW newImpulse = b2SubW( c->tangentImpulse2, negImpulse );
			    newImpulse = b2MaxW( b2SubW( b2ZeroW(), maxFriction ), b2MinW( newImpulse, maxFriction ) );
			    b2FloatW impulse = b2SubW( newImpulse, c->tangentImpulse2 );
			    c->tangentImpulse2 = newImpulse;

			    // Apply contact impulse
			    b2FloatW Px = b2MulW( impulse, tangentX );
			    b2FloatW Py = b2MulW( impulse, tangentY );

			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    // Rolling resistance
		    {
			    b2FloatW deltaLambda = b2MulW( c->rollingMass, b2SubW( bA.w, bB.w ) );
			    b2FloatW lambda = c->rollingImpulse;
			    b2FloatW maxLambda = b2MulW( c->rollingResistance, totalNormalImpulse );
			    c->rollingImpulse = b2SymClampW( b2AddW( lambda, deltaLambda ), maxLambda );
			    deltaLambda = b2SubW( c->rollingImpulse, lambda );

			    bA.w = b2MulSubW( bA.w, c->invIA, deltaLambda );
			    bB.w = b2MulAddW( bB.w, c->invIB, deltaLambda );
		    }

		    b2ScatterBodies( states, c->indexA, &bA );
		    b2ScatterBodies( states, c->indexB, &bB );
	    }

	    b2TracyCZoneEnd( solve_contact );
    }

    void b2ApplyRestitutionTask( int startIndex, int endIndex, b2StepContext* context, int colorIndex )
    {
	    b2TracyCZoneNC( restitution, "Restitution", b2_colorDodgerBlue, true );

	    b2BodyState* states = context->states;
	    b2ContactConstraintSIMD* constraints = context->graph->colors[colorIndex].simdConstraints;
	    b2FloatW threshold = b2SplatW( context->world->restitutionThreshold );
	    b2FloatW zero = b2ZeroW();

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    b2ContactConstraintSIMD* c = constraints + i;

		    if ( b2AllZeroW( c->restitution ) )
		    {
			    // No lanes have restitution. Common case.
			    continue;
		    }

		    // Create a mask based on restitution so that lanes with no restitution are not affected
		    // by the calculations below.
		    b2FloatW restitutionMask = b2EqualsW( c->restitution, zero );

		    b2BodyStateW bA = b2GatherBodies( states, c->indexA );
		    b2BodyStateW bB = b2GatherBodies( states, c->indexB );

		    // first point non-penetration constraint
		    {
			    // Set effective mass to zero if restitution should not be applied
			    b2FloatW mask1 = b2GreaterThanW( b2AddW( c->relativeVelocity1, threshold ), zero );
			    b2FloatW mask2 = b2EqualsW( c->totalNormalImpulse1, zero );
			    b2FloatW mask = b2OrW( b2OrW( mask1, mask2 ), restitutionMask );
			    b2FloatW mass = b2BlendW( c->normalMass1, zero, mask );

			    // fixed anchors for Jacobians
			    b2Vec2W rA = c->anchorA1;
			    b2Vec2W rB = c->anchorB1;

			    // Relative velocity at contact
			    b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			    b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			    b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = b2MulW( mass, b2AddW( vn, b2MulW( c->restitution, c->relativeVelocity1 ) ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse1, negImpulse ), b2ZeroW() );
			    b2FloatW deltaImpulse = b2SubW( newImpulse, c->normalImpulse1 );
			    c->normalImpulse1 = newImpulse;

			    // Add the incremental impulse rather than the full impulse because this is not a sub-step
			    c->totalNormalImpulse1 = b2AddW( c->totalNormalImpulse1, deltaImpulse );

			    // Apply contact impulse
			    b2FloatW Px = b2MulW( deltaImpulse, c->normal.X );
			    b2FloatW Py = b2MulW( deltaImpulse, c->normal.Y );

			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    // second point non-penetration constraint
		    {
			    // Set effective mass to zero if restitution should not be applied
			    b2FloatW mask1 = b2GreaterThanW( b2AddW( c->relativeVelocity2, threshold ), zero );
			    b2FloatW mask2 = b2EqualsW( c->totalNormalImpulse2, zero );
			    b2FloatW mask = b2OrW( b2OrW( mask1, mask2 ), restitutionMask );
			    b2FloatW mass = b2BlendW( c->normalMass2, zero, mask );

			    // fixed anchors for Jacobians
			    b2Vec2W rA = c->anchorA2;
			    b2Vec2W rB = c->anchorB2;

			    // Relative velocity at contact
			    b2FloatW dvx = b2SubW( b2SubW( bB.v.X, b2MulW( bB.w, rB.Y ) ), b2SubW( bA.v.X, b2MulW( bA.w, rA.Y ) ) );
			    b2FloatW dvy = b2SubW( b2AddW( bB.v.Y, b2MulW( bB.w, rB.X ) ), b2AddW( bA.v.Y, b2MulW( bA.w, rA.X ) ) );
			    b2FloatW vn = b2AddW( b2MulW( dvx, c->normal.X ), b2MulW( dvy, c->normal.Y ) );

			    // Compute normal impulse
			    b2FloatW negImpulse = b2MulW( mass, b2AddW( vn, b2MulW( c->restitution, c->relativeVelocity2 ) ) );

			    // Clamp the accumulated impulse
			    b2FloatW newImpulse = b2MaxW( b2SubW( c->normalImpulse2, negImpulse ), b2ZeroW() );
			    b2FloatW deltaImpulse = b2SubW( newImpulse, c->normalImpulse2 );
			    c->normalImpulse2 = newImpulse;

			    // Add the incremental impulse rather than the full impulse because this is not a sub-step
			    c->totalNormalImpulse2 = b2AddW( c->totalNormalImpulse2, deltaImpulse );

			    // Apply contact impulse
			    b2FloatW Px = b2MulW( deltaImpulse, c->normal.X );
			    b2FloatW Py = b2MulW( deltaImpulse, c->normal.Y );

			    bA.v.X = b2MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = b2MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = b2MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = b2MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = b2MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = b2MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    b2ScatterBodies( states, c->indexA, &bA );
		    b2ScatterBodies( states, c->indexB, &bB );
	    }

	    b2TracyCZoneEnd( restitution );
    }

    void b2StoreImpulsesTask( int startIndex, int endIndex, b2StepContext* context )
    {
	    b2TracyCZoneNC( store_impulses, "Store", b2_colorFireBrick, true );

	    b2ContactSim** contacts = context->contacts;
	    const b2ContactConstraintSIMD* constraints = context->simdContactConstraints;

	    b2Manifold dummy = { 0 };

	    for ( int constraintIndex = startIndex; constraintIndex < endIndex; ++constraintIndex )
	    {
		    const b2ContactConstraintSIMD* c = constraints + constraintIndex;
		    const float* rollingImpulse = (float*)&c->rollingImpulse;
		    const float* normalImpulse1 = (float*)&c->normalImpulse1;
		    const float* normalImpulse2 = (float*)&c->normalImpulse2;
		    const float* tangentImpulse1 = (float*)&c->tangentImpulse1;
		    const float* tangentImpulse2 = (float*)&c->tangentImpulse2;
		    const float* totalNormalImpulse1 = (float*)&c->totalNormalImpulse1;
		    const float* totalNormalImpulse2 = (float*)&c->totalNormalImpulse2;
		    const float* normalVelocity1 = (float*)&c->relativeVelocity1;
		    const float* normalVelocity2 = (float*)&c->relativeVelocity2;

		    int baseIndex = B2_SIMD_WIDTH * constraintIndex;

		    for ( int laneIndex = 0; laneIndex < B2_SIMD_WIDTH; ++laneIndex )
		    {
			    b2Manifold* m = contacts[baseIndex + laneIndex] == NULL ? &dummy : &contacts[baseIndex + laneIndex]->manifold;
			    m->rollingImpulse = rollingImpulse[laneIndex];

			    m->points[0].normalImpulse = normalImpulse1[laneIndex];
			    m->points[0].tangentImpulse = tangentImpulse1[laneIndex];
			    m->points[0].totalNormalImpulse = totalNormalImpulse1[laneIndex];
			    m->points[0].normalVelocity = normalVelocity1[laneIndex];

			    m->points[1].normalImpulse = normalImpulse2[laneIndex];
			    m->points[1].tangentImpulse = tangentImpulse2[laneIndex];
			    m->points[1].totalNormalImpulse = totalNormalImpulse2[laneIndex];
			    m->points[1].normalVelocity = normalVelocity2[laneIndex];
		    }
	    }

	    b2TracyCZoneEnd( store_impulses );
    }
}
