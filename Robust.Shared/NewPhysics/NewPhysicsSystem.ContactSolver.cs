using System.Numerics;
using Robust.Shared.NewPhysics.Contacts;
using Robust.Shared.Threading;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private void PrepareContactsTask(int startIndex, int endIndex)
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
				    const b2Manifold* manifold = &contactSim.manifold;

				    int indexA = contactSim.bodySimIndexA;
				    int indexB = contactSim.bodySimIndexB;

    #if DEBUG
				    b2Body* bodyA = bodies + contactSim.bodyIdA;
				    int validIndexA = bodyA.setIndex == (int) SetType.AwakeSet ? bodyA.localIndex : PhysicsConstants.NullIndex;
				    b2Body* bodyB = bodies + contactSim.bodyIdB;
				    int validIndexB = bodyB.setIndex == (int) SetType.AwakeSet ? bodyB.localIndex : PhysicsConstants.NullIndex;

				    DebugTools.Assert( indexA == validIndexA );
				    DebugTools.Assert( indexB == validIndexB );
    #endif
				    constraint.indexA[j] = indexA;
				    constraint.indexB[j] = indexB;

				    b2Vec2 vA = Vector2.Zero;
				    float wA = 0.0f;
				    float mA = contactSim.invMassA;
				    float iA = contactSim.invIA;
				    if ( indexA != PhysicsConstants.NullIndex )
				    {
					    b2BodyState* stateA = awakeStates + indexA;
					    vA = stateA.linearVelocity;
					    wA = stateA.angularVelocity;
				    }

				    b2Vec2 vB = Vector2.Zero;
				    float wB = 0.0f;
				    float mB = contactSim.invMassB;
				    float iB = contactSim.invIB;
				    if ( indexB != PhysicsConstants.NullIndex )
				    {
					    b2BodyState* stateB = awakeStates + indexB;
					    vB = stateB.linearVelocity;
					    wB = stateB.angularVelocity;
				    }

				    ( (float*)&constraint.invMassA )[j] = mA;
				    ( (float*)&constraint.invMassB )[j] = mB;
				    ( (float*)&constraint.invIA )[j] = iA;
				    ( (float*)&constraint.invIB )[j] = iB;

				    {
					    float k = iA + iB;
					    ( (float*)&constraint.rollingMass )[j] = k > 0.0f ? 1.0f / k : 0.0f;
				    }

				    b2Softness soft = contactSoftness;
				    if (indexA == PhysicsConstants.NullIndex || indexB == PhysicsConstants.NullIndex)
				    {
					    soft = staticSoftness;
				    }
				    else if (enableSoftening)
				    {
					    // todo experimental feature
					    float contactHertz = b2MinFloat( world.contactHertz, 0.125f * context.inv_h );
					    float ratio = 1.0f;
					    if ( mA < mB )
					    {
						    ratio = MathF.Max( 0.5f, mA / mB );
					    }
					    else if ( mB < mA )
					    {
						    ratio = MathF.Max( 0.5f, mB / mA );
					    }
					    soft = b2MakeSoft( ratio * contactHertz, ratio * world.contactDampingRatio, _h );
				    }

				    var normal = manifold.normal;
				    ( (float*)&constraint.normal.X )[j] = normal.x;
				    ( (float*)&constraint.normal.Y )[j] = normal.y;

				    ( (float*)&constraint.friction )[j] = contactSim.friction;
				    ( (float*)&constraint.tangentSpeed )[j] = contactSim.tangentSpeed;
				    ( (float*)&constraint.restitution )[j] = contactSim.restitution;
				    ( (float*)&constraint.rollingResistance )[j] = contactSim.rollingResistance;
				    ( (float*)&constraint.rollingImpulse )[j] = warmStartScale * manifold.rollingImpulse;

				    ( (float*)&constraint.biasRate )[j] = soft.biasRate;
				    ( (float*)&constraint.massScale )[j] = soft.massScale;
				    ( (float*)&constraint.impulseScale )[j] = soft.impulseScale;

				    var tangent = normal.RightPerp();

				    {
					    const b2ManifoldPoint* mp = manifold.points + 0;

					    b2Vec2 rA = mp.anchorA;
					    b2Vec2 rB = mp.anchorB;

					    ( (float*)&constraint.anchorA1.X )[j] = rA.x;
					    ( (float*)&constraint.anchorA1.Y )[j] = rA.y;
					    ( (float*)&constraint.anchorB1.X )[j] = rB.x;
					    ( (float*)&constraint.anchorB1.Y )[j] = rB.y;

					    ( (float*)&constraint.baseSeparation1 )[j] = mp.separation - Vector2.Dot( rB - rA, normal );

					    ( (float*)&constraint->normalImpulse1 )[j] = warmStartScale * mp->normalImpulse;
					    ( (float*)&constraint->tangentImpulse1 )[j] = warmStartScale * mp->tangentImpulse;
					    ( (float*)&constraint->totalNormalImpulse1 )[j] = 0.0f;

					    float rnA = Vector2Helpers.Cross( rA, normal );
					    float rnB = Vector2Helpers.Cross( rB, normal );
					    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					    ( (float*)&constraint->normalMass1 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					    float rtA = Vector2Helpers.Cross( rA, tangent );
					    float rtB = Vector2Helpers.Cross( rB, tangent );
					    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					    ( (float*)&constraint->tangentMass1 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					    // relative velocity for restitution
					    b2Vec2 vrA = b2Add( vA, Vector2Helpers.Cross( wA, rA ) );
					    b2Vec2 vrB = b2Add( vB, Vector2Helpers.Cross( wB, rB ) );
					    ( (float*)&constraint->relativeVelocity1 )[j] = Vector2.Dot( normal, b2Sub( vrB, vrA ) );
				    }

				    int pointCount = manifold->pointCount;
				    DebugTools.Assert( 0 < pointCount && pointCount <= 2 );

				    if ( pointCount == 2 )
				    {
					    const b2ManifoldPoint* mp = manifold->points + 1;

					    b2Vec2 rA = mp->anchorA;
					    b2Vec2 rB = mp->anchorB;

					    ( (float*)&constraint->anchorA2.X )[j] = rA.x;
					    ( (float*)&constraint->anchorA2.Y )[j] = rA.y;
					    ( (float*)&constraint->anchorB2.X )[j] = rB.x;
					    ( (float*)&constraint->anchorB2.Y )[j] = rB.y;

					    ( (float*)&constraint->baseSeparation2 )[j] = mp->separation - Vector2.Dot( b2Sub( rB, rA ), normal );

					    ( (float*)&constraint->normalImpulse2 )[j] = warmStartScale * mp->normalImpulse;
					    ( (float*)&constraint->tangentImpulse2 )[j] = warmStartScale * mp->tangentImpulse;
					    ( (float*)&constraint->totalNormalImpulse2 )[j] = 0.0f;

					    float rnA = Vector2Helpers.Cross( rA, normal );
					    float rnB = Vector2Helpers.Cross( rB, normal );
					    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
					    ( (float*)&constraint->normalMass2 )[j] = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

					    float rtA = Vector2Helpers.Cross( rA, tangent );
					    float rtB = Vector2Helpers.Cross( rB, tangent );
					    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
					    ( (float*)&constraint->tangentMass2 )[j] = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

					    // relative velocity for restitution
					    b2Vec2 vrA = b2Add( vA, Vector2Helpers.Cross( wA, rA ) );
					    b2Vec2 vrB = b2Add( vB, Vector2Helpers.Cross( wB, rB ) );
					    ( (float*)&constraint->relativeVelocity2 )[j] = Vector2.Dot( normal, b2Sub( vrB, vrA ) );
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
				    constraint->indexA[j] = PhysicsConstants.NullIndex;
				    constraint->indexB[j] = PhysicsConstants.NullIndex;

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
	    b2World* world = context->world;
	    b2ConstraintGraph* graph = context->graph;
	    b2GraphColor* color = graph->colors + PhysicsConstants.OverflowIndex;
	    b2ContactConstraint* constraints = color->overflowConstraints;
	    int contactCount = color->contactSims.count;
	    b2ContactSim* contacts = color->contactSims.data;
	    b2BodyState* awakeStates = context->states;

    #if B2_VALIDATE
	    b2Body* bodies = world->bodies.data;
    #endif

	    // Stiffer for static contacts to avoid bodies getting pushed through the ground
	    b2Softness contactSoftness = context->contactSoftness;
	    b2Softness staticSoftness = context->staticSoftness;

	    float warmStartScale = world->enableWarmStarting ? 1.0f : 0.0f;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    b2ContactSim* contactSim = contacts + i;

		    const b2Manifold* manifold = &contactSim->manifold;
		    int pointCount = manifold->pointCount;

		    DebugTools.Assert( 0 < pointCount && pointCount <= 2 );

		    int indexA = contactSim->bodySimIndexA;
		    int indexB = contactSim->bodySimIndexB;

    #if B2_VALIDATE
		    b2Body* bodyA = bodies + contactSim->bodyIdA;
		    int validIndexA = bodyA->setIndex == (int) SetType.AwakeSet ? bodyA->localIndex : PhysicsConstants.NullIndex;
		    DebugTools.Assert( indexA == validIndexA );

		    b2Body* bodyB = bodies + contactSim->bodyIdB;
		    int validIndexB = bodyB->setIndex == (int) SetType.AwakeSet ? bodyB->localIndex : PhysicsConstants.NullIndex;
		    DebugTools.Assert( indexB == validIndexB );
    #endif

		    b2ContactConstraint* constraint = constraints + i;
		    constraint->indexA = indexA;
		    constraint->indexB = indexB;
		    constraint->normal = manifold->normal;
		    constraint->friction = contactSim->friction;
		    constraint->restitution = contactSim->restitution;
		    constraint->rollingResistance = contactSim->rollingResistance;
		    constraint->rollingImpulse = warmStartScale * manifold->rollingImpulse;
		    constraint->tangentSpeed = contactSim->tangentSpeed;
		    constraint->pointCount = pointCount;

		    b2Vec2 vA = Vector2.Zero;
		    float wA = 0.0f;
		    float mA = contactSim->invMassA;
		    float iA = contactSim->invIA;
		    if ( indexA != PhysicsConstants.NullIndex )
		    {
			    b2BodyState* stateA = awakeStates + indexA;
			    vA = stateA->linearVelocity;
			    wA = stateA->angularVelocity;
		    }

		    b2Vec2 vB = Vector2.Zero;
		    float wB = 0.0f;
		    float mB = contactSim->invMassB;
		    float iB = contactSim->invIB;
		    if ( indexB != PhysicsConstants.NullIndex )
		    {
			    b2BodyState* stateB = awakeStates + indexB;
			    vB = stateB->linearVelocity;
			    wB = stateB->angularVelocity;
		    }

		    if ( indexA == PhysicsConstants.NullIndex || indexB == PhysicsConstants.NullIndex )
		    {
			    constraint->softness = staticSoftness;
		    }
		    else
		    {
			    constraint->softness = contactSoftness;
		    }

		    // copy mass into constraint to avoid cache misses during sub-stepping
		    constraint->invMassA = mA;
		    constraint->invIA = iA;
		    constraint->invMassB = mB;
		    constraint->invIB = iB;

		    {
			    float k = iA + iB;
			    constraint->rollingMass = k > 0.0f ? 1.0f / k : 0.0f;
		    }

		    b2Vec2 normal = constraint->normal;
		    b2Vec2 tangent = b2RightPerp( constraint->normal );

		    for ( int j = 0; j < pointCount; ++j )
		    {
			    const b2ManifoldPoint* mp = manifold->points + j;
			    b2ContactConstraintPoint* cp = constraint->points + j;

			    cp->normalImpulse = warmStartScale * mp->normalImpulse;
			    cp->tangentImpulse = warmStartScale * mp->tangentImpulse;
			    cp->totalNormalImpulse = 0.0f;

			    b2Vec2 rA = mp->anchorA;
			    b2Vec2 rB = mp->anchorB;

			    cp->anchorA = rA;
			    cp->anchorB = rB;
			    cp->baseSeparation = mp->separation - Vector2.Dot( b2Sub( rB, rA ), normal );

			    float rnA = Vector2Helpers.Cross( rA, normal );
			    float rnB = Vector2Helpers.Cross( rB, normal );
			    float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
			    cp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			    float rtA = Vector2Helpers.Cross( rA, tangent );
			    float rtB = Vector2Helpers.Cross( rB, tangent );
			    float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;
			    cp->tangentMass = kTangent > 0.0f ? 1.0f / kTangent : 0.0f;

			    // Save relative velocity for restitution
			    b2Vec2 vrA = b2Add( vA, Vector2Helpers.Cross( wA, rA ) );
			    b2Vec2 vrB = b2Add( vB, Vector2Helpers.Cross( wB, rB ) );
			    cp->relativeVelocity = Vector2.Dot( normal, b2Sub( vrB, vrA ) );
		    }
	    }

	    b2TracyCZoneEnd( prepare_overflow_contact );
    }

    private void WarmStartOverflowContacts()
    {
	    b2TracyCZoneNC( warmstart_overflow_contact, "WarmStart Overflow Contact", b2_colorDarkOrange, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2GraphColor* color = graph->colors + PhysicsConstants.OverflowIndex;
	    b2ContactConstraint* constraints = color->overflowConstraints;
	    int contactCount = color->contactSims.count;
	    b2World* world = context->world;
	    b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, (int) SetType.AwakeSet );
	    b2BodyState* states = awakeSet->bodyStates.data;

	    // This is a dummy state to represent a static body because static bodies don't have a solver body.
	    b2BodyState dummyState = bodyState.Identity;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    const b2ContactConstraint* constraint = constraints + i;

		    int indexA = constraint->indexA;
		    int indexB = constraint->indexB;

		    b2BodyState* stateA = indexA == PhysicsConstants.NullIndex ? &dummyState : states + indexA;
		    b2BodyState* stateB = indexB == PhysicsConstants.NullIndex ? &dummyState : states + indexB;

		    b2Vec2 vA = stateA->linearVelocity;
		    float wA = stateA->angularVelocity;
		    b2Vec2 vB = stateB->linearVelocity;
		    float wB = stateB->angularVelocity;

		    float mA = constraint->invMassA;
		    float iA = constraint->invIA;
		    float mB = constraint->invMassB;
		    float iB = constraint->invIB;

		    // Stiffer for static contacts to avoid bodies getting pushed through the ground
		    b2Vec2 normal = constraint->normal;
		    b2Vec2 tangent = b2RightPerp( constraint->normal );
		    int pointCount = constraint->pointCount;

		    for ( int j = 0; j < pointCount; ++j )
		    {
			    const b2ContactConstraintPoint* cp = constraint->points + j;

			    // fixed anchors
			    b2Vec2 rA = cp->anchorA;
			    b2Vec2 rB = cp->anchorB;

			    b2Vec2 P = b2Add( b2MulSV( cp->normalImpulse, normal ), b2MulSV( cp->tangentImpulse, tangent ) );
			    wA -= iA * Vector2Helpers.Cross( rA, P );
			    vA = Vector2Helpers.MulAdd( vA, -mA, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
		    }

		    wA -= iA * constraint->rollingImpulse;
		    wB += iB * constraint->rollingImpulse;

		    if ( stateA->flags & b2_dynamicFlag )
		    {
			    stateA->linearVelocity = vA;
			    stateA->angularVelocity = wA;
		    }

		    if ( stateB->flags & b2_dynamicFlag )
		    {
			    stateB->linearVelocity = vB;
			    stateB->angularVelocity = wB;
		    }
	    }

	    b2TracyCZoneEnd( warmstart_overflow_contact );
    }

    private void SolveOverflowContacts( bool useBias )
    {
	    b2TracyCZoneNC( solve_contact, "Solve Contact", b2_colorAliceBlue, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2GraphColor* color = graph->colors + PhysicsConstants.OverflowIndex;
	    b2ContactConstraint* constraints = color->overflowConstraints;
	    int contactCount = color->contactSims.count;
	    b2World* world = context->world;
	    b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, (int) SetType.AwakeSet );
	    b2BodyState* states = awakeSet->bodyStates.data;

	    float inv_h = context->inv_h;
	    const float contactSpeed = context->world->contactSpeed;

	    // This is a dummy body to represent a static body since static bodies don't have a solver body.
	    b2BodyState dummyState = bodyState.Identity;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    b2ContactConstraint* constraint = constraints + i;
		    float mA = constraint->invMassA;
		    float iA = constraint->invIA;
		    float mB = constraint->invMassB;
		    float iB = constraint->invIB;

		    b2BodyState* stateA = constraint->indexA == PhysicsConstants.NullIndex ? &dummyState : states + constraint->indexA;
		    b2Vec2 vA = stateA->linearVelocity;
		    float wA = stateA->angularVelocity;
		    b2Rot dqA = stateA->deltaRotation;

		    b2BodyState* stateB = constraint->indexB == PhysicsConstants.NullIndex ? &dummyState : states + constraint->indexB;
		    b2Vec2 vB = stateB->linearVelocity;
		    float wB = stateB->angularVelocity;
		    b2Rot dqB = stateB->deltaRotation;

		    b2Vec2 dp = b2Sub( stateB->deltaPosition, stateA->deltaPosition );

		    b2Vec2 normal = constraint->normal;
		    b2Vec2 tangent = b2RightPerp( normal );
		    float friction = constraint->friction;
		    b2Softness softness = constraint->softness;

		    int pointCount = constraint->pointCount;
		    float totalNormalImpulse = 0.0f;

		    // Non-penetration
		    for ( int j = 0; j < pointCount; ++j )
		    {
			    b2ContactConstraintPoint* cp = constraint->points + j;

			    // fixed anchor points
			    b2Vec2 rA = cp->anchorA;
			    b2Vec2 rB = cp->anchorB;

			    // compute current separation
			    // this is subject to round-off error if the anchor is far from the body center of mass
			    b2Vec2 ds = b2Add( dp, b2Sub( Quaternion2D.RotateVector( dqB, rB ), Quaternion2D.RotateVector( dqA, rA ) ) );
			    float s = cp->baseSeparation + Vector2.Dot( ds, normal );

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
			    b2Vec2 vrA = b2Add( vA, Vector2Helpers.Cross( wA, rA ) );
			    b2Vec2 vrB = b2Add( vB, Vector2Helpers.Cross( wB, rB ) );
			    float vn = Vector2.Dot( b2Sub( vrB, vrA ), normal );

			    // incremental normal impulse
			    float impulse = -cp->normalMass * ( massScale * vn + velocityBias ) - impulseScale * cp->normalImpulse;

			    // clamp the accumulated impulse
			    float newImpulse = MathF.Max( cp->normalImpulse + impulse, 0.0f );
			    impulse = newImpulse - cp->normalImpulse;
			    cp->normalImpulse = newImpulse;
			    cp->totalNormalImpulse += newImpulse;

			    totalNormalImpulse += newImpulse;

			    // apply normal impulse
			    b2Vec2 P = b2MulSV( impulse, normal );
			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * Vector2Helpers.Cross( rA, P );

			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
		    }

		    // Friction
		    for ( int j = 0; j < pointCount; ++j )
		    {
			    b2ContactConstraintPoint* cp = constraint->points + j;

			    // fixed anchor points
			    b2Vec2 rA = cp->anchorA;
			    b2Vec2 rB = cp->anchorB;

			    // relative tangent velocity at contact
			    b2Vec2 vrB = b2Add( vB, Vector2Helpers.Cross( wB, rB ) );
			    b2Vec2 vrA = b2Add( vA, Vector2Helpers.Cross( wA, rA ) );

			    // vt = dot(vrB - sB * tangent - (vrA + sA * tangent), tangent)
			    //    = dot(vrB - vrA, tangent) - (sA + sB)

			    float vt = Vector2.Dot( b2Sub( vrB, vrA ), tangent ) - constraint->tangentSpeed;

			    // incremental tangent impulse
			    float impulse = cp->tangentMass * ( -vt );

			    // clamp the accumulated force
			    float maxFriction = friction * cp->normalImpulse;
			    float newImpulse = Math.Clamp( cp->tangentImpulse + impulse, -maxFriction, maxFriction );
			    impulse = newImpulse - cp->tangentImpulse;
			    cp->tangentImpulse = newImpulse;

			    // apply tangent impulse
			    b2Vec2 P = b2MulSV( impulse, tangent );
			    vA = Vector2Helpers.MulSub( vA, mA, P );
			    wA -= iA * Vector2Helpers.Cross( rA, P );
			    vB = Vector2Helpers.MulAdd( vB, mB, P );
			    wB += iB * Vector2Helpers.Cross( rB, P );
		    }

		    // Rolling resistance
		    {
			    float deltaLambda = -constraint->rollingMass * ( wB - wA );
			    float lambda = constraint->rollingImpulse;
			    float maxLambda = constraint->rollingResistance * totalNormalImpulse;
			    constraint->rollingImpulse = Math.Clamp( lambda + deltaLambda, -maxLambda, maxLambda );
			    deltaLambda = constraint->rollingImpulse - lambda;

			    wA -= iA * deltaLambda;
			    wB += iB * deltaLambda;
		    }

		    if ( stateA->flags & b2_dynamicFlag )
		    {
			    stateA->linearVelocity = vA;
			    stateA->angularVelocity = wA;
		    }

		    if ( stateB->flags & b2_dynamicFlag )
		    {
			    stateB->linearVelocity = vB;
			    stateB->angularVelocity = wB;
		    }
	    }

	    b2TracyCZoneEnd( solve_contact );
    }

    private void ApplyOverflowRestitution()
    {
	    b2TracyCZoneNC( overflow_resitution, "Overflow Restitution", b2_colorViolet, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2GraphColor* color = graph->colors + PhysicsConstants.OverflowIndex;
	    b2ContactConstraint* constraints = color->overflowConstraints;
	    int contactCount = color->contactSims.count;
	    b2World* world = context->world;
	    b2SolverSet* awakeSet = b2SolverSetArray_Get( &world->solverSets, (int) SetType.AwakeSet );
	    b2BodyState* states = awakeSet->bodyStates.data;

	    float threshold = context->world->restitutionThreshold;

	    // dummy state to represent a static body
	    b2BodyState dummyState = bodyState.Identity;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    b2ContactConstraint* constraint = constraints + i;

		    float restitution = constraint->restitution;
		    if ( restitution == 0.0f )
		    {
			    continue;
		    }

		    float mA = constraint->invMassA;
		    float iA = constraint->invIA;
		    float mB = constraint->invMassB;
		    float iB = constraint->invIB;

		    b2BodyState* stateA = constraint->indexA == PhysicsConstants.NullIndex ? &dummyState : states + constraint->indexA;
		    b2Vec2 vA = stateA->linearVelocity;
		    float wA = stateA->angularVelocity;

		    b2BodyState* stateB = constraint->indexB == PhysicsConstants.NullIndex ? &dummyState : states + constraint->indexB;
		    b2Vec2 vB = stateB->linearVelocity;
		    float wB = stateB->angularVelocity;

		    b2Vec2 normal = constraint->normal;
		    int pointCount = constraint->pointCount;

		    // it is possible to get more accurate restitution by iterating
		    // this only makes a difference if there are two contact points
		    // for (int iter = 0; iter < 10; ++iter)
		    {
			    for ( int j = 0; j < pointCount; ++j )
			    {
				    b2ContactConstraintPoint* cp = constraint->points + j;

				    // if the normal impulse is zero then there was no collision
				    // this skips speculative contact points that didn't generate an impulse
				    // The max normal impulse is used in case there was a collision that moved away within the sub-step process
				    if ( cp->relativeVelocity > -threshold || cp->totalNormalImpulse == 0.0f )
				    {
					    continue;
				    }

				    // fixed anchor points
				    b2Vec2 rA = cp->anchorA;
				    b2Vec2 rB = cp->anchorB;

				    // relative normal velocity at contact
				    b2Vec2 vrB = b2Add( vB, Vector2Helpers.Cross( wB, rB ) );
				    b2Vec2 vrA = b2Add( vA, Vector2Helpers.Cross( wA, rA ) );
				    float vn = Vector2.Dot( b2Sub( vrB, vrA ), normal );

				    // compute normal impulse
				    float impulse = -cp->normalMass * ( vn + restitution * cp->relativeVelocity );

				    // clamp the accumulated impulse
				    // todo should this be stored?
				    float newImpulse = MathF.Max( cp->normalImpulse + impulse, 0.0f );
				    impulse = newImpulse - cp->normalImpulse;
				    cp->normalImpulse = newImpulse;

				    // Add the incremental impulse rather than the full impulse because this is not a sub-step
				    cp->totalNormalImpulse += impulse;

				    // apply contact impulse
				    b2Vec2 P = b2MulSV( impulse, normal );
				    vA = Vector2Helpers.MulSub( vA, mA, P );
				    wA -= iA * Vector2Helpers.Cross( rA, P );
				    vB = Vector2Helpers.MulAdd( vB, mB, P );
				    wB += iB * Vector2Helpers.Cross( rB, P );
			    }
		    }

		    if ( stateA->flags & b2_dynamicFlag )
		    {
			    stateA->linearVelocity = vA;
			    stateA->angularVelocity = wA;
		    }

		    if ( stateB->flags & b2_dynamicFlag )
		    {
			    stateB->linearVelocity = vB;
			    stateB->angularVelocity = wB;
		    }
	    }

	    b2TracyCZoneEnd( overflow_resitution );
    }

    private void StoreOverflowImpulses()
    {
	    b2TracyCZoneNC( store_impulses, "Store", b2_colorFireBrick, true );

	    b2ConstraintGraph* graph = context->graph;
	    b2GraphColor* color = graph->colors + PhysicsConstants.OverflowIndex;
	    b2ContactConstraint* constraints = color->overflowConstraints;
	    b2ContactSim* contacts = color->contactSims.data;
	    int contactCount = color->contactSims.count;

	    for ( int i = 0; i < contactCount; ++i )
	    {
		    const b2ContactConstraint* constraint = constraints + i;
		    b2ContactSim* contact = contacts + i;
		    b2Manifold* manifold = &contact->manifold;
		    int pointCount = manifold->pointCount;

		    for ( int j = 0; j < pointCount; ++j )
		    {
			    manifold->points[j].normalImpulse = constraint->points[j].normalImpulse;
			    manifold->points[j].tangentImpulse = constraint->points[j].tangentImpulse;
			    manifold->points[j].totalNormalImpulse = constraint->points[j].totalNormalImpulse;
			    manifold->points[j].normalVelocity = constraint->points[j].relativeVelocity;
		    }

		    manifold->rollingImpulse = constraint->rollingImpulse;
	    }

	    b2TracyCZoneEnd( store_impulses );
    }

    #endregion

    // Integrate velocities and apply damping
    private void IntegrateVelocitiesTask( int startIndex, int endIndex)
    {
	    b2TracyCZoneNC( integrate_velocity, "IntVel", b2_colorDeepPink, true );

	    b2BodyState* states = context->states;
	    b2BodySim* sims = context->sims;

	    b2Vec2 gravity = context->world->gravity;
	    float h = context->h;
	    float maxLinearSpeed = context->maxLinearVelocity;
	    float maxAngularSpeed = B2_MAX_ROTATION * context->inv_dt;
	    float maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed;
	    float maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed;

	    for ( int i = startIndex; i < endIndex; ++i )
	    {
		    b2BodySim* sim = sims + i;
		    b2BodyState* state = states + i;

		    b2Vec2 v = state->linearVelocity;
		    float w = state->angularVelocity;

		    // Apply forces, torque, gravity, and damping
		    // Apply damping.
		    // Differential equation: dv/dt + c * v = 0
		    // Solution: v(t) = v0 * exp(-c * t)
		    // Time step: v(t + dt) = v0 * exp(-c * (t + dt)) = v0 * exp(-c * t) * exp(-c * dt) = v(t) * exp(-c * dt)
		    // v2 = exp(-c * dt) * v1
		    // Pade approximation:
		    // v2 = v1 * 1 / (1 + c * dt)
		    float linearDamping = 1.0f / ( 1.0f + h * sim->linearDamping );
		    float angularDamping = 1.0f / ( 1.0f + h * sim->angularDamping );

		    // Gravity scale will be zero for kinematic bodies
		    float gravityScale = sim->invMass > 0.0f ? sim->gravityScale : 0.0f;

		    // lvd = h * im * f + h * g
		    b2Vec2 linearVelocityDelta = b2Add( b2MulSV( h * sim->invMass, sim->force ), b2MulSV( h * gravityScale, gravity ) );
		    float angularVelocityDelta = h * sim->invInertia * sim->torque;

		    v = Vector2Helpers.MulAdd( linearVelocityDelta, linearDamping, v );
		    w = angularVelocityDelta + angularDamping * w;

		    // Clamp to max linear speed
		    if ( Vector2.Dot( v, v ) > maxLinearSpeedSquared )
		    {
			    float ratio = maxLinearSpeed / b2Length( v );
			    v = b2MulSV( ratio, v );
			    sim->flags |= b2_isSpeedCapped;
		    }

		    // Clamp to max angular speed
		    if ( w * w > maxAngularSpeedSquared && ( sim->flags & b2_allowFastRotation ) == 0 )
		    {
			    float ratio = maxAngularSpeed / b2AbsFloat( w );
			    w *= ratio;
			    sim->flags |= b2_isSpeedCapped;
		    }

		    if ( state->flags & b2_lockLinearX )
		    {
			    v.x = 0.0f;
		    }

		    if ( state->flags & b2_lockLinearY )
		    {
			    v.y = 0.0f;
		    }

		    if ( state->flags & b2_lockAngularZ )
		    {
			    w = 0.0f;
		    }

		    state->linearVelocity = v;
		    state->angularVelocity = w;
	    }

	    b2TracyCZoneEnd( integrate_velocity );
    }

    private void WarmStartContactsTask( int startIndex, int endIndex, int colorIndex )
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
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, P.X );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, P.Y );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, P.X );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, P.Y );
		    }

		    {
			    // fixed anchors
			    b2Vec2W rA = c->anchorA2;
			    b2Vec2W rB = c->anchorB2;

			    b2Vec2W P;
			    P.X = b2AddW( b2MulW( c->normalImpulse2, c->normal.X ), b2MulW( c->tangentImpulse2, tangentX ) );
			    P.Y = b2AddW( b2MulW( c->normalImpulse2, c->normal.Y ), b2MulW( c->tangentImpulse2, tangentY ) );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2CrossW( rA, P ) );
			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, P.X );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, P.Y );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2CrossW( rB, P ) );
			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, P.X );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, P.Y );
		    }

		    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, c->rollingImpulse );
		    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, c->rollingImpulse );

		    b2ScatterBodies( states, c->indexA, &bA );
		    b2ScatterBodies( states, c->indexB, &bB );
	    }

	    b2TracyCZoneEnd( warm_start_contact );
    }

    private void SolveContactsTask( int startIndex, int endIndex, int colorIndex, bool useBias )
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
			    b2Vec2W rsA = Quaternion2D.RotateVectorW( bA.dq, rA );
			    b2Vec2W rsB = Quaternion2D.RotateVectorW( bB.dq, rB );

			    // compute current separation
			    // this is subject to round-off error if the anchor is far from the body center of mass
			    b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			    b2FloatW s = b2AddW( Vector2.DotW( c->normal, ds ), c->baseSeparation1 );

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

			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    // second point non-penetration constraint
		    {
			    // moving anchors for current separation
			    b2Vec2W rsA = Quaternion2D.RotateVectorW( bA.dq, c->anchorA2 );
			    b2Vec2W rsB = Quaternion2D.RotateVectorW( bB.dq, c->anchorB2 );

			    // compute current separation
			    b2Vec2W ds = { b2AddW( dp.X, b2SubW( rsB.X, rsA.X ) ), b2AddW( dp.Y, b2SubW( rsB.Y, rsA.Y ) ) };
			    b2FloatW s = b2AddW( Vector2.DotW( c->normal, ds ), c->baseSeparation2 );

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

			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
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

			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
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

			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    // Rolling resistance
		    {
			    b2FloatW deltaLambda = b2MulW( c->rollingMass, b2SubW( bA.w, bB.w ) );
			    b2FloatW lambda = c->rollingImpulse;
			    b2FloatW maxLambda = b2MulW( c->rollingResistance, totalNormalImpulse );
			    c->rollingImpulse = b2SymClampW( b2AddW( lambda, deltaLambda ), maxLambda );
			    deltaLambda = b2SubW( c->rollingImpulse, lambda );

			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, deltaLambda );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, deltaLambda );
		    }

		    b2ScatterBodies( states, c->indexA, &bA );
		    b2ScatterBodies( states, c->indexB, &bB );
	    }

	    b2TracyCZoneEnd( solve_contact );
    }

    private void ApplyRestitutionTask( int startIndex, int endIndex, int colorIndex )
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

			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
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

			    bA.v.X = Vector2Helpers.MulSubW( bA.v.X, c->invMassA, Px );
			    bA.v.Y = Vector2Helpers.MulSubW( bA.v.Y, c->invMassA, Py );
			    bA.w = Vector2Helpers.MulSubW( bA.w, c->invIA, b2SubW( b2MulW( rA.X, Py ), b2MulW( rA.Y, Px ) ) );

			    bB.v.X = Vector2Helpers.MulAddW( bB.v.X, c->invMassB, Px );
			    bB.v.Y = Vector2Helpers.MulAddW( bB.v.Y, c->invMassB, Py );
			    bB.w = Vector2Helpers.MulAddW( bB.w, c->invIB, b2SubW( b2MulW( rB.X, Py ), b2MulW( rB.Y, Px ) ) );
		    }

		    b2ScatterBodies( states, c->indexA, &bA );
		    b2ScatterBodies( states, c->indexB, &bB );
	    }

	    b2TracyCZoneEnd( restitution );
    }

    private void IntegratePositionsTask( int startIndex, int endIndex)
    {
        b2TracyCZoneNC( integrate_positions, "IntPos", b2_colorDarkSeaGreen, true );

        b2BodyState* states = context->states;
        float h = context->h;

        DebugTools.Assert( startIndex <= endIndex );

        for ( int i = startIndex; i < endIndex; ++i )
        {
            b2BodyState* state = states + i;

            if ( state->flags & b2_lockLinearX )
            {
                state->linearVelocity.x = 0.0f;
            }

            if ( state->flags & b2_lockLinearY )
            {
                state->linearVelocity.y = 0.0f;
            }

            if ( state->flags & b2_lockAngularZ )
            {
                state->angularVelocity = 0.0f;
            }

            state->deltaPosition = Vector2Helpers.MulAdd( state->deltaPosition, h, state->linearVelocity );
            state->deltaRotation = b2IntegrateRotation( state->deltaRotation, h * state->angularVelocity );
        }

        b2TracyCZoneEnd( integrate_positions );
    }

    private void StoreImpulsesTask( int startIndex, int endIndex )
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
