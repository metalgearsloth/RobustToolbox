using System.Diagnostics.Contracts;
using Robust.Shared.GameObjects;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private bool IsValid(in BodyId id)
    {
        if ( id.Index1 < 1 || _bodies.Count < id.Index1 )
        {
            // invalid index
            return false;
        }

        var ent = _bodies[id.Index1 - 1];
        var body = ent.Comp;

        if ( body.SetIndex == PhysicsConstants.NullIndex )
        {
            // this was freed
            return false;
        }

        DebugTools.Assert(body.LocalIndex != PhysicsConstants.NullIndex);

        if (ent.Owner != id.Uid)
        {
            // this id is orphaned
            return false;
        }

        return true;
    }

    private Entity<PhysicsComponent> GetBodyFullId(BodyId bodyId)
    {
        DebugTools.Assert(IsValid(bodyId));
        return _bodies[bodyId.Index1 - 1];
    }

    private BodyId MakeBodyId(int bodyId)
    {
        var body = _bodies[bodyId];
        return new BodyId()
        {
            Index1 = bodyId + 1,
            Uid = body.Owner,
        };
    }

    private ref BodySim GetBodySim(PhysicsComponent body)
    {
        // TODO: Dump storing bodysim / bodystate indefinitely and just derive it from physicscomponent.l

        var set = _solverSets[body.SetIndex];
        ref var sims = ref set.bodySims;
        return ref sims[body.LocalIndex];
    }

    [Pure]
    private BodyState? GetBodyState(PhysicsComponent body)
    {
        if (body.SetIndex == (int)SetType.AwakeSet)
        {
            var set = _solverSets[body.SetIndex];
            ref var states = ref set.bodyStates;
            return states[body.LocalIndex];
        }

        return null;
    }

    private void CreateIslandForBody(int setIndex, PhysicsComponent body)
    {
        DebugTools.Assert( body.IslandId == PhysicsConstants.NullIndex );
        DebugTools.Assert( body.islandPrev == PhysicsConstants.NullIndex );
        DebugTools.Assert( body.islandNext == PhysicsConstants.NullIndex );
        DebugTools.Assert( setIndex != (int) SetType.DisabledSet );

        var island = CreateIsland( setIndex );

        body.IslandId = island.islandId;
        island.headBody = body.Id;
        island.tailBody = body.Id;
        island.bodyCount = 1;
    }

    private void RemoveBodyFromIsland(Entity<PhysicsComponent> ent)
    {
        var body = ent.Comp;

        if ( body.IslandId == PhysicsConstants.NullIndex )
        {
            DebugTools.Assert( body.islandPrev == PhysicsConstants.NullIndex );
            DebugTools.Assert( body.islandNext == PhysicsConstants.NullIndex );
            return;
        }

        int islandId = body.IslandId;
        var island = _islands[islandId];

        // Fix the island's linked list of sims
        if ( body.islandPrev != PhysicsConstants.NullIndex )
        {
            var prevBody = _bodies[body.islandPrev].Comp;
            prevBody.islandNext = body.islandNext;
        }

        if ( body.islandNext != PhysicsConstants.NullIndex )
        {
            var nextBody = _bodies[body.islandNext].Comp;
            nextBody.islandPrev = body.islandPrev;
        }

        DebugTools.Assert( island.bodyCount > 0 );
        island.bodyCount -= 1;
        bool islandDestroyed = false;

        if ( island.headBody == body.Id )
        {
            island.headBody = body.islandNext;

            if ( island.headBody == PhysicsConstants.NullIndex )
            {
                // Destroy empty island
                DebugTools.Assert( island.tailBody == body.Id );
                DebugTools.Assert( island.bodyCount == 0 );
                DebugTools.Assert( island.contactCount == 0 );
                DebugTools.Assert( island.jointCount == 0 );

                // Free the island
                DestroyIsland( island.islandId );
                islandDestroyed = true;
            }
        }
        else if ( island.tailBody == body.Id )
        {
            island.tailBody = body.islandPrev;
        }

        if ( islandDestroyed == false )
        {
            ValidateIsland( islandId );
        }

        body.IslandId = PhysicsConstants.NullIndex;
        body.islandPrev = PhysicsConstants.NullIndex;
        body.islandNext = PhysicsConstants.NullIndex;
    }

    public void DestroyBodyContacts(PhysicsComponent body, bool wakeBodies)
    {
        var edgeKey = body.headContactKey;

        while (edgeKey != PhysicsConstants.NullIndex)
        {
            int contactId = edgeKey >> 1;
            int edgeIndex = edgeKey & 1;

            var contact = _contacts[contactId];
            edgeKey = contact.edges.AsSpan[edgeIndex].nextKey;
            DestroyContact( contact, wakeBodies );
        }

        ValidateSolverSets();
    }

    private void DestroyBody(Entity<PhysicsComponent> ent)
    {
        DebugTools.Assert(!_locked);

        if (_locked)
            return;

        // Ned to wake bodies attached to it.
        var wakeBodies = true;
        var body = ent.Comp;

        // Destroy the attached joints
	    int edgeKey = body.headJointKey;
	    while (edgeKey != PhysicsConstants.NullIndex)
	    {
		    int jointId = edgeKey >> 1;
		    int edgeIndex = edgeKey & 1;

		    var joint = _joints[jointId];
		    edgeKey = joint.Edges.AsSpan[edgeIndex].nextKey;

		    // Careful because this modifies the list being traversed
		    DestroyJointInternal(joint, wakeBodies);
	    }

	    // Destroy all contacts attached to this body.
	    DestroyBodyContacts(body, wakeBodies);

	    // Destroy the attached shapes and their broad-phase proxies.
	    int shapeId = body.headShapeId;
	    while ( shapeId != PhysicsConstants.NullIndex )
	    {
		    var shape = _shapes[shapeId];

		    if (shape.SensorIndex != PhysicsConstants.NullIndex)
		    {
			    DestroySensor(shape);
		    }

		    _broadphase.DestroyShapeProxy(shape);

		    // Return shape to free list.
            _shapeIdPool.FreeId(shapeId);
		    shape.Id = PhysicsConstants.NullIndex;

		    shapeId = shape.nextShapeId;
	    }

	    // Destroy the attached chains. The associated shapes have already been destroyed above.
	    int chainId = body.headChainId;
	    while ( chainId != PhysicsConstants.NullIndex )
	    {
		    var chain = _chainShapes[chainId];

		    FreeChainData(chain);

		    // Return chain to free list.
		    b2FreeId( &world->chainIdPool, chainId );
		    chain.id = PhysicsConstants.NullIndex;

		    chainId = chain.nextChainId;
	    }

	    RemoveBodyFromIsland(ent);

	    // Remove body sim from solver set that owns it
	    var set = _solverSets[body.SetIndex];
        var movedIndex = body.LocalIndex;
        set.bodySims.RemoveSwap(body.LocalIndex);

        if (movedIndex != PhysicsConstants.NullIndex)
	    {
		    // Fix moved body index
		    var movedBody = _bodies[body.LocalIndex];
		    DebugTools.Assert(movedBody.Comp.LocalIndex == movedIndex);
		    movedBody.Comp.LocalIndex = body.LocalIndex;
	    }

	    // Remove body state from awake set
	    if (body.SetIndex == (int) SetType.AwakeSet)
	    {
		    var result = set.bodyStates.RemoveSwap(body.LocalIndex);
		    DebugTools.Assert( result == movedIndex );
	    }
	    else if ( set.SetIndex >= (int) SetType.FirstSleepingSet && set.bodySims.Count == 0 )
	    {
		    // Remove solver set if it's now an orphan.
		    DestroySolverSet(set.setIndex);
	    }

	    // Free body and id (preserve body generation)
	    _bodyIdPool.FreeId(body.Id);

	    body.SetIndex = PhysicsConstants.NullIndex;
	    body.LocalIndex = PhysicsConstants.NullIndex;
	    body.Id = PhysicsConstants.NullIndex;

	    ValidateSolverSets();
    }

    private BodyId CreateBody()
    {
	    B2_ASSERT( b2IsValidVec2( def->position ) );
	    B2_ASSERT( b2IsValidRotation( def->rotation ) );
	    B2_ASSERT( b2IsValidVec2( def->linearVelocity ) );
	    B2_ASSERT( b2IsValidFloat( def->angularVelocity ) );
	    B2_ASSERT( b2IsValidFloat( def->linearDamping ) && def->linearDamping >= 0.0f );
	    B2_ASSERT( b2IsValidFloat( def->angularDamping ) && def->angularDamping >= 0.0f );
	    B2_ASSERT( b2IsValidFloat( def->sleepThreshold ) && def->sleepThreshold >= 0.0f );
	    B2_ASSERT( b2IsValidFloat( def->gravityScale ) );

	    b2World* world = b2GetWorldFromId( worldId );
	    B2_ASSERT( world->locked == false );

	    if ( world->locked )
	    {
		    return b2_nullBodyId;
	    }

	    bool isAwake = ( def->isAwake || def->enableSleep == false ) && def->isEnabled;

	    // determine the solver set
	    int setId;
	    if ( def->isEnabled == false )
	    {
		    // any body type can be disabled
		    setId = b2_disabledSet;
	    }
	    else if ( def->type == b2_staticBody )
	    {
		    setId = b2_staticSet;
	    }
	    else if ( isAwake == true )
	    {
		    setId = b2_awakeSet;
	    }
	    else
	    {
		    // new set for a sleeping body in its own island
		    setId = b2AllocId( &world->solverSetIdPool );
		    if ( setId == world->solverSets.count )
		    {
			    // Create a zero initialized solver set. All sub-arrays are also zero initialized.
			    b2SolverSetArray_Push( &world->solverSets, (b2SolverSet){ 0 } );
		    }
		    else
		    {
			    B2_ASSERT( world->solverSets.data[setId].setIndex == B2_NULL_INDEX );
		    }

		    world->solverSets.data[setId].setIndex = setId;
	    }

	    B2_ASSERT( 0 <= setId && setId < world->solverSets.count );

	    int bodyId = b2AllocId( &world->bodyIdPool );

	    uint32_t lockFlags = 0;
	    lockFlags |= def->motionLocks.linearX ? b2_lockLinearX : 0;
	    lockFlags |= def->motionLocks.linearY ? b2_lockLinearY : 0;
	    lockFlags |= def->motionLocks.angularZ ? b2_lockAngularZ : 0;

	    b2SolverSet* set = b2SolverSetArray_Get( &world->solverSets, setId );
	    b2BodySim* bodySim = b2BodySimArray_Add( &set->bodySims );
	    *bodySim = (b2BodySim){ 0 };
	    bodySim->transform.p = def->position;
	    bodySim->transform.q = def->rotation;
	    bodySim->center = def->position;
	    bodySim->rotation0 = bodySim->transform.q;
	    bodySim->center0 = bodySim->center;
	    bodySim->minExtent = B2_HUGE;
	    bodySim->maxExtent = 0.0f;
	    bodySim->linearDamping = def->linearDamping;
	    bodySim->angularDamping = def->angularDamping;
	    bodySim->gravityScale = def->gravityScale;
	    bodySim->bodyId = bodyId;
	    bodySim->flags = lockFlags;
	    bodySim->flags |= def->isBullet ? b2_isBullet : 0;
	    bodySim->flags |= def->allowFastRotation ? b2_allowFastRotation : 0;
	    bodySim->flags |= def->type == b2_dynamicBody ? b2_dynamicFlag : 0;

	    if ( setId == b2_awakeSet )
	    {
		    b2BodyState* bodyState = b2BodyStateArray_Add( &set->bodyStates );
		    B2_ASSERT( ( (uintptr_t)bodyState & 0x1F ) == 0 );

		    *bodyState = (b2BodyState){ 0 };
		    bodyState->linearVelocity = def->linearVelocity;
		    bodyState->angularVelocity = def->angularVelocity;
		    bodyState->deltaRotation = b2Rot_identity;
		    bodyState->flags = bodySim->flags;
	    }

	    if ( bodyId == world->bodies.count )
	    {
		    b2BodyArray_Push( &world->bodies, (b2Body){ 0 } );
	    }
	    else
	    {
		    B2_ASSERT( world->bodies.data[bodyId].id == B2_NULL_INDEX );
	    }

	    b2Body* body = b2BodyArray_Get( &world->bodies, bodyId );

	    if ( def->name )
	    {
		    int i = 0;
		    while ( i < B2_NAME_LENGTH - 1 && def->name[i] != 0 )
		    {
			    body->name[i] = def->name[i];
			    i += 1;
		    }

		    while ( i < B2_NAME_LENGTH )
		    {
			    body->name[i] = 0;
			    i += 1;
		    }
	    }
	    else
	    {
		    memset( body->name, 0, B2_NAME_LENGTH * sizeof( char ) );
	    }

	    body->userData = def->userData;
	    body->setIndex = setId;
	    body->localIndex = set->bodySims.count - 1;
	    body->generation += 1;
	    body->headShapeId = B2_NULL_INDEX;
	    body->shapeCount = 0;
	    body->headChainId = B2_NULL_INDEX;
	    body->headContactKey = B2_NULL_INDEX;
	    body->contactCount = 0;
	    body->headJointKey = B2_NULL_INDEX;
	    body->jointCount = 0;
	    body->islandId = B2_NULL_INDEX;
	    body->islandPrev = B2_NULL_INDEX;
	    body->islandNext = B2_NULL_INDEX;
	    body->bodyMoveIndex = B2_NULL_INDEX;
	    body->id = bodyId;
	    body->mass = 0.0f;
	    body->inertia = 0.0f;
	    body->sleepThreshold = def->sleepThreshold;
	    body->sleepTime = 0.0f;
	    body->type = def->type;
	    body->flags = bodySim->flags;
	    body->enableSleep = def->enableSleep;

	    // dynamic and kinematic bodies that are enabled need a island
	    if ( setId >= b2_awakeSet )
	    {
		    b2CreateIslandForBody( world, setId, body );
	    }

	    ValidateSolverSets();

	    b2BodyId id = { bodyId + 1, world->worldId, body->generation };
	    return id;
    }

    public bool WakeBody(Entity<PhysicsComponent> entity)
    {
        var body = entity.Comp;

        if ( body.SetIndex >= (int) SetType.FirstSleepingSet )
        {
            WakeSolverSet(body.SetIndex);
            ValidateSolverSets();
            return true;
        }

        return false;
    }
}
