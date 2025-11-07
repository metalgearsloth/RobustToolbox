using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using Robust.Shared.Collections;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Contacts;
using Robust.Shared.NewPhysics.Islands;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Events;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Threading;
using Robust.Shared.Utility;
using TerraFX.Interop.Windows;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private sealed class RebuildJob : IRobustJob
    {
        public SharedBroadphaseSystem Broadphase = default!;

        public void Execute()
        {
            Broadphase.Rebuild(false);
        }
    }

    private sealed class CollideJob : IParallelRobustJob
    {
        private NewPhysicsSystem _physics = default!;

        internal readonly List<ContactSim> ContactSims = new();

        // Store one per batch to avoid threading issues
        // These then get iterated sequentially later.
        internal readonly List<BitArray> ContactStateBits = new();

        public int BatchSize => 64;

        public void Execute(int contactIndex)
        {
            var batchIndex = contactIndex / BatchSize;
            var contactStateBit = ContactStateBits[batchIndex];

            var contactSim = ContactSims[contactIndex];

            int contactId = contactSim.contactId;

            var shapeA = contactSim.shapeA;
            var shapeB = contactSim.shapeB;

		    // Do proxies still overlap?
		    bool overlap = shapeA.fatAABB.Intersects(shapeB.fatAABB);
		    if (!overlap)
		    {
			    contactSim.simFlags |= ContactSimFlags.SimDisjoint;
			    contactSim.simFlags &= ~ContactSimFlags.SimTouchingFlag;
                contactStateBit.Set(contactId, true);
		    }
		    else
		    {
			    bool wasTouching = ( contactSim.simFlags & ContactSimFlags.SimTouchingFlag ) == ContactSimFlags.SimTouchingFlag;

			    // Update contact respecting shape/body order (A,B)
			    var bodyA = shapeA.Body;
			    var bodyB = shapeB.Body;
			    ref var bodySimA = ref _physics.GetBodySim(bodyA);
			    ref var bodySimB = ref _physics.GetBodySim(bodyB);

			    // avoid cache misses in b2PrepareContactsTask
			    contactSim.bodySimIndexA = bodyA.SetIndex == (int) SetType.AwakeSet ? bodyA.LocalIndex : PhysicsConstants.NullIndex;
			    contactSim.invMassA = bodySimA.invMass;
			    contactSim.invIA = bodySimA.invInertia;

			    contactSim.bodySimIndexB = bodyB.SetIndex == (int) SetType.AwakeSet ? bodyB.LocalIndex : PhysicsConstants.NullIndex;
			    contactSim.invMassB = bodySimB.invMass;
			    contactSim.invIB = bodySimB.invInertia;

			    var transformA = bodySimA.transform;
			    var transformB = bodySimB.transform;

                var centerOffsetA = Quaternion2D.RotateVector(transformA.Quaternion2D, bodySimA.localCenter);
                var centerOffsetB = Quaternion2D.RotateVector(transformB.Quaternion2D, bodySimB.localCenter);

			    // This updates solid contacts
			    bool touching =
				    _physics.UpdateContact(contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB, centerOffsetB );

			    // State changes that affect island connectivity. Also affects contact events.
			    if ( touching == true && wasTouching == false )
			    {
				    contactSim.simFlags |= ContactSimFlags.SimStartedTouching;
				    contactStateBit.Set(contactId, true);
			    }
			    else if ( touching == false && wasTouching == true )
			    {
				    contactSim.simFlags |= ContactSimFlags.SimStoppedTouching;
                    contactStateBit.Set(contactId, true);
			    }

			    // To make this work, the time of impact code needs to adjust the target
			    // distance based on the number of TOI events for a body.
			    // if (touching && bodySimB.isFast)
			    //{
			    //	b2Manifold* manifold = &contactSim.manifold;
			    //	int pointCount = manifold.pointCount;
			    //	for (int i = 0; i < pointCount; ++i)
			    //	{
			    //		// trick the solver into pushing the fast shapes apart
			    //		manifold.points[i].separation -= 0.25f * B2_SPECULATIVE_DISTANCE;
			    //	}
			    //}
		    }
        }
    }

    internal sealed class ConstraintGraph
    {
        // including overflow at the end
        public GraphColor[] colors = new GraphColor[PhysicsConstants.GraphColorCount];
    }

    internal sealed class GraphColor
    {
        // This bitset is indexed by bodyId so this is over-sized to encompass static bodies
        // however I never traverse these bits or use the bit count for anything
        // This bitset is unused on the overflow color.
        //
        // Dirk suggested having a uint64_t per body that tracks the graph color membership
        // but I think this would make debugging harder and be less flexible. With the bitset
        // I can trivially increase the number of graph colors beyond 64. See usage of b2CountSetBits
        // for validation.
        public BitArray BodySet = new(64);

        // cache friendly arrays
        public ValueList<ContactSim> ContactSims = new();
        public ValueList<JointSim> JointSims = new();

        // transient
        // Box2D uses a pointer here that references the context constraint but we won't mess with that.
        /// <summary>
        /// Stores the index of the SIMD constraints for this graph color in the step context.
        /// </summary>
        public int SimdConstraintIndex;

        public int SimdConstraintCount;

        // box2d uses a union for this but we don't have that luxury unless we start being unsafe with it.
        public ValueList<ContactConstraint> OverflowConstraints = new();
    }

    private ulong B2_SHAPE_PAIR_KEY(int K1, int K2)
    {
        return K1 < K2 ? (ulong) K1 << 32 | (uint)K2 : (ulong)K2 << 32 | (uint)K1;
    }

    private ref ContactSim GetContactSim(b2Contact contact)
    {
        if (contact.setIndex == (int) SetType.AwakeSet && contact.colorIndex != PhysicsConstants.NullIndex)
        {
            // contact lives in constraint graph
            DebugTools.Assert(0 <= contact.colorIndex && contact.colorIndex < PhysicsConstants.GraphColorCount);
            var color = _constraintGraph.colors[contact.colorIndex];
            ref var sims = ref color.ContactSims;

            ref var sim = ref sims[contact.localIndex];
            return ref sim;
        }

        var set = _solverSets[contact.setIndex];
        ref var setSims = ref set.contactSims;
        ref var setSim = ref setSims[contact.localIndex];

        return ref setSim;
    }

    private void Collide()
    {
        // Task that can be done in parallel with the narrow-phase
	    // - rebuild the collision tree for dynamic and kinematic bodies to keep their query performance good
	    // todo_erin move this to start when contacts are being created

        // This gets waited under the solve step once required.
        _rebuildHandle = _parallel.Process(_rebuildJob);

	    // gather contacts into a single array for easier parallel-for
	    int contactCount = 0;
	    var graphColors = _constraintGraph.colors;

	    for ( int i = 0; i < PhysicsConstants.GraphColorCount; ++i )
	    {
		    contactCount += graphColors[i].ContactSims.Count;
	    }

	    int nonTouchingCount = _solverSets[(int) SetType.AwakeSet].contactSims.Count;
	    contactCount += nonTouchingCount;

	    if (contactCount == 0)
	    {
		    return;
	    }

        var contactSims = _collideJob.ContactSims;
        contactSims.Clear();

	    for (int i = 0; i < PhysicsConstants.GraphColorCount; ++i )
        {
            var color = graphColors[i];

            contactSims.AddRange(color.ContactSims);
	    }

	    {
		    var based = _solverSets[(int) SetType.AwakeSet].contactSims;
            contactSims.AddRange(based);
	    }

	    DebugTools.Assert(contactSims.Count == contactCount);

	    // Contact bit set on ids because contact pointers are unstable as they move between touching and not touching.
	    int contactIdCapacity = _contactIdPool.Capacity;

        // We do this inline so we can get the bitset back from each job and combine them at the end.
        var batches = (contactCount / _collideJob.BatchSize) + 1;

        for (var i = _collideJob.ContactStateBits.Count; i < batches; i++)
        {
            _collideJob.ContactStateBits.Add(new BitArray(_collideJob.BatchSize));
        }

        _parallel.ProcessNow(_collideJob, contactCount);

	    // Serially update contact state
	    // todo_erin bring this zone together with island merge
	    int endEventArrayIndex = _endEventArrayIndex;
        ref var awakeSet = ref _solverSets[(int)SetType.AwakeSet];

        var shapes = _shapes;

        // For some reason box2d combines these first but seems easier to just iterate over them.
        for (var i = 0; i < batches; i++)
        {
            var bitset = _collideJob.ContactStateBits[i];
            for (var j = 0; j < _collideJob.BatchSize; j++)
            {
                var bit = bitset.Get(j);

                if (bit)
                {
                    var contactId = j * _collideJob.BatchSize + j;

                    var contact = _contacts[contactId];
			        DebugTools.Assert(contact.setIndex == (int) SetType.AwakeSet);

			        int colorIndex = contact.colorIndex;
			        int localIndex = contact.localIndex;

			        ContactSim contactSim;

			        if (colorIndex != PhysicsConstants.NullIndex)
			        {
				        // contact lives in constraint graph
				        DebugTools.Assert( 0 <= colorIndex && colorIndex < PhysicsConstants.GraphColorCount );
				        var color = graphColors[colorIndex];
				        contactSim = color.ContactSims[localIndex];
			        }
			        else
			        {
				        contactSim = awakeSet.contactSims[localIndex];
			        }

			        var shapeA = shapes[contact.shapeIdA];
			        var shapeB = shapes[contact.shapeIdB];

                    var flags = contact.flags;
                    var simFlags = contactSim.simFlags;

			        if ( (simFlags & ContactSimFlags.SimDisjoint) == ContactSimFlags.SimDisjoint )
			        {
				        // Bounding boxes no longer overlap
				        DestroyContact(contact, false );
			        }
			        else if ((simFlags & ContactSimFlags.SimStartedTouching ) == ContactSimFlags.SimStartedTouching)
			        {
				        DebugTools.Assert(contact.islandId == PhysicsConstants.NullIndex);

				        if ( (flags & ContactFlags.ContactEnableContactEvents) == ContactFlags.ContactEnableContactEvents )
				        {
                            // TODO: Startcollideevent
                            var ev = new ContactBeginTouchEvent();
                            _contactBeginEvents.Add(ev);
                        }

				        DebugTools.Assert(contactSim.manifold.pointCount > 0 );
				        DebugTools.Assert(contact.setIndex == (int) SetType.AwakeSet);

				        // Link first because this wakes colliding bodies and ensures the body sims
				        // are in the correct place.
				        contact.flags |= ContactFlags.ContactTouchingFlag;
				        LinkContact(contact);

				        // Make sure these didn't change
				        DebugTools.Assert(contact.colorIndex == PhysicsConstants.NullIndex);
				        DebugTools.Assert(contact.localIndex == localIndex);

				        // Contact sim pointer may have become orphaned due to awake set growth,
				        // so I just need to refresh it.
				        contactSim = awakeSet.contactSims[localIndex];

				        contactSim.simFlags &= ~ContactSimFlags.SimStartedTouching;

				        AddContactToGraph(ref contactSim, contact );
				        RemoveNonTouchingContact((int) SetType.AwakeSet, localIndex);
			        }
			        else if ( (simFlags & ContactSimFlags.SimStoppedTouching) == ContactSimFlags.SimStoppedTouching )
			        {
				        contactSim.simFlags &= ~ContactSimFlags.SimStoppedTouching;
				        contact.flags &= ~ContactFlags.ContactTouchingFlag;

				        if ( (contact.flags & ContactFlags.ContactEnableContactEvents) == ContactFlags.ContactEnableContactEvents )
				        {
                            // TODO: End collide event
                            var ev = new ContactEndTouchEvent();
                            _contactEndEvents[endEventArrayIndex].Add(ev);
                        }

				        DebugTools.Assert(contactSim.manifold.pointCount == 0 );

				        UnlinkContact(contact);
				        int bodyIdA = contact.edges._00.bodyId;
				        int bodyIdB = contact.edges._01.bodyId;

				        AddNonTouchingContact(contact, ref contactSim);
				        RemoveContactFromGraph(bodyIdA, bodyIdB, colorIndex, localIndex);
			        }
                }
            }

            // Clear data
            bitset.SetAll(false);
        }

	    ValidateSolverSets();
	    ValidateContacts();
    }

    private bool UpdateContact(ContactSim contactSim,
        Fixture fixtureA, Transform transformA, Vector2 centerOffsetA,
        Fixture fixtureB, Transform transformB, Vector2 centerOffsetB)
    {
        // Update the contact manifold and touching status.
        // Note: do not assume the shape AABBs are overlapping or are valid.
	    // Save old manifold
	    var oldManifold = contactSim.manifold;

	    // Compute new manifold
	    contactSim.manifold = GetManifold(fixtureA.Shape, transformA, fixtureB.Shape, transformB, contactSim.cache);

	    // Keep these updated in case the values on the shapes are modified
	    contactSim.friction = DefaultFrictionCallback(fixtureA.Material.Friction, fixtureA.Material.UserMaterialId,
													    fixtureB.Material.Friction, fixtureB.Material.UserMaterialId);

        contactSim.restitution = DefaultRestitutionCallback(fixtureA.Material.Restitution, fixtureA.Material.UserMaterialId,
														      fixtureB.Material.Restitution, fixtureB.Material.UserMaterialId);

	    if (fixtureA.Material.RollingResistance > 0.0f || fixtureB.Material.RollingResistance > 0.0f )
	    {
		    float radiusA = fixtureA.Shape.Radius;
		    float radiusB = fixtureB.Shape.Radius;
		    float maxRadius = MathF.Max(radiusA, radiusB);
		    contactSim.rollingResistance =
			    MathF.Max(fixtureA.Material.RollingResistance, fixtureB.Material.RollingResistance) * maxRadius;
	    }
	    else
	    {
		    contactSim.rollingResistance = 0.0f;
	    }

	    contactSim.tangentSpeed = fixtureA.Material.TangentSpeed + fixtureB.Material.TangentSpeed;

	    int pointCount = contactSim.manifold.pointCount;
	    bool touching = pointCount > 0;

	    if ( touching && ( contactSim.simFlags & ContactSimFlags.SimEnablePreSolveEvents ) != 0 )
        {
            var shapeIdA = fixtureA.Id + 1;
            var shapeIdB = fixtureB.Id + 1;

		    ref var manifold = ref contactSim.manifold;
		    float bestSeparation = manifold.points._00.separation;
		    Vector2 bestPoint = manifold.points._00.point;
            var pointSpan = manifold.points.AsSpan;

		    // Get deepest point
		    for ( int i = 1; i < manifold.pointCount; ++i )
		    {
			    float separation = pointSpan[i].separation;
			    if ( separation < bestSeparation )
			    {
				    bestSeparation = separation;
				    bestPoint = pointSpan[i].point;
			    }
		    }

            // TODO: Pre-Solve callback
	    }

	    // This flag is for testing
	    if (_enableSpeculative == false && pointCount == 2 )
	    {
		    if ( contactSim.manifold.points._00.separation > 1.5f * PhysicsConstants.LinearSlop )
		    {
			    contactSim.manifold.points._00 = contactSim.manifold.points._01;
			    contactSim.manifold.pointCount = 1;
		    }
		    else if ( contactSim.manifold.points._00.separation > 1.5f * PhysicsConstants.LinearSlop )
		    {
			    contactSim.manifold.pointCount = 1;
		    }

		    pointCount = contactSim.manifold.pointCount;
	    }

	    if ( touching && ( fixtureA.EnableHitEvents || fixtureB.EnableHitEvents ) )
	    {
		    contactSim.simFlags |= ContactSimFlags.SimEnableHitEvent;
	    }
	    else
	    {
		    contactSim.simFlags &= ~ContactSimFlags.SimEnableHitEvent;
	    }

	    if ( pointCount > 0 )
	    {
		    contactSim.manifold.rollingImpulse = oldManifold.rollingImpulse;
	    }

	    // Match old contact ids to new contact ids and copy the
	    // stored impulses to warm start the solver.
	    int unmatchedCount = 0;

	    for ( int i = 0; i < pointCount; ++i )
	    {
		    ref var mp2 = ref contactSim.manifold.points.AsSpan[i];

		    // shift anchors to be center of mass relative
		    mp2.anchorA = mp2.anchorA - centerOffsetA;
		    mp2.anchorB =  mp2.anchorB - centerOffsetB;

		    mp2.normalImpulse = 0.0f;
		    mp2.tangentImpulse = 0.0f;
		    mp2.totalNormalImpulse = 0.0f;
		    mp2.normalVelocity = 0.0f;
		    mp2.persisted = false;

		    var id2 = mp2.id;

		    for ( int j = 0; j < oldManifold.pointCount; ++j )
		    {
			    ref var mp1 = ref oldManifold.points.AsSpan[j];

			    if (mp1.id == id2 )
			    {
				    mp2.normalImpulse = mp1.normalImpulse;
				    mp2.tangentImpulse = mp1.tangentImpulse;
				    mp2.persisted = true;

				    // clear old impulse
				    mp1.normalImpulse = 0.0f;
				    mp1.tangentImpulse = 0.0f;
				    break;
			    }
		    }

		    unmatchedCount += mp2.persisted ? 0 : 1;
	    }

	    if ( touching )
	    {
		    contactSim.simFlags |= ContactSimFlags.SimTouchingFlag;
	    }
	    else
	    {
		    contactSim.simFlags &= ~ContactSimFlags.SimTouchingFlag;
	    }

	    return touching;
    }

    private void RemoveKey(ulong pairKey)
    {
        _pairSet.Remove(pairKey);
    }

    // A contact is destroyed when:
    // - broad-phase proxies stop overlapping
    // - a body is destroyed
    // - a body is disabled
    // - a body changes type from dynamic to kinematic or static
    // - a shape is destroyed
    // - contact filtering is modified
    private void DestroyContact(b2Contact contact, bool wakeBodies)
    {
	    // Remove pair from set
	    var pairKey = B2_SHAPE_PAIR_KEY(contact.shapeIdA, contact.shapeIdB );
	    RemoveKey(pairKey);

	    b2ContactEdge edgeA = contact.edges._00;
	    b2ContactEdge edgeB = contact.edges._01;

	    int bodyIdA = edgeA.bodyId;
	    int bodyIdB = edgeB.bodyId;
        var entA = _bodies[bodyIdA];
        var entB = _bodies[bodyIdB];
        var bodyA = _bodies[bodyIdA].Comp;
        var bodyB = _bodies[bodyIdB].Comp;

	    var flags = contact.flags;
	    bool touching = (flags & ContactFlags.ContactTouchingFlag) != 0;

	    // End touch event
	    if (touching && ( flags & ContactFlags.ContactEnableContactEvents) != 0 )
	    {
            _contactEndEvents[_endEventArrayIndex].Add(new ContactEndTouchEvent());
	    }

	    // Remove from body A
	    if ( edgeA.prevKey != PhysicsConstants.NullIndex )
        {
            var prevContact = _contacts[edgeA.prevKey >> 1];
		    ref var prevEdge = ref prevContact.edges._00;
		    prevEdge.nextKey = edgeA.nextKey;
	    }

	    if ( edgeA.nextKey != PhysicsConstants.NullIndex )
	    {
		    var nextContact = _contacts[edgeA.nextKey >> 1];
		    ref var nextEdge = ref nextContact.edges._01;
		    nextEdge.prevKey = edgeA.prevKey;
	    }

	    int contactId = contact.contactId;

	    int edgeKeyA = ( contactId << 1 ) | 0;
	    if ( bodyA.headContactKey == edgeKeyA )
	    {
		    bodyA.headContactKey = edgeA.nextKey;
	    }

	    bodyA.ContactCount -= 1;

	    // Remove from body B
	    if ( edgeB.prevKey != PhysicsConstants.NullIndex )
	    {
		    var prevContact = _contacts[edgeB.prevKey >> 1];
		    ref var prevEdge = ref prevContact.edges._00;
		    prevEdge.nextKey = edgeB.nextKey;
	    }

	    if ( edgeB.nextKey != PhysicsConstants.NullIndex )
	    {
		    var nextContact = _contacts[edgeB.nextKey >> 1];
		    ref var nextEdge = ref nextContact.edges._01;
		    nextEdge.prevKey = edgeB.prevKey;
	    }

	    int edgeKeyB = ( contactId << 1 ) | 1;
	    if ( bodyB.headContactKey == edgeKeyB )
	    {
		    bodyB.headContactKey = edgeB.nextKey;
	    }

	    bodyB.ContactCount -= 1;

	    // Remove contact from the array that owns it
	    if ( contact.islandId != PhysicsConstants.NullIndex )
	    {
		    UnlinkContact(contact);
	    }

	    if ( contact.colorIndex != PhysicsConstants.NullIndex )
	    {
		    // contact is an active constraint
		    DebugTools.Assert( contact.setIndex == (int) SetType.AwakeSet );
		    RemoveContactFromGraph( bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex );
	    }
	    else
	    {
		    // contact is non-touching or is sleeping
		    DebugTools.Assert( contact.setIndex != (int) SetType.AwakeSet || ( contact.flags & ContactFlags.ContactTouchingFlag ) == 0 );
		    var set = _solverSets[contact.setIndex];

		    var movedContactSim = set.contactSims.RemoveSwap(contact.localIndex);
            var movedIndex = set.contactSims.Count;

		    if ( movedIndex != PhysicsConstants.NullIndex )
		    {
			    var movedContact = _contacts[movedContactSim.contactId];
			    movedContact.localIndex = contact.localIndex;
                _contactSimPool.Return(movedContactSim);
		    }
	    }

	    // Free contact and id (preserve generation)
	    contact.contactId = PhysicsConstants.NullIndex;
	    contact.setIndex = PhysicsConstants.NullIndex;
	    contact.colorIndex = PhysicsConstants.NullIndex;
	    contact.localIndex = PhysicsConstants.NullIndex;
	    _contactIdPool.FreeId(contactId);

	    if ( wakeBodies && touching )
	    {
		    WakeBody(entA);
		    WakeBody(entB);
	    }
    }

    // Link a contact into an island.
    private void LinkContact(b2Contact contact)
    {
	    DebugTools.Assert((contact.flags & ContactFlags.ContactTouchingFlag) != 0);

	    int bodyIdA = contact.edges._00.bodyId;
	    int bodyIdB = contact.edges._01.bodyId;

	    var entA = _bodies[bodyIdA];
	    var entB = _bodies[bodyIdB];

        var bodyA = entA.Comp;
        var bodyB = entB.Comp;

	    DebugTools.Assert(bodyA.SetIndex != (int) SetType.DisabledSet && bodyB.SetIndex != (int) SetType.DisabledSet );
	    DebugTools.Assert(bodyA.SetIndex != (int) SetType.StaticSet || bodyB.SetIndex != (int) SetType.StaticSet );

	    // Wake bodyB if bodyA is awake and bodyB is sleeping
	    if (bodyA.SetIndex == (int) SetType.AwakeSet && bodyB.SetIndex >= (int) SetType.FirstSleepingSet)
	    {
		    WakeSolverSet(bodyB.SetIndex);
	    }

	    // Wake bodyA if bodyB is awake and bodyA is sleeping
	    if (bodyB.SetIndex == (int) SetType.AwakeSet && bodyA.SetIndex >= (int) SetType.FirstSleepingSet)
	    {
		    WakeSolverSet(bodyA.SetIndex);
	    }

	    int islandIdA = bodyA.IslandId;
	    int islandIdB = bodyB.IslandId;

	    // Static bodies have null island indices.
	    DebugTools.Assert( bodyA.SetIndex != (int) SetType.StaticSet || islandIdA == PhysicsConstants.NullIndex );
        DebugTools.Assert( bodyB.SetIndex != (int) SetType.StaticSet || islandIdB == PhysicsConstants.NullIndex );
        DebugTools.Assert( islandIdA != PhysicsConstants.NullIndex || islandIdB != PhysicsConstants.NullIndex );

	    // Merge islands. This will destroy one of the islands.
	    int finalIslandId = MergeIslands( islandIdA, islandIdB );

	    // Add contact to the island that survived
	    AddContactToIsland( finalIslandId, contact );
    }

    // This is called when a contact no longer has contact points or when a contact is destroyed.
    private void UnlinkContact(b2Contact contact)
    {
	    DebugTools.Assert(contact.islandId != PhysicsConstants.NullIndex);

	    // remove from island
	    int islandId = contact.islandId;
        var island = _islands[islandId];

	    if (contact.islandPrev != PhysicsConstants.NullIndex)
	    {
		    var prevContact = _contacts[contact.islandPrev];
		    DebugTools.Assert(prevContact.islandNext == contact.contactId);
		    prevContact.islandNext = contact.islandNext;
	    }

	    if (contact.islandNext != PhysicsConstants.NullIndex)
	    {
		    var nextContact = _contacts[contact.islandNext];
		    DebugTools.Assert(nextContact.islandPrev == contact.contactId);
		    nextContact.islandPrev = contact.islandPrev;
	    }

	    if ( island.headContact == contact.contactId )
	    {
		    island.headContact = contact.islandNext;
	    }

	    if ( island.tailContact == contact.contactId )
	    {
		    island.tailContact = contact.islandPrev;
	    }

	    DebugTools.Assert(island.contactCount > 0);
	    island.contactCount -= 1;
	    island.constraintRemoveCount += 1;

	    contact.islandId = PhysicsConstants.NullIndex;
	    contact.islandPrev = PhysicsConstants.NullIndex;
	    contact.islandNext = PhysicsConstants.NullIndex;

	    ValidateIsland(islandId);
    }

    // Contacts are always created as non-touching. They get cloned into the constraint
    // graph once they are found to be touching.
    private void AddContactToGraph(ref ContactSim contactSim, b2Contact contact)
    {
	    DebugTools.Assert(contactSim.manifold.pointCount > 0 );
        DebugTools.Assert((contactSim.simFlags & ContactSimFlags.SimTouchingFlag) == ContactSimFlags.SimTouchingFlag);
        DebugTools.Assert((contact.flags & ContactFlags.ContactTouchingFlag) == ContactFlags.ContactTouchingFlag);

	    var graph = _constraintGraph;
	    int colorIndex = PhysicsConstants.OverflowIndex;

	    var bodyIdA = contact.edges._00.bodyId;
	    var bodyIdB = contact.edges._01.bodyId;

        var entA = _bodies[bodyIdA];
        var entB = _bodies[bodyIdB];
        var bodyA = entA.Comp;
        var bodyB = entB.Comp;

	    var typeA = bodyA.BodyType;
	    var typeB = bodyB.BodyType;
	    DebugTools.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

	    if (typeA != BodyType.Static && typeB != BodyType.Static)
	    {
		    // Dynamic constraint colors cannot encroach on colors reserved for static constraints
		    for ( int i = 0; i < PhysicsConstants.DynamicColorCount; ++i )
		    {
			    var color = graph.colors[i];

			    if (color.BodySet.Get(bodyIdA) || color.BodySet.Get(bodyIdB))
			    {
				    continue;
			    }

			    if (typeA == BodyType.Dynamic)
			    {
                    Extensions.SetGrow(ref color.BodySet, bodyIdA, true);
			    }

			    if (typeB == BodyType.Dynamic)
			    {
                    Extensions.SetGrow(ref color.BodySet, bodyIdB, true);
			    }

			    colorIndex = i;
			    break;
		    }
	    }
	    else if (typeA == BodyType.Dynamic)
	    {
		    // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		    for ( int i = PhysicsConstants.OverflowIndex - 1; i >= 1; --i )
		    {
			    var color = graph.colors[i];

			    if (color.BodySet.Get(bodyIdA))
			    {
				    continue;
			    }

                Extensions.SetGrow(ref color.BodySet, bodyIdA, true);
			    colorIndex = i;
			    break;
		    }
	    }
	    else if (typeB == BodyType.Dynamic)
	    {
		    // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
		    for ( int i = PhysicsConstants.OverflowIndex - 1; i >= 1; --i )
		    {
			    var color = graph.colors[i];

                if (color.BodySet.Get(bodyIdB))
			    {
				    continue;
			    }

                Extensions.SetGrow(ref color.BodySet, bodyIdB, true);
			    colorIndex = i;
			    break;
		    }
	    }

	    var newColor = graph.colors[colorIndex];
	    contact.colorIndex = colorIndex;
	    contact.localIndex = newColor.ContactSims.Count;

        var newContact = _contactSimPool.Get();
        newColor.ContactSims.Add(newContact);

	    // todo perhaps skip this if the contact is already awake

	    if (typeA == BodyType.Static)
	    {
		    newContact.bodySimIndexA = PhysicsConstants.NullIndex;
		    newContact.invMassA = 0.0f;
		    newContact.invIA = 0.0f;
	    }
	    else
	    {
		    DebugTools.Assert(bodyA.SetIndex == (int) SetType.AwakeSet);
            var awakeSet = _solverSets[(int)SetType.AwakeSet];
            ref var awakeSims = ref awakeSet.bodySims;

		    int localIndex = bodyA.LocalIndex;
		    newContact.bodySimIndexA = localIndex;

		    ref var bodySimA = ref awakeSims[localIndex];
		    newContact.invMassA = bodySimA.invMass;
		    newContact.invIA = bodySimA.invInertia;
	    }

	    if (typeB == BodyType.Static)
	    {
		    newContact.bodySimIndexB = PhysicsConstants.NullIndex;
		    newContact.invMassB = 0.0f;
		    newContact.invIB = 0.0f;
	    }
	    else
	    {
		    DebugTools.Assert(bodyB.SetIndex == (int) SetType.AwakeSet);
            var awakeSet = _solverSets[(int)SetType.AwakeSet];
            ref var awakeSims = ref awakeSet.bodySims;

		    int localIndex = bodyB.LocalIndex;
		    newContact.bodySimIndexB = localIndex;

		    ref var bodySimB = ref awakeSims[localIndex];
		    newContact.invMassB = bodySimB.invMass;
		    newContact.invIB = bodySimB.invInertia;
	    }
    }

    private void RemoveContactFromGraph(int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
    {
	    var graph = _constraintGraph;

	    DebugTools.Assert(0 <= colorIndex && colorIndex < PhysicsConstants.GraphColorCount);
	    var color = graph.colors[colorIndex];

	    if (colorIndex != PhysicsConstants.OverflowIndex)
	    {
		    // This might clear a bit for a kinematic or static body, but this has no effect
            color.BodySet.Set(bodyIdA, false);
            color.BodySet.Set(bodyIdB, false);
	    }

	    color.ContactSims.RemoveSwap(localIndex);
        var movedIndex = color.ContactSims.Count;

        if (movedIndex <= 0)
            return;

        var sims = color.ContactSims;

        // Fix index on swapped contact
        ref var movedContactSim = ref sims[localIndex];

        // Fix moved contact
        int movedId = movedContactSim.contactId;
        var movedContact = _contacts[movedId];
        DebugTools.Assert(movedContact.setIndex == (int) SetType.AwakeSet);
        DebugTools.Assert(movedContact.colorIndex == colorIndex);
        DebugTools.Assert(movedContact.localIndex == movedIndex);
        movedContact.localIndex = localIndex;
    }

    private void AddNonTouchingContact(b2Contact contact, ref ContactSim contactSim)
    {
        DebugTools.Assert(contact.setIndex == (int) SetType.AwakeSet);
        var set = _solverSets[(int) SetType.AwakeSet];
        contact.colorIndex = PhysicsConstants.NullIndex;
        contact.localIndex = set.contactSims.Count;

        set.contactSims.Add(contactSim);
    }

    private void RemoveNonTouchingContact(int setIndex, int localIndex)
    {
        var set = _solverSets[setIndex];
        var removedSim = set.contactSims.RemoveSwap(localIndex);
        var movedIndex = set.contactSims.Count;
        _contactSimPool.Return(removedSim);

        if (movedIndex <= 0)
            return;

        ref var sims = ref set.contactSims;
        ref var movedContactSim = ref sims[localIndex];
        var movedContact = _contacts[movedContactSim.contactId];
        DebugTools.Assert(movedContact.setIndex == setIndex);
        DebugTools.Assert(movedContact.localIndex == movedIndex);
        DebugTools.Assert(movedContact.colorIndex == PhysicsConstants.NullIndex);
        movedContact.localIndex = localIndex;
    }

    private static float DefaultFrictionCallback( float frictionA, ulong materialA, float frictionB, ulong materialB )
    {
        return MathF.Sqrt( frictionA * frictionB );
    }

    private static float DefaultRestitutionCallback( float restitutionA, ulong materialA, float restitutionB, ulong materialB )
    {
        return MathF.Max( restitutionA, restitutionB );
    }
}
