using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using Robust.Shared.NewPhysics.Bodies;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

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
			    // if (touching && bodySimB->isFast)
			    //{
			    //	b2Manifold* manifold = &contactSim->manifold;
			    //	int pointCount = manifold->pointCount;
			    //	for (int i = 0; i < pointCount; ++i)
			    //	{
			    //		// trick the solver into pushing the fast shapes apart
			    //		manifold->points[i].separation -= 0.25f * B2_SPECULATIVE_DISTANCE;
			    //	}
			    //}
		    }
        }
    }

    private ref BodySim GetBodySim(PhysicsComponent body)
    {
        var set = _solverSets[body.SetIndex];
        return ref CollectionsMarshal.AsSpan(set.bodySims)[body.LocalIndex];
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
        public List<ContactSim> ContactSims = new();
        public List<JointSim> JointSims = new();

        // transient
        public List<ContactConstraint> _overflowConstraints = new();
    }

    private void Collide(StepContext context)
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

        // TODO: Bitset pool
        // Make them size 64 and then for each job allocate it, then process them all

        // We do this inline so we can get the bitset back from each job and combine them at the end.
        var batches = (contactCount / _collideJob.BatchSize) + 1;

        for (var i = _collideJob.ContactStateBits.Count; i < batches; i++)
        {
            _collideJob.ContactStateBits.Add(new BitArray(_collideJob.BatchSize));
        }

        _parallel.ProcessNow(_collideJob, contactCount);

	    // Serially update contact state
	    // todo_erin bring this zone together with island merge
        var awakeSet = _solverSets[(int)SetType.AwakeSet];

	    int endEventArrayIndex = _endEventArrayIndex;

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
			        DebugTools.Assert(contact.setIndex == SetType.AwakeSet);

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

                    var shapeIdA = shapeA.Id + 1;
                    var shapeIdB = shapeB.Id + 1;

                    var contactFullId = contactId + 1;

                    var flags = contact.flags;
                    var simFlags = contactSim.simFlags;

			        if ( (simFlags & ContactSimFlags.SimDisjoint) == ContactSimFlags.SimDisjoint )
			        {
				        // Bounding boxes no longer overlap
				        b2DestroyContact(contact, false );
				        contact = NULL;
				        contactSim = NULL;
			        }
			        else if ((simFlags & ContactSimFlags.SimStartedTouching ) == ContactSimFlags.SimStartedTouching)
			        {
				        DebugTools.Assert(contact.islandId == PhysicsConstants.NullIndex);

				        if ( flags & b2_contactEnableContactEvents )
				        {
                            // TODO: Startcollideevent
				        }

				        DebugTools.Assert(contactSim.manifold.pointCount > 0 );
				        DebugTools.Assert(contact.setIndex == SetType.AwakeSet );

				        // Link first because this wakes colliding bodies and ensures the body sims
				        // are in the correct place.
				        contact.flags |= ContactFlags.ContactTouchingFlag;
				        b2LinkContact( world, contact );

				        // Make sure these didn't change
				        DebugTools.Assert( contact.colorIndex == PhysicsConstants.NullIndex );
				        DebugTools.Assert( contact.localIndex == localIndex );

				        // Contact sim pointer may have become orphaned due to awake set growth,
				        // so I just need to refresh it.
				        contactSim = awakeSet.contactSims[localIndex];

				        contactSim.simFlags &= ~ContactSimFlags.SimStartedTouching;

				        b2AddContactToGraph( world, contactSim, contact );
				        b2RemoveNonTouchingContact( world, b2_awakeSet, localIndex );
				        contactSim = NULL;
			        }
			        else if ( (simFlags & ContactSimFlags.SimStoppedTouching) == ContactSimFlags.SimStoppedTouching )
			        {
				        contactSim.simFlags &= ~ContactSimFlags.SimStoppedTouching;
				        contact.flags &= ~ContactFlags.ContactTouchingFlag;

				        if ( (contact.flags & ContactFlags.ContactEnableContactEvents) == ContactFlags.ContactEnableContactEvents )
				        {
                            // TODO: End collide event

				        }

				        DebugTools.Assert(contactSim.manifold.pointCount == 0 );

				        b2UnlinkContact( world, contact );
				        int bodyIdA = contact.edges[0].bodyId;
				        int bodyIdB = contact.edges[1].bodyId;

				        b2AddNonTouchingContact( world, contact, contactSim );
				        b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
				        contact = NULL;
				        contactSim = NULL;
			        }
                }
            }

            // Clear data
            bitset.SetAll(false);
        }

	    b2ValidateSolverSets( world );
	    b2ValidateContacts( world );
    }


    private bool UpdateContact(ContactSim contactSim,
        Fixture shapeA, Transform transformA, Vector2 centerOffsetA,
        Fixture shapeB, Transform transformB, Vector2 centerOffsetB )
    {
        // Update the contact manifold and touching status.
        // Note: do not assume the shape AABBs are overlapping or are valid.
	    // Save old manifold
	    var oldManifold = contactSim.manifold;

	    // Compute new manifold
	    b2ManifoldFcn* fcn = s_registers[shapeA->type][shapeB->type].fcn;
	    contactSim.manifold = fcn( shapeA, transformA, shapeB, transformB, &contactSim->cache );

	    // Keep these updated in case the values on the shapes are modified
	    contactSim.friction = world->frictionCallback( shapeA->material.friction, shapeA->material.userMaterialId,
													    shapeB->material.friction, shapeB->material.userMaterialId );
	    contactSim.restitution = world->restitutionCallback( shapeA->material.restitution, shapeA->material.userMaterialId,
														      shapeB->material.restitution, shapeB->material.userMaterialId );

	    if (shapeA->material.rollingResistance > 0.0f || shapeB->material.rollingResistance > 0.0f )
	    {
		    float radiusA = b2GetShapeRadius( shapeA );
		    float radiusB = b2GetShapeRadius( shapeB );
		    float maxRadius = MathF.Max(radiusA, radiusB);
		    contactSim.rollingResistance =
			    MathF.Max( shapeA->material.rollingResistance, shapeB->material.rollingResistance ) * maxRadius;
	    }
	    else
	    {
		    contactSim.rollingResistance = 0.0f;
	    }

	    contactSim.tangentSpeed = shapeA->material.tangentSpeed + shapeB->material.tangentSpeed;

	    int pointCount = contactSim.manifold.pointCount;
	    bool touching = pointCount > 0;

	    if ( touching && world->preSolveFcn != NULL && ( contactSim.simFlags & ContactSimFlags.SimEnablePreSolveEvents ) != 0 )
        {
            var shapeIdA = shapeA.Id + 1;
            var shapeIdB = shapeB.Id + 1;

		    ref var manifold = ref contactSim.manifold;
		    float bestSeparation = manifold.points._00.separation;
		    Vector2 bestPoint = manifold.points._00.point;
            var pointSpan = manifold.points.AsSpan();

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

		    // this call assumes thread safety
		    touching = world->preSolveFcn( shapeIdA, shapeIdB, bestPoint, manifold->normal, world->preSolveContext );
		    if ( touching == false )
		    {
			    // disable contact
			    pointCount = 0;
			    manifold.pointCount = 0;
		    }
	    }

	    // This flag is for testing
	    if (_enableSpeculative == false && pointCount == 2 )
	    {
		    if ( contactSim.manifold.points.AsSpan()[0].separation > 1.5f * PhysicsConstants.LinearSlop )
		    {
			    contactSim.manifold.points[0] = contactSim.manifold.points[1];
			    contactSim.manifold.pointCount = 1;
		    }
		    else if ( contactSim.manifold.points[0].separation > 1.5f * PhysicsConstants.LinearSlop )
		    {
			    contactSim.manifold.pointCount = 1;
		    }

		    pointCount = contactSim.manifold.pointCount;
	    }

	    if ( touching && ( shapeA.enableHitEvents || shapeB.enableHitEvents ) )
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
		    ref var mp2 = ref contactSim.manifold.points.AsSpan()[i];

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
			    ref var mp1 = ref oldManifold.points.AsSpan()[j];

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
}
