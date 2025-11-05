using System;
using Robust.Shared.Collections;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Contacts;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.NewPhysics.Solver;
using Robust.Shared.Physics;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    /// <summary>
    /// Solve with graph coloring
    /// </summary>
    private void Solve()
    {
        // Are there any awake bodies? This scenario should not be important for profiling.
        var awakeSet = _solverSets[(int) SetType.AwakeSet];
        int awakeBodyCount = awakeSet.bodySims.Count;
        if (awakeBodyCount == 0)
        {
            // Nothing to simulate, however the tree rebuild must be finished.
            _rebuildHandle.WaitOne();

            ValidateNoEnlarged();
            return;
        }

        // Solve constraints using graph coloring
        {
            // Prepare buffers for bullets
            _bulletBodyCount = 0;
            _bulletBodies.Clear();
            _bulletBodies.EnsureCapacity(awakeBodyCount);

            var graph = _constraintGraph;
            var colors = graph.colors;

            _sims = awakeSet.bodySims;
            _states = awakeSet.bodyStates;

            // count contacts, joints, and colors
            int awakeJointCount = 0;
            int activeColorCount = 0;
            for (int i = 0; i < PhysicsConstants.GraphColorCount - 1; ++i)
            {
                int perColorContactCount = colors[i].ContactSims.Count;
                int perColorJointCount = colors[i].JointSims.Count;
                int occupancyCount = perColorContactCount + perColorJointCount;
                activeColorCount += occupancyCount > 0 ? 1 : 0;
                awakeJointCount += perColorJointCount;
            }

            // prepare for move events
            _bodyMoveEvents.EnsureCapacity(awakeBodyCount);

            // A block is a range of tasks, a start index and count as a sub-array.
            // Each worker receives at most M blocks of work. The workers may receive less blocks if there is not sufficient work.
            // Each block of work has a minimum number of elements (block size). This in turn may limit the number of blocks.
            // If there are many elements then the block size is increased so there are still at most M blocks of work per worker.
            // M is a tunable number that has two goals:
            // 1. keep M small to reduce overhead
            // 2. keep M large enough for other workers to be able to steal work
            // The block size is a power of two to make math efficient.

            int workerCount = _parallel.ParallelProcessCount;

            var maxBlockCount = blocksPerWorker * workerCount;

            // Configure blocks for tasks that parallel-for bodies
            int bodyBlockSize = 1 << 5;
            int bodyBlockCount;
            if (awakeBodyCount > bodyBlockSize * maxBlockCount)
            {
                // Too many blocks, increase block size
                bodyBlockSize = awakeBodyCount / maxBlockCount;
                bodyBlockCount = maxBlockCount;
            }
            else
            {
                // Divide by bodyBlockSize (32) and ensure there is at least one block
                bodyBlockCount = ((awakeBodyCount - 1) >> 5) + 1;
            }

            // Configure blocks for tasks parallel-for each active graph color
            var activeColorIndices = new int[PhysicsConstants.GraphColorCount];

            var colorContactCounts = new int[PhysicsConstants.GraphColorCount];
            var colorContactBlockSizes = new int[PhysicsConstants.GraphColorCount];
            var colorContactBlockCounts = new int[PhysicsConstants.GraphColorCount];

            var colorJointCounts = new int[PhysicsConstants.GraphColorCount];
            var colorJointBlockSizes = new int[PhysicsConstants.GraphColorCount];
            var colorJointBlockCounts = new int[PhysicsConstants.GraphColorCount];

            int graphBlockCount = 0;

            // This is where we start to diverge from box2d due to how SIMD implementation differences.

            // c is the active color index
            int simdContactCount = 0;
            int c = 0;
            for (int i = 0; i < PhysicsConstants.GraphColorCount - 1; ++i)
            {
                int colorContactCount = colors[i].ContactSims.Count;
                int colorJointCount = colors[i].JointSims.Count;

                if (colorContactCount + colorJointCount > 0)
                {
                    activeColorIndices[c] = i;

                    // 4/8-way SIMD
                    int colorContactCountSIMD = colorContactCount > 0 ? ((colorContactCount - 1) >> _simdShift) + 1 : 0;

                    // 4/8-way SIMD
                    colorContactCounts[c] = colorContactCountSIMD;

                    // determine the number of contact work blocks for this color
                    if (colorContactCountSIMD > blocksPerWorker * maxBlockCount)
                    {
                        // too many contact blocks per worker, so make bigger blocks
                        colorContactBlockSizes[c] = colorContactCountSIMD / maxBlockCount;
                        colorContactBlockCounts[c] = maxBlockCount;
                    }
                    else if (colorContactCountSIMD > 0)
                    {
                        // dividing by blocksPerWorker (4)
                        colorContactBlockSizes[c] = blocksPerWorker;

                        // This math makes sure there is at least one block
                        //colorContactBlockCounts[c] = ( ( colorContactCountSIMD - 1 ) >> 2 ) + 1;
                        colorContactBlockCounts[c] = ((colorContactCountSIMD - 1) / blocksPerWorker) + 1;
                    }
                    else
                    {
                        // no contacts in this color
                        colorContactBlockSizes[c] = 0;
                        colorContactBlockCounts[c] = 0;
                    }

                    colorJointCounts[c] = colorJointCount;

                    // determine number of joint work blocks for this color
                    if (colorJointCount > blocksPerWorker * maxBlockCount)
                    {
                        // too many joint blocks
                        colorJointBlockSizes[c] = colorJointCount / maxBlockCount;
                        colorJointBlockCounts[c] = maxBlockCount;
                    }
                    else if (colorJointCount > 0)
                    {
                        // dividing by blocksPerWorker (4)
                        colorJointBlockSizes[c] = blocksPerWorker;
                        //colorJointBlockCounts[c] = ( ( colorJointCount - 1 ) >> 2 ) + 1;
                        colorJointBlockCounts[c] = ((colorJointCount - 1) / 4) + 1;
                    }
                    else
                    {
                        colorJointBlockSizes[c] = 0;
                        colorJointBlockCounts[c] = 0;
                    }

                    graphBlockCount += colorContactBlockCounts[c] + colorJointBlockCounts[c];
                    simdContactCount += colorContactCountSIMD;
                    c += 1;
                }
            }

            activeColorCount = c;

            _contextContacts.Clear();
            _contextContacts.EnsureCapacity(_simdWidth + simdContactCount);

            // Gather joint pointers for easy parallel-for traversal.
            _contextJoints.Clear();
            _contextJoints.EnsureCapacity(awakeJointCount);

            int overflowContactCount = colors[PhysicsConstants.OverflowIndex].ContactSims.Count;
            var overflowContactConstraints = graph.colors[PhysicsConstants.OverflowIndex].OverflowConstraints;

            overflowContactConstraints.Clear();
            overflowContactConstraints.EnsureCapacity(overflowContactCount);

            // Distribute transient constraints to each graph color and build flat arrays of contact and joint pointers
            {
                int contactBase = 0;
                int jointBase = 0;
                for (int i = 0; i < activeColorCount; ++i)
                {
                    int j = activeColorIndices[i];
                    var color = colors[j];

                    int colorContactCount = color.ContactSims.Count;
                    color.SimdConstraints.Clear();

                    // Differs from box2d a bit as we don't allocate into a flat structure.
                    if (colorContactCount > 0)
                    {
                        for (int k = 0; k < colorContactCount; ++k)
                        {
                            // Box2D allocates into the array here but we already have the list and can just add so.
                            _contextContacts.Add(color.ContactSims[k]);
                        }

                        // remainder
                        int colorContactCountSIMD = ((colorContactCount - 1) >> _simdShift) + 1;
                        for (int k = colorContactCount; k < _simdWidth * colorContactCountSIMD; ++k)
                        {
                            _contextContacts.Add(null!);
                        }

                        contactBase += colorContactCountSIMD;
                    }

                    int colorJointCount = color.JointSims.Count;
                    for (int k = 0; k < colorJointCount; ++k)
                    {
                        _contextJoints[jointBase + k] = color.JointSims[k];
                    }

                    jointBase += colorJointCount;
                }

                DebugTools.Assert(contactBase == simdContactCount);
                DebugTools.Assert(jointBase == awakeJointCount);
            }

            // Define work blocks for preparing contacts and storing contact impulses
            int contactBlockSize = blocksPerWorker;
            //int contactBlockCount = simdContactCount > 0 ? ( ( simdContactCount - 1 ) >> 2 ) + 1 : 0;
            int contactBlockCount = simdContactCount > 0 ? ((simdContactCount - 1) / blocksPerWorker) + 1 : 0;
            if (simdContactCount > contactBlockSize * maxBlockCount)
            {
                // Too many blocks, increase block size
                contactBlockSize = simdContactCount / maxBlockCount;
                contactBlockCount = maxBlockCount;
            }

            // Define work blocks for preparing joints
            int jointBlockSize = blocksPerWorker;
            //int jointBlockCount = awakeJointCount > 0 ? ( ( awakeJointCount - 1 ) >> 2 ) + 1 : 0;
            int jointBlockCount = awakeJointCount > 0 ? ((awakeJointCount - 1) / blocksPerWorker) + 1 : 0;
            if (awakeJointCount > jointBlockSize * maxBlockCount)
            {
                // Too many blocks, increase block size
                jointBlockSize = awakeJointCount / maxBlockCount;
                jointBlockCount = maxBlockCount;
            }

            int stageCount = 0;

            // b2_stagePrepareJoints
            stageCount += 1;
            // b2_stagePrepareContacts
            stageCount += 1;
            // b2_stageIntegrateVelocities
            stageCount += 1;
            // b2_stageWarmStart
            stageCount += activeColorCount;
            // b2_stageSolve
            stageCount += Iterations * activeColorCount;
            // b2_stageIntegratePositions
            stageCount += 1;
            // b2_stageRelax
            stageCount += RelaxIterations * activeColorCount;
            // b2_stageRestitution
            stageCount += activeColorCount;
            // b2_stageStoreImpulses
            stageCount += 1;

            _contextStages.Clear();
            _contextBodyBlocks.Clear();
            _contextContactBlocks.Clear();
            _contextJointBlocks.Clear();
            _contextGraphBlocks.Clear();

            _contextStages.EnsureCapacity(stageCount);
            _contextBodyBlocks.EnsureCapacity(bodyBlockCount);
            _contextContactBlocks.EnsureCapacity(contactBlockCount);
            _contextJointBlocks.EnsureCapacity(jointBlockCount);
            _contextGraphBlocks.EnsureCapacity(graphBlockCount);

            var stages = new ValueList<SolverStage>(stageCount);
            var bodyBlocks = new ValueList<SolverBlock>(bodyBlockCount);
            var contactBlocks = new ValueList<SolverBlock>(contactBlockCount);
            var jointBlocks = new ValueList<SolverBlock>(jointBlockCount);
            var graphBlocks = new ValueList<SolverBlock>(graphBlockCount);

            // Split an awake island. This modifies:
            // - stack allocator
            // - world island array and solver set
            // - island indices on bodies, contacts, and joints
            // I'm squeezing this task in here because it may be expensive and this is a safe place to put it.
            // Note: cannot split islands in parallel with FinalizeBodies

            if (_splitIslandId != PhysicsConstants.NullIndex)
            {
                _splitHandle = _parallel.Process(_splitJob);
            }

            // Prepare body work blocks
            for (int i = 0; i < bodyBlockCount; ++i)
            {
                ref var block = ref bodyBlocks[i];
                block.startIndex = i * bodyBlockSize;
                block.count = (short) bodyBlockSize;
                block.blockType = (short) b2SolverBlockType.b2_bodyBlock;
                block.syncIndex = 0;
            }

            bodyBlocks[bodyBlockCount - 1].count = (short)(awakeBodyCount - (bodyBlockCount - 1) * bodyBlockSize);

            // Prepare joint work blocks
            for (int i = 0; i < jointBlockCount; ++i)
            {
                ref var block = ref jointBlocks[i];
                block.startIndex = i * jointBlockSize;
                block.count = (short)jointBlockSize;
                block.blockType = (short) b2SolverBlockType.b2_jointBlock;
                block.syncIndex = 0;
            }

            if (jointBlockCount > 0)
            {
                jointBlocks[jointBlockCount - 1].count =
                    (short)(awakeJointCount - (jointBlockCount - 1) * jointBlockSize);
            }

            // Prepare contact work blocks
            for (int i = 0; i < contactBlockCount; ++i)
            {
                ref var block = ref contactBlocks[i];
                block.startIndex = i * contactBlockSize;
                block.count = (short)contactBlockSize;
                block.blockType = (short) b2SolverBlockType.b2_contactBlock;
                block.syncIndex = 0;
            }

            if (contactBlockCount > 0)
            {
                contactBlocks[contactBlockCount - 1].count =
                    (short)(simdContactCount - (contactBlockCount - 1) * contactBlockSize);
            }

            // Prepare graph work blocks
            var graphColorBlocks = new SolverBlock[PhysicsConstants.GraphColorCount];
            var graphBlockIdx = 0;

            for (int i = 0; i < activeColorCount; ++i)
            {
                graphColorBlocks[i] = graphBlocks[graphBlockIdx];

                int colorJointBlockCount = colorJointBlockCounts[i];
                int colorJointBlockSize = colorJointBlockSizes[i];
                for (int j = 0; j < colorJointBlockCount; ++j)
                {
                    ref var block = ref graphBlocks[graphBlockIdx + j];
                    block.startIndex = j * colorJointBlockSize;
                    block.count = (short)colorJointBlockSize;
                    block.blockType = (short) b2SolverBlockType.b2_jointBlock;
                    block.syncIndex = 0;
                }

                if (colorJointBlockCount > 0)
                {
                    graphBlocks[graphBlockIdx + colorJointBlockCount - 1].count =
                        (short)(colorJointCounts[i] - (colorJointBlockCount - 1) * colorJointBlockSize);
                    graphBlockIdx += colorJointBlockCount;
                }

                int colorContactBlockCount = colorContactBlockCounts[i];
                int colorContactBlockSize = colorContactBlockSizes[i];
                for (int j = 0; j < colorContactBlockCount; ++j)
                {
                    ref var block = ref graphBlocks[graphBlockIdx + j];
                    block.startIndex = j * colorContactBlockSize;
                    block.count = (short)colorContactBlockSize;
                    block.blockType = (short) b2SolverBlockType.b2_graphContactBlock;
                    block.syncIndex = 0;
                }

                if (colorContactBlockCount > 0)
                {
                    graphBlocks[colorContactBlockCount - 1].count =
                        (short)(colorContactCounts[i] - (colorContactBlockCount - 1) * colorContactBlockSize);
                    graphBlockIdx += colorContactBlockCount;
                }
            }

            DebugTools.Assert((graphBlockIdx - graphBlocks.Count) == graphBlockCount);

            var stageIdx = 0;
            ref var stage = ref stages[stageIdx++];

            // Prepare joints
            stage.type = SolverStageType.b2_stagePrepareJoints;
            stage.blocks = jointBlocks;
            stage.blockCount = jointBlockCount;
            stage.colorIndex = -1;
            stage.completionCount = 0;

            // Prepare contacts
            ref var stage2 = ref stages[stageIdx++];
            stage2.type = SolverStageType.b2_stagePrepareContacts;
            stage2.blocks = contactBlocks;
            stage2.blockCount = contactBlockCount;
            stage2.colorIndex = -1;
            stage2.completionCount = 0;

            // Integrate velocities
            ref var stage3 = ref stages[stageIdx++];
            stage3.type = SolverStageType.b2_stageIntegrateVelocities;
            stage3.blocks = bodyBlocks;
            stage3.blockCount = bodyBlockCount;
            stage3.colorIndex = -1;
            stage3.completionCount = 0;

            // Warm start
            for (int i = 0; i < activeColorCount; ++i)
            {
                ref var warmStage = ref stages[stageIdx++];
                warmStage.type = SolverStageType.b2_stageWarmStart;
                warmStage.blocks.Add(graphColorBlocks[i]);
                warmStage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                warmStage.colorIndex = activeColorIndices[i];
                warmStage.completionCount = 0;
            }

            // Solve graph
            for (int j = 0; j < Iterations; ++j)
            {
                for (int i = 0; i < activeColorCount; ++i)
                {
                    ref var solveStage = ref stages[stageIdx++];
                    solveStage.type = SolverStageType.b2_stageSolve;
                    solveStage.blocks.Add(graphColorBlocks[i]);
                    solveStage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                    solveStage.colorIndex = activeColorIndices[i];
                    solveStage.completionCount = 0;
                }
            }

            // Integrate positions
            ref var integrateStage = ref stages[stageIdx++];
            integrateStage.type = SolverStageType.b2_stageIntegratePositions;
            integrateStage.blocks = bodyBlocks;
            integrateStage.blockCount = bodyBlockCount;
            integrateStage.colorIndex = -1;
            integrateStage.completionCount = 0;

            // Relax constraints
            for (int j = 0; j < RelaxIterations; ++j)
            {
                for (int i = 0; i < activeColorCount; ++i)
                {
                    ref var colorStage = ref stages[stageIdx++];
                    colorStage.type = SolverStageType.b2_stageRelax;
                    colorStage.blocks.Add(graphColorBlocks[i]);
                    colorStage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                    colorStage.colorIndex = activeColorIndices[i];
                    colorStage.completionCount = 0;
                }
            }

            // Restitution
            // Note: joint blocks mixed in, could have joint limit restitution
            for (int i = 0; i < activeColorCount; ++i)
            {
                ref var colorStage = ref stages[stageIdx++];
                colorStage.type = SolverStageType.b2_stageRestitution;
                colorStage.blocks.Add(graphColorBlocks[i]);
                colorStage.blockCount = colorJointBlockCounts[i] + colorContactBlockCounts[i];
                colorStage.colorIndex = activeColorIndices[i];
                colorStage.completionCount = 0;
            }

            // Store impulses
            ref var impulseStage = ref stages[stageIdx++];
            impulseStage.type = SolverStageType.b2_stageStoreImpulses;
            impulseStage.blocks = contactBlocks;
            impulseStage.blockCount = contactBlockCount;
            impulseStage.colorIndex = -1;
            stage.completionCount = 0;

            DebugTools.Assert(stageIdx == stageCount);

            _activeColorCount = activeColorCount;
            _stageCount = stageCount;

            SolverTask();

            // Finish island split
            _splitHandle.WaitOne();

            _splitIslandId = PhysicsConstants.NullIndex;

            // Finish constraint solve

            // Prepare contact, enlarged body, and island bit sets used in body finalization.
            int awakeIslandCount = awakeSet.islandSims.Count;
            for (int i = 0; i < world.workerCount; ++i)
            {
                b2TaskContext* taskContext = world.taskContexts.data + i;
                b2SensorHitArray_Clear(&taskContext.sensorHits);
                b2SetBitCountAndClear(&taskContext.enlargedSimBitSet, awakeBodyCount);
                b2SetBitCountAndClear(&taskContext.awakeIslandBitSet, awakeIslandCount);
                taskContext.splitIslandId = PhysicsConstants.NullIndex;
                taskContext.splitSleepTime = 0.0f;
            }

            // Finalize bodies. Must happen after the constraint solver and after island splitting.
            _parallel.ProcessNow(_finalizeJob, awakeBodyCount);

            // TODO: Store as fields and clear them here.
            b2FreeArenaItem(&world.arena, graphBlocks);
            b2FreeArenaItem(&world.arena, jointBlocks);
            b2FreeArenaItem(&world.arena, contactBlocks);
            b2FreeArenaItem(&world.arena, bodyBlocks);
            b2FreeArenaItem(&world.arena, stages);
            b2FreeArenaItem(&world.arena, overflowContactConstraints);
            b2FreeArenaItem(&world.arena, simdContactConstraints);
            b2FreeArenaItem(&world.arena, joints);
            b2FreeArenaItem(&world.arena, contacts);

            world.profile.transforms = b2GetMilliseconds(transformTicks);
            b2TracyCZoneEnd(update_transforms);
        }

        // Report joint events
        {
            // TODO: Put the jointstatebitset on the context and pass that in

            // Gather bits for all joints that have force/torque events
            b2BitSet* jointStateBitSet = &world.taskContexts.data[0].jointStateBitSet;
            for (int i = 1; i < world.workerCount; ++i)
            {
                b2InPlaceUnion(jointStateBitSet, &world.taskContexts.data[i].jointStateBitSet);
            }

            {
                uint32_t wordCount = jointStateBitSet.blockCount;
                uint64_t* bits = jointStateBitSet.bits;

                BaseJoint* jointArray = world.joints.data;

                for (uint32_t k = 0; k < wordCount; ++k)
                {
                    uint64_t word = bits[k];
                    while (word != 0)
                    {
                        uint32_t ctz = b2CTZ64(word);
                        int jointId = (int)(64 * k + ctz);

                        DebugTools.Assert(jointId < world.joints.capacity);

                        BaseJoint* joint = jointArray + jointId;

                        DebugTools.Assert(joint.setIndex == (int) SetType.AwakeSet);

                        b2JointEvent event = {
                            .jointId =
                            {
                                .index1 = jointId + 1,
                                    .world0 = worldIndex0,
                                    .generation = joint.generation,
                            },
                            .userData = joint.userData,
                        }
                        ;

                        b2JointEventArray_Push(&world.jointEvents,  event );

                        // Clear the smallest set bit
                        word = word & (word - 1);
                    }
                }
            }

            world.profile.jointEvents = b2GetMilliseconds(jointEventTicks);
            b2TracyCZoneEnd(joint_events);
        }

        // Report hit events
        // todo_erin perhaps optimize this with a bitset
        // todo_erin perhaps do this in parallel with other work below
        {
            b2TracyCZoneNC(hit_events, "Hit Events", b2_colorRosyBrown, true);
            uint64_t hitTicks = b2GetTicks();

            DebugTools.Assert(world.contactHitEvents.count == 0);

            float threshold = world.hitEventThreshold;
            b2GraphColor* colors = world.constraintGraph.colors;
            for (int i = 0; i < PhysicsConstants.GraphColorCount; ++i)
            {
                b2GraphColor* color = colors + i;
                int contactCount = color.contactSims.count;
                b2ContactSim* contactSims = color.contactSims.data;
                for (int j = 0; j < contactCount; ++j)
                {
                    b2ContactSim* contactSim = contactSims + j;
                    if ((contactSim.simFlags & b2_simEnableHitEvent) == 0)
                    {
                        continue;
                    }

                    b2ContactHitEvent event = {
                        0
                    }
                    ;
                    event.approachSpeed = threshold;

                    bool hit = false;
                    int pointCount = contactSim.manifold.pointCount;
                    for (int k = 0; k < pointCount; ++k)
                    {
                        b2ManifoldPoint* mp = contactSim.manifold.points + k;
                        float approachSpeed = -mp.normalVelocity;

                        // Need to check total impulse because the point may be speculative and not colliding
                        if (approachSpeed > event.approachSpeed && mp.totalNormalImpulse > 0.0f )
                        {
                            event.approachSpeed = approachSpeed;
                                event.point = mp.point;
                            hit = true;
                        }
                    }

                    if (hit == true)
                    {
                        event.normal = contactSim.manifold.normal;

                        b2Shape* shapeA = b2ShapeArray_Get(&world.shapes, contactSim.shapeIdA);
                        b2Shape* shapeB = b2ShapeArray_Get(&world.shapes, contactSim.shapeIdB);

                            event.shapeIdA = (b2ShapeId){
                            shapeA.id + 1, world.worldId, shapeA.generation
                        }
                        ;
                        event.shapeIdB = (b2ShapeId){
                            shapeB.id + 1, world.worldId, shapeB.generation
                        }
                        ;

                        b2ContactHitEventArray_Push(&world.contactHitEvents,  event );
                    }
                }
            }

            world.profile.hitEvents = b2GetMilliseconds(hitTicks);
            b2TracyCZoneEnd(hit_events);
        }

        {
            b2TracyCZoneNC(refit_bvh, "Refit BVH", b2_colorFireBrick, true);
            uint64_t refitTicks = b2GetTicks();

            // Finish the user tree task that was queued earlier in the time step. This must be complete before touching the
            // broad-phase.
            _rebuildHandle.WaitOne();

            ValidateNoEnlarged();

            // Gather bits for all sim bodies that have enlarged AABBs
            b2BitSet* enlargedBodyBitSet = &world.taskContexts.data[0].enlargedSimBitSet;
            for (int i = 1; i < world.workerCount; ++i)
            {
                b2InPlaceUnion(enlargedBodyBitSet, &world.taskContexts.data[i].enlargedSimBitSet);
            }

            // Enlarge broad-phase proxies and build move array
            // Apply shape AABB changes to broad-phase. This also create the move array which must be
            // in deterministic order. I'm tracking sim bodies because the number of shape ids can be huge.
            // This has to happen before bullets are processed.
            {
                b2BroadPhase* broadPhase = &world.broadPhase;
                uint32_t wordCount = enlargedBodyBitSet.blockCount;
                uint64_t* bits = enlargedBodyBitSet.bits;

                // Fast array access is important here
                b2Body* bodyArray = world.bodies.data;
                b2BodySim* bodySimArray = awakeSet.bodySims.data;
                b2Shape* shapeArray = world.shapes.data;

                for (uint32_t k = 0; k < wordCount; ++k)
                {
                    uint64_t word = bits[k];
                    while (word != 0)
                    {
                        uint32_t ctz = b2CTZ64(word);
                        uint32_t bodySimIndex = 64 * k + ctz;

                        b2BodySim* bodySim = bodySimArray + bodySimIndex;

                        b2Body* body = bodyArray + bodySim.bodyId;

                        int shapeId = body.headShapeId;
                        if ((bodySim.flags & (b2_isBullet | b2_isFast)) == (b2_isBullet | b2_isFast))
                        {
                            // Fast bullet bodies don't have their final AABB yet
                            while (shapeId != PhysicsConstants.NullIndex)
                            {
                                b2Shape* shape = shapeArray + shapeId;

                                // Shape is fast. It's aabb will be enlarged in continuous collision.
                                // Update the move array here for determinism because bullets are processed
                                // below in non-deterministic order.
                                b2BufferMove(broadPhase, shape.proxyKey);

                                shapeId = shape.nextShapeId;
                            }
                        }
                        else
                        {
                            while (shapeId != PhysicsConstants.NullIndex)
                            {
                                b2Shape* shape = shapeArray + shapeId;

                                // The AABB may not have been enlarged, despite the body being flagged as enlarged.
                                // For example, a body with multiple shapes may have not have all shapes enlarged.
                                // A fast body may have been flagged as enlarged despite having no shapes enlarged.
                                if (shape.enlargedAABB)
                                {
                                    b2BroadPhase_EnlargeProxy(broadPhase, shape.proxyKey, shape.fatAABB);
                                    shape.enlargedAABB = false;
                                }

                                shapeId = shape.nextShapeId;
                            }
                        }

                        // Clear the smallest set bit
                        word = word & (word - 1);
                    }
                }
            }

            b2ValidateBroadphase(&world.broadPhase);

            world.profile.refit = b2GetMilliseconds(refitTicks);
            b2TracyCZoneEnd(refit_bvh);
        }

        int bulletBodyCount = b2AtomicLoadInt(stepContext.bulletBodyCount);
        if (bulletBodyCount > 0)
        {
            b2TracyCZoneNC(bullets, "Bullets", b2_colorLightYellow, true);
            uint64_t bulletTicks = b2GetTicks();

            // Fast bullet bodies
            // Note: a bullet body may be moving slow
            int minRange = 8;
            void* userBulletBodyTask =
                world.enqueueTaskFcn(&b2BulletBodyTask,
                    bulletBodyCount,
                    minRange,
                    stepContext,
                    world.userTaskContext);
            world.taskCount += 1;
            if (userBulletBodyTask != NULL)
            {
                world.finishTaskFcn(userBulletBodyTask, world.userTaskContext);
            }

            // Serially enlarge broad-phase proxies for bullet shapes
            b2BroadPhase* broadPhase = &world.broadPhase;
            b2DynamicTree* dynamicTree = broadPhase.trees + b2_dynamicBody;

            // Fast array access is important here
            b2Body* bodyArray = world.bodies.data;
            b2BodySim* bodySimArray = awakeSet.bodySims.data;
            b2Shape* shapeArray = world.shapes.data;

            // Serially enlarge broad-phase proxies for bullet shapes
            int* bulletBodySimIndices = stepContext.bulletBodies;

            // This loop has non-deterministic order but it shouldn't affect the result
            for (int i = 0; i < bulletBodyCount; ++i)
            {
                b2BodySim* bulletBodySim = bodySimArray + bulletBodySimIndices[i];
                if ((bulletBodySim.flags & b2_enlargeBounds) == 0)
                {
                    continue;
                }

                // Clear flag
                bulletBodySim.flags &= ~b2_enlargeBounds;

                int bodyId = bulletBodySim.bodyId;
                DebugTools.Assert(0 <= bodyId && bodyId < world.bodies.count);
                b2Body* bulletBody = bodyArray + bodyId;

                int shapeId = bulletBody.headShapeId;
                while (shapeId != PhysicsConstants.NullIndex)
                {
                    b2Shape* shape = shapeArray + shapeId;
                    if (shape.enlargedAABB == false)
                    {
                        shapeId = shape.nextShapeId;
                        continue;
                    }

                    // Clear flag
                    shape.enlargedAABB = false;

                    int proxyKey = shape.proxyKey;
                    int proxyId = B2_PROXY_ID(proxyKey);
                    DebugTools.Assert(B2_PROXY_TYPE(proxyKey) == b2_dynamicBody);

                    // all fast bullet shapes should already be in the move buffer
                    DebugTools.Assert(b2ContainsKey(&broadPhase.moveSet, proxyKey + 1));

                    b2DynamicTree_EnlargeProxy(dynamicTree, proxyId, shape.fatAABB);

                    shapeId = shape.nextShapeId;
                }
            }

            world.profile.bullets = b2GetMilliseconds(bulletTicks);
            b2TracyCZoneEnd(bullets);
        }

        // Need to free this even if no bullets got processed.
        b2FreeArenaItem(&world.arena, stepContext.bulletBodies);
        stepContext.bulletBodies = NULL;
        b2AtomicStoreInt(&stepContext.bulletBodyCount, 0);

        // Report sensor hits. This may include bullets sensor hits.
        {
            b2TracyCZoneNC(sensor_hits, "Sensor Hits", b2_colorPowderBlue, true);
            uint64_t sensorHitTicks = b2GetTicks();

            int workerCount = world.workerCount;
            DebugTools.Assert(workerCount == world.taskContexts.count);

            for (int i = 0; i < workerCount; ++i)
            {
                b2TaskContext* taskContext = world.taskContexts.data + i;
                int hitCount = taskContext.sensorHits.count;
                b2SensorHit* hits = taskContext.sensorHits.data;

                for (int j = 0; j < hitCount; ++j)
                {
                    b2SensorHit hit = hits[j];
                    b2Shape* sensorShape = b2ShapeArray_Get(&world.shapes, hit.sensorId);
                    b2Shape* visitor = b2ShapeArray_Get(&world.shapes, hit.visitorId);

                    b2Sensor* sensor = b2SensorArray_Get(&world.sensors, sensorShape.sensorIndex);
                    b2Visitor shapeRef =
                    {
                        .shapeId = hit.visitorId,
                        .generation = visitor.generation,
                    };
                    b2VisitorArray_Push(&sensor.hits, shapeRef);
                }
            }

            world.profile.sensorHits = b2GetMilliseconds(sensorHitTicks);
            b2TracyCZoneEnd(sensor_hits);
        }

        // Island sleeping
        // This must be done last because putting islands to sleep invalidates the enlarged body bits.
        // todo_erin figure out how to do this in parallel with tree refit
        if (world.enableSleep == true)
        {
            b2TracyCZoneNC(sleep_islands, "Island Sleep", b2_colorLightSlateGray, true);
            uint64_t sleepTicks = b2GetTicks();

            // Collect split island candidate for the next time step. No need to split if sleeping is disabled.
            DebugTools.Assert(world.splitIslandId == PhysicsConstants.NullIndex);
            float splitSleepTimer = 0.0f;
            for (int i = 0; i < world.workerCount; ++i)
            {
                b2TaskContext* taskContext = world.taskContexts.data + i;
                if (taskContext.splitIslandId != PhysicsConstants.NullIndex && taskContext.splitSleepTime >= splitSleepTimer)
                {
                    DebugTools.Assert(taskContext.splitSleepTime > 0.0f);

                    // Tie breaking for determinism. Largest island id wins. Needed due to work stealing.
                    if (taskContext.splitSleepTime == splitSleepTimer &&
                        taskContext.splitIslandId < world.splitIslandId)
                    {
                        continue;
                    }

                    world.splitIslandId = taskContext.splitIslandId;
                    splitSleepTimer = taskContext.splitSleepTime;
                }
            }

            b2BitSet* awakeIslandBitSet = &world.taskContexts.data[0].awakeIslandBitSet;
            for (int i = 1; i < world.workerCount; ++i)
            {
                b2InPlaceUnion(awakeIslandBitSet, &world.taskContexts.data[i].awakeIslandBitSet);
            }

            // Need to process in reverse because this moves islands to sleeping solver sets.
            b2IslandSim* islands = awakeSet.islandSims.data;
            int count = awakeSet.islandSims.count;
            for (int islandIndex = count - 1; islandIndex >= 0; islandIndex -= 1)
            {
                if (b2GetBit(awakeIslandBitSet, islandIndex) == true)
                {
                    // this island is still awake
                    continue;
                }

                b2IslandSim* island = islands + islandIndex;
                int islandId = island.islandId;

                b2TrySleepIsland(world, islandId);
            }

            b2ValidateSolverSets(world);

            world.profile.sleepIslands = b2GetMilliseconds(sleepTicks);
            b2TracyCZoneEnd(sleep_islands);
        }
    }

    private sealed class FinalizeBodiesJob : IParallelRobustJob
    {
        public int BatchSize => 64;

        public void Execute(int simIndex)
        {
            // TODO: Do the same trick we do with old physics where we matrix transform the world pos
            // so we can avoid the grid change.

            // TODO: Given this is in parallel we need bitsets for "is physics dirty" and "is transform dirty"
            // Transform one should also implicitly do a moveevent
            b2TracyCZoneNC( finalize_transfprms, "Transforms", b2_colorMediumSeaGreen, true );

	        b2StepContext* stepContext = context;
	        b2World* world = stepContext.world;

	        DebugTools.Assert( (int)threadIndex < world.workerCount );

	        bool enableSleep = world.enableSleep;
	        b2BodyState* states = stepContext.states;
	        b2BodySim* sims = stepContext.sims;
	        b2Body* bodies = world.bodies.data;
	        float timeStep = stepContext.dt;
	        float invTimeStep = stepContext.inv_dt;

	        uint16_t worldId = world.worldId;

	        // The body move event array should already have the correct size
	        DebugTools.Assert( endIndex <= world.bodyMoveEvents.count );
	        b2BodyMoveEvent* moveEvents = world.bodyMoveEvents.data;

	        b2BitSet* enlargedSimBitSet = &world.taskContexts.data[threadIndex].enlargedSimBitSet;
	        b2BitSet* awakeIslandBitSet = &world.taskContexts.data[threadIndex].awakeIslandBitSet;
	        b2TaskContext* taskContext = world.taskContexts.data + threadIndex;

	        bool enableContinuous = world.enableContinuous;

	        const float speculativeDistance = B2_SPECULATIVE_DISTANCE;
	        const float aabbMargin = B2_AABB_MARGIN;

	        DebugTools.Assert( startIndex <= endIndex );

            // TODO: Split from above into its own thing

	        b2BodyState* state = states + simIndex;
		    b2BodySim* sim = sims + simIndex;

		    if ( state.flags & b2_lockLinearX )
		    {
			    state.linearVelocity.x = 0.0f;
		    }

		    if ( state.flags & b2_lockLinearY )
		    {
			    state.linearVelocity.y = 0.0f;
		    }

		    if ( state.flags & b2_lockAngularZ )
		    {
			    state.angularVelocity = 0.0f;
		    }

		    b2Vec2 v = state.linearVelocity;
		    float w = state.angularVelocity;

		    DebugTools.Assert( b2IsValidVec2( v ) );
		    DebugTools.Assert( b2IsValidFloat( w ) );

		    sim.center = b2Add( sim.center, state.deltaPosition );
		    sim.transform.q = b2NormalizeRot( b2MulRot( state.deltaRotation, sim.transform.q ) );

		    // Use the velocity of the farthest point on the body to account for rotation.
		    float maxVelocity = b2Length( v ) + b2AbsFloat( w ) * sim.maxExtent;

		    // Sleep needs to observe position correction as well as true velocity.
		    float maxDeltaPosition = b2Length( state.deltaPosition ) + b2AbsFloat( state.deltaRotation.s ) * sim.maxExtent;

		    // Position correction is not as important for sleep as true velocity.
		    float positionSleepFactor = 0.5f;

		    float sleepVelocity = MathF.Max( maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition );

		    // reset state deltas
		    state.deltaPosition = Vector2.Zero;
		    state.deltaRotation = b2Rot_identity;

		    sim.transform.p = b2Sub( sim.center, Quaternion2D.RotateVector( sim->transform.q, sim->localCenter ) );

		    // cache miss here, however I need the shape list below
		    b2Body* body = bodies + sim->bodyId;
		    body->bodyMoveIndex = simIndex;
		    moveEvents[simIndex].transform = sim->transform;
		    moveEvents[simIndex].bodyId = (b2BodyId){ sim->bodyId + 1, worldId, body->generation };
		    moveEvents[simIndex].userData = body->userData;
		    moveEvents[simIndex].fellAsleep = false;

		    // reset applied force and torque
		    sim->force = Vector2.Zero;
		    sim->torque = 0.0f;

		    body->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );
		    body->flags |= ( sim->flags & ( b2_isSpeedCapped | b2_hadTimeOfImpact ) );
		    sim->flags &= ~( b2_isFast | b2_isSpeedCapped | b2_hadTimeOfImpact );

		    if ( enableSleep == false || body->enableSleep == false || sleepVelocity > body->sleepThreshold )
		    {
			    // Body is not sleepy
			    body->sleepTime = 0.0f;

			    if ( body->type == b2_dynamicBody && enableContinuous && maxVelocity * timeStep > 0.5f * sim->minExtent )
			    {
				    // This flag is only retained for debug draw
				    sim->flags |= b2_isFast;

				    // Store in fast array for the continuous collision stage
				    // This is deterministic because the order of TOI sweeps doesn't matter
				    if ( sim->flags & b2_isBullet )
				    {
					    int bulletIndex = b2AtomicFetchAddInt( &stepContext->bulletBodyCount, 1 );
					    stepContext->bulletBodies[bulletIndex] = simIndex;
				    }
				    else
				    {
					    b2SolveContinuous( world, simIndex, taskContext );
				    }
			    }
			    else
			    {
				    // Body is safe to advance
				    sim->center0 = sim->center;
				    sim->rotation0 = sim->transform.q;
			    }
		    }
		    else
		    {
			    // Body is safe to advance and is falling asleep
			    sim->center0 = sim->center;
			    sim->rotation0 = sim->transform.q;
			    body->sleepTime += timeStep;
		    }

		    // Any single body in an island can keep it awake
		    var island = _islands[body.IslandId];
		    if ( body.SleepTime < _timeToSleep )
		    {
			    // keep island awake
			    int islandIndex = island->localIndex;
			    b2SetBit( awakeIslandBitSet, islandIndex );
		    }
		    else if ( island->constraintRemoveCount > 0 )
		    {
			    // body wants to sleep but its island needs splitting first
			    if ( body->sleepTime > taskContext->splitSleepTime )
			    {
				    // pick the sleepiest candidate
				    taskContext->splitIslandId = body->islandId;
				    taskContext->splitSleepTime = body->sleepTime;
			    }
		    }

		    // Update shapes AABBs
		    b2Transform transform = sim->transform;
		    bool isFast = ( sim->flags & b2_isFast ) != 0;
		    int shapeId = body->headShapeId;
		    while ( shapeId != PhysicsConstants.NullIndex )
		    {
			    b2Shape* shape = b2ShapeArray_Get( &world->shapes, shapeId );

			    if ( isFast )
			    {
				    // For fast non-bullet bodies the AABB has already been updated in b2SolveContinuous
				    // For fast bullet bodies the AABB will be updated at a later stage

				    // Add to enlarged shapes regardless of AABB changes.
				    // Bit-set to keep the move array sorted
				    b2SetBit( enlargedSimBitSet, simIndex );
			    }
			    else
			    {
				    b2AABB aabb = b2ComputeShapeAABB( shape, transform );
				    aabb.lowerBound.x -= speculativeDistance;
				    aabb.lowerBound.y -= speculativeDistance;
				    aabb.upperBound.x += speculativeDistance;
				    aabb.upperBound.y += speculativeDistance;
				    shape->aabb = aabb;

				    DebugTools.Assert( shape->enlargedAABB == false );

				    if ( b2AABB_Contains( shape->fatAABB, aabb ) == false )
				    {
					    b2AABB fatAABB;
					    fatAABB.lowerBound.x = aabb.lowerBound.x - aabbMargin;
					    fatAABB.lowerBound.y = aabb.lowerBound.y - aabbMargin;
					    fatAABB.upperBound.x = aabb.upperBound.x + aabbMargin;
					    fatAABB.upperBound.y = aabb.upperBound.y + aabbMargin;
					    shape->fatAABB = fatAABB;

					    shape->enlargedAABB = true;

					    // Bit-set to keep the move array sorted
					    b2SetBit( enlargedSimBitSet, simIndex );
				    }
			    }

			    shapeId = shape->nextShapeId;
		    }
        }
    }
}
