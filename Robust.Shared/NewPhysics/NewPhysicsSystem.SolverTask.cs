using System;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.NewPhysics.Solver;
using Robust.Shared.Threading;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private sealed class SolveStageJob : IParallelRobustJob
    {
        public NewPhysicsSystem System = default!;
        public StepContext Context = default!;
        public SolverStage Stage;

        public void Execute(int index)
        {
            // Equivalent to b2ExecuteBlock
            var block = Stage.blocks[index];

            var stageType = Stage.type;
            var blockType = (b2SolverBlockType) block.blockType;
            int startIndex = block.startIndex;
            int endIndex = startIndex + block.count;

            switch ( stageType )
            {
                case SolverStageType.b2_stagePrepareJoints:
                    PrepareJointsTask(startIndex, endIndex, Context);
                    break;

                case SolverStageType.b2_stagePrepareContacts:
                    PrepareContactsTask( startIndex, endIndex, in Context);
                    break;

                case SolverStageType.b2_stageIntegrateVelocities:
                    IntegrateVelocitiesTask(startIndex, endIndex, Context);
                    break;

                case SolverStageType.b2_stageWarmStart:
                    if ( blockType == b2SolverBlockType.b2_graphContactBlock )
                    {
                        WarmStartContactsTask( startIndex, endIndex, context, stage->colorIndex );
                    }
                    else if ( blockType == b2SolverBlockType.b2_graphJointBlock )
                    {
                        WarmStartJointsTask( startIndex, endIndex, context, stage->colorIndex );
                    }
                    break;

                case SolverStageType.b2_stageSolve:
                    if ( blockType == b2SolverBlockType.b2_graphContactBlock )
                    {
                        SolveContactsTask( startIndex, endIndex, context, stage->colorIndex, true );
                    }
                    else if ( blockType == b2SolverBlockType.b2_graphJointBlock )
                    {
                        SolveJointsTask( startIndex, endIndex, context, stage->colorIndex, true, workerIndex );
                    }
                    break;

                case SolverStageType.b2_stageIntegratePositions:
                    IntegratePositionsTask( startIndex, endIndex, context );
                    break;

                case SolverStageType.b2_stageRelax:
                    if ( blockType == b2SolverBlockType.b2_graphContactBlock )
                    {
                        SolveContactsTask( startIndex, endIndex, context, stage->colorIndex, false );
                    }
                    else if ( blockType == b2SolverBlockType.b2_graphJointBlock )
                    {
                        SolveJointsTask( startIndex, endIndex, context, stage->colorIndex, false, workerIndex );
                    }
                    break;

                case SolverStageType.b2_stageRestitution:
                    if ( blockType == b2SolverBlockType.b2_graphContactBlock )
                    {
                        ApplyRestitutionTask( startIndex, endIndex, context, stage->colorIndex );
                    }
                    break;

                case SolverStageType.b2_stageStoreImpulses:
                    StoreImpulsesTask( startIndex, endIndex, context );
                    break;
            }
        }

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

		        v = b2MulAdd( linearVelocityDelta, linearDamping, v );
		        w = angularVelocityDelta + angularDamping * w;

		        // Clamp to max linear speed
		        if ( b2Dot( v, v ) > maxLinearSpeedSquared )
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
    }

    private void ExecuteMainStage(in SolverStage stage, ref StepContext context)
    {
        _solveJob.Stage = stage;
        _parallel.ProcessNow(_solveJob, stage.blockCount);
    }

    private void SolverTask(ref StepContext context)
    {
        // Okay now we have all the work blocks.
        // In Box2D it makes a task for each thread and does manual work-stealing, with the main thread orchestrating it.
        // It uses atomics to pipeline each thread onto each stage.
        // For us we'll just use discrete tasks to make it simpler but still keep the same general outline for simplicity.

		/*
		b2_stagePrepareJoints,
		b2_stagePrepareContacts,
		b2_stageIntegrateVelocities,
		b2_stageWarmStart,
		b2_stageSolve,
		b2_stageIntegratePositions,
		b2_stageRelax,
		b2_stageRestitution,
		b2_stageStoreImpulses
		*/

		int bodySyncIndex = 1;
		int stageIndex = 0;
        var stages = context.Stages;

		// This stage loops over all awake joints
		ExecuteMainStage(stages[stageIndex], ref context);

		// This stage loops over all contact constraints
		DebugTools.Assert(stages[stageIndex].type == SolverStageType.b2_stagePrepareContacts );
		ExecuteMainStage(stages[stageIndex], ref context);
		stageIndex += 1;

		int graphSyncIndex = 1;

		// Single-threaded overflow work. These constraints don't fit in the graph coloring.
		PrepareOverflowJoints( context );
		PrepareOverflowContacts( context );

		int subStepCount = context.subStepCount;

		for ( int i = 0; i < subStepCount; ++i )
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int iterStageIndex = stageIndex;

			// integrate velocities
			syncBits = ( bodySyncIndex << 16 ) | iterStageIndex;
			B2_ASSERT( stages[iterStageIndex].type == b2_stageIntegrateVelocities );
			ExecuteMainStage( stages[iterStageIndex], context, syncBits );
			iterStageIndex += 1;
			bodySyncIndex += 1;

			profile->integrateVelocities += b2GetMillisecondsAndReset( &ticks );

			// warm start constraints
			b2WarmStartOverflowJoints( context );
			b2WarmStartOverflowContacts( context );

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageWarmStart );
				b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			graphSyncIndex += 1;

			profile->warmStart += b2GetMillisecondsAndReset( &ticks );

			// solve constraints
			bool useBias = true;

			for ( int j = 0; j < ITERATIONS; ++j )
			{
				// Overflow constraints have lower priority
				SolveOverflowJoints( context, useBias );
				SolveOverflowContacts( context, useBias );

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
					B2_ASSERT( stages[iterStageIndex].type == b2_stageSolve );
					b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
					iterStageIndex += 1;
				}
				graphSyncIndex += 1;
			}

			profile->solveImpulses += b2GetMillisecondsAndReset( &ticks );

			// integrate positions
			B2_ASSERT( stages[iterStageIndex].type == b2_stageIntegratePositions );
			syncBits = ( bodySyncIndex << 16 ) | iterStageIndex;
			ExecuteMainStage(stages[iterStageIndex], ref context);
			iterStageIndex += 1;
			bodySyncIndex += 1;

			profile->integratePositions += b2GetMillisecondsAndReset( &ticks );

			// relax constraints
			useBias = false;
			for ( int j = 0; j < RELAX_ITERATIONS; ++j )
			{
				b2SolveOverflowJoints( context, useBias );
				b2SolveOverflowContacts( context, useBias );

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
					B2_ASSERT( stages[iterStageIndex].type == b2_stageRelax );
					b2ExecuteMainStage( stages + iterStageIndex, context, syncBits );
					iterStageIndex += 1;
				}
				graphSyncIndex += 1;
			}
        }

		// advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + ITERATIONS * activeColorCount + 1 + RELAX_ITERATIONS * activeColorCount;

		// Restitution
		{
			ApplyOverflowRestitution( context );

			int iterStageIndex = stageIndex;
			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				syncBits = ( graphSyncIndex << 16 ) | iterStageIndex;
				B2_ASSERT( stages[iterStageIndex].type == b2_stageRestitution );
				ExecuteMainStage( stages + iterStageIndex, context, syncBits );
				iterStageIndex += 1;
			}
			// graphSyncIndex += 1;
			stageIndex += activeColorCount;
		}

		profile->applyRestitution += b2GetMillisecondsAndReset( &ticks );

		StoreOverflowImpulses( context );

		syncBits = ( contactSyncIndex << 16 ) | stageIndex;
		B2_ASSERT( stages[stageIndex].type == b2_stageStoreImpulses );
		ExecuteMainStage( stages + stageIndex, context, syncBits );

		profile->storeImpulses += b2GetMillisecondsAndReset( &ticks );

		// Signal workers to finish
		b2AtomicStoreU32( &context->atomicSyncBits, UINT_MAX );

		B2_ASSERT( stageIndex + 1 == context->stageCount );
		return;
    }
}
