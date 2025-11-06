using System;
using System.Collections;
using System.Collections.Generic;
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
        public SolverStage Stage;

        public List<BitArray> JointStateBitSet = new();

        public void Execute(int index)
        {
            // Equivalent to b2ExecuteBlock
            var block = Stage.blocks[index];

            var stageType = Stage.type;
            var blockType = (b2SolverBlockType)block.blockType;
            int startIndex = block.startIndex;
            int endIndex = startIndex + block.count;

            switch (stageType)
            {
                case SolverStageType.b2_stagePrepareJoints:
                    System.PrepareJointsTask(startIndex, endIndex);
                    break;

                case SolverStageType.b2_stagePrepareContacts:
                    System.PrepareContactsTask(startIndex, endIndex);
                    break;

                case SolverStageType.b2_stageIntegrateVelocities:
                    System.IntegrateVelocitiesTask(startIndex, endIndex);
                    break;

                case SolverStageType.b2_stageWarmStart:
                    if (blockType == b2SolverBlockType.b2_graphContactBlock)
                    {
                        System.WarmStartContactsTask(startIndex, endIndex, Stage.colorIndex);
                    }
                    else if (blockType == b2SolverBlockType.b2_graphJointBlock)
                    {
                        System.WarmStartJointsTask(startIndex, endIndex, Stage.colorIndex);
                    }

                    break;

                case SolverStageType.b2_stageSolve:
                    if (blockType == b2SolverBlockType.b2_graphContactBlock)
                    {
                        System.SolveContactsTask(startIndex, endIndex, Stage.colorIndex, true);
                    }
                    else if (blockType == b2SolverBlockType.b2_graphJointBlock)
                    {
                        System.SolveJointsTask(startIndex, endIndex, Stage.colorIndex, true, index);
                    }

                    break;

                case SolverStageType.b2_stageIntegratePositions:
                    System.IntegratePositionsTask(startIndex, endIndex);
                    break;

                case SolverStageType.b2_stageRelax:
                    if (blockType == b2SolverBlockType.b2_graphContactBlock)
                    {
                        System.SolveContactsTask(startIndex, endIndex, Stage.colorIndex, false);
                    }
                    else if (blockType == b2SolverBlockType.b2_graphJointBlock)
                    {
                        System.SolveJointsTask(startIndex, endIndex, Stage.colorIndex, false, index);
                    }

                    break;

                case SolverStageType.b2_stageRestitution:
                    if (blockType == b2SolverBlockType.b2_graphContactBlock)
                    {
                        System.ApplyRestitutionTask(startIndex, endIndex, Stage.colorIndex);
                    }

                    break;

                case SolverStageType.b2_stageStoreImpulses:
                    System.StoreImpulsesTask(startIndex, endIndex);
                    break;
            }
        }
    }

    private void ExecuteMainStage(in SolverStage stage)
    {
        _solveJob.Stage = stage;
        _parallel.ProcessNow(_solveJob, stage.blockCount);
    }

    private void SolverTask()
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

		int stageIndex = 0;
        var activeColorCount = _activeColorCount;

		// This stage loops over all awake joints
		ExecuteMainStage(_contextStages[stageIndex++]);

		// This stage loops over all contact constraints
		DebugTools.Assert(_contextStages[stageIndex].type == SolverStageType.b2_stagePrepareContacts );
		ExecuteMainStage(_contextStages[stageIndex++]);
		stageIndex += 1;

        // Single-threaded overflow work. These constraints don't fit in the graph coloring.
		PrepareOverflowJoints();
		PrepareOverflowContacts();

		int subStepCount = _substepCount;

		for ( int i = 0; i < subStepCount; ++i )
		{
			// stage index restarted each iteration
			// syncBits still increases monotonically because the upper bits increase each iteration
			int iterStageIndex = stageIndex;

			// integrate velocities
			DebugTools.Assert( _contextStages[iterStageIndex].type == SolverStageType.b2_stageIntegrateVelocities );
			ExecuteMainStage( _contextStages[iterStageIndex]);
			iterStageIndex += 1;

			// warm start constraints
			WarmStartOverflowJoints();
			WarmStartOverflowContacts();

			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				DebugTools.Assert(_contextStages[iterStageIndex].type == SolverStageType.b2_stageWarmStart);
				ExecuteMainStage(_contextStages[iterStageIndex]);
				iterStageIndex += 1;
			}

            // solve constraints
			bool useBias = true;

			for ( int j = 0; j < Iterations; ++j )
			{
				// Overflow constraints have lower priority
				SolveOverflowJoints(useBias);
				SolveOverflowContacts(useBias);

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					DebugTools.Assert(_contextStages[iterStageIndex].type == SolverStageType.b2_stageSolve);
					ExecuteMainStage(_contextStages[iterStageIndex]);
					iterStageIndex += 1;
				}
            }

			// integrate positions
			DebugTools.Assert( _contextStages[iterStageIndex].type == SolverStageType.b2_stageIntegratePositions );
			ExecuteMainStage(_contextStages[iterStageIndex]);
			iterStageIndex += 1;

			// relax constraints
			useBias = false;
			for ( int j = 0; j < RelaxIterations; ++j )
			{
				SolveOverflowJoints(useBias);
				SolveOverflowContacts(useBias);

				for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
				{
					DebugTools.Assert(_contextStages[iterStageIndex].type == SolverStageType.b2_stageRelax);
					ExecuteMainStage(_contextStages[iterStageIndex]);
					iterStageIndex += 1;
				}
            }
        }

		// advance the stage according to the sub-stepping tasks just completed
		// integrate velocities / warm start / solve / integrate positions / relax
		stageIndex += 1 + activeColorCount + Iterations * activeColorCount + 1 + RelaxIterations * activeColorCount;

		// Restitution
		{
			ApplyOverflowRestitution();

			int iterStageIndex = stageIndex;
			for ( int colorIndex = 0; colorIndex < activeColorCount; ++colorIndex )
			{
				DebugTools.Assert(_contextStages[iterStageIndex].type == SolverStageType.b2_stageRestitution);
				ExecuteMainStage(_contextStages[iterStageIndex]);
				iterStageIndex += 1;
			}
			// graphSyncIndex += 1;
			stageIndex += activeColorCount;
		}

		StoreOverflowImpulses();

		DebugTools.Assert(_contextStages[stageIndex].type == SolverStageType.b2_stageStoreImpulses);
		ExecuteMainStage(_contextStages[stageIndex]);

		DebugTools.Assert(stageIndex + 1 == _stageCount);
		return;
    }
}
