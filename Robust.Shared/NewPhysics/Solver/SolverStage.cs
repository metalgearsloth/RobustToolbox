using Robust.Shared.Collections;

namespace Robust.Shared.NewPhysics.Solver;

internal record struct SolverStage
{
    // Each stage must be completed before going to the next stage.
    // Non-iterative stages use a stage instance once while iterative stages re-use the same instance each iteration.
    public SolverStageType type;
    public ValueList<SolverBlock> blocks;
    public int blockCount;
    public int colorIndex;
    // todo consider false sharing of this atomic
    public int completionCount;
}
