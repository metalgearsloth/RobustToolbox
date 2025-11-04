namespace Robust.Shared.NewPhysics;

internal record struct WorkerContext
{
    public StepContext context;
    public int workerIndex;
    public void* userTask;
}
