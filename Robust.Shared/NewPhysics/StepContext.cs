namespace Robust.Shared.NewPhysics;

// Context for a time step. Recreated each time step.
internal record struct StepContext
{
    // time step
    public float dt;

    // inverse time step (0 if dt == 0).
    public float inv_dt;

    // sub-step
    public float h;
    public float inv_h;

    public int subStepCount;
}
