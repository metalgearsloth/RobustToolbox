using Robust.Shared.GameObjects;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem : EntitySystem
{
    internal enum SetType : byte
    {
        Static = 0,
        Disabled = 1,
        Awake = 2,
        FirstSleeping = 3,
    }

    // TODO: Cvar
    private int _substeps = 4;

    internal bool _locked;

    public override void Update(float frameTime)
    {
        base.Update(frameTime);
        Step(frameTime, _substeps);
    }
}
