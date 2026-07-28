using Robust.Shared.GameStates;

namespace Robust.Shared.GameObjects;

public sealed class PredictedSpawnSystem : EntitySystem
{
    public override void Initialize()
    {
        SubscribeLocalEvent<PredictedSpawnComponent, ComponentGetState>(OnGetState);
        SubscribeLocalEvent<PredictedSpawnComponent, ComponentHandleState>(OnHandleState);
    }

    private void OnGetState(EntityUid uid, PredictedSpawnComponent component, ref ComponentGetState args)
    {
        args.State = new PredictedSpawnComponentState(
            component.SpawnTick,
            component.SpawnIndex,
            component.SpawnId,
            component.SpawnOwner);
    }

    private void OnHandleState(EntityUid uid, PredictedSpawnComponent component, ref ComponentHandleState args)
    {
        if (args.Current is not PredictedSpawnComponentState state)
            return;

        EntityManager.SetPredictedSpawnReference(
            uid,
            component,
            state.SpawnTick,
            state.SpawnIndex,
            state.SpawnId,
            state.SpawnOwner,
            false);
    }
}
