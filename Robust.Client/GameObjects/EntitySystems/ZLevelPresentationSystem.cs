using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Map.Components;

namespace Robust.Client.GameObjects;

/// <summary>
/// Converts replicated height changes into the same render-pose timeline used by transform interpolation.
/// </summary>
public sealed class ZLevelPresentationSystem : SharedZLevelPresentationSystem
{
    private readonly Dictionary<EntityUid, float> _lastHeights = new();

    public override void Initialize()
    {
        base.Initialize();
        SubscribeLocalEvent<ZLevelPresentationComponent, ComponentStartup>(OnStartup);
        SubscribeLocalEvent<ZLevelPresentationComponent, ComponentShutdown>(OnShutdown);
        SubscribeLocalEvent<ZLevelPresentationComponent, AfterAutoHandleStateEvent>(OnAfterState);
    }

    private void OnStartup(Entity<ZLevelPresentationComponent> entity, ref ComponentStartup args)
        => _lastHeights[entity.Owner] = entity.Comp.LocalHeight;

    private void OnShutdown(Entity<ZLevelPresentationComponent> entity, ref ComponentShutdown args)
        => _lastHeights.Remove(entity.Owner);

    private void OnAfterState(Entity<ZLevelPresentationComponent> entity, ref AfterAutoHandleStateEvent args)
    {
        var oldHeight = _lastHeights.GetValueOrDefault(entity.Owner, entity.Comp.LocalHeight);
        if (oldHeight.Equals(entity.Comp.LocalHeight))
            return;

        _lastHeights[entity.Owner] = entity.Comp.LocalHeight;
        var ev = new ZLevelPresentationChangedEvent(
            oldHeight + entity.Comp.VisualHeight,
            entity.Comp.LocalHeight + entity.Comp.VisualHeight);
        RaiseLocalEvent(entity.Owner, ref ev);
    }

    protected override void OnLocalHeightChanged(EntityUid uid, float height)
        => _lastHeights[uid] = height;
}
