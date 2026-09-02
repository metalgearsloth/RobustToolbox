using System;
using Robust.Shared.Map.Components;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Authoritative mutation API for continuous z presentation height.
/// </summary>
public abstract class SharedZLevelPresentationSystem : EntitySystem
{
    public void SetLocalHeight(Entity<ZLevelPresentationComponent?> entity, float height)
    {
        if (!Resolve(entity, ref entity.Comp) || !float.IsFinite(height) || entity.Comp.LocalHeight.Equals(height))
            return;

        var oldHeight = entity.Comp.LocalHeight + entity.Comp.VisualHeight;
        entity.Comp.LocalHeight = height;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPresentationComponent.LocalHeight));

        OnLocalHeightChanged(entity.Owner, height);
        var ev = new ZLevelPresentationChangedEvent(oldHeight, height + entity.Comp.VisualHeight);
        RaiseLocalEvent(entity.Owner, ref ev);
    }

    /// <summary>
    /// Sets a transient visual-only height without changing authoritative vertical physics.
    /// </summary>
    public void SetVisualHeight(Entity<ZLevelPresentationComponent?> entity, float height)
    {
        if (!float.IsFinite(height))
            return;

        var presentation = EnsureComp<ZLevelPresentationComponent>(entity.Owner);
        if (presentation.VisualHeight.Equals(height))
            return;

        var oldHeight = presentation.LocalHeight + presentation.VisualHeight;
        presentation.VisualHeight = height;
        var changed = new ZLevelPresentationChangedEvent(oldHeight, presentation.LocalHeight + height);
        RaiseLocalEvent(entity.Owner, ref changed);
    }

    protected virtual void OnLocalHeightChanged(EntityUid uid, float height)
    {
    }
}

[ByRefEvent]
public readonly record struct ZLevelPresentationChangedEvent(float OldHeight, float NewHeight);
