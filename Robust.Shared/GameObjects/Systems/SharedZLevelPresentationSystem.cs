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

        var oldHeight = entity.Comp.LocalHeight;
        entity.Comp.LocalHeight = height;
        DirtyField(entity.Owner, entity.Comp, nameof(ZLevelPresentationComponent.LocalHeight));

        OnLocalHeightChanged(entity.Owner, height);
        var ev = new ZLevelPresentationChangedEvent(oldHeight, height);
        RaiseLocalEvent(entity.Owner, ref ev);
    }

    protected virtual void OnLocalHeightChanged(EntityUid uid, float height)
    {
    }
}

[ByRefEvent]
public readonly record struct ZLevelPresentationChangedEvent(float OldHeight, float NewHeight);
