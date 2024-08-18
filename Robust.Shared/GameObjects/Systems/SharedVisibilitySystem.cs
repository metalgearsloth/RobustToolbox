using Robust.Server.GameObjects;

namespace Robust.Shared.GameObjects;

public abstract class SharedVisibilitySystem : EntitySystem
{
    public void AddLayer(Entity<VisibilityComponent?> ent, ushort layer, bool refresh = true) { }

    public void RemoveLayer(Entity<VisibilityComponent?> ent, ushort layer, bool refresh = true) { }

    public virtual void SetLayer(Entity<VisibilityComponent?> ent, ushort layer, bool refresh = true) { }

    public virtual void RefreshVisibility(EntityUid uid, VisibilityComponent? visibilityComponent = null, MetaDataComponent? meta = null) {}

    public virtual void RefreshVisibility(Entity<VisibilityComponent?, MetaDataComponent?> ent) {}
}
