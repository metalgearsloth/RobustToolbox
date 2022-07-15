using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Players;

namespace Robust.Shared.Placement;

/// <summary>
/// Handles exclusive placement of various data, such as tiles, entities etc. Mutually exclusive with other placement systems
/// </summary>
public abstract class PlacementSystem : EntitySystem
{
    public bool Enabled { get; set; }

    protected virtual void OnEnable() {}

    protected virtual void OnDisable() {}

    /// <summary>
    /// Changes from one placement system to another.
    /// </summary>
    public void SwitchTo(PlacementSystem system)
    {
        if (Enabled)
        {
            Enabled = false;
            system.Enabled = true;
        }
    }

    public virtual bool CanPlace(ICommonSession session)
    {
        return false;
    }
}

public enum PlacementMode : byte
{
    Single,
    Grid,
    Line,
}
