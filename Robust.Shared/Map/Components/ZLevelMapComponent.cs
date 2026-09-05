using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Analyzers;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Map.Components;

/// <summary>
/// Marks a map as a member of a z-level network.
/// </summary>
[RegisterComponent, NetworkedComponent, AutoGenerateComponentState(true, fieldDeltas: true), UnsavedComponent]
[Access(typeof(ZLevelSystem))]
public sealed partial class ZLevelMapComponent : Component, IComponentDelta
{
    /// <summary>
    /// Z-level map network this map belongs to.
    /// </summary>
    [ViewVariables, AutoNetworkedField]
    public EntityUid Network;

    /// <summary>
    /// Zero-based depth of this map inside its network, ordered from lowest to highest.
    /// </summary>
    [DataField, AutoNetworkedField]
    public int Depth;
}
