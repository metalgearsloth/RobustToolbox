using System.Collections.Generic;
using System.Numerics;
using Robust.Shared.Analyzers;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Maths;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Map.Components;

/// <summary>
/// Runtime map group and replicated presentation configuration for ordered z-level rendering.
/// </summary>
[RegisterComponent, NetworkedComponent, AutoGenerateComponentState(true, fieldDeltas: true)]
[Access(typeof(ZLevelSystem))]
public sealed partial class ZLevelMapNetworkComponent : Component, IComponentDelta
{
    /// <summary>
    /// Runtime z-level maps in lower-to-upper order.
    /// </summary>
    [ViewVariables, Access(Other = AccessPermissions.ReadExecute)]
    public IReadOnlyList<EntityUid> SortedZLevels => SortedZLevelsInternal;

    /// <summary>
    /// Mutable runtime z-level map storage.
    /// </summary>
    [AutoNetworkedField, Access(typeof(ZLevelSystem), Other = AccessPermissions.None)]
    internal readonly List<EntityUid> SortedZLevelsInternal = new();

    /// <summary>
    /// Maps in lower-to-upper order, stored only in full saves so the runtime network can be rebuilt after loading.
    /// </summary>
    [DataField]
    public List<EntityUid> Maps = new();

    /// <summary>
    /// Map-space visual displacement per unit of absolute z. Physics and authored transforms are never displaced.
    /// </summary>
    [DataField, AutoNetworkedField]
    public Vector2 ProjectionOffset = new(0f, 0.7f);

    /// <summary>
    /// Maximum number of lower maps rendered and included in PVS for an eye in this network.
    /// </summary>
    [DataField, AutoNetworkedField]
    public int VisibleLevelsBelow = 3;

    /// <summary>
    /// Maximum number of upper maps rendered and included in PVS for an eye in this network.
    /// </summary>
    [DataField, AutoNetworkedField]
    public int VisibleLevelsAbove;

    /// <summary>
    /// Stops drawing deeper maps after an opaque level covers the viewport.
    /// </summary>
    [DataField, AutoNetworkedField]
    public bool StopAtOpaque = true;

    /// <summary>
    /// Whether lower-level post-processing is enabled. Tint remains independently configurable.
    /// </summary>
    [DataField, AutoNetworkedField]
    public bool LowerPostShaderEnabled = true;

    /// <summary>
    /// Optional game-provided shader prototype for lower levels. Null uses the engine default.
    /// </summary>
    [DataField, AutoNetworkedField]
    public string? LowerPostShader;

    /// <summary>
    /// Blur radius in screen pixels for the engine lower-level shader.
    /// </summary>
    [DataField, AutoNetworkedField]
    public float LowerBlurRadius = 2f;

    /// <summary>
    /// Darkening strength for the engine lower-level shader.
    /// </summary>
    [DataField, AutoNetworkedField]
    public float LowerDarkenStrength = 0.3f;

    /// <summary>
    /// Additional lower-level tint composited after the post-shader.
    /// </summary>
    [DataField, AutoNetworkedField]
    public Color LowerTint = Color.Transparent;
}
