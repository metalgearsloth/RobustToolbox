using Robust.Shared.Analyzers;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Physics.Components;

/// <summary>
/// Defines a walkable height profile on an anchored entity, such as a wall top or stairway.
/// </summary>
[RegisterComponent, NetworkedComponent, AutoGenerateComponentState(fieldDeltas: true)]
[Access(typeof(ZLevelPhysicsSystem), typeof(ZLevelSupportSystem))]
public sealed partial class ZLevelHighGroundComponent : Component, IComponentDelta
{
    public const string DefaultSurfaceFixture = "zLevelTop";

    /// <summary>
    /// Flat walkable surface height above the provider's map plane.
    /// </summary>
    [DataField, AutoNetworkedField]
    public float Height = 1.05f;

    /// <summary>
    /// Whether fixtures beneath the walkable surface represent a solid vertical volume. Walls use this; flat platforms usually do not.
    /// </summary>
    [DataField, AutoNetworkedField]
    public bool SolidVolume = true;

    /// <summary>
    /// Authored fixture whose XY shape defines the walkable surface.
    /// </summary>
    [DataField, AutoNetworkedField]
    public string SurfaceFixture = DefaultSurfaceFixture;

    [ViewVariables]
    public float MinimumHeight => Height;

    [ViewVariables]
    public float MaximumHeight => Height;
}
