using System.Collections.Generic;
using System.Linq;
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
    /// Optional evenly-spaced height samples along the anchored entity's cardinal facing direction.
    /// Empty uses <see cref="Height"/> as a flat surface.
    /// </summary>
    [DataField, AutoNetworkedField]
    public List<float> HeightCurve = [];

    /// <summary>
    /// Blends both local axes when evaluating <see cref="HeightCurve"/>, for corner stairs.
    /// </summary>
    [DataField, AutoNetworkedField]
    public bool Corner;

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
    public float MinimumHeight => HeightCurve.Count == 0 ? Height : HeightCurve.Min();

    [ViewVariables]
    public float MaximumHeight => HeightCurve.Count == 0 ? Height : HeightCurve.Max();
}
