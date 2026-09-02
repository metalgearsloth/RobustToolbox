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
[Access(typeof(ZLevelPhysicsSystem))]
public sealed partial class ZLevelHighGroundComponent : Component, IComponentDelta
{
    /// <summary>
    /// Evenly-spaced height samples along the anchored entity's cardinal facing direction.
    /// </summary>
    [DataField, AutoNetworkedField]
    public List<float> HeightCurve = [1.05f, 1.05f];

    /// <summary>
    /// Whether fixtures beneath the walkable surface represent a solid vertical volume. Walls use this; ramps do not.
    /// </summary>
    [DataField, AutoNetworkedField]
    public bool SolidVolume = true;

    /// <summary>
    /// Blends both local axes when evaluating the height profile, for corner stairs.
    /// </summary>
    [DataField, AutoNetworkedField]
    public bool Corner;

    /// <summary>
    /// Optional fixture whose XY shape defines the walkable surface. Empty uses all provider fixtures.
    /// </summary>
    [DataField, AutoNetworkedField]
    public string SurfaceFixture = string.Empty;

    /// <summary>
    /// Stable authored priority used after height and hysteresis. Entity UID is the final tie-breaker.
    /// </summary>
    [DataField, AutoNetworkedField]
    public int Priority;

    [ViewVariables]
    public float MinimumHeight => HeightCurve.Count == 0 ? 0f : HeightCurve.Min();

    [ViewVariables]
    public float MaximumHeight => HeightCurve.Count == 0 ? 0f : HeightCurve.Max();
}
