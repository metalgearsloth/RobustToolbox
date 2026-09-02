using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.Map.Components;

/// <summary>
/// Gives an entity continuous render-only height inside an ordered z-map network.
/// </summary>
/// <remarks>
/// The canonical transform remains two-dimensional and is used by physics. Presentation uses
/// <c>absoluteZ = mapDepth + localHeight</c> without modifying authored sprite offsets.
/// </remarks>
[RegisterComponent, NetworkedComponent, AutoGenerateComponentState(raiseAfterAutoHandleState: true, fieldDeltas: true)]
public sealed partial class ZLevelPresentationComponent : Component, IComponentDelta
{
    /// <summary>
    /// Authoritative local physics height. This is replicated and is never used for a merely visual incline.
    /// </summary>
    [DataField, AutoNetworkedField, ViewVariables]
    public float LocalHeight;

    /// <summary>
    /// Transient, locally derived render-only height added to <see cref="LocalHeight"/>.
    /// </summary>
    /// <remarks>
    /// This deliberately is not serialized or networked. Gameplay features can derive it from predicted XY state
    /// without changing vertical physics, while the generic render-pose path still keeps sprites and eyes aligned.
    /// </remarks>
    [ViewVariables]
    public float VisualHeight;
}
