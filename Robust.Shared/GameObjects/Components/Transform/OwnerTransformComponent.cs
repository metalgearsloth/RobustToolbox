using System.Numerics;
using Robust.Shared.GameStates;
using Robust.Shared.Maths;
using Robust.Shared.ViewVariables;

namespace Robust.Shared.GameObjects;

/// <summary>
/// Gives the attached client some level of ownership over the <see cref="TransformComponent"/> for this entity.
/// </summary>
[RegisterComponent, NetworkedComponent]
public sealed partial class OwnerTransformComponent : Component
{
    [ViewVariables]
    public bool Enabled = true;

    [ViewVariables]
    public Vector2 LastPosition;

    [ViewVariables]
    public Angle LastRotation;
}
