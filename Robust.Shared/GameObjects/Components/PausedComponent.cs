namespace Robust.Shared.GameObjects;

/// <summary>
///     Internal tag component present on entities that are currently paused.
/// </summary>
[RegisterComponent, UnsavedComponent]
public sealed partial class PausedComponent : Component;
