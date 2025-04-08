using Robust.Shared.Audio.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Serialization.Manager.Attributes;

namespace Robust.Shared.Audio.Components;

/// <summary>
/// Marks this entity as being spawned for audio presets.
/// </summary>
[RegisterComponent, NetworkedComponent, AutoGenerateComponentState(fieldDeltas: true), Access(typeof(SharedAudioSystem))]
public sealed partial class AudioPresetComponent : Component
{
    /// <summary>
    /// The <see cref="AudioPresetPrototype"/> used to spawn this entity.
    /// </summary>
    [DataField, AutoNetworkedField]
    public string? Preset;
}
