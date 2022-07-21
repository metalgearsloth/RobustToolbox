using System.Collections.Generic;
using Robust.Client.Audio;

namespace Robust.Client.GameObjects;

public sealed partial class AudioSystem
{
    /// <summary>
    /// OpenAL requires effects to be bound to auxiliary effect handles. This means you could have 1 handle
    /// for an entire area but re-use the effect across multiple areas.
    /// In our case we handle it for content.
    /// </summary>
    private Dictionary<string, AuxiliaryAudioEffect> _auxiliary = new();
    private Dictionary<string, AudioEffect> _effects = new ();

    private void InitializeEffects()
    {

    }

    private void ShutdownEffects()
    {
        foreach (var (_, aux) in _auxiliary)
        {
            aux.Dispose();
        }

        _auxiliary.Clear();

        foreach (var (_, effect) in _effects)
        {
            effect.Dispose();
        }

        _effects.Clear();
    }
}
