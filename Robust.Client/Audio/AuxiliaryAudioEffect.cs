using System;
using OpenToolkit.Audio.OpenAL;
using OpenToolkit.Audio.OpenAL.Extensions.Creative.EFX;
using Robust.Shared.Maths;

namespace Robust.Client.Audio;

/// <summary>
/// Auxiliary effect slot for audio. Can be set to a specific effect.
/// You can also set audio sources to this auxiliary audio effect slot.
/// </summary>
public struct AuxiliaryAudioEffect : IDisposable
{
    internal readonly int AuxiliaryHandle;

    public AuxiliaryAudioEffect()
    {
        AuxiliaryHandle = EFX.GenAuxiliaryEffectSlot();
    }

    public void SetAudioEffect(AudioEffect effect)
    {
        // TODO: Suss this shit out
        EFX.GetSource(0, EFXSourceInteger3.AuxiliarySendFilter, out var value1, out var value2, out var value3);
        AL.Source(0, Auxi);

        if (!EFX.IsEffect(effect.EffectHandle))
            throw new InvalidOperationException($"Tried to set an invalid effect handle for auxiliary audio!");

        EFX.AuxiliaryEffectSlot(AuxiliaryHandle, EffectSlotInteger.Effect, effect.EffectHandle);
    }

    private void _validate()
    {
        if (!EFX.IsAuxiliaryEffectSlot(AuxiliaryHandle))
            throw new InvalidOperationException($"Handle {AuxiliaryHandle} is not an auxiliary effect!");
    }

    public void Dispose()
    {
        _validate();
        EFX.DeleteAuxiliaryEffectSlot(AuxiliaryHandle);
    }
}
