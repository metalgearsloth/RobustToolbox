using Robust.Shared.Maths;

namespace Robust.Shared.Audio;

public struct ReverbProperties
{
    /// <summary>
    /// An identifier used on the client to know if it needs to use a different auxiliary effect slot.
    /// </summary>
    public readonly string ID;

    public float Density = default;

    public float Diffusion = default;

    public float Gain = default;

    public float GainHF = default;

    public float GainLF = default;

    public float DecayTime = default;

    public float DecayHFRatio = default;

    public float DecayLFRatio = default;

    public float ReflectionsGain = default;

    public float ReflectionsDelay = default;

    public Vector3 ReflectionsPan = default;

    public float LateReverbGain = default;

    public float LateReverbDelay = default;

    public Vector3 LateReverbPan = default;

    public float EchoTime = default;

    public float EchoDepth = default;

    public float ModulationTime = default;

    public float ModulationDepth = default;

    public float AirAbsorptionGainHF = default;

    public float HFReference = default;

    public float LFReference = default;

    public float RoomRolloffFactor = default;

    public int DecayHFLimit = default;

    public ReverbProperties(string id)
    {
        ID = id;
    }
}
