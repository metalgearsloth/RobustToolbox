using System;
using JetBrains.Annotations;
using Robust.Client.Audio;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics
{
    public interface IClydeAudioSource : IDisposable
    {
        /// <summary>
        /// Auxiliary audio effect. If you wish to have a specific effect play then you need to bind
        /// the auxiliary audio effect to an effect.
        /// </summary>
        AuxiliaryAudioEffect? Effect {get; set; }
        float RolloffFactor { get; set; }
        float ReferenceDistance { get; set; }
        float MaxDistance { get; set; }
        float Pitch { get; set; }
        Vector2 Velocity { get; set; }

        void StartPlaying();
        void StopPlaying();

        bool IsPlaying { get; }

        bool IsLooping { get; set; }

        [MustUseReturnValue]
        bool SetPosition(Vector2 position);
        void SetGlobal();
        void SetVolume(float decibels);
        void SetVolumeDirect(float decibels);
        void SetOcclusion(float blocks);
        void SetPlaybackPosition(float seconds);
    }
}
