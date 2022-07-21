using System;
using JetBrains.Annotations;
using Robust.Client.Audio;
using Robust.Shared.Audio;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics
{
    public interface IClydeAudioSource : IDisposable
    {
        /// <summary>
        /// Audio effect to use. The Auxiliary audio effect for OpenAL is handled by the engine.
        /// </summary>
        AudioEffect? Effect { get; }
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
