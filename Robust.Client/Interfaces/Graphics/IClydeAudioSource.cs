using System;
using JetBrains.Annotations;
using Robust.Client.Graphics.Clyde;
using Robust.Shared.Maths;

namespace Robust.Client.Interfaces.Graphics
{
    public interface IClydeAudioSource : IDisposable
    {
        void StartPlaying();
        void StopPlaying();

        bool IsPlaying { get; }

        bool IsLooping { get; set; }

        [MustUseReturnValue]
        bool SetPosition(Vector2 position);
        void SetPitch(float pitch);
        void SetGlobal();
        void SetEffect(AudioEffect effect);
        void SetVolume(float decibels);
        void SetOcclusion(float blocks);
        void SetPlaybackPosition(float seconds);
    }
}
