namespace Robust.Shared.Audio
{
    public interface IPlayingAudioStream
    {
        // TODO: Maybe just don't expose AuxiliaryEffectSlot to content and manage it ourselves?
        AudioEffect
        bool IsLooping { get; set; }
        void Stop();
    }
}
