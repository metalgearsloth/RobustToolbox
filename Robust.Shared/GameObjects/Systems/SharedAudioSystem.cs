using JetBrains.Annotations;
using Robust.Shared.Audio;
using Robust.Shared.Configuration;
using Robust.Shared.IoC;
using Robust.Shared.Map;

namespace Robust.Shared.GameObjects
{
    public abstract class SharedAudioSystem : EntitySystem
    {
        [Dependency] protected readonly IConfigurationManager ConfigManager = default!;
        [Dependency] private readonly IMapManager _mapManager = default!;

        /// <summary>
        /// Default max range at which the sound can be heard.
        /// </summary>
        public static float DefaultSoundRange { get; private set; }

        public static float DefaultRolloffFactor { get; private set; }

        public static float DefaultReferenceDistance { get; private set; }

        public AudioParams Default { get; private set; } = new(
            0,
            1,
            "Master",
            DefaultSoundRange,
            DefaultRolloffFactor,
            DefaultReferenceDistance,
            false,
            0f);

        public override void Initialize()
        {
            base.Initialize();
            ConfigManager.OnValueChanged(CVars.AudioMaxDistance, SetAudioMaxDistance, true);
            ConfigManager.OnValueChanged(CVars.AudioRolloffFactor, SetAudioRolloffFactor, true);
            ConfigManager.OnValueChanged(CVars.AudioReferenceDistance, SetAudioReferenceDistance, true);
        }

        public override void Shutdown()
        {
            base.Shutdown();
            ConfigManager.UnsubValueChanged(CVars.AudioMaxDistance, SetAudioMaxDistance);
            ConfigManager.UnsubValueChanged(CVars.AudioRolloffFactor, SetAudioRolloffFactor);
            ConfigManager.UnsubValueChanged(CVars.AudioReferenceDistance, SetAudioReferenceDistance);
        }

        private void SetAudioReferenceDistance(float obj)
        {
            DefaultReferenceDistance = obj;
            Default = Default.WithReferenceDistance(obj);
            AudioParams.Default = AudioParams.Default.WithReferenceDistance(obj);
        }

        private void SetAudioRolloffFactor(float obj)
        {
            DefaultRolloffFactor = obj;
            Default = Default.WithRolloffFactor(obj);
            AudioParams.Default = AudioParams.Default.WithRolloffFactor(obj);
        }

        private void SetAudioMaxDistance(float obj)
        {
            DefaultSoundRange = obj;
            Default = Default.WithMaxDistance(obj);
            AudioParams.Default = AudioParams.Default.WithMaxDistance(obj);
        }

        protected EntityCoordinates GetFallbackCoordinates(MapCoordinates mapCoordinates)
        {
            if (_mapManager.TryFindGridAt(mapCoordinates, out var mapGrid))
            {
                return new EntityCoordinates(mapGrid.GridEntityId,
                    mapGrid.WorldToLocal(mapCoordinates.Position));
            }

            if (_mapManager.HasMapEntity(mapCoordinates.MapId))
            {
                return new EntityCoordinates(_mapManager.GetMapEntityId(mapCoordinates.MapId),
                    mapCoordinates.Position);
            }

            return EntityCoordinates.Invalid;
        }
    }
}
