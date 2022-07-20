using System.Threading;
using Robust.Client.Player;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.Timing;

namespace Robust.Client.Graphics.Audio
{
    internal partial class ClydeAudio
    {
        [Shared.IoC.Dependency] private readonly IConfigurationManager _cfg = default!;
        [Shared.IoC.Dependency] private readonly IEntityManager _entManager = default!;
        [Shared.IoC.Dependency] private readonly IEyeManager _eyeManager = default!;
        [Shared.IoC.Dependency] private readonly IPlayerManager _playerManager = default!;

        private Thread? _gameThread;

        public bool InitializePostWindowing()
        {
            _gameThread = Thread.CurrentThread;
            return _initializeAudio();
        }

        public void FrameProcess(FrameEventArgs eventArgs)
        {
            _updateAudio();
        }

        public void Shutdown()
        {
            _shutdownAudio();
        }

        private bool IsMainThread()
        {
            return Thread.CurrentThread == _gameThread;
        }
    }
}
