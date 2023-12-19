using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading;
using OpenTK.Audio.OpenAL;
using OpenTK.Audio.OpenAL.Extensions.Creative.EFX;
using Robust.Client.Audio.Sources;
using Robust.Shared;
using Robust.Shared.Audio;
using Robust.Shared.Configuration;
using Robust.Shared.Log;
using Robust.Shared.Utility;

namespace Robust.Client.Audio;

internal sealed partial class AudioManager : IAudioInternal
{
    [Shared.IoC.Dependency] private readonly IConfigurationManager _cfg = default!;
    [Shared.IoC.Dependency] private readonly ILogManager _logMan = default!;

    private Thread? _gameThread;

    private ALDevice _openALDevice;
    private ALContext _openALContext;

    private readonly List<LoadedAudioSample> _audioSampleBuffers = new();

    private readonly Dictionary<int, WeakReference<BaseAudioSource>> _audioSources =
        new();

    private readonly Dictionary<int, WeakReference<BufferedAudioSource>> _bufferedAudioSources =
        new();

    private readonly HashSet<string> _alcDeviceExtensions = new();
    private readonly HashSet<string> _alContextExtensions = new();
    private Attenuation _attenuation;

    public bool HasAlDeviceExtension(string extension) => _alcDeviceExtensions.Contains(extension);
    public bool HasAlContextExtension(string extension) => _alContextExtensions.Contains(extension);

    internal bool IsEfxSupported;

    internal ISawmill OpenALSawmill = default!;

    private bool _audioOpenDevice()
    {
        string? preferredDevice = _cfg.GetCVar(CVars.AudioDevice);

        // Open device.
        if (string.IsNullOrEmpty(preferredDevice))
        {
            preferredDevice = null;
        }

        return SetAudioDevice(preferredDevice);
    }

    private void InitializeAudio()
    {
        OpenALSawmill = _logMan.GetSawmill("clyde.oal");

        if (!_audioOpenDevice())
            return;

        _cfg.OnValueChanged(CVars.AudioMasterVolume, SetMasterGain, true);
    }

    internal bool IsMainThread()
    {
        return Thread.CurrentThread == _gameThread;
    }

    private static void RemoveEfx((int sourceHandle, int filterHandle) handles)
    {
        if (handles.filterHandle != 0)
            EFX.DeleteFilter(handles.filterHandle);
    }

    private void _checkAlcError(ALDevice device,
        [CallerMemberName] string callerMember = "",
        [CallerLineNumber] int callerLineNumber = -1)
    {
        var error = ALC.GetError(device);
        if (error != AlcError.NoError)
        {
            OpenALSawmill.Error("[{0}:{1}] ALC error: {2}", callerMember, callerLineNumber, error);
        }
    }

    /// <summary>
    /// Like _checkAlError but allows custom data to be passed in as relevant.
    /// </summary>
    internal void LogALError(string message, [CallerMemberName] string callerMember = "", [CallerLineNumber] int callerLineNumber = -1)
    {
        var error = AL.GetError();
        if (error != ALError.NoError)
        {
            OpenALSawmill.Error("[{0}:{1}] AL error: {2}, {3}", callerMember, callerLineNumber, error, message);
        }
    }

    public void _checkAlError([CallerMemberName] string callerMember = "", [CallerLineNumber] int callerLineNumber = -1)
    {
        var error = AL.GetError();
        if (error != ALError.NoError)
        {
            OpenALSawmill.Error("[{0}:{1}] AL error: {2}", callerMember, callerLineNumber, error);
        }
    }

    private sealed class LoadedAudioSample
    {
        public readonly int BufferHandle;

        public LoadedAudioSample(int bufferHandle)
        {
            BufferHandle = bufferHandle;
        }
    }
}
