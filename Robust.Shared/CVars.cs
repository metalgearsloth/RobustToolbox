using System;
using System.Runtime.InteropServices;
using Robust.Shared.Configuration;
using Robust.Shared.Log;
using Robust.Shared.Network;

namespace Robust.Shared
{
    [CVarDefs]
    public abstract class CVars
    {
        protected CVars()
        {
            throw new InvalidOperationException("This class must not be instantiated");
        }

        /*
         * NET
         */

        public static readonly CVarDef<int> NetPort =
            CVarDef.Create("net.port", 1212, CVar.ARCHIVE);

        public static readonly CVarDef<int> NetSendBufferSize =
            CVarDef.Create("net.sendbuffersize", 131071, CVar.ARCHIVE);

        public static readonly CVarDef<int> NetReceiveBufferSize =
            CVarDef.Create("net.receivebuffersize", 131071, CVar.ARCHIVE);

        public static readonly CVarDef<bool> NetVerbose =
            CVarDef.Create("net.verbose", false);

        public static readonly CVarDef<string> NetServer =
            CVarDef.Create("net.server", "127.0.0.1", CVar.ARCHIVE | CVar.CLIENTONLY);

        public static readonly CVarDef<int> NetUpdateRate =
            CVarDef.Create("net.updaterate", 20, CVar.ARCHIVE | CVar.CLIENTONLY);

        public static readonly CVarDef<int> NetCmdRate =
            CVarDef.Create("net.cmdrate", 30, CVar.ARCHIVE | CVar.CLIENTONLY);

        public static readonly CVarDef<int> NetRate =
            CVarDef.Create("net.rate", 10240, CVar.ARCHIVE | CVar.REPLICATED | CVar.CLIENTONLY);

        // That's comma-separated, btw.
        public static readonly CVarDef<string> NetBindTo =
            CVarDef.Create("net.bindto", "0.0.0.0,::", CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<bool> NetDualStack =
            CVarDef.Create("net.dualstack", false, CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<bool> NetInterp =
            CVarDef.Create("net.interp", true, CVar.ARCHIVE);

        public static readonly CVarDef<int> NetInterpRatio =
            CVarDef.Create("net.interp_ratio", 0, CVar.ARCHIVE);

        public static readonly CVarDef<bool> NetLogging =
            CVarDef.Create("net.logging", false, CVar.ARCHIVE);

        public static readonly CVarDef<bool> NetPredict =
            CVarDef.Create("net.predict", true, CVar.ARCHIVE);

        public static readonly CVarDef<int> NetPredictTickBias =
            CVarDef.Create("net.predict_tick_bias", 1, CVar.ARCHIVE);

        // On Windows we default this to 16ms lag bias, to account for time period lag in the Lidgren thread.
        // Basically due to how time periods work on Windows, messages are (at worst) time period-delayed when sending.
        // BUT! Lidgren's latency calculation *never* measures this due to how it works.
        // This broke some prediction calculations quite badly so we bias them to mask it.
        // This is not necessary on Linux because Linux, for better or worse,
        // just has the Lidgren thread go absolute brr polling.
        public static readonly CVarDef<float> NetPredictLagBias = CVarDef.Create(
                "net.predict_lag_bias",
                RuntimeInformation.IsOSPlatform(OSPlatform.Windows) ? 0.016f : 0,
                CVar.ARCHIVE);

        public static readonly CVarDef<int> NetStateBufMergeThreshold =
            CVarDef.Create("net.state_buf_merge_threshold", 5, CVar.ARCHIVE);

        public static readonly CVarDef<bool> NetPVS =
            CVarDef.Create("net.pvs", true, CVar.ARCHIVE | CVar.REPLICATED | CVar.SERVER);

        public static readonly CVarDef<float> NetMaxUpdateRange =
            CVarDef.Create("net.maxupdaterange", 12.5f, CVar.ARCHIVE | CVar.REPLICATED | CVar.SERVER);

        public static readonly CVarDef<bool> NetLogLateMsg =
            CVarDef.Create("net.log_late_msg", true);

        public static readonly CVarDef<int> NetTickrate =
            CVarDef.Create("net.tickrate", 60, CVar.ARCHIVE | CVar.REPLICATED | CVar.SERVER);

        public static readonly CVarDef<int> SysWinTickPeriod =
            CVarDef.Create("sys.win_tick_period", 3, CVar.SERVERONLY);

#if DEBUG
        public static readonly CVarDef<float> NetFakeLoss = CVarDef.Create("net.fakeloss", 0f, CVar.CHEAT);
        public static readonly CVarDef<float> NetFakeLagMin = CVarDef.Create("net.fakelagmin", 0f, CVar.CHEAT);
        public static readonly CVarDef<float> NetFakeLagRand = CVarDef.Create("net.fakelagrand", 0f, CVar.CHEAT);
        public static readonly CVarDef<float> NetFakeDuplicates = CVarDef.Create("net.fakeduplicates", 0f, CVar.CHEAT);
#endif

        /*
         * METRICS
         */

        public static readonly CVarDef<bool> MetricsEnabled =
            CVarDef.Create("metrics.enabled", false, CVar.SERVERONLY);

        public static readonly CVarDef<string> MetricsHost =
            CVarDef.Create("metrics.host", "localhost", CVar.SERVERONLY);

        public static readonly CVarDef<int> MetricsPort =
            CVarDef.Create("metrics.port", 44880, CVar.SERVERONLY);

        /*
         * STATUS
         */

        public static readonly CVarDef<bool> StatusEnabled =
            CVarDef.Create("status.enabled", true, CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<string> StatusBind =
            CVarDef.Create("status.bind", "*:1212", CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<int> StatusMaxConnections =
            CVarDef.Create("status.max_connections", 5, CVar.SERVERONLY);

        public static readonly CVarDef<string> StatusConnectAddress =
            CVarDef.Create("status.connectaddress", "", CVar.ARCHIVE | CVar.SERVERONLY);

        /*
         * BUILD
         */

        public static readonly CVarDef<string> BuildEngineVersion =
            CVarDef.Create("build.engine_version", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> BuildForkId =
            CVarDef.Create("build.fork_id", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> BuildVersion =
            CVarDef.Create("build.version", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> BuildDownloadUrl =
            CVarDef.Create("build.download_url", string.Empty, CVar.SERVERONLY);

        public static readonly CVarDef<string> BuildHash =
            CVarDef.Create("build.hash", "", CVar.SERVERONLY);

        /*
         * WATCHDOG
         */

        public static readonly CVarDef<string> WatchdogToken =
            CVarDef.Create("watchdog.token", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> WatchdogKey =
            CVarDef.Create("watchdog.key", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> WatchdogBaseUrl =
            CVarDef.Create("watchdog.baseUrl", "http://localhost:5000", CVar.SERVERONLY);

        /*
         * GAME
         */

        public static readonly CVarDef<int> GameMaxPlayers =
            CVarDef.Create("game.maxplayers", 32, CVar.ARCHIVE | CVar.REPLICATED | CVar.SERVER);

        public static readonly CVarDef<string> GameHostName =
            CVarDef.Create("game.hostname", "MyServer", CVar.ARCHIVE | CVar.REPLICATED | CVar.SERVER);

        /*
         * LOG
         */

        public static readonly CVarDef<bool> LogEnabled =
            CVarDef.Create("log.enabled", true, CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<string> LogPath =
            CVarDef.Create("log.path", "logs", CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<string> LogFormat =
            CVarDef.Create("log.format", "log_%(date)s-T%(time)s.txt", CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<LogLevel> LogLevel =
            CVarDef.Create("log.level", Log.LogLevel.Info, CVar.ARCHIVE | CVar.SERVERONLY);

        public static readonly CVarDef<bool> LogRuntimeLog =
            CVarDef.Create("log.runtimelog", true, CVar.ARCHIVE | CVar.SERVERONLY);

        /*
         * LOKI
         */

        public static readonly CVarDef<bool> LokiEnabled =
            CVarDef.Create("loki.enabled", false, CVar.SERVERONLY);

        public static readonly CVarDef<string> LokiName =
            CVarDef.Create("loki.name", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> LokiAddress =
            CVarDef.Create("loki.address", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> LokiUsername =
            CVarDef.Create("loki.username", "", CVar.SERVERONLY);

        public static readonly CVarDef<string> LokiPassword =
            CVarDef.Create("loki.password", "", CVar.SERVERONLY);

        /*
         * AUTH
         */

        public static readonly CVarDef<int> AuthMode =
            CVarDef.Create("auth.mode", (int) Network.AuthMode.Optional, CVar.SERVERONLY);

        public static readonly CVarDef<bool> AuthAllowLocal =
            CVarDef.Create("auth.allowlocal", true, CVar.SERVERONLY);

        // Only respected on server, client goes through IAuthManager for security.
        public static readonly CVarDef<string> AuthServer =
            CVarDef.Create("auth.server", AuthManager.DefaultAuthServer, CVar.SERVERONLY);

        /*
         * DISPLAY
         */

        public static readonly CVarDef<bool> DisplayVSync =
            CVarDef.Create("display.vsync", true, CVar.ARCHIVE | CVar.CLIENTONLY);

        public static readonly CVarDef<int> DisplayWindowMode =
            CVarDef.Create("display.windowmode", 0, CVar.ARCHIVE | CVar.CLIENTONLY);

        public static readonly CVarDef<int> DisplayWidth =
            CVarDef.Create("display.width", 1280, CVar.CLIENTONLY);

        public static readonly CVarDef<int> DisplayHeight =
            CVarDef.Create("display.height", 720, CVar.CLIENTONLY);

        public static readonly CVarDef<int> DisplayLightMapDivider =
            CVarDef.Create("display.lightmapdivider", 2, CVar.CLIENTONLY | CVar.ARCHIVE);

        public static readonly CVarDef<int> DisplayMaxLightsPerScene =
            CVarDef.Create("display.maxlightsperscene", 128, CVar.CLIENTONLY | CVar.ARCHIVE);

        public static readonly CVarDef<bool> DisplaySoftShadows =
            CVarDef.Create("display.softshadows", true, CVar.CLIENTONLY | CVar.ARCHIVE);

        public static readonly CVarDef<float> DisplayUIScale =
            CVarDef.Create("display.uiScale", 0f, CVar.ARCHIVE | CVar.CLIENTONLY);

        public static readonly CVarDef<int> DisplayRenderer =
            CVarDef.Create("display.renderer", 0, CVar.CLIENTONLY);

        public static readonly CVarDef<int> DisplayFontDpi =
            CVarDef.Create("display.fontdpi", 96, CVar.CLIENTONLY);

        public static readonly CVarDef<string> DisplayOGLOverrideVersion =
            CVarDef.Create("display.ogl_override_version", string.Empty, CVar.CLIENTONLY);

        public static readonly CVarDef<bool> DisplayOGLCheckErrors =
            CVarDef.Create("display.ogl_check_errors", false, CVar.CLIENTONLY);

        /*
         * AUDIO
         */

        public static readonly CVarDef<string> AudioDevice =
            CVarDef.Create("audio.device", string.Empty, CVar.CLIENTONLY);

        public static readonly CVarDef<float> AudioMasterVolume =
            CVarDef.Create("audio.mastervolume", 1.0f, CVar.ARCHIVE | CVar.CLIENTONLY);

        /*
         * PLAYER
         */

        public static readonly CVarDef<string> PlayerName =
            CVarDef.Create("player.name", "JoeGenero", CVar.ARCHIVE | CVar.CLIENTONLY);

        /*
         * PHYSICS
         */

        // - Sleep
        public static readonly CVarDef<float> AngularSleepTolerance =
            CVarDef.Create("physics.angsleeptol", 2.0f / 180.0f * MathF.PI);

        public static readonly CVarDef<float> LinearSleepTolerance =
            CVarDef.Create("physics.linsleeptol", 0.001f);

        public static readonly CVarDef<bool> SleepAllowed =
            CVarDef.Create("physics.sleepallowed", true);

        // Box2D default is 0.5f
        public static readonly CVarDef<float> TimeToSleep =
            CVarDef.Create("physics.timetosleep", 0.50f);

        // - Solver
        // These are the minimum recommended by Box2D with the standard being 8 velocity 3 position iterations.
        // Trade-off is obviously performance vs how long it takes to stabilise.
        public static readonly CVarDef<int> PositionIterations =
            CVarDef.Create("physics.positer", 3);

        public static readonly CVarDef<int> VelocityIterations =
            CVarDef.Create("physics.veliter", 8);

        public static readonly CVarDef<bool> WarmStarting =
            CVarDef.Create("physics.warmstart", true);

        /// <summary>
        /// A velocity threshold for elastic collisions. Any collision with a relative linear
        /// velocity below this threshold will be treated as inelastic.
        /// </summary>
        public static readonly CVarDef<float> VelocityThreshold =
            CVarDef.Create("physics.velocitythreshold", 0.5f);

        // TODO: Copy Box2D's comments on baumgarte I think it's on the solver class.
        /// <summary>
        ///     How much overlap is resolved per tick.
        /// </summary>
        public static readonly CVarDef<float> Baumgarte =
            CVarDef.Create("physics.baumgarte", 0.2f);

        /// <summary>
        /// A small length used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        /// </summary>
        public static readonly CVarDef<float> LinearSlop =
            CVarDef.Create("physics.linearslop", 0.005f);

        /// <summary>
        /// A small angle used as a collision and constraint tolerance. Usually it is
        /// chosen to be numerically significant, but visually insignificant.
        /// </summary>
        public static readonly CVarDef<float> AngularSlop =
            CVarDef.Create("physics.angularslop", 2.0f / 180.0f * MathF.PI);

        /// <summary>
        /// The radius of the polygon/edge shape skin. This should not be modified. Making
        /// this smaller means polygons will have an insufficient buffer for continuous collision.
        /// Making it larger may create artifacts for vertex collision.
        /// </summary>
        /// <remarks>
        ///     Default is set to be 2 x linearslop. TODO Should we listen to linearslop changes?
        /// </remarks>
        public static readonly CVarDef<float> PolygonRadius =
            CVarDef.Create("physics.polygonradius", 2 * 0.005f);

        /// <summary>
        /// If true, it will run a GiftWrap convex hull on all polygon inputs.
        /// This makes for a more stable engine when given random input,
        /// but if speed of the creation of polygons are more important,
        /// you might want to set this to false.
        /// </summary>
        public static readonly CVarDef<bool> ConvexHullPolygons =
            CVarDef.Create("physics.convexhullpolygons", true);

        public static readonly CVarDef<int> MaxPolygonVertices =
            CVarDef.Create("physics.maxpolygonvertices", 8);

        public static readonly CVarDef<float> MaxLinearCorrection =
            CVarDef.Create("physics.maxlinearcorrection", 0.2f);

        public static readonly CVarDef<float> MaxAngularCorrection =
            CVarDef.Create("physics.maxangularcorrection", 8.0f / 180.0f * MathF.PI);

        // - Maximums
        // Squared
        public static readonly CVarDef<float> MaxLinVelocity =
            CVarDef.Create("physics.maxlinvelocity", 4.0f);

        // Squared
        public static readonly CVarDef<float> MaxAngVelocity =
            CVarDef.Create("physics.maxangvelocity", 0.5f * MathF.PI);

        /*
         * DISCORD
         */

        public static readonly CVarDef<bool> DiscordEnabled =
            CVarDef.Create("discord.enabled", true, CVar.CLIENTONLY);
    }
}
