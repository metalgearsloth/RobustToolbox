﻿#if !FULL_RELEASE
using System;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;
using Robust.Client.Utility;
using Robust.Shared.Console;
using Robust.Shared.IoC;
using Robust.Shared.Network;

namespace Robust.Client.Console.Commands
{
    internal sealed class LauncherAuthCommand : IConsoleCommand
    {
        public string Command => "launchauth";
        public string Description => "Load authentication tokens from launcher data to aid in testing of live servers";
        public string Help => "launchauth [account name]";

        public void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            var wantName = args.Length > 0 ? args[0] : null;

            var basePath = Path.GetDirectoryName(UserDataDir.GetUserDataDir())!;
            var cfgPath = Path.Combine(basePath, "launcher", "launcher_config.json");

            var data = JsonSerializer.Deserialize<LauncherConfig>(File.ReadAllText(cfgPath))!;

            var login = wantName != null
                ? data.Logins.FirstOrDefault(p => p.Username == wantName)
                : data.Logins.FirstOrDefault();

            if (login == null)
            {
                shell.WriteLine("Unable to find a matching login");
                return;
            }

            var token = login.Token.Token;
            var userId = login.UserId;

            var cfg = IoCManager.Resolve<IAuthManager>();
            cfg.Token = token;
            cfg.UserId = new NetUserId(Guid.Parse(userId));
        }

        private sealed class LauncherConfig
        {
            [JsonInclude] [JsonPropertyName("logins")]
            public LauncherLogin[] Logins = default!;
        }

        private sealed class LauncherLogin
        {
            [JsonInclude] public string Username = default!;
            [JsonInclude] public string UserId = default!;
            [JsonInclude] public LauncherToken Token = default!;
        }

        private sealed class LauncherToken
        {
            [JsonInclude] public string Token = default!;
        }
    }
}

#endif
