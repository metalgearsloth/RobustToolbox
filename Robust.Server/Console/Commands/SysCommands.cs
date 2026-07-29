using System.Text;
using Robust.Shared.Configuration;
using Robust.Shared.Console;
using Robust.Shared.IoC;
using Robust.Shared.Network;
using Robust.Shared.Timing;

namespace Robust.Server.Console.Commands
{
    /*
    // Disabled for now since it doesn't actually work.
    sealed class RestartCommand : LocalizedCommands
    {
        [Dependency] private IBaseServer _server = default!;

        public override string Command => "restart";

        public override void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            _server.Restart();
        }
    }
    */

    sealed partial class ShutdownCommand : LocalizedCommands
    {
        [Dependency] private IBaseServer _server = default!;

        public override string Command => "shutdown";

        public override void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            _server.Shutdown(argStr[$"{Command} ".Length..].Trim());
        }
    }

    sealed partial class NetworkAuditCommand : LocalizedCommands
    {
        [Dependency] private INetManager _netManager = default!;

        public override string Command => "netaudit";
        public override void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            var callbacks = ((NetManager)_netManager).CallbackAudit;

            var sb = new StringBuilder();

            foreach (var kvCallback in callbacks)
            {
                var msgType = kvCallback.Key;
                var call = kvCallback.Value;

                sb.AppendLine($"Type: {msgType.Name.PadRight(16)} Call:{call.Target}");
            }

            shell.WriteLine(sb.ToString());
        }
    }

    sealed partial class NetworkStatsCommand : LocalizedCommands
    {
        [Dependency] private INetManager _netManager = default!;

        public override string Command => "netstats";

        public override void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            if (args.Length != 0)
            {
                shell.WriteError(Help);
                return;
            }

            var manager = (NetManager) _netManager;
            var peers = manager.NetPeers;
            var builder = new StringBuilder();

            builder.AppendLine(
                $"Network manager: mode={(manager.IsServer ? "server" : "client")}, running={manager.IsRunning}, connected={manager.IsConnected}, peers={peers.Count}, channels={manager.ChannelCount}");

            for (var peerIndex = 0; peerIndex < peers.Count; peerIndex++)
            {
                var peer = peers[peerIndex];
                var config = peer.Configuration;
                var socket = peer.Socket;
                var localEndPoint = socket?.LocalEndPoint;
                var statistics = peer.Statistics;
                var connections = peer.Connections;

                builder.AppendLine();
                builder.AppendLine($"Peer {peerIndex}: status={peer.Status}, connections={connections.Count}");
                builder.AppendLine(
                    $"  Socket: local={localEndPoint?.ToString() ?? "(not bound)"}, family={socket?.AddressFamily.ToString() ?? config.LocalAddress.AddressFamily.ToString()}, dual-stack={socket?.DualMode ?? config.DualStack}");
                builder.AppendLine(
                    $"  MTU config: IPv4={config.MaximumTransmissionUnit}, IPv6={config.MaximumTransmissionUnitV6}, auto-expand={config.AutoExpandMTU}, probe-frequency={config.ExpandMTUFrequency:F2}s, max-failures={config.ExpandMTUFailAttempts}, unreliable-above-MTU={config.UnreliableSizeBehaviour}");
                builder.AppendLine(
                    $"  Buffers: configured receive={config.ReceiveBufferSize}, send={config.SendBufferSize}; socket receive={socket?.ReceiveBufferSize.ToString() ?? "n/a"}, send={socket?.SendBufferSize.ToString() ?? "n/a"}");
                builder.AppendLine(
                    $"  Limits: packets/heartbeat={config.MaximumPacketsPerHeartbeat}, bytes/heartbeat={config.MaximumBytesPerHeartbeat}, ping-interval={config.PingInterval:F2}s, timeout={config.ConnectionTimeout:F2}s");
                builder.AppendLine(
                    $"  Totals: sent={statistics.SentBytes} B/{statistics.SentPackets} packets/{statistics.SentMessages} messages; received={statistics.ReceivedBytes} B/{statistics.ReceivedPackets} packets/{statistics.ReceivedMessages} messages; dropped-messages={statistics.DroppedMessages}, resent-messages-delay={statistics.ResentMessagesDueToDelay}, resent-messages-hole={statistics.ResentMessagesDueToHole}");

                for (var connectionIndex = 0; connectionIndex < connections.Count; connectionIndex++)
                {
                    var connection = connections[connectionIndex];
                    var connectionStats = connection.Statistics;
                    var failedMtu = connection.SmallestFailedMTU?.ToString() ?? "none";
                    var remoteAddress = connection.RemoteEndPoint.Address;
                    var remoteFamily = remoteAddress.IsIPv4MappedToIPv6
                        ? "InterNetwork (IPv4-mapped IPv6)"
                        : remoteAddress.AddressFamily.ToString();

                    builder.AppendLine(
                        $"  Connection {connectionIndex}: remote={connection.RemoteEndPoint}, family={remoteFamily}, status={connection.Status}, RTT={connection.AverageRoundtripTime * 1000:F1}ms");
                    builder.AppendLine(
                        $"    MTU: current={connection.CurrentMTU}, expansion={connection.MTUExpansionStatus}, largest-success={connection.LargestSuccessfulMTU}, smallest-failure={failedMtu}, last-probe={connection.LastSentMTUAttemptSize}, probe-failures={connection.MTUAttemptFailures}/{config.ExpandMTUFailAttempts}, send-too-large={connection.MTUSendFailures}");
                    builder.AppendLine(
                        $"    Traffic: sent={connectionStats.SentBytes} B/{connectionStats.SentPackets} packets/{connectionStats.SentMessages} messages; received={connectionStats.ReceivedBytes} B/{connectionStats.ReceivedPackets} packets/{connectionStats.ReceivedMessages} messages; dropped-messages={connectionStats.DroppedMessages}, resent-messages-delay={connectionStats.ResentMessagesDueToDelay}, resent-messages-hole={connectionStats.ResentMessagesDueToHole}");
                }
            }

            shell.WriteLine(builder.ToString().TrimEnd());
        }
    }

    sealed partial class ShowTimeCommand : LocalizedCommands
    {
        [Dependency] private IGameTiming _timing = default!;

        public override string Command => "showtime";

        public override void Execute(IConsoleShell shell, string argStr, string[] args)
        {
            shell.WriteLine($"Paused: {_timing.Paused}, CurTick: {_timing.CurTick}, CurTime: {_timing.CurTime}, RealTime: {_timing.RealTime}");
        }
    }
}
