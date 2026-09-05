using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Shared;
using Robust.Shared.Analyzers;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Player;
using Robust.Shared.Reflection;
using Robust.UnitTesting;

namespace Robust.UnitTesting.Shared.Map;

[TestFixture]
internal sealed class ZLevelLocalEntityEventTest : RobustIntegrationTest
{
    [Test]
    public async Task LocalEntityChangingZLevelsRaisesEvent()
    {
        var clientOptions = new ClientIntegrationOptions { Pool = false };
        clientOptions.BeforeStart = () =>
        {
            IoCManager.Resolve<IEntitySystemManager>().LoadExtraSystemType<ZLevelChangeRecorderSystem>();
        };

        await using var pair = await StartConnectedPair(new ServerIntegrationOptions { Pool = false }, clientOptions);
        var (client, server) = pair;

        var sEntMan = server.ResolveDependency<IEntityManager>();
        var cEntMan = client.ResolveDependency<IEntityManager>();
        var sConfig = server.ResolveDependency<IConfigurationManager>();
        var sPlayerMan = server.ResolveDependency<ISharedPlayerManager>();
        var cPlayerMan = client.ResolveDependency<ISharedPlayerManager>();
        var map = sEntMan.System<SharedMapSystem>();
        var zLevels = sEntMan.System<ZLevelSystem>();

        await server.WaitPost(() => sConfig.SetCVar(CVars.NetPVS, true));
        await RunTicksSync(server, client, 10);

        EntityUid player = default;
        EntityUid other = default;
        NetEntity playerNet = default;
        NetEntity lowerMapNet = default;
        NetEntity upperMapNet = default;
        await server.WaitPost(() =>
        {
            var lowerMap = map.CreateMap(out var lowerMapId);
            var upperMap = map.CreateMap(out var upperMapId);
            lowerMapNet = sEntMan.GetNetEntity(lowerMap);
            upperMapNet = sEntMan.GetNetEntity(upperMap);

            var lowerGrid = map.CreateGridEntity(lowerMapId);
            var upperGrid = map.CreateGridEntity(upperMapId);
            map.SetTile(lowerGrid, Vector2i.Zero, new Tile(1));
            map.SetTile(upperGrid, Vector2i.Zero, new Tile(1));
            Assert.That(zLevels.TryCreateMapNetwork([lowerMap, upperMap], out _), Is.True);

            player = sEntMan.SpawnEntity(null, new EntityCoordinates(lowerGrid.Owner, new Vector2(0.5f, 0.5f)));
            playerNet = sEntMan.GetNetEntity(player);
            other = sEntMan.SpawnEntity(null, new EntityCoordinates(lowerGrid.Owner, new Vector2(0.25f, 0.25f)));
            var session = sPlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, player);
            sPlayerMan.JoinGame(session);
        });

        await RunTicksSync(server, client, 10);
        await client.WaitPost(() =>
        {
            Assert.That(cEntMan.GetNetEntity(cPlayerMan.LocalEntity), Is.EqualTo(playerNet));
            cEntMan.System<ZLevelChangeRecorderSystem>().Events.Clear();
        });

        await server.WaitPost(() => Assert.That(zLevels.TryMoveEntityToMapOffset(other, 1), Is.True));
        await RunTicksSync(server, client, 10);
        await client.WaitPost(() =>
        {
            var events = cEntMan.System<ZLevelChangeRecorderSystem>().Events;
            Assert.That(events, Is.Empty);
        });

        await server.WaitPost(() => Assert.That(zLevels.TryMoveEntityToMapOffset(player, 1), Is.True));
        await RunTicksSync(server, client, 10);
        await client.WaitPost(() =>
        {
            var events = cEntMan.System<ZLevelChangeRecorderSystem>().Events;
            Assert.That(events, Has.Count.EqualTo(1));
            var ev = events[0];
            Assert.Multiple(() =>
            {
                Assert.That(cEntMan.GetNetEntity(ev.Entity), Is.EqualTo(playerNet));
                Assert.That(cEntMan.GetNetEntity(ev.OldMap), Is.EqualTo(lowerMapNet));
                Assert.That(cEntMan.GetNetEntity(ev.NewMap), Is.EqualTo(upperMapNet));
                Assert.That(ev.Offset, Is.EqualTo(1));
            });
        });

        await server.WaitPost(() => Assert.That(zLevels.TryMoveEntityToMapOffset(player, -1), Is.True));
        await RunTicksSync(server, client, 10);
        await client.WaitPost(() =>
        {
            var events = cEntMan.System<ZLevelChangeRecorderSystem>().Events;
            Assert.That(events, Has.Count.EqualTo(2));
            var ev = events[1];
            Assert.Multiple(() =>
            {
                Assert.That(cEntMan.GetNetEntity(ev.OldMap), Is.EqualTo(upperMapNet));
                Assert.That(cEntMan.GetNetEntity(ev.NewMap), Is.EqualTo(lowerMapNet));
                Assert.That(ev.Offset, Is.EqualTo(-1));
            });
        });
    }

}

[Reflect(false)]
internal sealed partial class ZLevelChangeRecorderSystem : EntitySystem
{
    public readonly List<LocalEntityZLevelChangedEvent> Events = [];

    public override void Initialize()
    {
        base.Initialize();
    }

    [SubscribeLocalEvent]
    private void OnLocalEntityZLevelChanged(LocalEntityZLevelChangedEvent args)
    {
        Events.Add(args);
    }
}
