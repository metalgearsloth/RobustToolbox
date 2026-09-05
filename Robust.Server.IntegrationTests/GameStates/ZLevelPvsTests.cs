using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Player;

namespace Robust.UnitTesting.Server.GameStates;

public sealed class ZLevelPvsTests : RobustIntegrationTest
{
    [Test]
    public async Task ReplicatedVisibleLevelsAreClampedByServerCaps()
    {
        await using var pair = await StartConnectedPair();
        var (client, server) = pair;

        var sEntMan = server.ResolveDependency<IEntityManager>();
        var cEntMan = client.ResolveDependency<IEntityManager>();
        var sConfig = server.ResolveDependency<IConfigurationManager>();
        var sPlayerMan = server.ResolveDependency<ISharedPlayerManager>();
        var mapSystem = sEntMan.System<SharedMapSystem>();
        var zLevels = sEntMan.System<ZLevelSystem>();

        await server.WaitPost(() =>
        {
            sConfig.SetCVar(CVars.NetPVS, true);
            sConfig.SetCVar(CVars.NetPvsZLevelsBelow, 1);
            sConfig.SetCVar(CVars.NetPvsZLevelsAbove, 1);
        });

        await RunTicksSync(server, client, 10);

        EntityUid player = default;
        NetEntity playerNet = default;
        EntityUid lowerEnt = default;
        NetEntity lowerNet = default;
        NetEntity networkNet = default;
        NetEntity lowerMapNet = default;
        NetEntity upperMapNet = default;
        NetEntity topMapNet = default;
        Entity<ZLevelMapNetworkComponent> networkEntity = default;

        await server.WaitPost(() =>
        {
            var lowerMap = mapSystem.CreateMap(out var lowerMapId);
            var upperMap = mapSystem.CreateMap(out var upperMapId);
            var topMap = mapSystem.CreateMap(out var topMapId);
            lowerMapNet = sEntMan.GetNetEntity(lowerMap);
            upperMapNet = sEntMan.GetNetEntity(upperMap);
            topMapNet = sEntMan.GetNetEntity(topMap);

            var lowerGrid = mapSystem.CreateGridEntity(lowerMapId);
            var upperGrid = mapSystem.CreateGridEntity(upperMapId);
            var topGrid = mapSystem.CreateGridEntity(topMapId);
            mapSystem.SetTile(lowerGrid, Vector2i.Zero, new Tile(1));
            mapSystem.SetTile(upperGrid, Vector2i.Zero, new Tile(1));
            mapSystem.SetTile(topGrid, Vector2i.Zero, new Tile(1));

            networkEntity = zLevels.CreateMapNetwork();
            zLevels.SetProjectionOffset(networkEntity.Owner, new Vector2(0f, 0.7f));
            zLevels.SetVisibleLevels(networkEntity.Owner, below: 0, above: 1);
            zLevels.SetLowerLevelEffects(
                networkEntity.Owner,
                enabled: false,
                shader: "test-z-shader",
                blurRadius: 3f,
                darkenStrength: 0.4f,
                tint: new Color(0.1f, 0.2f, 0.3f, 0.25f));
            networkNet = sEntMan.GetNetEntity(networkEntity);
            Assert.That(zLevels.TryAddMaps(networkEntity, new Dictionary<EntityUid, int>
            {
                [lowerMap] = 0,
                [upperMap] = 1,
                [topMap] = 2,
            }));

            player = sEntMan.SpawnEntity(null, new EntityCoordinates(upperGrid.Owner, new Vector2(0.5f, 0.5f)));
            playerNet = sEntMan.GetNetEntity(player);
            sEntMan.AddComponent<ZLevelPresentationComponent>(player);
            sEntMan.System<Robust.Server.GameObjects.ZLevelPresentationSystem>().SetLocalHeight(player, 0.25f);
            lowerEnt = sEntMan.SpawnEntity(null, new EntityCoordinates(lowerGrid.Owner, new Vector2(0.5f, 0.5f)));
            lowerNet = sEntMan.GetNetEntity(lowerEnt);

            var session = sPlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, player);
            sPlayerMan.JoinGame(session);
        });

        await RunTicksSync(server, client, 10);
        Assert.That(cEntMan.TryGetEntity(networkNet, out var clientNetwork), Is.True);
        Assert.That(cEntMan.HasComponent<ZLevelMapNetworkComponent>(clientNetwork), Is.True);
        Assert.That(cEntMan.TryGetEntity(lowerNet, out _), Is.False);
        Assert.That(cEntMan.TryGetEntity(topMapNet, out var clientTopMap), Is.True);
        var clientNetworkComp = cEntMan.GetComponent<ZLevelMapNetworkComponent>(clientNetwork!.Value);
        Assert.Multiple(() =>
        {
            Assert.That(clientNetworkComp.ProjectionOffset, Is.EqualTo(new Vector2(0f, 0.7f)));
            Assert.That(clientNetworkComp.VisibleLevelsBelow, Is.EqualTo(0));
            Assert.That(clientNetworkComp.VisibleLevelsAbove, Is.EqualTo(1));
            Assert.That(clientNetworkComp.LowerPostShaderEnabled, Is.False);
            Assert.That(clientNetworkComp.LowerPostShader, Is.EqualTo("test-z-shader"));
            Assert.That(clientNetworkComp.LowerBlurRadius, Is.EqualTo(3f));
            Assert.That(clientNetworkComp.LowerDarkenStrength, Is.EqualTo(0.4f));
            Assert.That(clientNetworkComp.LowerTint, Is.EqualTo(new Color(0.1f, 0.2f, 0.3f, 0.25f)));
        });

        var clientPlayer = cEntMan.GetEntity(playerNet);
        var samples = new RenderLayerSample[2];
        var sampleCount = cEntMan.System<TransformSystem>().GetRenderLayerSamples(clientPlayer, samples);
        Assert.Multiple(() =>
        {
            Assert.That(sampleCount, Is.EqualTo(2));
            Assert.That(samples[0].Opacity, Is.EqualTo(0.75f).Within(0.001f));
            Assert.That(samples[1].Opacity, Is.EqualTo(0.25f).Within(0.001f));
            Assert.That(samples[1].Map, Is.EqualTo(clientTopMap!.Value));
        });

        await server.WaitPost(() =>
        {
            zLevels.SetVisibleLevels(networkEntity.Owner, below: 1, above: 0);
        });
        await RunTicksSync(server, client, 10);
        Assert.That(clientNetworkComp.VisibleLevelsBelow, Is.EqualTo(1));
        Assert.That(cEntMan.TryGetEntityData(lowerNet, out _, out var meta), Is.True);
        Assert.That(meta!.Flags & MetaDataFlags.Detached, Is.EqualTo(MetaDataFlags.None));
        Assert.That(cEntMan.TryGetEntity(upperMapNet, out var clientUpperMap), Is.True);
        Assert.That(cEntMan.TryGetEntity(lowerMapNet, out var clientLowerMap), Is.True);
        Assert.That(cEntMan.System<ZLevelSystem>().TryGetMapOffset(clientUpperMap!.Value, -1, out var clientOffsetMap), Is.True);
        Assert.That(clientOffsetMap, Is.EqualTo(clientLowerMap!.Value));

        await server.WaitPost(() => sConfig.SetCVar(CVars.NetPvsZLevelsBelow, 0));
        await RunTicksSync(server, client, 10);
        Assert.That(meta.Flags & MetaDataFlags.Detached, Is.EqualTo(MetaDataFlags.Detached));
    }

}
