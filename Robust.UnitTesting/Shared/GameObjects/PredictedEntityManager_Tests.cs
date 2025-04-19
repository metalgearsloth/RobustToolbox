using System.Linq;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Player;

namespace Robust.UnitTesting.Shared.GameObjects;

[TestFixture]
public sealed class PredictedEntityManager_Tests : RobustIntegrationTest
{
    [Test]
    public async Task TestPredictedDel()
    {
        var server = StartServer();
        var client = StartClient();

        await Task.WhenAll(client.WaitIdleAsync(), server.WaitIdleAsync());

        var confMan = server.ResolveDependency<IConfigurationManager>();
        var sPlayerMan = server.ResolveDependency<ISharedPlayerManager>();
        await client.ConnectTo(server);

        server.Post(() => confMan.SetCVar(CVars.NetPVS, false));

        // Attach player.
        var player = server.EntMan.Spawn(null, MapCoordinates.Nullspace);

        var session = sPlayerMan.Sessions.First();
        server.PlayerMan.SetAttachedEntity(session, player);
        sPlayerMan.JoinGame(session);

        var ent1 = server.EntMan.Spawn(null, MapCoordinates.Nullspace);

        await WaitRunTicks(client, server, 10);

        // Check client got it
        Assert.That(server.EntMan.EntityCount, Is.EqualTo(2));
        Assert.That(client.EntMan.EntityCount, Is.EqualTo(2));
        Assert.That(client.EntMan.GetEntity(new NetEntity(1)), Is.EqualTo(ent1));
    }
}
