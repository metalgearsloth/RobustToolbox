using Moq;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Input;
using Robust.Shared.Player;
using Robust.Shared.Timing;
using Robust.UnitTesting;

namespace Robust.Shared.IntegrationTests.GameObjects;

[TestFixture]
public sealed class PredictedSpawnReferenceTest : RobustIntegrationTest
{
    [Test]
    public async Task PredictedSpawnRegistersReferenceNormalSpawnDoesNot()
    {
        var server = StartServer();
        await server.WaitIdleAsync();

        var entMan = server.ResolveDependency<IEntityManager>();
        var session = Mock.Of<ICommonSession>();
        var tick = new GameTick(10);
        var function = new KeyFunctionId(42);

        await server.WaitAssertion(() =>
        {
            using (entMan.PushPredictedSpawnContext(session, tick, 7, function))
            {
                var normal = entMan.Spawn();
                var predicted = entMan.PredictedSpawn();

                Assert.That(entMan.GetNetEntityReference(normal).IsPredicted, Is.False);

                var reference = new EntityNetReference(new PredictedEntityReference(tick, 7, function, 0));
                Assert.That(entMan.GetEntity(reference, session), Is.EqualTo(predicted));
            }
        });
    }

    [Test]
    public async Task PredictedReferenceResolvesOnlyForOwningSession()
    {
        var server = StartServer();
        await server.WaitIdleAsync();

        var entMan = server.ResolveDependency<IEntityManager>();
        var owner = Mock.Of<ICommonSession>();
        var other = Mock.Of<ICommonSession>();
        var tick = new GameTick(20);
        var function = new KeyFunctionId(24);
        var predictedReference = new EntityNetReference(new PredictedEntityReference(tick, 3, function, 0));

        await server.WaitAssertion(() =>
        {
            EntityUid predicted;
            using (entMan.PushPredictedSpawnContext(owner, tick, 3, function))
            {
                predicted = entMan.PredictedSpawn();
            }

            Assert.That(entMan.GetEntity(predictedReference, owner), Is.EqualTo(predicted));
            Assert.That(entMan.GetEntity(predictedReference, other), Is.EqualTo(EntityUid.Invalid));
            Assert.That(entMan.GetEntity(predictedReference), Is.EqualTo(EntityUid.Invalid));
        });
    }

    [Test]
    public async Task PredictedSpawnOrdinalsAreStableWithinInput()
    {
        var server = StartServer();
        await server.WaitIdleAsync();

        var entMan = server.ResolveDependency<IEntityManager>();
        var session = Mock.Of<ICommonSession>();
        var tick = new GameTick(30);
        var function = new KeyFunctionId(12);

        await server.WaitAssertion(() =>
        {
            EntityUid first;
            EntityUid second;
            using (entMan.PushPredictedSpawnContext(session, tick, 1, function))
            {
                first = entMan.PredictedSpawn();
                second = entMan.PredictedSpawn();
            }

            var firstRef = new EntityNetReference(new PredictedEntityReference(tick, 1, function, 0));
            var secondRef = new EntityNetReference(new PredictedEntityReference(tick, 1, function, 1));

            Assert.That(entMan.GetEntity(firstRef, session), Is.EqualTo(first));
            Assert.That(entMan.GetEntity(secondRef, session), Is.EqualTo(second));
        });
    }
}
