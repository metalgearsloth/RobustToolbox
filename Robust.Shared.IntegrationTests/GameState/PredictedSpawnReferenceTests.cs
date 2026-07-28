using System;
using System.Linq;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared;
using Robust.Client;
using Robust.Client.GameStates;
using Robust.Client.Input;
using Robust.Client.Player;
using Robust.Shared.GameObjects;
using Robust.Shared.Input;
using Robust.Shared.Input.Binding;
using Robust.Shared.Map;
using Robust.Shared.Network;
using Robust.Shared.Serialization;
using Robust.Shared.Timing;

namespace Robust.UnitTesting.Shared.GameState;

internal sealed partial class PredictedSpawnReferenceTests : RobustIntegrationTest
{
    [Test]
    public async Task PredictedSpawnCanBeTargetedBeforeAuthoritativeStateArrives()
    {
        await using var pair = await StartConnectedPair();
        var server = pair.Server;
        var client = pair.Client;
        await RunTicksSync(server, client, 10);

        NetEntityReference reference = default;
        EntityUid predictedUid = default;
        EntityUid serverTarget = default;
        NetEntity authoritativeNet = default;
        NetEntity parentNet = default;
        var serverMoveInputs = 0;

        await server.WaitPost(() =>
        {
            server.CfgMan.SetCVar(CVars.NetPVS, false);
            var map = server.System<SharedMapSystem>().CreateMap();
            parentNet = server.EntMan.GetNetEntity(map);
            var player = server.EntMan.SpawnAttachedTo(null, new EntityCoordinates(map, Vector2.Zero));
            var session = server.PlayerMan.Sessions.First();
            server.PlayerMan.SetAttachedEntity(session, player);
            server.PlayerMan.JoinGame(session);
            var input = server.System<Robust.Server.GameObjects.InputSystem>();

            CommandBinds.Builder
                .Bind(EngineKeyFunctions.Use, new PointerInputCmdHandler((_, _, uid) =>
                {
                    serverTarget = uid;
                    return true;
                }))
                .Bind(EngineKeyFunctions.MoveUp, InputCmdHandler.FromDelegate(_ => serverMoveInputs++))
                .Register<PredictedSpawnReferenceTests>(input.BindRegistry);
        });

        await RunTicksSync(server, client, 5);

        await client.WaitPost(() =>
        {
            Assert.That(client.EntMan.GetEntity(parentNet), Is.Not.EqualTo(EntityUid.Invalid));
            client.EntMan.RaisePredictiveEvent(new PredictSpawnMessage(parentNet, "target"));

            predictedUid = client.System<PredictedSpawnReferenceTestSystem>().LastSpawned;
            reference = client.EntMan.GetNetEntityReference(predictedUid);

            var input = client.System<Robust.Client.GameObjects.InputSystem>();
            var inputManager = client.Resolve<IInputManager>();
            var timing = client.Resolve<IGameTiming>();
            var session = client.Resolve<IPlayerManager>().LocalSession;
            var coordinates = new EntityCoordinates(EntityUid.Invalid, Vector2.Zero);

            input.HandleInputCommand(
                session,
                EngineKeyFunctions.Use,
                new ClientFullInputCmdMessage(
                    timing.CurTick,
                    timing.TickFraction,
                    inputManager.NetworkBindMap.KeyFunctionID(EngineKeyFunctions.Use),
                    coordinates,
                    default,
                    BoundKeyState.Down,
                    predictedUid));

            input.HandleInputCommand(
                session,
                EngineKeyFunctions.MoveUp,
                new ClientFullInputCmdMessage(
                    timing.CurTick,
                    timing.TickFraction,
                    inputManager.NetworkBindMap.KeyFunctionID(EngineKeyFunctions.MoveUp),
                    coordinates,
                    default,
                    BoundKeyState.Down,
                    EntityUid.Invalid));

            Assert.That(reference.IsPredicted, Is.True);
        });

        await RunTicksSync(server, client, 10);

        await server.WaitPost(() =>
        {
            var spawned = server.System<PredictedSpawnReferenceTestSystem>().LastSpawned;
            authoritativeNet = server.EntMan.GetNetEntity(spawned);

            Assert.That(serverTarget, Is.EqualTo(spawned));
            Assert.That(serverMoveInputs, Is.EqualTo(1));
        });

        await client.WaitPost(() =>
        {
            var authoritative = client.EntMan.GetEntity(reference);
            var authoritativeByNet = client.EntMan.GetEntity(authoritativeNet);

            Assert.That(authoritativeByNet, Is.Not.EqualTo(EntityUid.Invalid));
            Assert.That(client.EntMan.HasComponent<PredictedSpawnComponent>(authoritativeByNet), Is.True);
            Assert.That(authoritative, Is.Not.EqualTo(EntityUid.Invalid));
            Assert.That(client.EntMan.IsClientSide(authoritative), Is.False);
            Assert.That(authoritative, Is.Not.EqualTo(predictedUid));
        });
    }

    [Test]
    public async Task PredictedSpawnReferenceUsesPerIdentifierIndex()
    {
        using var server = StartServer();
        using var client = StartClient();
        await client.WaitPost(() => client.Resolve<IBaseClient>().StartSinglePlayer());

        EntityUid secondServerEnt = default;
        EntityUid firstClientEnt = default;
        EntityUid secondClientEnt = default;
        NetEntityReference firstReference = default;
        NetEntityReference secondReference = default;

        await server.WaitPost(() =>
        {
            server.EntMan.PredictedSpawn(null, predictedSpawnId: "same");
            secondServerEnt = server.EntMan.PredictedSpawn(null, predictedSpawnId: "same");
        });

        await client.WaitPost(() =>
        {
            firstClientEnt = client.EntMan.PredictedSpawn(null, predictedSpawnId: "same");
            secondClientEnt = client.EntMan.PredictedSpawn(null, predictedSpawnId: "same");

            firstReference = client.EntMan.GetNetEntityReference(firstClientEnt);
            secondReference = client.EntMan.GetNetEntityReference(secondClientEnt);
        });

        await server.WaitAssertion(() =>
        {
            Assert.That(firstReference.PredictedSpawnIndex, Is.EqualTo(0));
            Assert.That(secondReference.PredictedSpawnIndex, Is.EqualTo(1));
            Assert.That(server.EntMan.GetEntity(secondReference), Is.EqualTo(secondServerEnt));
        });

        await client.WaitPost(() =>
        {
            client.EntMan.DeleteEntity(firstClientEnt);
            client.EntMan.DeleteEntity(secondClientEnt);

            var replacement = client.EntMan.PredictedSpawn(null, predictedSpawnId: "same");
            var replacementReference = client.EntMan.GetNetEntityReference(replacement);
            Assert.That(replacementReference.PredictedSpawnIndex, Is.EqualTo(0));
        });
    }

    [Test]
    public async Task StableIdentifierDoesNotResolveAcrossDifferentSpawnTicks()
    {
        using var server = StartServer();
        using var client = StartClient();
        await client.WaitPost(() => client.Resolve<IBaseClient>().StartSinglePlayer());

        var owner = new NetUserId(Guid.NewGuid());
        NetEntityReference clientReference = default;
        EntityUid serverEntity = default;

        await client.WaitPost(() =>
        {
            using (client.EntMan.WithPredictionContext(new GameTick(5), owner))
            {
                var entity = client.EntMan.PredictedSpawn(null, predictedSpawnId: "delayed-spawn");
                clientReference = client.EntMan.GetNetEntityReference(entity);
            }
        });

        await server.WaitPost(() =>
        {
            using (server.EntMan.WithPredictionContext(new GameTick(8), owner))
            {
                serverEntity = server.EntMan.PredictedSpawn(null, predictedSpawnId: "delayed-spawn");
            }

            Assert.That(server.EntMan.GetEntity(clientReference), Is.EqualTo(EntityUid.Invalid));
        });
    }

    [Test]
    public async Task PredictedSpawnReferenceUsesInputTickDuringClientReplay()
    {
        using var client = StartClient();
        await client.WaitPost(() => client.Resolve<IBaseClient>().StartSinglePlayer());

        await client.WaitAssertion(() =>
        {
            var input = client.System<Robust.Client.GameObjects.InputSystem>();
            var timing = client.Resolve<IGameTiming>();
            var inputTick = new GameTick(5);
            var replayTick = new GameTick(10);
            var callCount = 0;
            EntityUid spawned = default;
            EntityUid replayTarget = default;

            CommandBinds.Builder
                .Bind(EngineKeyFunctions.Use, new PointerInputCmdHandler((_, _, uid) =>
                {
                    callCount++;

                    if (callCount == 1)
                    {
                        spawned = client.EntMan.PredictedSpawn(null, predictedSpawnId: "replay-test");
                        return true;
                    }

                    replayTarget = uid;
                    return true;
                }))
                .Register<PredictedSpawnReferenceTests>(input.BindRegistry);

            var functionId = client.Resolve<Robust.Client.Input.IInputManager>()
                .NetworkBindMap
                .KeyFunctionID(EngineKeyFunctions.Use);
            var coordinates = new NetCoordinates(NetEntity.Invalid, default);
            var owner = client.Resolve<IPlayerManager>().LocalSession!.UserId;
            var reference = new NetEntityReference(NetEntity.Invalid, inputTick, 0, "replay-test", owner);

            timing.CurTick = replayTick;

            input.PredictInputCommand(new FullInputCmdMessage(
                inputTick,
                0,
                functionId,
                BoundKeyState.Down,
                coordinates,
                default,
                NetEntityReference.Invalid));

            input.PredictInputCommand(new FullInputCmdMessage(
                inputTick,
                0,
                functionId,
                BoundKeyState.Down,
                coordinates,
                default,
                reference));

            Assert.That(client.EntMan.GetNetEntityReference(spawned).PredictedSpawnTick, Is.EqualTo(inputTick));
            Assert.That(replayTarget, Is.EqualTo(spawned));
        });
    }

    [Test]
    public async Task MissingPredictedSpawnReferenceDoesNotBlockLaterServerInputs()
    {
        await using var pair = await StartConnectedPair();
        var server = pair.Server;
        var client = pair.Client;
        await RunTicksSync(server, client, 10);

        var useInputs = 0;
        var moveInputs = 0;
        uint movementSequence = 0;

        await server.WaitPost(() =>
        {
            var input = server.System<Robust.Server.GameObjects.InputSystem>();

            CommandBinds.Builder
                .Bind(EngineKeyFunctions.Use, new PointerInputCmdHandler((_, _, _) =>
                {
                    useInputs++;
                    return true;
                }))
                .Bind(EngineKeyFunctions.MoveUp, InputCmdHandler.FromDelegate(_ => moveInputs++))
                .Register<PredictedSpawnReferenceTests>(input.BindRegistry);
        });

        await client.WaitPost(() =>
        {
            var gameStates = client.Resolve<IClientGameStateManager>();
            var inputManager = client.Resolve<IInputManager>();
            var timing = client.Resolve<IGameTiming>();
            var useFunctionId = inputManager.NetworkBindMap.KeyFunctionID(EngineKeyFunctions.Use);
            var moveFunctionId = inputManager.NetworkBindMap.KeyFunctionID(EngineKeyFunctions.MoveUp);
            var coordinates = new NetCoordinates(NetEntity.Invalid, default);
            var unresolvedPredictedReference = new NetEntityReference(
                NetEntity.Invalid,
                timing.CurTick,
                0,
                "missing-predicted-target");

            var missingInput = new FullInputCmdMessage(
                timing.CurTick,
                0,
                useFunctionId,
                BoundKeyState.Down,
                coordinates,
                default,
                unresolvedPredictedReference);

            gameStates.InputCommandDispatched(
                new ClientFullInputCmdMessage(timing.CurTick, 0, useFunctionId),
                missingInput);
            client.EntMan.EntityNetManager!.SendSystemNetworkMessage(missingInput, missingInput.InputSequence);

            var wrongOwnerInput = new FullInputCmdMessage(
                timing.CurTick,
                1,
                useFunctionId,
                BoundKeyState.Up,
                coordinates,
                default,
                new NetEntityReference(
                    NetEntity.Invalid,
                    timing.CurTick,
                    0,
                    "wrong-owner",
                    new NetUserId(Guid.NewGuid())));

            gameStates.InputCommandDispatched(
                new ClientFullInputCmdMessage(timing.CurTick, 1, useFunctionId),
                wrongOwnerInput);
            client.EntMan.EntityNetManager.SendSystemNetworkMessage(wrongOwnerInput, wrongOwnerInput.InputSequence);

            var movementInput = new FullInputCmdMessage(
                timing.CurTick,
                2,
                moveFunctionId,
                BoundKeyState.Down,
                coordinates,
                default,
                NetEntityReference.Invalid);

            gameStates.InputCommandDispatched(
                new ClientFullInputCmdMessage(timing.CurTick, 2, moveFunctionId),
                movementInput);
            client.EntMan.EntityNetManager.SendSystemNetworkMessage(movementInput, movementInput.InputSequence);
            movementSequence = movementInput.InputSequence;
        });

        await RunTicksSync(server, client, 5);

        await server.WaitPost(() =>
        {
            var input = server.System<Robust.Server.GameObjects.InputSystem>();
            var session = server.PlayerMan.Sessions.First();

            Assert.That(useInputs, Is.EqualTo(1));
            Assert.That(moveInputs, Is.EqualTo(1));
            Assert.That(input.GetLastInputCommand(session), Is.EqualTo(movementSequence));
        });
    }

    [Test]
    public async Task PredictedSpawnReferencesAreScopedByPredictingUser()
    {
        using var client = StartClient();
        await client.WaitPost(() => client.Resolve<IBaseClient>().StartSinglePlayer());

        await client.WaitAssertion(() =>
        {
            var tick = client.Resolve<IGameTiming>().CurTick;
            var firstUser = new NetUserId(Guid.NewGuid());
            var secondUser = new NetUserId(Guid.NewGuid());
            EntityUid first;
            EntityUid second;

            using (client.EntMan.WithPredictionContext(tick, firstUser))
            {
                first = client.EntMan.PredictedSpawn(null, predictedSpawnId: "same");
            }

            using (client.EntMan.WithPredictionContext(tick, secondUser))
            {
                second = client.EntMan.PredictedSpawn(null, predictedSpawnId: "same");
            }

            var firstReference = client.EntMan.GetNetEntityReference(first);
            var secondReference = client.EntMan.GetNetEntityReference(second);

            Assert.That(firstReference.PredictedSpawnOwner, Is.EqualTo(firstUser));
            Assert.That(secondReference.PredictedSpawnOwner, Is.EqualTo(secondUser));
            Assert.That(firstReference.PredictedSpawnIndex, Is.EqualTo(0));
            Assert.That(secondReference.PredictedSpawnIndex, Is.EqualTo(0));
            Assert.That(client.EntMan.GetEntity(firstReference), Is.EqualTo(first));
            Assert.That(client.EntMan.GetEntity(secondReference), Is.EqualTo(second));
        });
    }

    public sealed partial class PredictedSpawnReferenceTestSystem : EntitySystem
    {
        public EntityUid LastSpawned { get; private set; }

        public override void Initialize()
        {
            base.Initialize();
            SubscribeAllEvent<PredictSpawnMessage>(OnPredictSpawn);
        }

        private void OnPredictSpawn(PredictSpawnMessage message, EntitySessionEventArgs args)
        {
            var parent = GetEntity(message.Parent);
            LastSpawned = EntityManager.PredictedSpawnAttachedTo(
                null,
                new EntityCoordinates(parent, Vector2.Zero),
                message.Id);
        }
    }

    [Serializable, NetSerializable]
    public sealed class PredictSpawnMessage(NetEntity parent, string id) : EntityEventArgs
    {
        public NetEntity Parent { get; } = parent;
        public string Id { get; } = id;
    }
}
