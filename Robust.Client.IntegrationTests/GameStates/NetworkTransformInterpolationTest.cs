using System.Numerics;
using System.Reflection;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Client.GameStates;
using Robust.Client.Timing;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Timing;
using Robust.UnitTesting;

namespace Robust.Client.IntegrationTests.GameStates;

[TestFixture, NonParallelizable]
public sealed class NetworkTransformInterpolationTest : RobustUnitTest
{
    private const float TickDistance = 0.25f;

    public override UnitTestProject Project => UnitTestProject.Client;

    private EntityManager _entities = default!;
    private IConfigurationManager _configuration = default!;
    private IComponentFactory _componentFactory = default!;
    private IClientGameTiming _timing = default!;
    private TransformSystem _transforms = default!;
    private SharedMapSystem _maps = default!;
    private IClientGameStateManager _stateManager = default!;
    private int _nextNetEntity = 1;

    [OneTimeSetUp]
    public void OneTimeSetup()
    {
        _entities = IoCManager.Resolve<EntityManager>();
        _configuration = IoCManager.Resolve<IConfigurationManager>();
        _componentFactory = IoCManager.Resolve<IComponentFactory>();
        _componentFactory.GenerateNetIds();
        _timing = IoCManager.Resolve<IClientGameTiming>();
        _transforms = _entities.System<TransformSystem>();
        _maps = _entities.System<SharedMapSystem>();
        _stateManager = IoCManager.Resolve<IClientGameStateManager>();
        _stateManager.Initialize();
    }

    [SetUp]
    public void Setup()
    {
        _stateManager.Reset();
        _transforms.ResetRenderPoses();
    }

    [Test]
    public void NormalRemoteInterpolationUsesOneTickDuration()
    {
        var context = SetupClientState();
        var sourceTick = new GameTick(100);
        var targetTick = sourceTick + 1;
        var current = MakeTransformState(
            sourceTick,
            targetTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                new Vector2(TickDistance, 0f),
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var future = MakeTransformState(
            targetTick,
            targetTick + 1,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                new Vector2(TickDistance * 2f, 0f),
                Angle.Zero,
                context.MapNet,
                false,
                false));

        context.Timing.CurTick = targetTick;
        context.Timing.LastRealTick = targetTick;
        context.Timing.LastProcessedTick = targetTick;
        context.StateManager.UpdateFullRep(current);
        context.StateManager.ApplyGameState(current, future);

        AssertInterpolationSegment(context, TickDistance, TickDistance, 1, sourceTick, targetTick);
    }

    [TestCase(2, false)]
    [TestCase(3, false)]
    [TestCase(2, true)]
    public void RemoteInterpolationUsesBufferedFutureAuthoritativeSpan(int tickSpan, bool deltaFuture)
    {
        var context = SetupClientState();
        var sourceTick = new GameTick(100);
        var targetTick = sourceTick + (uint) tickSpan;
        var targetPosition = new Vector2(TickDistance * tickSpan, 0f);
        var current = MakeTransformState(
            sourceTick - 1,
            sourceTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                Vector2.Zero,
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var future = MakeTransformState(
            sourceTick,
            targetTick,
            context.EntityNet,
            context.TransformNetId,
            deltaFuture
                ? new TransformComponentDeltaState
                {
                    ChangedFields = 1UL << SharedTransformSystem.TransformLocalPositionIndex,
                    LocalPosition = targetPosition
                }
                : new TransformComponentState(
                    targetPosition,
                    Angle.Zero,
                    context.MapNet,
                    false,
                    false));

        context.StateManager.UpdateFullRep(current);
        context.StateManager.ApplyGameState(current, future);

        AssertInterpolationSegment(context, targetPosition.X, TickDistance, tickSpan, sourceTick, targetTick);
    }

    [Test]
    public void ApplyGameStateStartsFutureInterpolationWhenDroppedStateLeavesNoCurrentState()
    {
        var context = SetupClientState();
        var sourceTick = new GameTick(100);
        var targetTick = sourceTick + 2u;
        var targetPosition = new Vector2(TickDistance * 2f, 0f);
        var source = MakeTransformState(
            sourceTick - 1,
            sourceTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                Vector2.Zero,
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var future = MakeTransformState(
            sourceTick,
            targetTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                targetPosition,
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var processor = GetProcessor();
        processor.OnFullStateReceived();

        context.StateManager.UpdateFullRep(source);
        Assert.That(processor.AddNewState(future), Is.True);

        ApplyBufferedGameState(context);

        Assert.That(context.Transforms.TryGetRenderPoseDebugData(context.Entity, out var data), Is.True);
        Assert.That(data.Type, Is.EqualTo(RenderInterpolationType.NetworkInterpolation));
        Assert.That(data.Target.Position.X - data.Source.Position.X, Is.EqualTo(targetPosition.X).Within(0.001f));
        Assert.That(context.Transforms.TryGetNetworkInterpolationDebugTicks(context.Entity, out var actualSourceTick, out var actualTargetTick, out var actualTickSpan), Is.True);
        Assert.That(actualSourceTick, Is.EqualTo(sourceTick));
        Assert.That(actualTargetTick, Is.EqualTo(targetTick));
        Assert.That(actualTickSpan, Is.EqualTo(2u));

        var frameTime = (float) context.Timing.TickPeriod.TotalSeconds / 4f;
        SetTickAlpha(context.Timing, 0.75f);
        context.Transforms.FrameUpdate(frameTime);
        var samples = new List<float> { context.Transforms.GetRenderWorldPosition(context.Entity).X };
        Assert.That(samples[0], Is.EqualTo(TickDistance * 0.75f).Within(0.001f));

        ApplyBufferedGameState(context);

        Assert.That(context.Transforms.GetWorldPosition(context.Entity).X, Is.EqualTo(targetPosition.X).Within(0.001f));
        Assert.That(context.Transforms.TryGetRenderPoseDebugData(context.Entity, out data), Is.True);
        Assert.That(data.Type, Is.EqualTo(RenderInterpolationType.NetworkInterpolation));
        Assert.That(data.Alpha, Is.GreaterThan(0f));

        for (var frame = 0; frame < 4; frame++)
        {
            SetTickAlpha(context.Timing, frame / 4f);
            context.Transforms.FrameUpdate(frameTime);
            samples.Add(context.Transforms.GetRenderWorldPosition(context.Entity).X);
        }

        SetTickAlpha(context.Timing, 1f);
        context.Transforms.FrameUpdate(frameTime);
        samples.Add(context.Transforms.GetRenderWorldPosition(context.Entity).X);

        Assert.That(samples[^1], Is.EqualTo(targetPosition.X).Within(0.001f));

        for (var i = 1; i < samples.Count; i++)
        {
            Assert.That(samples[i] - samples[i - 1], Is.EqualTo(TickDistance / 4f).Within(0.015f));
        }
    }

    [Test]
    public void BufferedFutureParentChangeUsesVariableSpanInterpolation()
    {
        const int tickSpan = 2;

        var context = SetupClientState();
        var sourceTick = new GameTick(100);
        var targetTick = sourceTick + (uint) tickSpan;
        var parent = _entities.SpawnEntity(null, new MapCoordinates(new Vector2(TickDistance * tickSpan, 0f), context.MapId));
        var parentNet = AssignNetEntity(parent);
        var current = MakeTransformState(
            sourceTick - 1,
            sourceTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                Vector2.Zero,
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var future = MakeTransformState(
            sourceTick,
            targetTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                Vector2.Zero,
                Angle.Zero,
                parentNet,
                false,
                false));

        context.StateManager.UpdateFullRep(current);
        context.StateManager.ApplyGameState(current, future);

        Assert.That(context.Transforms.TryGetRenderPoseDebugData(context.Entity, out var data), Is.True);
        Assert.That(data.Parent, Is.EqualTo(parent));
        AssertInterpolationSegment(context, TickDistance * tickSpan, TickDistance, tickSpan, sourceTick, targetTick);
    }

    [Test]
    public void BufferedFutureDiscontinuityDoesNotStartVariableSpanInterpolation()
    {
        var context = SetupClientState();
        var sourceTick = new GameTick(100);
        var targetTick = sourceTick + 2u;
        var current = MakeTransformState(
            sourceTick - 1,
            sourceTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                Vector2.Zero,
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var future = MakeTransformState(
            sourceTick,
            targetTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                new Vector2(1000f, 0f),
                Angle.Zero,
                context.MapNet,
                false,
                false));

        context.StateManager.UpdateFullRep(current);
        context.StateManager.ApplyGameState(current, future);

        Assert.That(context.Transforms.TryGetRenderPoseDebugData(context.Entity, out _), Is.False);
        Assert.That(context.Transforms.GetRenderWorldPosition(context.Entity).X, Is.EqualTo(0f).Within(0.001f));
    }

    [Test]
    public void BufferedFutureDiscontinuityKeepsCurrentInterpolation()
    {
        var context = SetupClientState();
        var sourceTick = new GameTick(100);
        var currentTick = sourceTick + 1u;
        var futureTick = currentTick + 2u;
        var current = MakeTransformState(
            sourceTick,
            currentTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                new Vector2(TickDistance, 0f),
                Angle.Zero,
                context.MapNet,
                false,
                false));
        var future = MakeTransformState(
            currentTick,
            futureTick,
            context.EntityNet,
            context.TransformNetId,
            new TransformComponentState(
                new Vector2(1000f, 0f),
                Angle.Zero,
                context.MapNet,
                false,
                false));

        context.Timing.CurTick = currentTick;
        context.Timing.LastRealTick = currentTick;
        context.Timing.LastProcessedTick = currentTick;
        context.StateManager.UpdateFullRep(current);
        context.StateManager.ApplyGameState(current, future);

        AssertInterpolationSegment(context, TickDistance, TickDistance, 1, sourceTick, currentTick);
    }

    private TestContextState SetupClientState()
    {
        _configuration.SetCVar(CVars.NetPredict, false);
        _timing.SetTickRateAt(10, _timing.CurTick);
        _timing.TickTimingAdjustment = 0f;
        ((GameTiming) _timing).FreezeTickTimingAdjustment();
        _timing.TickRemainder = TimeSpan.Zero;
        _timing.CurTick = new GameTick(100);
        _timing.LastRealTick = new GameTick(100);
        _timing.LastProcessedTick = new GameTick(100);

        var map = _maps.CreateMap(out var mapId);
        var entity = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var entityMeta = _entities.GetComponent<MetaDataComponent>(entity);
        var entityXform = _entities.GetComponent<TransformComponent>(entity);
        var mapNet = AssignNetEntity(map);
        var entityNet = AssignNetEntity(entity);

        entityXform.LastModifiedTick = _timing.LastRealTick;
        entityMeta.LastStateApplied = _timing.LastRealTick;

        return new TestContextState(
            _timing,
            _transforms,
            _stateManager,
            entity,
            mapId,
            entityNet,
            mapNet,
            _componentFactory.GetRegistration(typeof(TransformComponent)).NetID!.Value);
    }

    private NetEntity AssignNetEntity(EntityUid entity)
    {
        var meta = _entities.GetComponent<MetaDataComponent>(entity);
        var netEntity = new NetEntity(_nextNetEntity++);
        _entities.ClearNetEntity(meta.NetEntity);
        _entities.SetNetEntity(entity, netEntity, meta);
        return netEntity;
    }

    private GameStateProcessor GetProcessor()
    {
        var field = typeof(ClientGameStateManager).GetField("_processor", BindingFlags.Instance | BindingFlags.NonPublic);
        Assert.That(field, Is.Not.Null);
        return (GameStateProcessor) field!.GetValue(_stateManager)!;
    }

    private static void ApplyBufferedGameState(TestContextState context)
    {
        var wasInSimulation = context.Timing.InSimulation;
        context.Timing.InSimulation = true;
        try
        {
            context.StateManager.ApplyGameState();
        }
        finally
        {
            context.Timing.InSimulation = wasInSimulation;
        }
    }

    private static GameState MakeTransformState(
        GameTick fromTick,
        GameTick toTick,
        NetEntity entity,
        ushort transformNetId,
        IComponentState state)
    {
        return new GameState(
            fromTick,
            toTick,
            0,
            new[]
            {
                new EntityState(
                    entity,
                    new[]
                    {
                        new ComponentChange(transformNetId, state, toTick)
                    },
                    toTick)
            },
            Array.Empty<SessionState>(),
            Array.Empty<NetEntity>());
    }

    private static void AssertInterpolationSegment(
        TestContextState context,
        float targetDisplacement,
        float expectedVelocityDistancePerTick,
        int tickSpan,
        GameTick sourceTick,
        GameTick targetTick,
        float? expectedFinalPosition = null)
    {
        Assert.That(context.Transforms.TryGetRenderPoseDebugData(context.Entity, out var data), Is.True);
        Assert.That(data.Type, Is.EqualTo(RenderInterpolationType.NetworkInterpolation));
        Assert.That(data.Target.Position.X - data.Source.Position.X, Is.EqualTo(targetDisplacement).Within(0.001f));
        Assert.That(context.Transforms.TryGetNetworkInterpolationDebugTicks(context.Entity, out var actualSourceTick, out var actualTargetTick, out var actualTickSpan), Is.True);
        Assert.That(actualSourceTick, Is.EqualTo(sourceTick));
        Assert.That(actualTargetTick, Is.EqualTo(targetTick));
        Assert.That(actualTickSpan, Is.EqualTo((uint) tickSpan));

        var samples = new List<float>();
        var frameTime = (float) context.Timing.TickPeriod.TotalSeconds / 4f;

        for (var tick = 0; tick < tickSpan; tick++)
        {
            for (var frame = 0; frame < 4; frame++)
            {
                SetTickAlpha(context.Timing, frame / 4f);
                context.Transforms.FrameUpdate(frameTime);
                samples.Add(context.Transforms.GetRenderWorldPosition(context.Entity).X);
            }
        }

        SetTickAlpha(context.Timing, 1f);
        context.Transforms.FrameUpdate(frameTime);
        samples.Add(context.Transforms.GetRenderWorldPosition(context.Entity).X);

        Assert.That(samples[^1], Is.EqualTo(expectedFinalPosition ?? targetDisplacement).Within(0.001f));

        for (var i = 1; i < samples.Count; i++)
        {
            var distance = samples[i] - samples[i - 1];
            Assert.That(distance, Is.EqualTo(expectedVelocityDistancePerTick / 4f).Within(0.015f));
        }
    }

    private static void SetTickAlpha(IClientGameTiming timing, float alpha)
    {
        timing.TickRemainder = TimeSpan.FromTicks((long) (timing.TickPeriod.Ticks * alpha));
    }

    private sealed record TestContextState(
        IClientGameTiming Timing,
        TransformSystem Transforms,
        IClientGameStateManager StateManager,
        EntityUid Entity,
        MapId MapId,
        NetEntity EntityNet,
        NetEntity MapNet,
        ushort TransformNetId);
}
