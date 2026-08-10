using System;
using System.Collections.Generic;
using System.Numerics;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Client.Timing;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Timing;

namespace Robust.UnitTesting.Client.GameObjects;

[TestFixture]
[TestOf(typeof(TransformSystem))]
[NonParallelizable]
public sealed class TransformRenderCorrectionTests : RobustUnitTest
{
    public override UnitTestProject Project => UnitTestProject.Client;

    [Test]
    public void NonPredictedStateInterpolationUsesRenderPositionWithoutMutatingTransform()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);
        var parent = entMan.GetNetEntity(map);

        ApplyState(
            ent,
            xform,
            xformSystem,
            timing,
            new TransformComponentState(Vector2.Zero, Angle.Zero, parent, false, false),
            new TransformComponentState(Vector2.UnitX, Angle.Zero, parent, false, false));

        var samples = SampleTick(xformSystem, timing, xform, 4);

        AssertLinearTrajectory(samples, 0f, 1f);
        Assert.That(xform.LocalPosition.X, Is.EqualTo(0f));
    }

    [Test]
    public void RenderMapCoordinatesUseRenderPositionDuringNormalInterpolation()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);
        var parent = entMan.GetNetEntity(map);

        ApplyState(
            ent,
            xform,
            xformSystem,
            timing,
            new TransformComponentState(Vector2.Zero, Angle.Zero, parent, false, false),
            new TransformComponentState(Vector2.UnitX, Angle.Zero, parent, false, false));

        timing.TickRemainder = TimeSpan.FromTicks(timing.TickPeriod.Ticks / 2);
        xformSystem.FrameUpdate(0f);

        Assert.That(RenderX(xformSystem, xform), Is.EqualTo(0.5f).Within(0.001f));
        Assert.That(xformSystem.GetRenderMapCoordinates(xform).Position.X, Is.EqualTo(0.5f).Within(0.001f));
        Assert.That(xform.LocalPosition.X, Is.EqualTo(0f));
    }

    [Test]
    public void PredictedMovementLerpsBetweenTicksWithoutMutatingTransform()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);

        xformSystem.SetLocalPosition(ent, Vector2.UnitX, xform);

        timing.TickRemainder = TimeSpan.FromTicks(timing.TickPeriod.Ticks / 2);
        xformSystem.FrameUpdate(0f);

        Assert.That(RenderX(xformSystem, xform), Is.EqualTo(0.5f).Within(0.001f));
        Assert.That(xform.LocalPosition.X, Is.EqualTo(1f));
    }

    [Test]
    public void PredictedMispredictionCorrectionIsRenderOnlyAndRejoinsPrediction()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);
        var parent = entMan.GetNetEntity(map);

        xformSystem.SetLocalPosition(ent, Vector2.UnitX, xform);
        timing.TickRemainder = timing.TickPeriod;
        xformSystem.FrameUpdate(0f);
        Assert.That(RenderX(xformSystem, xform), Is.EqualTo(1f).Within(0.001f));

        ReconcilePrediction(xformSystem, timing, ent, xform, parent, 0.5f, 0.75f, 0.75f);
        var samples = SampleCorrectionAtTickEnd(xformSystem, timing, xform, 4);

        AssertLinearTrajectory(samples, 1f, 0.75f);
        Assert.That(xform.LocalPosition.X, Is.EqualTo(0.75f));
    }

    [Test]
    public void PredictedRotationMispredictionCorrectionIsRenderOnly()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);
        var parent = entMan.GetNetEntity(map);

        xformSystem.SetLocalRotation(ent, Angle.FromDegrees(90), xform);
        timing.TickRemainder = timing.TickPeriod;
        xformSystem.FrameUpdate(0f);
        Assert.That(RenderAngle(xformSystem, xform).Degrees, Is.EqualTo(90).Within(0.001));

        timing.TickRemainder = timing.TickPeriod;
        xformSystem.BeginPredictionCorrection();
        xformSystem.Reset();

        using (timing.StartStateApplicationArea())
        {
            var state = new TransformComponentState(Vector2.Zero, Angle.Zero, parent, false, false);
            var handleState = new ComponentHandleState(state, null);
            xformSystem.OnHandleState(ent, xform, ref handleState);
        }

        xformSystem.SetLocalRotation(ent, Angle.FromDegrees(45), xform);
        timing.TickRemainder = timing.TickPeriod;
        xformSystem.CapturePredictionReplayTick();
        xformSystem.EndPredictionCorrection();

        timing.TickRemainder = timing.TickPeriod;
        Assert.That(RenderAngle(xformSystem, xform).Degrees, Is.EqualTo(90).Within(0.001));
        Assert.That(xform.LocalRotation.Degrees, Is.EqualTo(45).Within(0.001));

        timing.TickRemainder = timing.TickPeriod;
        xformSystem.FrameUpdate((float) timing.TickPeriod.TotalSeconds / 2f);
        Assert.That(RenderAngle(xformSystem, xform).Degrees, Is.EqualTo(67.5).Within(0.001));

        xformSystem.FrameUpdate((float) timing.TickPeriod.TotalSeconds / 2f);
        Assert.That(RenderAngle(xformSystem, xform).Degrees, Is.EqualTo(45).Within(0.001));
    }

    [Test]
    public void NonPredictedDroppedStateHoldsAndSmoothsToNextState()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);
        var parent = entMan.GetNetEntity(map);

        ApplyState(
            ent,
            xform,
            xformSystem,
            timing,
            new TransformComponentState(Vector2.Zero, Angle.Zero, parent, false, false),
            new TransformComponentState(Vector2.UnitX, Angle.Zero, parent, false, false));

        timing.TickRemainder = timing.TickPeriod;
        xformSystem.FrameUpdate(0f);
        Assert.That(RenderX(xformSystem, xform), Is.EqualTo(1f).Within(0.001f));

        xformSystem.NotifyStateMissing();

        timing.TickRemainder = TimeSpan.Zero;
        xformSystem.FrameUpdate((float) timing.TickPeriod.TotalSeconds);
        Assert.That(RenderX(xformSystem, xform), Is.EqualTo(1f).Within(0.001f));

        ApplyState(
            ent,
            xform,
            xformSystem,
            timing,
            new TransformComponentState(new Vector2(2, 0), Angle.Zero, parent, false, false),
            new TransformComponentState(new Vector2(3, 0), Angle.Zero, parent, false, false),
            stateLoss: true);

        var samples = SampleTick(xformSystem, timing, xform, 4);
        AssertLinearTrajectory(samples, 1f, 3f);
    }

    [Test]
    public void PredictedDroppedStateSmoothsWhenReplayReachesComparisonTick()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var xform = entMan.GetComponent<TransformComponent>(ent);
        var parent = entMan.GetNetEntity(map);

        timing.CurTick = new GameTick(10);
        xformSystem.SetLocalPosition(ent, Vector2.UnitX, xform);
        timing.TickRemainder = timing.TickPeriod;
        xformSystem.FrameUpdate(0f);
        Assert.That(RenderX(xformSystem, xform), Is.EqualTo(1f).Within(0.001f));

        xformSystem.NotifyStateMissing();

        ReconcilePrediction(xformSystem, timing, ent, xform, parent, 0.5f, 0.75f, 0.75f);
        var samples = SampleCorrectionAtTickEnd(xformSystem, timing, xform, 4);

        AssertLinearTrajectory(samples, 1f, 0.75f);
        Assert.That(xform.LocalPosition.X, Is.EqualTo(0.75f));
    }

    [Test]
    public void ParentChangePreservesRenderWorldPositionAndSmoothsToNewParent()
    {
        var (entMan, xformSystem, timing, map) = Setup();
        var parent = entMan.SpawnEntity(null, new EntityCoordinates(map, new Vector2(10, 0)));
        var ent = entMan.SpawnEntity(null, new EntityCoordinates(map, Vector2.UnitX));
        var xform = entMan.GetComponent<TransformComponent>(ent);

        ApplyState(
            ent,
            xform,
            xformSystem,
            timing,
            new TransformComponentState(new Vector2(-8, 0), Angle.Zero, entMan.GetNetEntity(parent), false, false),
            null);

        var samples = SampleCorrectionAtTickEnd(xformSystem, timing, xform, 4);

        AssertLinearTrajectory(samples, 1f, 2f);
        Assert.That(xform.LocalPosition.X, Is.EqualTo(-8f));
        Assert.That(xform.ParentUid, Is.EqualTo(parent));
    }

    private static void ReconcilePrediction(
        TransformSystem xformSystem,
        IClientGameTiming timing,
        EntityUid ent,
        TransformComponent xform,
        NetEntity parent,
        float current,
        float next,
        float replayed)
    {
        timing.TickRemainder = timing.TickPeriod;
        xformSystem.BeginPredictionCorrection();
        xformSystem.Reset();

        using (timing.StartStateApplicationArea())
        {
            var currentState = new TransformComponentState(new Vector2(current, 0), Angle.Zero, parent, false, false);
            var nextState = new TransformComponentState(new Vector2(next, 0), Angle.Zero, parent, false, false);
            var handleState = new ComponentHandleState(currentState, nextState);
            xformSystem.OnHandleState(ent, xform, ref handleState);
        }

        xformSystem.SetLocalPosition(ent, new Vector2(replayed, 0), xform);
        timing.TickRemainder = timing.TickPeriod;
        xformSystem.CapturePredictionReplayTick();
        xformSystem.EndPredictionCorrection();
    }

    private static void ApplyState(
        EntityUid ent,
        TransformComponent xform,
        TransformSystem xformSystem,
        IClientGameTiming timing,
        TransformComponentState current,
        TransformComponentState? next,
        bool stateLoss = false)
    {
        timing.TickRemainder = TimeSpan.Zero;
        xformSystem.BeginPredictionCorrection();
        xformSystem.Reset();
        xformSystem.SetStateLossCorrection(stateLoss);

        try
        {
            using (timing.StartStateApplicationArea())
            {
                var handleState = new ComponentHandleState(current, next);
                xformSystem.OnHandleState(ent, xform, ref handleState);
            }
        }
        finally
        {
            xformSystem.SetStateLossCorrection(false);
        }

        xformSystem.EndPredictionCorrection();
    }

    private static List<float> SampleTick(
        TransformSystem xformSystem,
        IClientGameTiming timing,
        TransformComponent xform,
        int frames)
    {
        var samples = new List<float> { RenderX(xformSystem, xform) };
        var frameTime = (float) timing.TickPeriod.TotalSeconds / frames;

        for (var frame = 1; frame <= frames; frame++)
        {
            timing.TickRemainder = TimeSpan.FromTicks(timing.TickPeriod.Ticks * frame / frames);
            xformSystem.FrameUpdate(frameTime);
            samples.Add(RenderX(xformSystem, xform));
        }

        return samples;
    }

    private static List<float> SampleCorrectionAtTickEnd(
        TransformSystem xformSystem,
        IClientGameTiming timing,
        TransformComponent xform,
        int frames)
    {
        timing.TickRemainder = timing.TickPeriod;
        var samples = new List<float> { RenderX(xformSystem, xform) };
        var frameTime = (float) timing.TickPeriod.TotalSeconds / frames;

        for (var frame = 1; frame <= frames; frame++)
        {
            xformSystem.FrameUpdate(frameTime);
            samples.Add(RenderX(xformSystem, xform));
        }

        return samples;
    }

    private static void AssertLinearTrajectory(IReadOnlyList<float> samples, float start, float end)
    {
        Assert.That(samples[0], Is.EqualTo(start).Within(0.001f));
        Assert.That(samples[^1], Is.EqualTo(end).Within(0.001f));

        for (var i = 0; i < samples.Count; i++)
        {
            var expected = float.Lerp(start, end, i / (float) (samples.Count - 1));
            Assert.That(samples[i], Is.EqualTo(expected).Within(0.001f),
                $"sample {i}: {string.Join(", ", samples)}");
        }
    }

    private static float RenderX(TransformSystem xformSystem, TransformComponent xform)
    {
        return xformSystem.GetRenderWorldPositionRotation(xform).WorldPosition.X;
    }

    private static Angle RenderAngle(TransformSystem xformSystem, TransformComponent xform)
    {
        return xformSystem.GetRenderWorldPositionRotation(xform).WorldRotation;
    }

    private static (IEntityManager EntMan, TransformSystem XformSystem, IClientGameTiming Timing, EntityUid Map) Setup()
    {
        var entMan = IoCManager.Resolve<IEntityManager>();
        var mapSystem = entMan.System<SharedMapSystem>();
        var xformSystem = entMan.System<TransformSystem>();
        var timing = IoCManager.Resolve<IClientGameTiming>();
        xformSystem.Reset();
        timing.CurTick = GameTick.Zero;
        timing.LastRealTick = GameTick.Zero;
        timing.TickRemainder = TimeSpan.Zero;

        var map = mapSystem.CreateMap();
        return (entMan, xformSystem, timing, map);
    }
}
