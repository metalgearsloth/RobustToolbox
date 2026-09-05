using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Client.Graphics.Clyde;
using Robust.Client.Timing;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;
using Robust.Shared.Timing;

namespace Robust.UnitTesting.Client.GameObjects;

[TestFixture, NonParallelizable]
public sealed class RenderTransformSystemTest : RobustUnitTest
{
    public override UnitTestProject Project => UnitTestProject.Client;

    private IEntityManager _entities = default!;
    private ContainerSystem _containers = default!;
    private IConfigurationManager _configuration = default!;
    private SharedMapSystem _maps = default!;
    private TransformSystem _transforms = default!;
    private SpriteSystem _sprites = default!;
    private EyeSystem _eyes = default!;
    private ZLevelSystem _zLevels = default!;
    private ZLevelPresentationSystem _zPresentation = default!;
    private IClientGameTiming _timing = default!;

    [OneTimeSetUp]
    public void OneTimeSetup()
    {
        _entities = IoCManager.Resolve<IEntityManager>();
        _configuration = IoCManager.Resolve<IConfigurationManager>();
        _containers = _entities.System<ContainerSystem>();
        _maps = _entities.System<SharedMapSystem>();
        _transforms = _entities.System<TransformSystem>();
        _sprites = _entities.System<SpriteSystem>();
        _eyes = _entities.System<EyeSystem>();
        _zLevels = _entities.System<ZLevelSystem>();
        _zPresentation = _entities.System<ZLevelPresentationSystem>();
        _timing = IoCManager.Resolve<IClientGameTiming>();
    }

    [SetUp]
    public void Setup()
    {
        _transforms.ResetRenderPoses();
        _timing.TickRemainder = TimeSpan.Zero;
        _timing.TickTimingAdjustment = 0f;
        ((GameTiming) _timing).LatchTickTimingAdjustment();
        _timing.CurTick = new GameTick(_timing.CurTick.Value + 1);
        _timing.LastRealTick = _timing.CurTick;
    }

    [Test]
    public void ComponentStateApplicationInterpolatesToTheFinalSimulationPose()
    {
        var (map, mapId) = CreateMap();
        var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        MakeRemote(xform);
        var state = new TransformComponentState(
            Vector2.UnitX,
            Angle.Zero,
            _entities.GetNetEntity(map),
            false,
            false);
        var nextState = new TransformComponentState(
            new Vector2(1.5f, 0f),
            Angle.FromDegrees(45),
            _entities.GetNetEntity(map),
            false,
            false);
        var handleState = new ComponentHandleState(state, nextState);

        using (_timing.StartStateApplicationArea())
            _entities.EventBus.RaiseComponentEvent(uid, xform, ref handleState);

        // The real client runs prediction after state application and before FrameUpdate.
        _timing.CurTick = new GameTick(_timing.LastRealTick.Value + 12);
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        Assert.Multiple(() =>
        {
            AssertVector(xform.LocalPosition, Vector2.UnitX);
            AssertVector(_transforms.GetWorldPosition(uid), Vector2.UnitX);
            AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f));
        });
    }

    [Test]
    public void SameParentInterpolationSamplesWholeTickWithoutMutatingSimulation()
    {
        var (_, mapId) = CreateMap();
        var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        MakeRemote(xform);

        var moveEvents = 0;
        void CountMove(ref MoveEvent args)
        {
            if (args.Sender == uid)
                moveEvents++;
        }

        _transforms.OnGlobalMoveEvent += CountMove;
        try
        {
            ApplyRemote(() => _transforms.SetLocalPositionRotation(uid, Vector2.One, Angle.FromDegrees(90), xform));
            var countBeforeFrame = moveEvents;
            var parent = xform.ParentUid;
            var localPosition = xform.LocalPosition;
            var localRotation = xform.LocalRotation;
            var lastModified = xform.LastModifiedTick;
            var broadphase = xform.Broadphase;

            SetTickAlpha(0f);
            _transforms.FrameUpdate(0f);

            Assert.Multiple(() =>
            {
                AssertVector(xform.LocalPosition, Vector2.One);
                AssertVector(_transforms.GetWorldPosition(uid), Vector2.One);
                AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.Zero);
                Assert.That(_transforms.GetRenderWorldRotation(uid).Degrees, Is.EqualTo(0).Within(0.001));
            });

            SetTickAlpha(0.5f);
            _transforms.FrameUpdate(0f);

            Assert.Multiple(() =>
            {
                AssertVector(xform.LocalPosition, Vector2.One);
                Assert.That(xform.ParentUid, Is.EqualTo(parent));
                Assert.That(xform.LocalRotation, Is.EqualTo(localRotation));
                Assert.That(xform.LastModifiedTick, Is.EqualTo(lastModified));
                Assert.That(xform.Broadphase, Is.EqualTo(broadphase));
                Assert.That(moveEvents, Is.EqualTo(countBeforeFrame));
                AssertVector(_transforms.GetWorldPosition(uid), Vector2.One);
                AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f));
                Assert.That(_transforms.GetRenderWorldRotation(uid).Degrees, Is.EqualTo(45).Within(0.001));
            });

            SetTickAlpha(1f);
            _transforms.FrameUpdate(0f);

            Assert.Multiple(() =>
            {
                AssertVector(xform.LocalPosition, Vector2.One);
                AssertVector(_transforms.GetWorldPosition(uid), Vector2.One);
                AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.One);
                Assert.That(_transforms.GetRenderWorldRotation(uid).Degrees, Is.EqualTo(90).Within(0.001));
                Assert.That(moveEvents, Is.EqualTo(countBeforeFrame));
            });
        }
        finally
        {
            _transforms.OnGlobalMoveEvent -= CountMove;
        }
    }

    [TestCase(ParentTransition.GridToMap, false)]
    [TestCase(ParentTransition.MapToGrid, false)]
    [TestCase(ParentTransition.GridToGrid, false)]
    public void CrossParentTransitionsInterpolateInRenderSpace(ParentTransition transition, bool predicted)
    {
        var (map, mapId) = CreateMap();
        var gridA = _maps.CreateGridEntity(mapId).Owner;
        var gridB = _maps.CreateGridEntity(mapId).Owner;
        _transforms.SetLocalPosition(gridB, Vector2.UnitX);

        var source = transition == ParentTransition.MapToGrid ? map : gridA;
        var target = transition == ParentTransition.GridToMap ? map : gridB;
        var targetLocal = transition == ParentTransition.GridToMap ? Vector2.UnitX : Vector2.Zero;
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(source, Vector2.Zero));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        xform.GridTraversal = false;
        MakeRemote(xform);

        void Move() => _transforms.SetCoordinates(
            uid,
            xform,
            new EntityCoordinates(target, targetLocal),
            Angle.Zero,
            unanchor: false);

        if (predicted)
        {
            _timing.CurTick = new GameTick(_timing.LastRealTick.Value + 1);
            Move();
        }
        else
        {
            ApplyRemote(Move);
        }

        SetTickAlpha(0f);
        _transforms.FrameUpdate(0f);
        Assert.Multiple(() =>
        {
            Assert.That(xform.ParentUid, Is.EqualTo(target));
            AssertVector(_transforms.GetWorldPosition(uid), Vector2.UnitX);
            AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.Zero);
        });

        SetTickAlpha(0.5f);
        _transforms.FrameUpdate(0f);

        Assert.Multiple(() =>
        {
            Assert.That(xform.ParentUid, Is.EqualTo(target));
            AssertVector(_transforms.GetWorldPosition(uid), Vector2.UnitX);
            AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f));
        });

        if (predicted)
        {
            using (_timing.StartStateApplicationArea())
                _transforms.SetCoordinates(uid, xform, new EntityCoordinates(source, Vector2.Zero), Angle.Zero, false);
            using (_timing.StartPastPredictionArea())
                _transforms.SetCoordinates(uid, xform, new EntityCoordinates(target, targetLocal), Angle.Zero, false);

            Assert.Multiple(() =>
            {
                Assert.That(xform.ParentUid, Is.EqualTo(target));
                AssertVector(_transforms.GetWorldPosition(uid), Vector2.UnitX);
                AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f),
                    "cross-parent prediction replay must preserve the displayed midpoint");
            });
        }

        SetTickAlpha(1f);
        _transforms.FrameUpdate(0f);
        Assert.Multiple(() =>
        {
            Assert.That(xform.ParentUid, Is.EqualTo(target));
            AssertVector(_transforms.GetWorldPosition(uid), Vector2.UnitX);
            AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.UnitX);
        });
    }

    [Test]
    public void CrossParentEndpointsFollowMovingRotatingParentsAndNestedChildren()
    {
        var (map, mapId) = CreateMap();
        var parentA = _entities.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var parentB = _entities.SpawnEntity(null, new EntityCoordinates(map, Vector2.UnitX));
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(parentA, new Vector2(0.5f, 0f)));
        var nested = _entities.SpawnEntity(null, new EntityCoordinates(uid, new Vector2(0.25f, 0f)));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        xform.GridTraversal = false;
        MakeRemote(parentA, parentB, uid);

        ApplyRemote(() =>
        {
            _transforms.SetCoordinates(uid, xform, new EntityCoordinates(parentB, new Vector2(0.5f, 0f)), Angle.Zero, false);
            _transforms.SetLocalPositionRotation(parentA, new Vector2(0f, 0.2f), Angle.FromDegrees(90));
            _transforms.SetLocalPositionRotation(parentB, new Vector2(1f, 0.2f), Angle.FromDegrees(-90));
        });

        SetTickAlpha(0f);
        _transforms.FrameUpdate(0f);
        Assert.Multiple(() =>
        {
            AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f));
            AssertVector(_transforms.GetRenderWorldPosition(nested), new Vector2(0.75f, 0f));
            AssertVector(_transforms.GetWorldPosition(uid), new Vector2(1f, -0.3f));
        });

        SetTickAlpha(0.5f);
        _transforms.FrameUpdate(0f);

        var halfRoot = new Vector2(MathF.Sqrt(0.125f), 0.1f + MathF.Sqrt(0.125f));
        var halfTarget = new Vector2(1f + MathF.Sqrt(0.125f), 0.1f - MathF.Sqrt(0.125f));
        var expected = Vector2.Lerp(halfRoot, halfTarget, 0.5f);
        var pose = _transforms.GetRenderWorldPose(uid);
        var nestedExpected = pose.Position + pose.Rotation.RotateVec(new Vector2(0.25f, 0f));

        Assert.Multiple(() =>
        {
            AssertVector(pose.Position, expected);
            AssertVector(_transforms.GetRenderWorldPosition(nested), nestedExpected);
            AssertVector(_transforms.GetWorldPosition(uid), new Vector2(1f, -0.3f));
        });

        SetTickAlpha(1f);
        _transforms.FrameUpdate(0f);
        var finalPose = _transforms.GetRenderWorldPose(uid);
        Assert.Multiple(() =>
        {
            AssertVector(finalPose.Position, new Vector2(1f, -0.3f));
            Assert.That(finalPose.Rotation.Degrees, Is.EqualTo(-90).Within(0.001));
            AssertVector(_transforms.GetRenderWorldPosition(nested), new Vector2(1f, -0.55f));
        });
    }

    [Test]
    public void RemoteSpriteAndEyeCoordinatesUseTheSameRenderPose()
    {
        var (_, mapId) = CreateMap();
        var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        var sprite = _entities.AddComponent<SpriteComponent>(uid);
        var eye = _entities.AddComponent<EyeComponent>(uid);
        MakeRemote(xform);

        ApplyRemote(() => _transforms.SetLocalPosition(uid, Vector2.UnitX, xform));
        SetHalfTick();
        _transforms.FrameUpdate(0f);
        _eyes.FrameUpdate(0f);

        var renderCoordinates = _transforms.GetRenderMapCoordinates(uid, xform);
        Assert.Multiple(() =>
        {
            AssertVector(_sprites.GetSpriteWorldPosition((uid, sprite, xform)), renderCoordinates.Position);
            AssertVector(eye.Eye.Position.Position, renderCoordinates.Position);
            Assert.That(eye.Eye.Position.MapId, Is.EqualTo(renderCoordinates.MapId));
            Assert.That(renderCoordinates.MapId, Is.EqualTo(mapId));
            AssertVector(renderCoordinates.Position, new Vector2(0.5f, 0f));
        });
    }

    [TestCase(-0.1f)]
    [TestCase(0.1f)]
    public void MovementAt5TpsUsesAdjustedTickPhase(float tickTimingAdjustment)
    {
        var oldTickRate = _timing.TickRate;
        var oldTimingAdjustment = _timing.TickTimingAdjustment;

        try
        {
            _timing.SetTickRateAt(5, _timing.CurTick);
            _timing.TickTimingAdjustment = tickTimingAdjustment;
            ((GameTiming) _timing).LatchTickTimingAdjustment();
            var adjustedPeriod = (float) _timing.CalcAdjustedTickPeriod().TotalSeconds;
            const float frameTime = 1f / 119f;
            var (_, mapId) = CreateMap();
            var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
            var xform = _entities.GetComponent<TransformComponent>(uid);
            MakeRemote(xform);

            var accumulator = 0f;
            var tick = 0;
            float? previousPosition = null;
            var velocities = new List<float>();

            ApplyRemote(() => _transforms.SetLocalPosition(uid, Vector2.UnitX, xform));

            for (var frame = 0; tick < 10; frame++)
            {
                accumulator += frameTime;
                while (accumulator >= adjustedPeriod)
                {
                    accumulator -= adjustedPeriod;
                    tick++;
                    if (tick >= 10)
                        break;

                    _timing.LastRealTick = new GameTick(_timing.LastRealTick.Value + 1);
                    _timing.CurTick = _timing.LastRealTick;
                    ApplyRemote(() => _transforms.SetLocalPosition(uid, new Vector2(tick + 1, 0f), xform));
                }

                if (tick >= 10)
                    break;

                _timing.TickRemainder = TimeSpan.FromSeconds(accumulator);
                _transforms.FrameUpdate(frameTime);

                var position = _transforms.GetRenderWorldPosition(uid).X;
                if (previousPosition is { } previous)
                    velocities.Add((position - previous) / frameTime);

                previousPosition = position;
            }

            var expectedVelocity = 1f / adjustedPeriod;
            Assert.Multiple(() =>
            {
                Assert.That(velocities.Min(), Is.EqualTo(expectedVelocity).Within(0.08f));
                Assert.That(velocities.Max(), Is.EqualTo(expectedVelocity).Within(0.08f));
            });
        }
        finally
        {
            _timing.SetTickRateAt(oldTickRate, _timing.CurTick);
            _timing.TickTimingAdjustment = oldTimingAdjustment;
            ((GameTiming) _timing).LatchTickTimingAdjustment();
            _timing.TickRemainder = TimeSpan.Zero;
        }
    }

    [Test]
    public void TimingAdjustmentChangeMidTickDoesNotMoveRenderPoseWithoutElapsedTime()
    {
        var oldTickRate = _timing.TickRate;
        var oldTimingAdjustment = _timing.TickTimingAdjustment;

        try
        {
            _timing.SetTickRateAt(30, _timing.CurTick);
            _timing.TickTimingAdjustment = 0f;
            ((GameTiming) _timing).LatchTickTimingAdjustment();
            var (_, mapId) = CreateMap();
            var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
            var xform = _entities.GetComponent<TransformComponent>(uid);
            MakeRemote(xform);

            ApplyRemote(() => _transforms.SetLocalPosition(uid, Vector2.UnitX, xform));
            SetHalfTick();
            _transforms.FrameUpdate(0f);

            var before = _transforms.GetRenderWorldPosition(uid);
            var phaseBefore = _timing.TickPhase;

            _timing.TickTimingAdjustment = 0.1f;
            _transforms.FrameUpdate(0f);

            Assert.Multiple(() =>
            {
                Assert.That(_timing.TickPhase, Is.EqualTo(phaseBefore).Within(0.00001f));
                AssertVector(_transforms.GetRenderWorldPosition(uid), before);
            });
        }
        finally
        {
            _timing.SetTickRateAt(oldTickRate, _timing.CurTick);
            _timing.TickTimingAdjustment = oldTimingAdjustment;
            ((GameTiming) _timing).LatchTickTimingAdjustment();
            _timing.TickRemainder = TimeSpan.Zero;
        }
    }

    [Test]
    public void TeleportsNullspaceContainersAndUnrelatedMapsSnap()
    {
        var (mapA, mapAId) = CreateMap();
        var (mapB, _) = CreateMap();
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(mapA, Vector2.Zero));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        xform.GridTraversal = false;
        MakeRemote(xform);

        ApplyRemote(() => _transforms.SetCoordinates(uid, xform, new EntityCoordinates(mapB, Vector2.UnitX), Angle.Zero, false));
        AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.UnitX, "unrelated maps must snap");

        ApplyRemote(() => _transforms.SetCoordinates(uid, xform, new EntityCoordinates(mapA, Vector2.Zero), Angle.Zero, false));
        ApplyRemote(() => _transforms.SetLocalPosition(uid, Vector2.UnitX, xform));
        _transforms.SnapRenderPose(uid);
        AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.UnitX, "explicit teleport must snap");

        ApplyRemote(() => _transforms.SetLocalPosition(uid, new Vector2(10f, 0f), xform));
        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(10f, 0f), "large deltas must snap");

        var containerOwner = _entities.SpawnEntity(null, new EntityCoordinates(mapA, new Vector2(10.5f, 0f)));
        var container = _containers.EnsureContainer<Container>(containerOwner, "test");
        var inserted = false;
        ApplyRemote(() => inserted = _containers.Insert(uid, container, force: true));
        Assert.That(inserted, Is.True);
        AssertVector(_transforms.GetRenderWorldPosition(uid),
            new Vector2(10.5f, 0f),
            "container transitions must snap");

        ApplyRemote(() => _containers.Remove(uid, container, reparent: false, force: true));
        ApplyRemote(() => _transforms.DetachEntity(uid, xform));
        Assert.That(xform.MapID, Is.EqualTo(MapId.Nullspace));
        Assert.That(_transforms.GetRenderWorldPose(uid).CoordinateSpace.IsValid(), Is.False);
        Assert.That(mapAId, Is.Not.EqualTo(MapId.Nullspace));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void ContainerAndHandSlotInsertionRemovalSnap(bool handSlot)
    {
        var (map, mapId) = CreateMap();
        var holder = _entities.SpawnEntity(null, new EntityCoordinates(map, Vector2.UnitX));
        var item = _entities.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var holderXform = _entities.GetComponent<TransformComponent>(holder);
        var itemXform = _entities.GetComponent<TransformComponent>(item);
        MakeRemote(holderXform);
        MakeRemote(itemXform);

        BaseContainer container = handSlot
            ? _containers.EnsureContainer<ContainerSlot>(holder, "hand")
            : _containers.EnsureContainer<Container>(holder, "container");

        ApplyRemote(() => _transforms.SetLocalPosition(item, new Vector2(0.5f, 0f), itemXform));
        SetHalfTick();
        _transforms.FrameUpdate(0f);
        AssertVector(_transforms.GetRenderWorldPosition(item), new Vector2(0.25f, 0f));

        var inserted = false;
        ApplyRemote(() => inserted = _containers.Insert(item, container, force: true));
        Assert.Multiple(() =>
        {
            Assert.That(inserted, Is.True);
            Assert.That(itemXform.ParentUid, Is.EqualTo(holder));
            AssertVector(_transforms.GetWorldPosition(item), Vector2.UnitX);
            AssertVector(_transforms.GetRenderWorldPosition(item),
                Vector2.UnitX,
                "insertion must discard the active render lerp");
        });

        ApplyRemote(() => _transforms.SetLocalPosition(holder, new Vector2(1.5f, 0f), holderXform));
        SetHalfTick();
        _transforms.FrameUpdate(0f);
        AssertVector(_transforms.GetRenderWorldPosition(item), new Vector2(1.25f, 0f));

        var removed = false;
        ApplyRemote(() => removed = _containers.Remove(
            item,
            container,
            force: true,
            destination: new EntityCoordinates(map, new Vector2(2f, 0f))));
        Assert.Multiple(() =>
        {
            Assert.That(removed, Is.True);
            Assert.That(itemXform.ParentUid, Is.EqualTo(map));
            AssertVector(_transforms.GetWorldPosition(item), new Vector2(2f, 0f));
            AssertVector(_transforms.GetRenderWorldPosition(item),
                new Vector2(2f, 0f),
                "removal must discard the inherited render offset");
            Assert.That(itemXform.MapID, Is.EqualTo(mapId));
        });
    }

    [Test]
    public void RapidPredictedContainerCycleClearsStaleVerticalPoseAndVelocity()
    {
        var (maps, _, _) = CreateZNetwork(2, new Vector2(0f, 0.7f));
        var holder = _entities.SpawnEntity(null, new EntityCoordinates(maps[1], Vector2.Zero));
        var item = _entities.SpawnEntity(null, new EntityCoordinates(maps[1], Vector2.Zero));
        var holderPresentation = _entities.AddComponent<ZLevelPresentationComponent>(holder);
        var itemPhysics = _entities.AddComponent<ZLevelPhysicsComponent>(item);
        var itemPresentation = _entities.GetComponent<ZLevelPresentationComponent>(item);
        var slot = _containers.EnsureContainer<ContainerSlot>(holder, "hand");
        _zPresentation.SetLocalHeight((holder, holderPresentation), 0.2f);
        _zPresentation.SetLocalHeight((item, itemPresentation), -0.4f);
        itemPhysics.Velocity = -6f;
        _timing.CurTick = new GameTick(_timing.LastRealTick.Value + 2);

        Assert.That(_containers.Insert(item, slot, force: true), Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(itemPresentation.LocalHeight, Is.EqualTo(0.2f).Within(0.0001f));
            Assert.That(itemPhysics.Velocity, Is.Zero);
        });

        Assert.That(_containers.Remove(
            item,
            slot,
            force: true,
            destination: new EntityCoordinates(maps[1], Vector2.Zero)), Is.True);
        _transforms.SnapRenderPose(item);
        Assert.That(_transforms.GetRenderWorldPose(item).AbsoluteZ, Is.EqualTo(1.2f).Within(0.0001f));

        _zPresentation.SetLocalHeight((item, itemPresentation), -0.1f);
        itemPhysics.Velocity = -3f;
        Assert.That(_containers.Insert(item, slot, force: true), Is.True);
        Assert.That(_containers.Remove(
            item,
            slot,
            force: true,
            destination: new EntityCoordinates(maps[1], Vector2.Zero)), Is.True);
        _transforms.SnapRenderPose(item);

        Assert.Multiple(() =>
        {
            Assert.That(itemPresentation.LocalHeight, Is.EqualTo(0.2f).Within(0.0001f));
            Assert.That(itemPhysics.Velocity, Is.Zero);
            Assert.That(_transforms.GetRenderWorldPose(item).AbsoluteZ, Is.EqualTo(1.2f).Within(0.0001f));
        });
    }

    [Test]
    public void CompatibilityHookAllowsCrossMapInterpolation()
    {
        var (mapA, mapAId) = CreateMap();
        var (mapB, _) = CreateMap();
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(mapA, Vector2.Zero));
        var child = _entities.SpawnEntity(null, new EntityCoordinates(uid, new Vector2(0.25f, 0f)));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        xform.GridTraversal = false;
        MakeRemote(xform);

        void Compatible(ref RenderSpaceCompatibilityEvent args)
        {
            if ((args.First == mapA && args.Second == mapB) || (args.First == mapB && args.Second == mapA))
                args.CommonSpace = mapA;
        }

        _transforms.RenderSpaceCompatibility += Compatible;
        try
        {
            ApplyRemote(() => _transforms.SetCoordinates(uid, xform, new EntityCoordinates(mapB, Vector2.UnitX), Angle.Zero, false));
            _timing.TickRemainder = TimeSpan.FromTicks(_timing.TickPeriod.Ticks / 4);
            _transforms.FrameUpdate(0f);
            AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.25f, 0f));
            AssertVector(_transforms.GetRenderWorldPosition(child), new Vector2(0.5f, 0f));
            var pose = _transforms.GetRenderWorldPose(child);
            Assert.That(pose.CoordinateSpace, Is.EqualTo(mapA));
            Assert.That(pose.SourceRenderSpace, Is.EqualTo(mapA));
            Assert.That(pose.TargetRenderSpace, Is.EqualTo(mapB));
            Assert.That(pose.RenderSpaceAlpha, Is.EqualTo(0.25f).Within(0.001f));
            Assert.That(_transforms.GetRenderMapCoordinates(child).MapId, Is.EqualTo(mapAId));

            SetTickAlpha(0.75f);
            _transforms.FrameUpdate(0f);
            pose = _transforms.GetRenderWorldPose(child);
            Assert.That(pose.CoordinateSpace, Is.EqualTo(mapA), "the coordinate space must not switch mid-lerp");
            Assert.That(pose.SourceRenderSpace, Is.EqualTo(mapA));
            Assert.That(pose.TargetRenderSpace, Is.EqualTo(mapB));
            Assert.That(pose.RenderSpaceAlpha, Is.EqualTo(0.75f).Within(0.001f));
            Assert.That(_transforms.GetRenderMapCoordinates(child).MapId, Is.EqualTo(mapAId));
        }
        finally
        {
            _transforms.RenderSpaceCompatibility -= Compatible;
        }
    }

    [Test]
    public void RenderCullingBoundsIncludeTheSimulationTreeEntry()
    {
        var (_, mapId) = CreateMap();
        var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        MakeRemote(xform);

        ApplyRemote(() => _transforms.SetLocalPosition(uid, new Vector2(1.5f, 0f), xform));
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var viewport = new Box2Rotated(new Box2(-0.1f, -0.1f, 0.85f, 0.1f), Angle.Zero, Vector2.Zero);
        var queryBounds = _transforms.GetRenderCullingBounds(mapId, viewport);
        Assert.Multiple(() =>
        {
            Assert.That(viewport.CalcBoundingBox().Contains(_transforms.GetRenderWorldPosition(uid)), Is.True);
            Assert.That(viewport.CalcBoundingBox().Contains(_transforms.GetWorldPosition(uid)), Is.False);
            Assert.That(queryBounds.Contains(_transforms.GetWorldPosition(uid)), Is.True);
        });
    }

    [Test]
    public void RenderCullingAccountsForChildrenOfRotatingParents()
    {
        var (map, mapId) = CreateMap();
        var parent = _entities.SpawnEntity(null, new EntityCoordinates(map, Vector2.Zero));
        var child = _entities.SpawnEntity(null, new EntityCoordinates(parent, new Vector2(10f, 0f)));
        var parentXform = _entities.GetComponent<TransformComponent>(parent);
        MakeRemote(parentXform);

        ApplyRemote(() => _transforms.SetLocalRotation(parent, Angle.FromDegrees(90), parentXform));
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var rendered = _transforms.GetRenderWorldPosition(child);
        var simulation = _transforms.GetWorldPosition(child);
        var viewportAabb = Box2.CenteredAround(rendered, new Vector2(0.2f));
        var viewport = new Box2Rotated(viewportAabb, Angle.Zero, viewportAabb.Center);
        var queryBounds = _transforms.GetRenderCullingBounds(mapId, viewport);

        Assert.Multiple(() =>
        {
            Assert.That(viewportAabb.Contains(rendered), Is.True);
            Assert.That(viewportAabb.Contains(simulation), Is.False);
            Assert.That(queryBounds.Contains(simulation), Is.True);
        });
    }

    [Test]
    public void RenderCullingAccountsForRotatingGridExtents()
    {
        var (_, mapId) = CreateMap();
        var grid = _maps.CreateGridEntity(mapId);
        _maps.SetTile(grid, new Vector2i(9, 0), new Tile(1));
        var xform = _entities.GetComponent<TransformComponent>(grid.Owner);
        MakeRemote(xform);

        ApplyRemote(() => _transforms.SetLocalRotation(grid.Owner, Angle.FromDegrees(90), xform));
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var localPoint = grid.Comp.LocalAABB.TopRight;
        var renderedPose = _transforms.GetRenderWorldPose(grid.Owner, xform);
        var rendered = renderedPose.Position + renderedPose.Rotation.RotateVec(localPoint);
        var simulation = _transforms.GetWorldPosition(xform)
                         + _transforms.GetWorldRotation(xform).RotateVec(localPoint);
        var viewportAabb = Box2.CenteredAround(rendered, new Vector2(0.2f));
        var queryBounds = _transforms.GetRenderCullingBounds(mapId, viewportAabb);

        Assert.Multiple(() =>
        {
            Assert.That(viewportAabb.Contains(rendered), Is.True);
            Assert.That(viewportAabb.Contains(simulation), Is.False);
            Assert.That(queryBounds.Contains(simulation), Is.True);
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void RendererLayerSamplesBlendOneContinuousProjectedPose(float verticalOffset)
    {
        var (maps, _, network) = CreateZNetwork(3, new Vector2(0f, verticalOffset));
        var canonical = new Vector2(2f, 3f);
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(maps[1], canonical));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        var sprite = _entities.AddComponent<SpriteComponent>(uid);
        var authoredOffset = new Vector2(0.125f, -0.25f);
        _sprites.SetOffset((uid, sprite), authoredOffset);
        _entities.AddComponent<ZLevelPresentationComponent>(uid);
        MakeRemote(xform);

        ApplyRemote(() => _zPresentation.SetLocalHeight(uid, 1f));
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var pose = _transforms.GetRenderWorldPose(uid, xform);
        var samples = new RenderLayerSample[2];
        var count = _transforms.GetRenderLayerSamples(uid, samples, xform);

        Assert.Multiple(() =>
        {
            Assert.That(count, Is.EqualTo(2));
            Assert.That(pose.AbsoluteZ, Is.EqualTo(1.5f).Within(0.001f));
            AssertVector(pose.CanonicalPosition, canonical);
            AssertVector(pose.Position, canonical + network.Comp.ProjectionOffset * 0.5f);
            Assert.That(samples[0].Map, Is.EqualTo(maps[1]));
            Assert.That(samples[1].Map, Is.EqualTo(maps[2]));
            Assert.That(samples[0].Opacity, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(samples[1].Opacity, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(samples[0].Opacity + samples[1].Opacity, Is.EqualTo(1f).Within(0.001f));
            Assert.That(sprite.Offset, Is.EqualTo(authoredOffset),
                "z presentation must not rewrite the authored sprite offset");
        });

        for (var i = 0; i < count; i++)
        {
            var layerEyeOffset = ZLevelProjection.GetLayerEyeOffset(
                samples[i].Depth - pose.ReferenceDepth,
                network.Comp.ProjectionOffset);
            AssertVector(samples[i].Position - layerEyeOffset, pose.Position,
                "both layer draws must land at the same final projected position");
        }

        SetTickAlpha(1f);
        _transforms.FrameUpdate(0f);
        count = _transforms.GetRenderLayerSamples(uid, samples, xform);
        Assert.Multiple(() =>
        {
            Assert.That(count, Is.EqualTo(1), "an entity outside a transition must be drawn once");
            Assert.That(samples[0].Map, Is.EqualTo(maps[2]));
            Assert.That(samples[0].Opacity, Is.EqualTo(1f));
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void MapOverlayCoordinatesProjectIntoTheSelectedRenderLayer(float verticalOffset)
    {
        var (maps, mapIds, network) = CreateZNetwork(3, new Vector2(0f, verticalOffset));
        var lower = new MapCoordinates(new Vector2(2f, 3f), mapIds[0]);
        var upper = new MapCoordinates(new Vector2(2f, 3f), mapIds[2]);

        Assert.Multiple(() =>
        {
            Assert.That(_transforms.TryProjectMapCoordinatesForLayer(lower, maps[2], out var lowerOnUpper), Is.True);
            AssertVector(lowerOnUpper, lower.Position - network.Comp.ProjectionOffset * 2f);

            Assert.That(_transforms.TryProjectMapCoordinatesForLayer(upper, maps[0], out var upperOnLower), Is.True);
            AssertVector(upperOnLower, upper.Position + network.Comp.ProjectionOffset * 2f);

            Assert.That(_transforms.TryProjectMapCoordinatesForLayer(lower, maps[0], out var sameLayer), Is.True);
            AssertVector(sameLayer, lower.Position);
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void OverlayQueryBoundsAreUnprojectedIntoVisibleMaps(float verticalOffset)
    {
        var (maps, mapIds, _) = CreateZNetwork(3, new Vector2(0f, verticalOffset));
        var layerBounds = new Box2Rotated(
            new Box2(-2f, -3f, 4f, 5f),
            Angle.FromDegrees(20),
            new Vector2(1f, 1f));

        Assert.That(
            _transforms.TryGetMapRenderBoundsForLayer(
                maps[0],
                maps[2],
                layerBounds,
                out var sourceMapId,
                out var sourceBounds),
            Is.True);

        var projectedOrigin = ZLevelProjection.Reproject(
            Vector2.Zero,
            fromReferenceDepth: 0,
            toReferenceDepth: 2,
            new Vector2(0f, verticalOffset));
        Assert.Multiple(() =>
        {
            Assert.That(sourceMapId, Is.EqualTo(mapIds[0]));
            AssertVector(sourceBounds.Box.BottomLeft, layerBounds.Box.BottomLeft - projectedOrigin);
            AssertVector(sourceBounds.Box.TopRight, layerBounds.Box.TopRight - projectedOrigin);
            AssertVector(sourceBounds.Origin, layerBounds.Origin - projectedOrigin);
            Assert.That(sourceBounds.Rotation, Is.EqualTo(layerBounds.Rotation));
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void PostStackEntityAttachmentCombinesOnlyVisibleRendererSamples(float verticalOffset)
    {
        var (maps, _, network) = CreateZNetwork(3, new Vector2(0f, verticalOffset));
        var canonical = new Vector2(2f, 3f);
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(maps[1], canonical));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        _entities.AddComponent<ZLevelPresentationComponent>(uid);
        MakeRemote(xform);

        ApplyRemote(() => _zPresentation.SetLocalHeight(uid, 1f));
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var allVisible = new HashSet<EntityUid> { maps[1], maps[2] };
        var upperOnly = new HashSet<EntityUid> { maps[2] };
        var unrelated = new HashSet<EntityUid> { maps[0] };

        Assert.Multiple(() =>
        {
            Assert.That(_transforms.TryGetPresentedViewSample(uid, maps[2], allVisible, out var combined), Is.True);
            AssertVector(combined.Position, canonical - network.Comp.ProjectionOffset * 0.5f);
            Assert.That(combined.Opacity, Is.EqualTo(1f).Within(0.001f));
            Assert.That(combined.AbsoluteZ, Is.EqualTo(1.5f).Within(0.001f));

            Assert.That(_transforms.TryGetPresentedViewSample(uid, maps[2], upperOnly, out var clipped), Is.True);
            AssertVector(clipped.Position, combined.Position);
            Assert.That(clipped.Opacity, Is.EqualTo(0.5f).Within(0.001f));

            Assert.That(_transforms.TryGetPresentedViewSample(uid, maps[2], unrelated, out _), Is.False);
        });
    }

    [TestCase(0.00001f, 1, 1f)]
    [TestCase(0.25f, 2, 0.25f)]
    [TestCase(0.99999f, 1, 1f)]
    [TestCase(1f, 1, 1f)]
    [TestCase(1.00001f, 1, 1f)]
    public void RendererLayerSelectionHandlesIntegerBoundaries(float height, int expectedCount, float upperWeight)
    {
        var (maps, _, _) = CreateZNetwork(3, new Vector2(0f, 0.7f));
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(maps[0], Vector2.Zero));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        _entities.AddComponent<ZLevelPresentationComponent>(uid);
        MakeRemote(xform);

        ApplyRemote(() => _zPresentation.SetLocalHeight(uid, height));
        SetTickAlpha(1f);
        _transforms.FrameUpdate(0f);

        var samples = new RenderLayerSample[2];
        var count = _transforms.GetRenderLayerSamples(uid, samples, xform);
        Assert.That(count, Is.EqualTo(expectedCount));
        Assert.That(samples[count - 1].Opacity, Is.EqualTo(upperWeight).Within(0.0001f));
        if (count == 2)
            Assert.That(samples[0].Opacity + samples[1].Opacity, Is.EqualTo(1f).Within(0.0001f));
    }

    [Test]
    public void CrossLevelGridTransitionFollowsMovingRotatingParentsAndCamera()
    {
        var (maps, mapIds, network) = CreateZNetwork(2, new Vector2(0f, 0.7f));
        var lowerGridEntity = _maps.CreateGridEntity(mapIds[0]);
        var upperGridEntity = _maps.CreateGridEntity(mapIds[1]);
        var lowerGrid = lowerGridEntity.Owner;
        var upperGrid = upperGridEntity.Owner;
        _maps.SetTile(lowerGridEntity, new Vector2i(1, 0), new Tile(1));
        _maps.SetTile(upperGridEntity, new Vector2i(1, 0), new Tile(1));
        _transforms.SetWorldPosition(upperGrid, new Vector2(0.5f, 0f));
        _transforms.SnapRenderPose(upperGrid);

        var uid = _entities.SpawnEntity(null, new EntityCoordinates(maps[0], Vector2.UnitX));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        xform.GridTraversal = false;
        _transforms.SetCoordinates(uid, xform, new EntityCoordinates(lowerGrid, Vector2.UnitX), Angle.Zero, false);
        Assert.That(xform.ParentUid, Is.EqualTo(lowerGrid));
        _entities.AddComponent<ZLevelPresentationComponent>(uid);
        var eye = _entities.AddComponent<EyeComponent>(uid);
        MakeRemote(lowerGrid, upperGrid, uid);

        ApplyRemote(() =>
        {
            _transforms.SetCoordinates(uid, xform, new EntityCoordinates(upperGrid, Vector2.UnitX), Angle.Zero, false);
            _transforms.SetWorldPositionRotation(lowerGrid, new Vector2(0f, 1f), Angle.FromDegrees(90));
            _transforms.SetWorldPositionRotation(upperGrid, new Vector2(0.5f, 1f), Angle.FromDegrees(-90));
        });

        SetHalfTick();
        _transforms.FrameUpdate(0f);
        _eyes.FrameUpdate(0f);

        var halfRoot = new Vector2(MathF.Sqrt(0.5f), 0.5f + MathF.Sqrt(0.5f));
        var halfTarget = new Vector2(0.5f + MathF.Sqrt(0.5f), 0.5f - MathF.Sqrt(0.5f));
        var expectedCanonical = Vector2.Lerp(halfRoot, halfTarget, 0.5f);
        var pose = _transforms.GetRenderWorldPose(uid, xform);
        Assert.That(_transforms.TryGetRenderPoseDebugData(uid, out var debugPose), Is.True);
        Assert.That(debugPose.SourceParent, Is.EqualTo(lowerGrid));
        var samples = new RenderLayerSample[2];
        var count = _transforms.GetRenderLayerSamples(uid, samples, xform);

        Assert.Multiple(() =>
        {
            AssertVector(pose.CanonicalPosition, expectedCanonical);
            Assert.That(pose.AbsoluteZ, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(pose.CoordinateSpace, Is.EqualTo(maps[1]));
            AssertVector(pose.Position, expectedCanonical - network.Comp.ProjectionOffset * 0.5f);
            Assert.That(count, Is.EqualTo(2));
            Assert.That(samples[0].Opacity + samples[1].Opacity, Is.EqualTo(1f).Within(0.001f));
            AssertVector(eye.Eye.Position.Position, pose.Position,
                "camera and controlled entity must consume the same presented pose");
            Assert.That(eye.Eye.Position.MapId, Is.EqualTo(mapIds[1]));
            Assert.That(eye.Eye.PresentedAbsoluteZ, Is.EqualTo(pose.AbsoluteZ).Within(0.001f));
        });

        var viewport = Box2.CenteredAround(samples[0].Position, new Vector2(0.05f));
        var queryBounds = _transforms.GetRenderCullingBounds(mapIds[1], viewport);
        Assert.That(queryBounds.Contains(_transforms.GetWorldPosition(uid)), Is.True,
            "the target map tree query must retain a source-layer render sample");
    }

    [TestCase(ParentTransition.GridToMap)]
    [TestCase(ParentTransition.MapToGrid)]
    public void CrossLevelGridMapTransitionsUseOneTimeline(ParentTransition transition)
    {
        var (maps, mapIds, _) = CreateZNetwork(2, new Vector2(0f, 0.7f));
        var lowerGrid = _maps.CreateGridEntity(mapIds[0]).Owner;
        var upperGrid = _maps.CreateGridEntity(mapIds[1]).Owner;
        _transforms.SetWorldPosition(upperGrid, Vector2.UnitX);
        _transforms.SnapRenderPose(upperGrid);

        var source = transition == ParentTransition.MapToGrid ? maps[0] : lowerGrid;
        var destination = transition == ParentTransition.GridToMap ? maps[1] : upperGrid;
        var destinationLocal = transition == ParentTransition.GridToMap ? Vector2.UnitX : Vector2.Zero;
        var uid = _entities.SpawnEntity(null, new EntityCoordinates(maps[0], Vector2.Zero));
        var xform = _entities.GetComponent<TransformComponent>(uid);
        xform.GridTraversal = false;
        _transforms.SetCoordinates(uid, xform, new EntityCoordinates(source, Vector2.Zero), Angle.Zero, false);
        _transforms.SnapRenderPose(uid);
        _entities.AddComponent<ZLevelPresentationComponent>(uid);
        MakeRemote(uid);

        ApplyRemote(() => _transforms.SetCoordinates(
            uid,
            xform,
            new EntityCoordinates(destination, destinationLocal),
            Angle.Zero,
            false));
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var pose = _transforms.GetRenderWorldPose(uid, xform);
        var samples = new RenderLayerSample[2];
        var count = _transforms.GetRenderLayerSamples(uid, samples, xform);
        Assert.Multiple(() =>
        {
            AssertVector(pose.CanonicalPosition, new Vector2(0.5f, 0f));
            Assert.That(pose.AbsoluteZ, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(count, Is.EqualTo(2));
            Assert.That(samples[0].Opacity, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(samples[1].Opacity, Is.EqualTo(0.5f).Within(0.001f));
        });
    }

    [Test]
    public void RendererEffectSelectionHonorsReplicatedShaderConfiguration()
    {
        var (_, _, network) = CreateZNetwork(2, new Vector2(0f, 0.7f));

        _zLevels.SetLowerLevelEffects(
            network.Owner,
            enabled: false,
            shader: "game-z-shader",
            blurRadius: 4f,
            darkenStrength: 0.6f,
            tint: new Color(0.1f, 0.2f, 0.3f, 0.4f));
        var disabled = Clyde.ResolveZLevelLayerEffects(network.Comp, -1, gameShaderAvailable: true, effectStrength: 1f);

        _zLevels.SetLowerLevelEffects(
            network.Owner,
            enabled: true,
            shader: "game-z-shader",
            blurRadius: 4f,
            darkenStrength: 0.6f,
            tint: new Color(0.1f, 0.2f, 0.3f, 0.4f));
        var game = Clyde.ResolveZLevelLayerEffects(network.Comp, -1, gameShaderAvailable: true, effectStrength: 1f);
        var fallback = Clyde.ResolveZLevelLayerEffects(network.Comp, -1, gameShaderAvailable: false, effectStrength: 1f);
        var transitioning = Clyde.ResolveZLevelLayerEffects(
            network.Comp,
            -1,
            gameShaderAvailable: false,
            effectStrength: 0.5f);
        var current = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, gameShaderAvailable: true, effectStrength: 0f);

        Assert.Multiple(() =>
        {
            Assert.That(disabled.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.None));
            Assert.That(disabled.Tint, Is.EqualTo(new Color(0.1f, 0.2f, 0.3f, 0.4f)));
            Assert.That(game.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.GameShader));
            Assert.That(fallback.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.EngineDefault));
            Assert.That(fallback.BlurRadius, Is.EqualTo(4f));
            Assert.That(fallback.DarkenStrength, Is.EqualTo(0.6f));
            Assert.That(transitioning.Strength, Is.EqualTo(0.5f));
            Assert.That(transitioning.BlurRadius, Is.EqualTo(2f));
            Assert.That(transitioning.DarkenStrength, Is.EqualTo(0.3f));
            Assert.That(transitioning.Tint.A, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(current.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.None));
            Assert.That(current.Tint, Is.EqualTo(Color.Transparent));
        });
    }

    [Test]
    public void ZPhysicsDoesNotChangeChatTwoLayerOrShaderSelection()
    {
        var (maps, mapIds, network) = CreateZNetwork(4, new Vector2(0f, 0.7f));
        _zLevels.SetLowerLevelEffects(
            network.Owner,
            enabled: true,
            shader: "known-good-lower-shader",
            blurRadius: 3f,
            darkenStrength: 0.4f,
            tint: Color.Transparent);

        var belowMaps = new List<MapId>();
        var aboveMaps = new List<MapId>();
        _zLevels.CollectRenderableMaps(
            maps[2],
            mapIds[2],
            network.Comp.VisibleLevelsBelow,
            network.Comp.VisibleLevelsAbove,
            default,
            default,
            belowMaps,
            aboveMaps);
        var lowerBefore = Clyde.ResolveZLevelLayerEffects(network.Comp, -1, gameShaderAvailable: true, effectStrength: 1f);
        var currentBefore = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, gameShaderAvailable: true, effectStrength: 0f);
        var farthestRenderedLower = belowMaps.Count > 0 ? belowMaps[^1] : mapIds[2];

        var falling = _entities.SpawnEntity(null, new EntityCoordinates(maps[2], Vector2.Zero));
        var xform = _entities.GetComponent<TransformComponent>(falling);
        _entities.AddComponent<ZLevelPhysicsComponent>(falling);
        MakeRemote(falling);
        ApplyRemote(() =>
        {
            _transforms.SetCoordinates(
                falling,
                xform,
                new EntityCoordinates(maps[1], Vector2.Zero),
                Angle.Zero,
                false);
            _zPresentation.SetLocalHeight(falling, 0.25f);
        });
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        var lowerAfter = Clyde.ResolveZLevelLayerEffects(network.Comp, -1, gameShaderAvailable: true, effectStrength: 1f);
        var currentAfter = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, gameShaderAvailable: true, effectStrength: 0f);
        Assert.Multiple(() =>
        {
            Assert.That(belowMaps, Is.EqualTo(new[] { mapIds[1], mapIds[0] }),
                "the current map keeps the configured lower-map stack");
            Assert.That(aboveMaps, Is.Empty, "the default viewport must not include the map immediately above");
            Assert.That(farthestRenderedLower, Is.EqualTo(mapIds[0]),
                "background/parallax must be associated with the farthest map in the actual lower render stack");
            Assert.That(network.Comp.VisibleLevelsAbove, Is.Zero);
            Assert.That(lowerBefore.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.GameShader));
            Assert.That(currentBefore.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.None));
            Assert.That(lowerAfter, Is.EqualTo(lowerBefore),
                "a received z/map transition must not disable or bypass the configured lower shader");
            Assert.That(currentAfter, Is.EqualTo(currentBefore));
        });
    }

    [Test]
    public void PresentedDescentKeepsDestinationEffectsOnContinuousTimeline()
    {
        var (_, mapIds, network) = CreateZNetwork(2, new Vector2(0f, 0.7f));
        _zLevels.SetLowerLevelEffects(
            network.Owner,
            enabled: true,
            shader: null,
            blurRadius: 2f,
            darkenStrength: 0.4f,
            tint: Color.Transparent);

        // After a downward reparent the lower destination is relative depth zero, but the presented eye is still
        // 0.75 planes above it. The effect must fade continuously instead of switching off with the map parent.
        var beforeReparent = Clyde.ResolveZLevelLayerEffects(network.Comp, -1, false, 0.75f);
        var afterReparent = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, false, 0.75f);
        var halfway = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, false, 0.5f);
        var landed = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, false, 0f);

        Assert.Multiple(() =>
        {
            Assert.That(
                Clyde.ResolveZLevelLightingMap(new Clyde.ZLevelRenderLayer(mapIds[0], -1)),
                Is.EqualTo(mapIds[0]),
                "ambient and point lights must use the destination layer before reparenting");
            Assert.That(
                Clyde.ResolveZLevelLightingMap(new Clyde.ZLevelRenderLayer(mapIds[0], 0)),
                Is.EqualTo(mapIds[0]),
                "ambient and point lights must keep using the destination after reparenting");
            Assert.That(afterReparent, Is.EqualTo(beforeReparent));
            Assert.That(afterReparent.DarkenStrength, Is.EqualTo(0.3f).Within(0.001f));
            Assert.That(halfway.DarkenStrength, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(landed.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.None));
            Assert.That(landed.Strength, Is.Zero);
        });

        for (var repetition = 0; repetition < 3; repetition++)
        {
            var descending = new[] { 1f, 0.75f, 0.5f, 0.25f, 0f }
                .Select(strength => Clyde.ResolveZLevelLayerEffects(network.Comp, 0, false, strength).DarkenStrength)
                .ToArray();
            Assert.That(descending, Is.EqualTo(new[] { 0.4f, 0.3f, 0.2f, 0.1f, 0f }).Within(0.001f));
        }

        _zLevels.SetLowerLevelEffects(
            network.Owner,
            enabled: false,
            shader: null,
            blurRadius: 2f,
            darkenStrength: 0.4f,
            tint: Color.Transparent);
        var disabled = Clyde.ResolveZLevelLayerEffects(network.Comp, 0, false, 0.75f);
        Assert.That(disabled.Shader, Is.EqualTo(Clyde.ZLevelPostShaderSelection.None));
    }

    [Test]
    public void ProjectedRendererSamplesDriveYSort()
    {
        var (maps, _, _) = CreateZNetwork(2, new Vector2(0f, 0.7f));
        var lower = _entities.SpawnEntity(null, new EntityCoordinates(maps[0], new Vector2(0f, 0.2f)));
        var crossing = _entities.SpawnEntity(null, new EntityCoordinates(maps[0], Vector2.Zero));
        _entities.AddComponent<ZLevelPresentationComponent>(crossing);
        _zPresentation.SetLocalHeight(crossing, 0.5f);
        SetHalfTick();
        _transforms.FrameUpdate(0f);

        Assert.That(_transforms.TryGetRenderLayerSample(lower, maps[0], out var lowerSample), Is.True);
        Assert.That(_transforms.TryGetRenderLayerSample(crossing, maps[0], out var crossingSample), Is.True);

        var localBounds = new Box2(-0.5f, -0.5f, 0.5f, 0.5f);
        var scale = new Vector2(1f, -1f);
        var lowerBounds = Clyde.TransformCenteredBox(localBounds, 0f, lowerSample.Position, scale);
        var crossingBounds = Clyde.TransformCenteredBox(localBounds, 0f, crossingSample.Position, scale);
        var lowerSort = new Clyde.SpriteSortItem(0, 0, 0, lowerBounds.Top, lower);
        var crossingSort = new Clyde.SpriteSortItem(1, 0, 0, crossingBounds.Top, crossing);

        Assert.That(Math.Sign(lowerSort.CompareTo(crossingSort)),
            Is.EqualTo(Math.Sign(lowerBounds.Top.CompareTo(crossingBounds.Top))));
        Assert.That(lowerBounds.Top, Is.Not.EqualTo(crossingBounds.Top),
            "continuous z projection must participate in the renderer's y-sort key");
    }

    [TestCase(30, 144f)]
    [TestCase(60, 144f)]
    public void PredictionReplayDoesNotDisturbConstantRenderVelocity(int tickRate, float renderFps)
    {
        var oldTickRate = _timing.TickRate;
        try
        {
            _timing.SetTickRateAt((ushort) tickRate, _timing.CurTick);
            var tickPeriod = (float) _timing.TickPeriod.TotalSeconds;
            var frameTime = 1f / renderFps;
            const float speed = 4.5f;
            var tickDistance = speed / tickRate;
            var (_, mapId) = CreateMap();
            var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
            var xform = _entities.GetComponent<TransformComponent>(uid);
            var accumulator = 0f;
            var simulationTick = 0;
            float? previousRender = null;
            var velocities = new List<float>();
            var simulationDisplacements = new List<float>();

            _timing.CurTick = _timing.LastRealTick + 1;
            _transforms.SetLocalPosition(uid, new Vector2(tickDistance, 0f), xform);
            var previousSimulationEndpoint = _transforms.GetWorldPosition(uid).X;

            for (var frame = 0; simulationTick < 24; frame++)
            {
                accumulator += frameTime;

                while (accumulator >= tickPeriod)
                {
                    accumulator -= tickPeriod;
                    simulationTick++;
                    ReplayPredictionTick(simulationTick);
                    var simulationEndpoint = _transforms.GetWorldPosition(uid).X;
                    simulationDisplacements.Add(simulationEndpoint - previousSimulationEndpoint);
                    previousSimulationEndpoint = simulationEndpoint;
                }

                _timing.TickRemainder = TimeSpan.FromSeconds(accumulator);
                _transforms.FrameUpdate(frameTime);

                var render = _transforms.GetRenderWorldPosition(uid).X;
                if (previousRender is { } previous && simulationTick >= 2)
                    velocities.Add((render - previous) / frameTime);

                previousRender = render;
            }

            Assert.Multiple(() =>
            {
                Assert.That(simulationDisplacements.Min(), Is.EqualTo(tickDistance).Within(0.0001f));
                Assert.That(simulationDisplacements.Max(), Is.EqualTo(tickDistance).Within(0.0001f));
                Assert.That(velocities.Min(), Is.EqualTo(speed).Within(0.25f));
                Assert.That(velocities.Max(), Is.EqualTo(speed).Within(0.25f));
            });

            void ReplayPredictionTick(int realTickIndex)
            {
                var previousRealTick = _timing.LastRealTick;
                var nextRealTick = previousRealTick + 1;

                using (_timing.StartStateApplicationArea())
                    _transforms.SetLocalPosition(uid, new Vector2((realTickIndex - 1) * tickDistance, 0f), xform);

                xform.LastModifiedTick = previousRealTick;

                _timing.CurTick = _timing.LastRealTick = nextRealTick;
                using (_timing.StartStateApplicationArea())
                    _transforms.SetLocalPosition(uid, new Vector2(realTickIndex * tickDistance, 0f), xform);

                _timing.CurTick = nextRealTick + 1;
                using (_timing.StartPastPredictionArea())
                    _transforms.SetLocalPosition(uid, new Vector2((realTickIndex + 1) * tickDistance, 0f), xform);
            }
        }
        finally
        {
            _timing.SetTickRateAt(oldTickRate, _timing.CurTick);
            _timing.TickRemainder = TimeSpan.Zero;
        }
    }

    [Test]
    public void SnappedLocalMispredictionUsesPredictionCorrection()
    {
        var (_, mapId) = CreateMap();
        var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var xform = _entities.GetComponent<TransformComponent>(uid);

        _timing.CurTick = new GameTick(_timing.LastRealTick.Value + 2);
        _transforms.SetLocalPosition(uid, Vector2.UnitX, xform);
        _transforms.SnapRenderPose(uid, true);

        AssertVector(_transforms.GetWorldPosition(uid), Vector2.UnitX);
        AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.UnitX);

        using (_timing.StartStateApplicationArea())
            _transforms.SetLocalPosition(uid, Vector2.Zero, xform);

        Assert.Multiple(() =>
        {
            AssertVector(_transforms.GetWorldPosition(uid), Vector2.Zero);
            AssertVector(_transforms.GetRenderWorldPosition(uid), Vector2.UnitX);
            Assert.That(_transforms.TryGetRenderPoseDebugData(uid, out var data), Is.True);
            Assert.That(data.Type, Is.EqualTo(RenderInterpolationType.PredictionCorrection));
        });

        _transforms.FrameUpdate(CorrectionHalfLifeForTest);
        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f));
    }

    [Test]
    public void PredictionReplayPreservesProgressAndUpdatesDestinationContinuously()
    {
        var (_, mapId) = CreateMap();
        var uid = _entities.SpawnEntity(null, new MapCoordinates(Vector2.Zero, mapId));
        var xform = _entities.GetComponent<TransformComponent>(uid);

        _timing.CurTick = new GameTick(_timing.LastRealTick.Value + 1);
        _transforms.SetLocalPosition(uid, Vector2.UnitX, xform);
        SetTickAlpha(0.25f);
        _transforms.FrameUpdate(0f);
        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.25f, 0f));

        using (_timing.StartStateApplicationArea())
            _transforms.SetLocalPosition(uid, Vector2.Zero, xform);
        using (_timing.StartPastPredictionArea())
            _transforms.SetLocalPosition(uid, Vector2.UnitX, xform);

        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.25f, 0f));

        SetTickAlpha(0.5f);
        _transforms.FrameUpdate(0f);
        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f));

        using (_timing.StartStateApplicationArea())
            _transforms.SetLocalPosition(uid, Vector2.Zero, xform);
        using (_timing.StartPastPredictionArea())
            _transforms.SetLocalPosition(uid, new Vector2(2f, 0f), xform);

        Assert.Multiple(() =>
        {
            AssertVector(xform.LocalPosition, new Vector2(2f, 0f));
            AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(0.5f, 0f));
        });

        SetTickAlpha(0.75f);
        _transforms.FrameUpdate(0f);
        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(1.25f, 0f));

        SetTickAlpha(1f);
        _transforms.FrameUpdate(0f);
        AssertVector(_transforms.GetRenderWorldPosition(uid), new Vector2(2f, 0f));
    }

    private (EntityUid[] Maps, MapId[] MapIds, Entity<ZLevelMapNetworkComponent> Network) CreateZNetwork(
        int count,
        Vector2 projectionOffset)
    {
        var maps = new EntityUid[count];
        var mapIds = new MapId[count];
        for (var i = 0; i < count; i++)
            maps[i] = _maps.CreateMap(out mapIds[i]);

        var networkUid = _entities.SpawnEntity(null, MapCoordinates.Nullspace);
        var networkComp = _entities.AddComponent<ZLevelMapNetworkComponent>(networkUid);
        Entity<ZLevelMapNetworkComponent> network = (networkUid, networkComp);
        var depths = maps.Select((map, depth) => (map, depth)).ToDictionary(pair => pair.map, pair => pair.depth);
        Assert.That(_zLevels.TryAddMaps(network, depths), Is.True);
        _zLevels.SetProjectionOffset(networkUid, projectionOffset);
        return (maps, mapIds, network);
    }

    private (EntityUid Uid, MapId Id) CreateMap()
    {
        var uid = _maps.CreateMap(out var id);
        return (uid, id);
    }

    private void ApplyRemote(Action action)
    {
        using (_timing.StartStateApplicationArea())
            action();
    }

    private void SetHalfTick()
    {
        SetTickAlpha(0.5f);
    }

    private void SetTickAlpha(float alpha)
    {
        _timing.TickRemainder = TimeSpan.FromTicks((long) (_timing.TickPeriod.Ticks * alpha));
    }

    private float CorrectionHalfLifeForTest => _configuration.GetCVar(CVars.NetInterpCorrectionHalfLife);

    private void MakeRemote(params EntityUid[] entities)
    {
        foreach (var uid in entities)
            MakeRemote(_entities.GetComponent<TransformComponent>(uid));
    }

    private void MakeRemote(TransformComponent xform)
    {
        xform.LastModifiedTick = _timing.LastRealTick;
    }

    private static void AssertVector(Vector2 actual, Vector2 expected, string? message = null)
    {
        Assert.That(actual.X, Is.EqualTo(expected.X).Within(0.001f), message);
        Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(0.001f), message);
    }

    public enum ParentTransition : byte
    {
        GridToMap,
        MapToGrid,
        GridToGrid,
    }
}
