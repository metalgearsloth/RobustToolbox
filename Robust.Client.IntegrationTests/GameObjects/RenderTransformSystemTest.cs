using System;
using System.Numerics;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Client.Timing;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.GameStates;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
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
        _timing = IoCManager.Resolve<IClientGameTiming>();
    }

    [SetUp]
    public void Setup()
    {
        _transforms.ResetRenderPoses();
        _timing.TickRemainder = TimeSpan.Zero;
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
