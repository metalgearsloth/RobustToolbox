using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Dynamics;
using Robust.Shared.Physics.Systems;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.Physics;

[TestFixture]
[Parallelizable(ParallelScope.All | ParallelScope.Fixtures)]
[TestOf(typeof(ZLevelPhysicsSystem))]
internal sealed class ZLevelSupportTest
{
    private const string BodyFixture = "body";
    private const string VolumeFixture = "volume";

    [Test]
    public void SupportQueryIsPureAndUsesCanonicalFootPoint()
    {
        var world = CreateWorld(1);
        var platform = SpawnPlatform(world, 0, new Vector2(0.5f), 0.2f, new Vector2(0.5f), solidVolume: false);
        var body = SpawnMob(world, 0, new Vector2(0.5f), 0.2f);
        GroundAtCurrentHeight(world, body);

        var proposed = new Vector2(1.06f, 0.5f);
        var originalPosition = world.Transform.GetWorldPosition(body.Uid);
        var originalMap = world.Transform.GetMap(body.Uid);
        var originalHeight = body.Presentation.LocalHeight;
        var originalVelocity = body.Physics.Velocity;
        var originalGround = body.Physics.GroundState;
        var originalProvider = body.Physics.SupportProvider;
        var diagnostics = new List<ZLevelSupportCandidateDebug>();

        Assert.That(world.Support.TryQuerySupport(
            (body.Uid, body.Physics),
            proposed,
            GetAbsoluteZ(world, body),
            world.ZPhysics.MaxStepUp,
            out var result,
            diagnostics), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(result.Provider, Is.EqualTo(platform));
            Assert.That(result.Surface, Is.EqualTo(ZLevelSupportSurface.HighGround));
            Assert.That(result.AbsoluteHeight, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(result.SamplePoint.X, Is.EqualTo(proposed.X).Within(0.0001f),
                "the sampled support point must remain the body's canonical foot point, not a clamped provider point");
            Assert.That(result.SamplePoint.Y, Is.EqualTo(proposed.Y).Within(0.0001f));
            Assert.That(world.Transform.GetWorldPosition(body.Uid), Is.EqualTo(originalPosition));
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(originalMap));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(originalHeight));
            Assert.That(body.Physics.Velocity, Is.EqualTo(originalVelocity));
            Assert.That(body.Physics.GroundState, Is.EqualTo(originalGround));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(originalProvider));
            Assert.That(diagnostics.Exists(candidate =>
                candidate.Provider == platform &&
                candidate.Rejection == ZLevelSupportRejection.None), Is.True);
        });
    }

    [Test]
    public void SupportQueryUsesAuthoredFixtureInsteadOfUnionedProviderBounds()
    {
        var world = CreateWorld(1);
        var platform = SpawnPlatform(
            world,
            0,
            new Vector2(0.5f),
            0.2f,
            supportHalfExtents: new Vector2(0.2f),
            solidVolume: true,
            volumeHalfExtents: new Vector2(1f));
        var body = SpawnMob(world, 0, new Vector2(1.25f, 0.5f), 0.2f);
        GroundAtCurrentHeight(world, body);
        var diagnostics = new List<ZLevelSupportCandidateDebug>();

        Assert.That(world.Support.TryQuerySupport(
            (body.Uid, body.Physics),
            world.Transform.GetWorldPosition(body.Uid),
            GetAbsoluteZ(world, body),
            world.ZPhysics.MaxStepUp,
            out _,
            diagnostics), Is.False);

        Assert.That(diagnostics.Exists(candidate =>
            candidate.Provider == platform &&
            candidate.Rejection == ZLevelSupportRejection.OutsideFootprint), Is.True,
            "a large hard volume fixture must not expand the authored walkable support fixture");
    }

    [Test]
    public void SupportQueryEvaluatesRampHeightCurveAtSamplePoint()
    {
        var world = CreateWorld(1);
        var ramp = SpawnPlatform(world, 0, new Vector2(0.5f), 0f, new Vector2(0.5f), solidVolume: false);
        var highGround = world.Entities.GetComponent<ZLevelHighGroundComponent>(ramp);
        SetField(highGround, nameof(ZLevelHighGroundComponent.HeightCurve), new List<float> { 0.1f, 1.05f });
        var body = SpawnMob(world, 0, new Vector2(0.5f, 0.75f), 1.05f);

        Assert.That(world.Support.TryQuerySupport(
            (body.Uid, body.Physics),
            world.Transform.GetWorldPosition(body.Uid),
            GetAbsoluteZ(world, body),
            world.ZPhysics.MaxStepUp,
            out var result), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(result.Provider, Is.EqualTo(ramp));
            Assert.That(result.Surface, Is.EqualTo(ZLevelSupportSurface.HighGround));
            Assert.That(result.AbsoluteHeight, Is.EqualTo(0.3375f).Within(0.001f));
        });
    }

    [Test]
    public void ActualMovementWalksOnFlatFloor()
    {
        var world = CreateWorld(1, floorLevels: [0], floorLength: 6);
        var body = SpawnMob(world, 0, new Vector2(0.5f), 0f);
        GroundAtCurrentHeight(world, body);

        world.SharedPhysics.SetLinearVelocity(body.Uid, new Vector2(1.2f, 0f), body: body.Body);
        RunFor(world, 1.5f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetWorldPosition(body.Uid).X, Is.GreaterThan(1.5f));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(world.FloorGrids[0]));
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.Tile));
            Assert.That(body.Physics.SupportHeight, Is.Zero.Within(0.001f));
            Assert.That(body.Presentation.LocalHeight, Is.Zero.Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });
    }

    [Test]
    public void ActualMovementStepsOntoLowPlatformAndStepsDown()
    {
        var world = CreateWorld(1, floorLevels: [0], floorLength: 7);
        var platform = SpawnPlatform(world, 0, new Vector2(2.5f, 0.5f), 0.2f, new Vector2(0.5f), solidVolume: false);
        var body = SpawnMob(world, 0, new Vector2(0.5f, 0.5f), 0f);
        GroundAtCurrentHeight(world, body);

        world.SharedPhysics.SetLinearVelocity(body.Uid, new Vector2(1.2f, 0f), body: body.Body);
        RunUntil(world, () => body.Physics.SupportProvider == platform, 3f);

        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.HighGround));
            Assert.That(body.Physics.SupportHeight, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });

        RunUntil(world, () =>
            world.Transform.GetWorldPosition(body.Uid).X > 3.2f &&
            body.Physics.SupportSurface == ZLevelSupportSurface.Tile, 3f);

        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(world.FloorGrids[0]));
            Assert.That(body.Physics.SupportHeight, Is.Zero.Within(0.001f));
            Assert.That(body.Presentation.LocalHeight, Is.Zero.Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });
    }

    [Test]
    public void ActualMovementBlocksPlatformAboveMaxStepUp()
    {
        var world = CreateWorld(1, floorLevels: [0], floorLength: 5);
        var highPlatform = SpawnPlatform(
            world,
            0,
            new Vector2(2f, 0.5f),
            0.4f,
            new Vector2(0.5f),
            solidVolume: true);
        var body = SpawnMob(world, 0, new Vector2(0.5f, 0.5f), 0f);
        GroundAtCurrentHeight(world, body);

        world.SharedPhysics.SetLinearVelocity(body.Uid, new Vector2(1.5f, 0f), body: body.Body);
        RunFor(world, 2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetWorldPosition(body.Uid).X, Is.LessThan(1.75f),
                "the hard platform volume must block ordinary XY movement when its top is above MaxStepUp");
            Assert.That(body.Physics.SupportProvider, Is.Not.EqualTo(highPlatform));
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.Tile));
            Assert.That(body.Presentation.LocalHeight, Is.Zero.Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });
    }

    [Test]
    public void ActualLowBodyIsBlockedByWallVolume()
    {
        var world = CreateWorld(1, floorLevels: [0], floorLength: 4);
        var wall = SpawnWall(world, 0, new Vector2(1.5f, 0.5f));
        var body = SpawnMob(world, 0, new Vector2(0.5f, 0.5f), 0f);
        GroundAtCurrentHeight(world, body);

        world.SharedPhysics.SetLinearVelocity(body.Uid, new Vector2(1.5f, 0f), body: body.Body);
        RunFor(world, 2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetWorldPosition(body.Uid).X, Is.LessThan(0.85f),
                "a body below the wall top step range must never enter the wall volume");
            Assert.That(body.Physics.SupportProvider, Is.Not.EqualTo(wall));
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.Tile));
            Assert.That(body.Presentation.LocalHeight, Is.Zero.Within(0.001f));
        });
    }

    [Test]
    public void ActualMovementWalksOffHighWallAndFalls()
    {
        var world = CreateWorld(2);
        var upperFloor = CreateGridWithTiles(world, 1, 0, 1);
        var wall = SpawnWall(world, 0, new Vector2(1.5f, 0.5f));
        var body = SpawnMob(world, 1, new Vector2(0.5f, 0.5f), 0f);
        GroundAtCurrentHeight(world, body);

        Assert.That(body.Physics.SupportProvider, Is.EqualTo(upperFloor));

        world.SharedPhysics.SetLinearVelocity(body.Uid, new Vector2(0.8f, 0f), body: body.Body);
        var minimumWallAbsoluteZ = float.PositiveInfinity;

        for (var i = 0; i < 240; i++)
        {
            Step(world);

            if (body.Physics.SupportProvider == wall)
                minimumWallAbsoluteZ = MathF.Min(minimumWallAbsoluteZ, GetAbsoluteZ(world, body));

            if (world.Transform.GetWorldPosition(body.Uid).X > 2.15f &&
                body.Physics.GroundState == ZLevelGroundState.Airborne)
            {
                break;
            }
        }

        Assert.Multiple(() =>
        {
            Assert.That(minimumWallAbsoluteZ, Is.EqualTo(1.05f).Within(0.001f),
                "the body must only occupy the wall XY footprint while carried by the authored top surface");
            Assert.That(world.Transform.GetWorldPosition(body.Uid).X, Is.GreaterThan(2.15f));
            Assert.That(body.Physics.SupportProvider, Is.Not.EqualTo(wall));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Airborne));
            Assert.That(body.Physics.Velocity, Is.LessThan(0f));
        });
    }

    [Test]
    public void ActualFallLandsOnWallTopAndNormalizesMap()
    {
        var world = CreateWorld(2);
        var wall = SpawnWall(world, 0, new Vector2(0.5f));
        var falling = SpawnMob(world, 0, new Vector2(0.5f), 1.5f, zVelocity: -1f);

        RunUntil(world, () => falling.Physics.GroundState == ZLevelGroundState.Grounded, 2f);

        Assert.Multiple(() =>
        {
            Assert.That(falling.Physics.SupportProvider, Is.EqualTo(wall));
            Assert.That(falling.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.HighGround));
            Assert.That(falling.Physics.SupportHeight, Is.EqualTo(1.05f).Within(0.001f));
            Assert.That(world.Transform.GetMap(falling.Uid), Is.EqualTo(world.Maps[1]));
            Assert.That(falling.Presentation.LocalHeight, Is.EqualTo(0.05f).Within(0.001f));
            Assert.That(GetAbsoluteZ(world, falling), Is.EqualTo(1.05f).Within(0.001f));
        });
    }

    [Test]
    public void MovingAndRotatingLinkedGridsKeepSupportUsingTrackedBodies()
    {
        var world = CreateWorld(2);
        var lowerGrid = world.Map.CreateGridEntity(world.MapIds[0]);
        var upperGrid = world.Map.CreateGridEntity(world.MapIds[1]);
        Assert.That(world.ZLevels.TryLinkGrids(lowerGrid, upperGrid), Is.True);

        var wall = SpawnWall(world, lowerGrid, new Vector2(0.5f));
        var body = SpawnMob(world, upperGrid, new Vector2(0.5f), 0.05f);
        GroundAtCurrentHeight(world, body);
        var lowerBefore = world.Transform.GetWorldPosition(lowerGrid);
        var upperBefore = world.Transform.GetWorldPosition(upperGrid);

        Assert.That(body.Physics.SupportProvider, Is.EqualTo(wall));

        Assert.That(world.ZLevels.TryMoveLinkedGrids(
            lowerGrid,
            new Vector2(3f, -2f),
            Angle.FromDegrees(90f)), Is.True);
        Step(world);
        var lowerAfter = world.Transform.GetWorldPosition(lowerGrid);
        var upperAfter = world.Transform.GetWorldPosition(upperGrid);

        Assert.Multiple(() =>
        {
            Assert.That(lowerAfter, Is.Not.EqualTo(lowerBefore));
            Assert.That(upperAfter, Is.Not.EqualTo(upperBefore));
            Assert.That(world.Transform.GetWorldRotation(lowerGrid).Degrees, Is.EqualTo(90f).Within(0.001f));
            Assert.That(world.Transform.GetWorldRotation(upperGrid).Degrees, Is.EqualTo(90f).Within(0.001f));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(wall));
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.HighGround));
            Assert.That(body.Physics.SupportHeight, Is.EqualTo(1.05f).Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[1]));
        });
    }

    [Test]
    public void SupportRemovalStartsAuthoritativeFall()
    {
        var world = CreateWorld(1);
        var platform = SpawnPlatform(world, 0, new Vector2(0.5f), 0.2f, new Vector2(0.5f), solidVolume: false);
        var body = SpawnMob(world, 0, new Vector2(0.5f), 0.2f);
        GroundAtCurrentHeight(world, body);

        Assert.That(body.Physics.SupportProvider, Is.EqualTo(platform));

        world.Entities.DeleteEntity(platform);
        Step(world);

        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.None));
            Assert.That(body.Physics.SupportProvider, Is.Null);
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Airborne));
            Assert.That(body.Physics.Velocity, Is.LessThan(0f));
            Assert.That(world.ZPhysics.ActiveBodies, Does.Contain(body.Uid));
        });
    }

    [Test]
    public void SameProviderDoesNotBypassAuthoritativeStepDownLimit()
    {
        var world = CreateWorld(1);
        var platform = SpawnPlatform(world, 0, new Vector2(0.5f), 0.2f, new Vector2(0.5f), solidVolume: false);
        var body = SpawnMob(world, 0, new Vector2(0.5f), 0.2f);
        GroundAtCurrentHeight(world, body);
        var originalMap = world.Transform.GetMap(body.Uid);

        var highGround = world.Entities.GetComponent<ZLevelHighGroundComponent>(platform);
        SetField(highGround, nameof(ZLevelHighGroundComponent.Height), -0.2f);
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));

        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(platform));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Airborne),
                "reusing the same provider must not snap through the maximum step-down distance");
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(originalMap));
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void ProjectedSurfaceAndFootContactUseTheSameProjection(float offsetY)
    {
        var world = CreateWorld(2);
        world.ZLevels.SetProjectionOffset(world.Network.Owner, new Vector2(0f, offsetY));
        var provider = SpawnWall(world, 0, new Vector2(0.5f));
        Span<ZLevelSupportPoint> points = stackalloc ZLevelSupportPoint[4];

        Assert.That(world.ZPhysics.TryGetSupportSurfacePoints(
            (provider, world.Entities.GetComponent<ZLevelHighGroundComponent>(provider)),
            points,
            out var count), Is.True);
        Assert.That(count, Is.EqualTo(4));

        for (var i = 0; i < count; i++)
        {
            Assert.That(world.ZLevels.TryProjectAbsolutePosition(
                world.Maps[0],
                points[i].CanonicalPosition,
                points[i].AbsoluteHeight,
                out var projected), Is.True);
            var expected = ZLevelProjection.ProjectSupportPoint(
                points[i].CanonicalPosition,
                points[i].AbsoluteHeight,
                0,
                new Vector2(0f, offsetY));
            Assert.That(projected.X, Is.EqualTo(expected.X).Within(0.0001f));
            Assert.That(projected.Y, Is.EqualTo(expected.Y).Within(0.0001f));
            var unprojected = ZLevelProjection.UnprojectSupportPoint(
                projected,
                points[i].AbsoluteHeight,
                0,
                new Vector2(0f, offsetY));
            Assert.That(unprojected.X, Is.EqualTo(points[i].CanonicalPosition.X).Within(0.0001f));
            Assert.That(unprojected.Y, Is.EqualTo(points[i].CanonicalPosition.Y).Within(0.0001f));
        }
    }

    private static TestWorld CreateWorld(
        int levels,
        IReadOnlyCollection<int>? floorLevels = null,
        int floorLength = 1)
    {
        var simulation = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = simulation.Resolve<IEntityManager>();
        var config = simulation.Resolve<IConfigurationManager>();
        var map = entities.System<SharedMapSystem>();
        var maps = new EntityUid[levels];
        var mapIds = new MapId[levels];
        var floorGrids = new EntityUid?[levels];

        config.SetCVar(CVars.PhysicsZLevelMaxStepUp, 0.25f);
        config.SetCVar(CVars.PhysicsZLevelMaxStepDown, 0.25f);
        config.SetCVar(CVars.PhysicsZLevelGroundSnapDistance, 0.25f);
        config.SetCVar(CVars.PhysicsZLevelVelocityLimit, 50f);
        config.SetCVar(CVars.SleepAllowed, false);

        for (var i = 0; i < levels; i++)
        {
            maps[i] = map.CreateMap(out mapIds[i]);
            if (floorLevels?.Contains(i) == true)
                floorGrids[i] = CreateGridWithTiles(entities, map, mapIds[i], 0, floorLength);
        }

        var zLevels = entities.System<ZLevelSystem>();
        Assert.That(zLevels.TryCreateMapNetwork(maps, out var network), Is.True);

        return new TestWorld(
            entities,
            map,
            entities.System<FixtureSystem>(),
            entities.System<SharedPhysicsSystem>(),
            entities.System<SharedTransformSystem>(),
            zLevels,
            entities.System<ZLevelPhysicsSystem>(),
            entities.System<ZLevelSupportSystem>(),
            maps,
            mapIds,
            floorGrids,
            network!.Value);
    }

    private static EntityUid CreateGridWithTiles(TestWorld world, int level, int startX, int length)
    {
        var grid = CreateGridWithTiles(world.Entities, world.Map, world.MapIds[level], startX, length);
        world.FloorGrids[level] ??= grid;
        return grid;
    }

    private static EntityUid CreateGridWithTiles(
        IEntityManager entities,
        SharedMapSystem map,
        MapId mapId,
        int startX,
        int length)
    {
        var grid = map.CreateGridEntity(mapId);
        for (var x = startX; x < startX + length; x++)
            map.SetTile(grid, new Vector2i(x, 0), new Tile(1));

        return grid.Owner;
    }

    private static EntityUid SpawnWall(TestWorld world, int level, Vector2 worldPosition)
        => SpawnPlatform(world, level, worldPosition, 1.05f, new Vector2(0.5f), solidVolume: true);

    private static EntityUid SpawnWall(TestWorld world, EntityUid parent, Vector2 localPosition)
        => SpawnPlatform(world, parent, localPosition, 1.05f, new Vector2(0.5f), solidVolume: true);

    private static EntityUid SpawnPlatform(
        TestWorld world,
        int level,
        Vector2 worldPosition,
        float height,
        Vector2 supportHalfExtents,
        bool solidVolume,
        Vector2? volumeHalfExtents = null)
    {
        var uid = world.Entities.SpawnEntity(null, new MapCoordinates(worldPosition, world.MapIds[level]));
        InitializePlatform(world, uid, height, supportHalfExtents, solidVolume, volumeHalfExtents);
        return uid;
    }

    private static EntityUid SpawnPlatform(
        TestWorld world,
        EntityUid parent,
        Vector2 localPosition,
        float height,
        Vector2 supportHalfExtents,
        bool solidVolume,
        Vector2? volumeHalfExtents = null)
    {
        var uid = world.Entities.SpawnEntity(null, new EntityCoordinates(parent, localPosition));
        InitializePlatform(world, uid, height, supportHalfExtents, solidVolume, volumeHalfExtents);
        return uid;
    }

    private static void InitializePlatform(
        TestWorld world,
        EntityUid uid,
        float height,
        Vector2 supportHalfExtents,
        bool solidVolume,
        Vector2? volumeHalfExtents)
    {
        var body = world.Entities.AddComponent<PhysicsComponent>(uid);
        world.SharedPhysics.SetBodyType(uid, BodyType.Static, body: body);

        if (solidVolume)
        {
            var volumeShape = new PolygonShape();
            var volume = volumeHalfExtents ?? supportHalfExtents;
            volumeShape.SetAsBox(volume.X, volume.Y);
            world.Fixtures.CreateFixture(uid, VolumeFixture, new Fixture(volumeShape, 1, 1, true), body: body);
        }

        var supportShape = new PolygonShape();
        supportShape.SetAsBox(supportHalfExtents.X, supportHalfExtents.Y);
        world.Fixtures.CreateFixture(
            uid,
            ZLevelHighGroundComponent.DefaultSurfaceFixture,
            new Fixture(supportShape, 1, 1, false),
            body: body);
        world.SharedPhysics.SetCanCollide(uid, true, body: body);

        var highGround = world.Entities.AddComponent<ZLevelHighGroundComponent>(uid);
        SetField(highGround, nameof(ZLevelHighGroundComponent.Height), height);
        SetField(highGround, nameof(ZLevelHighGroundComponent.SolidVolume), solidVolume);
        SetField(highGround, nameof(ZLevelHighGroundComponent.SurfaceFixture), ZLevelHighGroundComponent.DefaultSurfaceFixture);
    }

    private static TestBody SpawnMob(
        TestWorld world,
        int level,
        Vector2 worldPosition,
        float localHeight,
        float radius = 0.35f,
        float zVelocity = 0f,
        bool gravity = true)
    {
        var uid = world.Entities.SpawnEntity(null, new MapCoordinates(worldPosition, world.MapIds[level]));
        return InitializeMob(world, uid, localHeight, radius, zVelocity, gravity);
    }

    private static TestBody SpawnMob(
        TestWorld world,
        EntityUid parent,
        Vector2 localPosition,
        float localHeight,
        float radius = 0.35f,
        float zVelocity = 0f,
        bool gravity = true)
    {
        var uid = world.Entities.SpawnEntity(null, new EntityCoordinates(parent, localPosition));
        return InitializeMob(world, uid, localHeight, radius, zVelocity, gravity);
    }

    private static TestBody InitializeMob(
        TestWorld world,
        EntityUid uid,
        float localHeight,
        float radius,
        float zVelocity,
        bool gravity)
    {
        var body = world.Entities.AddComponent<PhysicsComponent>(uid);
        world.SharedPhysics.SetBodyType(uid, BodyType.KinematicController, body: body);
        world.Fixtures.CreateFixture(
            uid,
            BodyFixture,
            new Fixture(new PhysShapeCircle(radius), 1, 1, true),
            body: body);
        world.SharedPhysics.SetCanCollide(uid, true, body: body);

        var physics = world.Entities.AddComponent<ZLevelPhysicsComponent>(uid);
        SetField(physics, nameof(ZLevelPhysicsComponent.VelocityGravity), gravity);
        world.ZPhysics.SetZPosition((uid, physics), localHeight);
        world.ZPhysics.SetZVelocity((uid, physics), zVelocity);
        world.ZPhysics.RefreshSupport((uid, physics));
        world.ZPhysics.RefreshBody((uid, physics));
        return new TestBody(
            uid,
            body,
            physics,
            world.Entities.GetComponent<ZLevelPresentationComponent>(uid));
    }

    private static void GroundAtCurrentHeight(TestWorld world, TestBody body)
    {
        body.Physics.GroundState = ZLevelGroundState.Grounded;
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        world.ZPhysics.RefreshBody((body.Uid, body.Physics));
    }

    private static void RunUntil(TestWorld world, Func<bool> condition, float seconds)
    {
        var steps = (int) MathF.Round(seconds * 60f);
        for (var i = 0; i < steps; i++)
        {
            Step(world);
            if (condition())
                return;
        }

        Assert.Fail("Timed out waiting for simulated physics condition.");
    }

    private static void RunFor(TestWorld world, float seconds)
    {
        var steps = (int) MathF.Round(seconds * 60f);
        for (var i = 0; i < steps; i++)
            Step(world);
    }

    private static void Step(TestWorld world)
    {
        const float delta = 1f / 60f;
        world.SharedPhysics.Update(delta);
        world.ZPhysics.Update(delta);
    }

    private static float GetAbsoluteZ(TestWorld world, TestBody body)
    {
        var map = world.Transform.GetMap(body.Uid);
        Assert.That(map, Is.Not.Null);
        Assert.That(world.ZLevels.TryGetMapDepth(map!.Value, out var depth), Is.True);
        return ZLevelProjection.GetAbsoluteZ(depth!.Value, body.Presentation.LocalHeight);
    }

    private static void SetField<TComponent, TValue>(TComponent component, string field, TValue value)
    {
        typeof(TComponent)
            .GetField(field)!
            .SetValue(component, value);
    }

    private sealed record TestWorld(
        IEntityManager Entities,
        SharedMapSystem Map,
        FixtureSystem Fixtures,
        SharedPhysicsSystem SharedPhysics,
        SharedTransformSystem Transform,
        ZLevelSystem ZLevels,
        ZLevelPhysicsSystem ZPhysics,
        ZLevelSupportSystem Support,
        EntityUid[] Maps,
        MapId[] MapIds,
        EntityUid?[] FloorGrids,
        Entity<ZLevelMapNetworkComponent> Network);

    private readonly record struct TestBody(
        EntityUid Uid,
        PhysicsComponent Body,
        ZLevelPhysicsComponent Physics,
        ZLevelPresentationComponent Presentation);
}
