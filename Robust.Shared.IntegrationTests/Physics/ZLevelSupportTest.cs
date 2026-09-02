using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Reflection;
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
using Robust.Shared.Physics.Events;
using Robust.Shared.Physics.Systems;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.Physics;

[TestFixture]
[Parallelizable(ParallelScope.All | ParallelScope.Fixtures)]
[TestOf(typeof(ZLevelPhysicsSystem))]
internal sealed class ZLevelSupportTest
{
    [Test]
    public void FootprintFindsSupportWhenBodyCenterIsOutsideSurface()
    {
        var world = CreateWorld(1);
        var provider = SpawnProvider(world, 0, new Vector2(0.5f), 0.1f, new Vector2(0.5f));
        var body = SpawnBody(world, 0, new Vector2(1.18f, 0.5f), 0.1f, 0.25f);

        GroundAtCurrentHeight(world, body);

        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(provider));
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.HighGround));
            Assert.That(body.Physics.SupportPoint.X, Is.EqualTo(1f).Within(0.001f));
        });
    }

    [Test]
    public void HighestSupportWinsDeterministicallyAndCurrentSupportHasHysteresis()
    {
        var world = CreateWorld(1);
        var lower = SpawnProvider(world, 0, new Vector2(0.5f), 0.1f, new Vector2(0.5f));
        var body = SpawnBody(world, 0, new Vector2(0.5f), 0.1f, 0.2f);
        GroundAtCurrentHeight(world, body);
        Assert.That(body.Physics.SupportProvider, Is.EqualTo(lower));

        var nearHigher = SpawnProvider(world, 0, new Vector2(0.5f), 0.14f, new Vector2(0.5f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.That(body.Physics.SupportProvider, Is.EqualTo(lower), "a 0.04 rise is inside the 0.05 support hysteresis");

        world.ZPhysics.SetHeightCurve(
            (nearHigher, world.Entities.GetComponent<ZLevelHighGroundComponent>(nearHigher)),
            [0.16f, 0.16f]);
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.That(body.Physics.SupportProvider, Is.EqualTo(nearHigher));

        world.ZPhysics.SetHeightCurve(
            (nearHigher, world.Entities.GetComponent<ZLevelHighGroundComponent>(nearHigher)),
            [0.1f, 0.1f]);
        var tieBody = SpawnBody(world, 0, new Vector2(0.5f), 0.1f, 0.1f);
        GroundAtCurrentHeight(world, tieBody);
        var expectedTieWinner = lower.CompareTo(nearHigher) < 0 ? lower : nearHigher;
        Assert.That(tieBody.Physics.SupportProvider, Is.EqualTo(expectedTieWinner));
    }

    [Test]
    public void StepUpSnapDownAndLargeDropUseConfiguredThresholds()
    {
        var world = CreateWorld(1, floor: true);
        var body = SpawnBody(world, 0, new Vector2(0.5f), 0f, 0.15f);
        GroundAtCurrentHeight(world, body);

        var smallRise = SpawnProvider(world, 0, new Vector2(1.5f, 0.5f), 0.2f, new Vector2(0.5f));
        world.Transform.SetWorldPosition(body.Uid, new Vector2(1.5f, 0.5f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(smallRise));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });

        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportSurface, Is.EqualTo(ZLevelSupportSurface.Tile));
            Assert.That(body.Presentation.LocalHeight, Is.Zero.Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });

        var excessiveRise = SpawnProvider(world, 0, new Vector2(2.5f, 0.5f), 0.4f, new Vector2(0.5f));
        world.Transform.SetWorldPosition(body.Uid, new Vector2(2.5f, 0.5f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.That(body.Physics.SupportProvider, Is.Not.EqualTo(excessiveRise));
        Assert.That(body.Physics.LastSupportCandidates.Any(candidate =>
            candidate.Provider == excessiveRise && candidate.Rejection == ZLevelSupportRejection.AboveStepLimit));

        world.ZPhysics.SetZPosition((body.Uid, body.Physics), 0.4f);
        body.Physics.GroundState = ZLevelGroundState.Grounded;
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.That(body.Physics.SupportProvider, Is.EqualTo(excessiveRise));

        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Airborne));
            Assert.That(body.Physics.ReconciliationState, Is.EqualTo(ZLevelReconciliationState.AuthoritativeFall));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.4f).Within(0.001f));
        });
    }

    [Test]
    public void RampHeightIsContinuousAndDeterministicAtEdgesAndCorners()
    {
        var world = CreateWorld(1);
        var ramp = SpawnProvider(
            world,
            0,
            new Vector2(0.5f),
            [0f, 0.5f, 1f],
            new Vector2(0.5f),
            solidVolume: false);
        var body = SpawnBody(world, 0, new Vector2(0.5f, 0.75f), 0.25f, 0.08f);
        GroundAtCurrentHeight(world, body);

        Assert.That(body.Physics.SupportHeight, Is.EqualTo(0.25f).Within(0.001f));

        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f, 0.749f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        var first = body.Physics.SupportHeight;
        for (var i = 0; i < 20; i++)
            world.ZPhysics.RefreshSupport((body.Uid, body.Physics));

        Assert.Multiple(() =>
        {
            Assert.That(first, Is.EqualTo(0.251f).Within(0.001f));
            Assert.That(body.Physics.SupportHeight, Is.EqualTo(first).Within(0.00001f));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(ramp));
        });

        var corner = SpawnProvider(
            world,
            0,
            new Vector2(2.5f, 0.5f),
            [0f, 1f],
            new Vector2(0.5f),
            solidVolume: false,
            corner: true);
        world.ZPhysics.SetZPosition((body.Uid, body.Physics), 0.5f);
        body.Physics.GroundState = ZLevelGroundState.Grounded;
        world.Transform.SetWorldPosition(body.Uid, new Vector2(2.5f, 0.5f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(corner));
            Assert.That(body.Physics.SupportHeight, Is.EqualTo(0.5f).Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });
    }

    [Test]
    public void RampCrossesMapPlanesOnceAndRetainsConfirmedSupport()
    {
        var world = CreateWorld(2);
        var ramp = SpawnProvider(
            world,
            0,
            new Vector2(0.5f),
            [0.1f, 1.05f],
            new Vector2(0.5f),
            solidVolume: false);
        var body = SpawnBody(world, 0, new Vector2(0.5f, 0.75f), 0.3375f, 0.08f);
        GroundAtCurrentHeight(world, body);

        // Climb to the high edge. The surface deliberately extends through the plane hysteresis band.
        for (var y = 0.7f; y >= 0f; y -= 0.05f)
        {
            world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f, y));
            world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        }
        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f, 0f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[1]));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(ramp));
            Assert.That(body.Physics.SupportHeight, Is.EqualTo(1.05f).Within(0.001f));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.05f).Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
            Assert.That(body.Physics.ReconciliationState, Is.EqualTo(ZLevelReconciliationState.Confirmed));
        });

        // Small reversals inside the boundary band do not bounce between map parents.
        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f, 0.08f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[1]));

        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f, 0.12f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));

        world.Transform.SetWorldPosition(body.Uid, new Vector2(0.5f, 0.08f));
        world.ZPhysics.RefreshSupport((body.Uid, body.Physics));
        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(ramp));
            Assert.That(body.Physics.ReconciliationState, Is.EqualTo(ZLevelReconciliationState.Confirmed));
        });
    }

    [Test]
    public void WallTopLandsButWallVolumeBlocksEntry()
    {
        var world = CreateWorld(2);
        var wall = SpawnProvider(world, 0, new Vector2(0.5f), 1.05f, new Vector2(0.5f));
        var falling = SpawnBody(world, 0, new Vector2(0.5f), 1.5f, 0.2f, velocity: -1f);

        RunFor(world, 1f);
        Assert.Multiple(() =>
        {
            Assert.That(falling.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
            Assert.That(falling.Physics.SupportProvider, Is.EqualTo(wall));
            Assert.That(falling.Presentation.LocalHeight, Is.EqualTo(1.05f).Within(0.001f));
        });

        var crossing = SpawnBody(world, 1, new Vector2(0.5f), 0f, 0.2f);
        Assert.That(world.ZPhysics.HasTransitionObstruction(crossing.Uid, -1), Is.True);
        Assert.That(world.ZPhysics.TryMoveDown(crossing.Uid), Is.False);
        Assert.That(world.Transform.GetMap(crossing.Uid), Is.EqualTo(world.Maps[1]));

        var bodyFixture = world.Entities.GetComponent<FixturesComponent>(falling.Uid).Fixtures[BodyFixture];
        var wallFixture = world.Entities.GetComponent<FixturesComponent>(wall).Fixtures[SupportFixture];
        var wallBody = world.Entities.GetComponent<PhysicsComponent>(wall);
        var contact = new PreventCollideEvent(
            falling.Uid,
            wall,
            falling.Body,
            wallBody,
            bodyFixture,
            wallFixture);
        world.Entities.EventBus.RaiseLocalEvent(falling.Uid, ref contact, true);
        Assert.That(contact.Cancelled, Is.True, "the walkable top must not collide as vertical wall volume");

        world.ZPhysics.SetZPosition((falling.Uid, falling.Physics), 0f);
        contact = new PreventCollideEvent(
            falling.Uid,
            wall,
            falling.Body,
            wallBody,
            bodyFixture,
            wallFixture);
        world.Entities.EventBus.RaiseLocalEvent(falling.Uid, ref contact, true);
        Assert.That(contact.Cancelled, Is.False, "the volume beneath the top must remain solid");
    }

    [Test]
    public void DestinationClearanceUsesBodyAndFixtureShapesAtFootprintEdge()
    {
        var world = CreateWorld(2);
        SpawnObstacle(world, 1, new Vector2(0.5f), new Vector2(0.5f));
        var body = SpawnBody(world, 0, new Vector2(1.18f, 0.5f), 0.95f, 0.25f);

        Assert.That(world.ZPhysics.HasTransitionObstruction(body.Uid, 1), Is.True,
            "the overlapping edge must block even though the body centre is outside the fixture");

        world.Transform.SetWorldPosition(body.Uid, new Vector2(1.4f, 0.5f));
        Assert.That(world.ZPhysics.HasTransitionObstruction(body.Uid, 1), Is.False);
    }

    [Test]
    public void MovingAndRotatingGridKeepsConfirmedContactAttached()
    {
        var world = CreateWorld(1);
        var grid = world.Map.CreateGridEntity(world.MapIds[0]);
        var provider = SpawnProvider(world, grid.Owner, new Vector2(0.5f), 0.2f, new Vector2(0.5f));
        var body = SpawnBody(world, grid.Owner, new Vector2(0.5f), 0.2f, 0.15f);
        GroundAtCurrentHeight(world, body);

        world.Transform.SetWorldPosition(grid.Owner, new Vector2(3f, -2f));
        world.Transform.SetWorldRotation(grid.Owner, Angle.FromDegrees(90f));

        var expected = world.Transform.GetWorldPosition(body.Uid);
        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.SupportProvider, Is.EqualTo(provider));
            Assert.That(body.Physics.SupportPoint.X, Is.EqualTo(expected.X).Within(0.001f));
            Assert.That(body.Physics.SupportPoint.Y, Is.EqualTo(expected.Y).Within(0.001f));
            Assert.That(body.Physics.SupportHeight, Is.EqualTo(0.2f).Within(0.001f));
            Assert.That(body.Physics.GroundState, Is.EqualTo(ZLevelGroundState.Grounded));
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void ProjectedSurfaceAndFootContactUseTheSameProjection(float offsetY)
    {
        var world = CreateWorld(2);
        world.ZLevels.SetProjectionOffset(world.Network.Owner, new Vector2(0f, offsetY));
        var provider = SpawnProvider(world, 0, new Vector2(0.5f), 1.05f, new Vector2(0.5f));
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

    private const string BodyFixture = "body";
    private const string SupportFixture = "support";

    private static TestWorld CreateWorld(int levels, bool floor = false)
    {
        var simulation = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = simulation.Resolve<IEntityManager>();
        var config = simulation.Resolve<IConfigurationManager>();
        var map = entities.System<SharedMapSystem>();
        var maps = new EntityUid[levels];
        var mapIds = new MapId[levels];
        config.SetCVar(CVars.PhysicsZLevelMaxStepUp, 0.25f);
        config.SetCVar(CVars.PhysicsZLevelMaxStepDown, 0.25f);
        config.SetCVar(CVars.PhysicsZLevelGroundSnapDistance, 0.25f);
        config.SetCVar(CVars.PhysicsZLevelSupportHysteresis, 0.05f);

        for (var i = 0; i < levels; i++)
        {
            maps[i] = map.CreateMap(out mapIds[i]);
            if (floor && i == 0)
            {
                var grid = map.CreateGridEntity(mapIds[i]);
                for (var x = 0; x < 4; x++)
                    map.SetTile(grid, new Vector2i(x, 0), new Tile(1));
            }
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
            maps,
            mapIds,
            network!.Value);
    }

    private static EntityUid SpawnProvider(
        TestWorld world,
        int level,
        Vector2 worldPosition,
        float height,
        Vector2 halfExtents)
        => SpawnProvider(world, level, worldPosition, [height, height], halfExtents);

    private static EntityUid SpawnProvider(
        TestWorld world,
        int level,
        Vector2 worldPosition,
        IReadOnlyList<float> curve,
        Vector2 halfExtents,
        bool solidVolume = true,
        bool corner = false)
        => SpawnProvider(
            world,
            world.Maps[level],
            worldPosition,
            curve,
            halfExtents,
            solidVolume,
            corner);

    private static EntityUid SpawnProvider(
        TestWorld world,
        EntityUid parent,
        Vector2 localPosition,
        float height,
        Vector2 halfExtents)
        => SpawnProvider(world, parent, localPosition, [height, height], halfExtents);

    private static EntityUid SpawnProvider(
        TestWorld world,
        EntityUid parent,
        Vector2 localPosition,
        IReadOnlyList<float> curve,
        Vector2 halfExtents,
        bool solidVolume = true,
        bool corner = false)
    {
        var uid = world.Entities.SpawnEntity(null, new EntityCoordinates(parent, localPosition));
        var body = world.Entities.AddComponent<PhysicsComponent>(uid);
        world.SharedPhysics.SetBodyType(uid, BodyType.Static, body: body);
        var shape = new PolygonShape();
        shape.SetAsBox(halfExtents.X, halfExtents.Y);
        world.Fixtures.CreateFixture(uid, SupportFixture, new Fixture(shape, 1, 1, solidVolume), body: body);
        world.SharedPhysics.SetCanCollide(uid, true, body: body);
        var highGround = world.Entities.AddComponent<ZLevelHighGroundComponent>(uid);
        world.ZPhysics.SetHeightCurve((uid, highGround), curve);
        SetField(highGround, nameof(ZLevelHighGroundComponent.SolidVolume), solidVolume);
        SetField(highGround, nameof(ZLevelHighGroundComponent.Corner), corner);
        return uid;
    }

    private static EntityUid SpawnObstacle(
        TestWorld world,
        int level,
        Vector2 worldPosition,
        Vector2 halfExtents)
    {
        var uid = world.Entities.SpawnEntity(
            null,
            new MapCoordinates(worldPosition, world.MapIds[level]));
        var body = world.Entities.AddComponent<PhysicsComponent>(uid);
        world.SharedPhysics.SetBodyType(uid, BodyType.Static, body: body);
        var shape = new PolygonShape();
        shape.SetAsBox(halfExtents.X, halfExtents.Y);
        world.Fixtures.CreateFixture(uid, SupportFixture, new Fixture(shape, 1, 1, true), body: body);
        world.SharedPhysics.SetCanCollide(uid, true, body: body);
        return uid;
    }

    private static TestBody SpawnBody(
        TestWorld world,
        int level,
        Vector2 worldPosition,
        float localHeight,
        float radius,
        float velocity = 0f)
        => SpawnBody(world, world.Maps[level], worldPosition, localHeight, radius, velocity);

    private static TestBody SpawnBody(
        TestWorld world,
        EntityUid parent,
        Vector2 localPosition,
        float localHeight,
        float radius,
        float velocity = 0f)
    {
        var uid = world.Entities.SpawnEntity(null, new EntityCoordinates(parent, localPosition));
        var body = world.Entities.AddComponent<PhysicsComponent>(uid);
        world.SharedPhysics.SetBodyType(uid, BodyType.Dynamic, body: body);
        world.Fixtures.CreateFixture(
            uid,
            BodyFixture,
            new Fixture(new PhysShapeCircle(radius), 1, 1, true),
            body: body);
        world.SharedPhysics.SetCanCollide(uid, true, body: body);
        var physics = world.Entities.AddComponent<ZLevelPhysicsComponent>(uid);
        world.ZPhysics.SetZPosition((uid, physics), localHeight);
        world.ZPhysics.SetZVelocity((uid, physics), velocity);
        world.ZPhysics.RefreshSupport((uid, physics));
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
    }

    private static void RunFor(TestWorld world, float seconds)
    {
        for (var i = 0; i < (int) (seconds * 60f); i++)
            world.ZPhysics.Update(1f / 60f);
    }

    private static void SetField<T>(ZLevelHighGroundComponent component, string field, T value)
        => typeof(ZLevelHighGroundComponent).GetField(field, BindingFlags.Instance | BindingFlags.Public)!
            .SetValue(component, value);

    private sealed record TestWorld(
        IEntityManager Entities,
        SharedMapSystem Map,
        FixtureSystem Fixtures,
        SharedPhysicsSystem SharedPhysics,
        SharedTransformSystem Transform,
        ZLevelSystem ZLevels,
        ZLevelPhysicsSystem ZPhysics,
        EntityUid[] Maps,
        MapId[] MapIds,
        Entity<ZLevelMapNetworkComponent> Network);

    private readonly record struct TestBody(
        EntityUid Uid,
        PhysicsComponent Body,
        ZLevelPhysicsComponent Physics,
        ZLevelPresentationComponent Presentation);
}
