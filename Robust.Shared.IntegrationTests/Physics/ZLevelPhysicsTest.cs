using System.Numerics;
using System.Reflection;
using NUnit.Framework;
using Robust.Server.Containers;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.Containers;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Systems;
using Robust.Shared.Reflection;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.Physics;

[TestFixture]
[Parallelizable(ParallelScope.All | ParallelScope.Fixtures)]
[TestOf(typeof(ZLevelPhysicsSystem))]
internal sealed class ZLevelPhysicsTest
{
    [TestCase(1)]
    [TestCase(2)]
    [TestCase(5)]
    [TestCase(10)]
    public void FallsAcrossRequestedNumberOfLevelsAndUsesFirstFloor(int levels)
    {
        var world = CreateWorld(levels + 1, floorLevels: [0]);
        var body = SpawnBody(world, levels, 0.8f, 0f);

        RunFor(world, 4f, 60);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0f).Within(0.001f));
            Assert.That(body.Physics.Velocity, Is.EqualTo(0f).Within(0.001f));
            Assert.That(body.Body.BodyStatus, Is.EqualTo(BodyStatus.OnGround));
        });
    }

    [Test]
    public void MultipleCrossingsAreConsumedInOnePhysicsTick()
    {
        var world = CreateWorld(10, tickRate: 5, velocityLimit: 100f);
        var body = SpawnBody(world, 9, 0.75f, -31f, gravity: false);

        world.ZPhysics.Update(0.2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[3]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.55f).Within(0.001f));
            Assert.That(body.Physics.LastStep.Crossings, Is.EqualTo(6));
            Assert.That(body.Physics.LastStep.RemainingTime, Is.EqualTo(0f).Within(0.00001f));
            Assert.That(body.Physics.LastStep.IterationLimitReached, Is.False);
        });
    }

    [Test]
    public void UpwardCrossingsCanConsumeSeveralLevelsInOneTick()
    {
        var world = CreateWorld(10, tickRate: 5, velocityLimit: 100f);
        var body = SpawnBody(world, 0, 0.25f, 21f, gravity: false);

        world.ZPhysics.Update(0.2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[4]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.45f).Within(0.001f));
            Assert.That(body.Physics.LastStep.Crossings, Is.EqualTo(4));
        });
    }

    [Test]
    public void MapCrossingsPreserveResidualTime()
    {
        var world = CreateWorld(6, tickRate: 5, velocityLimit: 100f);
        var body = SpawnBody(world, 0, 0.25f, 12f, gravity: false);

        world.ZPhysics.Update(0.2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[2]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.65f).Within(0.001f));
            Assert.That(GetAbsoluteZ(world, body), Is.EqualTo(2.65f).Within(0.001f));
        });
    }

    [TestCase(5)]
    [TestCase(10)]
    [TestCase(30)]
    [TestCase(60)]
    public void FixedRatesProduceTheSameConstantVelocityResult(int tickRate)
    {
        var world = CreateWorld(8, tickRate: tickRate, velocityLimit: 100f);
        var body = SpawnBody(world, 0, 0.2f, 5f, gravity: false);

        RunFor(world, 1f, tickRate);

        Assert.That(GetAbsoluteZ(world, body), Is.EqualTo(5.2f).Within(0.001f));
    }

    [Test]
    public void ExcessiveInitialVelocityIsClampedBeforeIntegration()
    {
        var world = CreateWorld(20, tickRate: 5, velocityLimit: 20f);
        var body = SpawnBody(world, 10, 0.5f, -1_000_000f, gravity: false);

        world.ZPhysics.Update(0.2f);

        Assert.Multiple(() =>
        {
            Assert.That(body.Physics.Velocity, Is.EqualTo(-20f));
            Assert.That(GetAbsoluteZ(world, body), Is.EqualTo(6.5f).Within(0.001f));
            Assert.That(body.Physics.LastStep.Crossings, Is.EqualTo(4));
            Assert.That(body.Physics.LastStep.IterationLimitReached, Is.False);
        });
    }

    [Test]
    public void GravityStopsAtTerminalVelocity()
    {
        var world = CreateWorld(50, velocityLimit: 20f);
        var body = SpawnBody(world, 49, 0.5f, 0f);

        RunFor(world, 2.5f, 60);

        Assert.That(body.Physics.Velocity, Is.EqualTo(-20f).Within(0.001f));
    }

    [Test]
    public void FirstValidFloorWinsBeforeTheNextMapBoundary()
    {
        var world = CreateWorld(3, floorLevels: [0, 1], tickRate: 5, velocityLimit: 100f, recordEvents: true);
        var body = SpawnBody(world, 2, 0.1f, -20f, gravity: false);

        world.ZPhysics.Update(0.2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[1]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0f).Within(0.001f));
            Assert.That(world.Events!.Events, Is.EqualTo(new[] { "transition:-1", "impact:Floor", "landing:Floor" }));
        });
    }

    [Test]
    public void BlockedUpwardSpaceRaisesCeilingBeforeAnyTransition()
    {
        var world = CreateWorld(2, floorLevels: [1], velocityLimit: 100f, recordEvents: true);
        var body = SpawnBody(world, 0, 0.95f, 10f, gravity: false);

        world.ZPhysics.Update(1f / 60f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(1f).Within(0.001f));
            Assert.That(world.Events!.Events, Is.EqualTo(new[] { "impact:Ceiling", "landing:Ceiling" }));
        });
    }

    [TestCase(-1, ZLevelImpactSurface.Floor, 0f)]
    [TestCase(1, ZLevelImpactSurface.Ceiling, 1f)]
    public void MissingDestinationMapsResolveAsBlockedBoundaries(
        int direction,
        ZLevelImpactSurface surface,
        float expectedHeight)
    {
        var world = CreateWorld(1, velocityLimit: 100f, recordEvents: true);
        var body = SpawnBody(
            world,
            0,
            direction < 0 ? 0.05f : 0.95f,
            direction * 10f,
            gravity: false);

        world.ZPhysics.Update(1f / 60f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(expectedHeight).Within(0.001f));
            Assert.That(world.Events!.Events, Is.EqualTo(new[]
            {
                $"boundary:{direction}",
                $"impact:{surface}",
                $"landing:{surface}",
            }));
        });
    }

    [Test]
    public void TransitionAndLandingAreRaisedExactlyOnceAfterRepeatedUpdates()
    {
        var world = CreateWorld(2, floorLevels: [0], tickRate: 5, velocityLimit: 100f, recordEvents: true);
        var body = SpawnBody(world, 1, 0.1f, -10f);

        world.ZPhysics.Update(0.2f);
        for (var i = 0; i < 10; i++)
            world.ZPhysics.Update(0.2f);

        Assert.Multiple(() =>
        {
            Assert.That(world.Events!.Events.Count(e => e.StartsWith("transition")), Is.EqualTo(1));
            Assert.That(world.Events.Events.Count(e => e.StartsWith("landing")), Is.EqualTo(1));
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
        });
    }

    [Test]
    public void ArbitraryMapReparentPreservesAbsoluteZAndWorldXy()
    {
        var world = CreateWorld(2);
        var body = SpawnBody(world, 0, 0.75f, 0f, gravity: false);
        var position = world.Transform.GetWorldPosition(body.Uid);

        Assert.That(world.ZLevels.TryMoveEntityToMapOffset(body.Uid, 1), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(GetAbsoluteZ(world, body), Is.EqualTo(0.75f).Within(0.001f));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(-0.25f).Within(0.001f));
            Assert.That(world.Transform.GetWorldPosition(body.Uid), Is.EqualTo(position));
        });
    }

    [Test]
    public void LinkingZMapsDoesNotWakeOrReparentContainedBodies()
    {
        var simulation = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = simulation.Resolve<IEntityManager>();
        var map = entities.System<SharedMapSystem>();
        var physics = entities.System<SharedPhysicsSystem>();
        var containers = entities.System<ContainerSystem>();
        var zLevels = entities.System<ZLevelSystem>();
        var zPhysics = entities.System<ZLevelPhysicsSystem>();
        var currentMap = map.CreateMap(out var currentMapId);
        var lowerMap = map.CreateMap(out _);
        var grid = map.CreateGridEntity(currentMapId);
        map.SetTile(grid, Vector2i.Zero, new Tile(1));
        var coordinates = new EntityCoordinates(grid.Owner, new Vector2(0.5f));

        var owner = entities.SpawnEntity(null, coordinates);
        var ownerBody = entities.AddComponent<PhysicsComponent>(owner);
        physics.SetBodyType(owner, BodyType.Dynamic, body: ownerBody);
        var ownerVertical = entities.AddComponent<ZLevelPhysicsComponent>(owner);

        var contained = entities.SpawnEntity(null, coordinates);
        var containedBody = entities.AddComponent<PhysicsComponent>(contained);
        physics.SetBodyType(contained, BodyType.Dynamic, body: containedBody);
        var containedVertical = entities.AddComponent<ZLevelPhysicsComponent>(contained);
        zPhysics.SetZPosition((contained, containedVertical), 0.25f);
        var slot = containers.EnsureContainer<ContainerSlot>(owner, "contained-z-body");
        Assert.That(containers.Insert(contained, slot), Is.True);

        var nested = entities.SpawnEntity(null, MapCoordinates.Nullspace);
        var nestedBody = entities.AddComponent<PhysicsComponent>(nested);
        physics.SetBodyType(nested, BodyType.Dynamic, body: nestedBody);
        var nestedVertical = entities.AddComponent<ZLevelPhysicsComponent>(nested);
        zPhysics.SetZPosition((nested, nestedVertical), -0.4f);
        var nestedSlot = containers.EnsureContainer<ContainerSlot>(contained, "nested-z-body");
        Assert.That(containers.Insert(nested, nestedSlot), Is.True);

        var containedParent = entities.GetComponent<TransformComponent>(contained).ParentUid;
        var nestedParent = entities.GetComponent<TransformComponent>(nested).ParentUid;
        Assert.That(zLevels.TryCreateMapNetwork([lowerMap, currentMap], out _), Is.True);

        for (var i = 0; i < 30; i++)
            zPhysics.Update(1f / 60f);

        Assert.Multiple(() =>
        {
            Assert.That(slot.ContainedEntity, Is.EqualTo(contained));
            Assert.That(nestedSlot.ContainedEntity, Is.EqualTo(nested));
            Assert.That(entities.GetComponent<TransformComponent>(contained).ParentUid, Is.EqualTo(containedParent));
            Assert.That(entities.GetComponent<TransformComponent>(nested).ParentUid, Is.EqualTo(nestedParent));
            Assert.That(containers.IsEntityInContainer(contained), Is.True);
            Assert.That(containers.IsEntityInContainer(nested), Is.True);
            Assert.That(entities.GetComponent<ZLevelPresentationComponent>(contained).LocalHeight,
                Is.EqualTo(0.25f).Within(0.0001f));
            Assert.That(entities.GetComponent<ZLevelPresentationComponent>(nested).LocalHeight,
                Is.EqualTo(-0.4f).Within(0.0001f));
            Assert.That(containedVertical.Velocity, Is.Zero);
            Assert.That(nestedVertical.Velocity, Is.Zero);
            Assert.That(zPhysics.ActiveBodies, Does.Not.Contain(contained));
            Assert.That(zPhysics.ActiveBodies, Does.Not.Contain(nested));
            Assert.That(zPhysics.ActiveBodies, Does.Not.Contain(owner),
                "a body already supported by the existing floor must not be woken just because its map was linked");
            Assert.That(ownerVertical.Velocity, Is.Zero);
        });
    }

    [Test]
    public void ReleasedContainerBodyStartsAtCarriersCurrentHeight()
    {
        var world = CreateWorld(2, floorLevels: [0]);
        var containers = world.Entities.System<ContainerSystem>();
        var carrier = SpawnBody(world, 0, 0f, 0f, gravity: false);
        var item = SpawnBody(world, 0, -0.4f, 0f, gravity: true);
        var slot = containers.EnsureContainer<ContainerSlot>(carrier.Uid, "held");

        Assert.That(containers.Insert(item.Uid, slot), Is.True);
        Assert.That(item.Presentation.LocalHeight, Is.Zero.Within(0.0001f));

        world.ZPhysics.SetZPosition((carrier.Uid, carrier.Physics), 0.75f);
        Assert.That(containers.Remove(item.Uid, slot), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(item.Presentation.LocalHeight, Is.EqualTo(0.75f).Within(0.0001f));
            Assert.That(containers.IsEntityInContainer(item.Uid), Is.False);
            Assert.That(world.ZPhysics.ActiveBodies, Does.Contain(item.Uid));
            Assert.That(world.Entities.GetComponent<PhysicsComponent>(item.Uid).BodyType,
                Is.EqualTo(BodyType.Dynamic));
        });
    }

    [Test]
    public void RapidContainerCycleDoesNotReusePreviousFallVelocity()
    {
        var world = CreateWorld(2);
        var containers = world.Entities.System<ContainerSystem>();
        var carrier = SpawnBody(world, 1, 0.5f, 0f, gravity: false);
        var item = SpawnBody(world, 1, 0.5f, -6f, gravity: true);
        var slot = containers.EnsureContainer<ContainerSlot>(carrier.Uid, "held");
        const float step = 1f / 60f;

        Assert.That(containers.Insert(item.Uid, slot), Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(item.Physics.Velocity, Is.Zero, "pickup must cancel the old fall velocity");
            Assert.That(item.Presentation.LocalHeight, Is.EqualTo(0.5f).Within(0.0001f));
        });

        Assert.That(containers.Remove(item.Uid, slot), Is.True);
        Assert.That(item.Physics.Velocity, Is.Zero, "a fresh drop must start without inherited velocity");
        world.ZPhysics.Update(step);
        var firstDropVelocity = item.Physics.Velocity;
        Assert.That(firstDropVelocity, Is.LessThan(0f));

        Assert.That(containers.Insert(item.Uid, slot), Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(item.Physics.Velocity, Is.Zero, "a rapid pickup must cancel the in-progress fall");
            Assert.That(item.Presentation.LocalHeight, Is.EqualTo(0.5f).Within(0.0001f));
        });

        Assert.That(containers.Remove(item.Uid, slot), Is.True);
        Assert.That(item.Physics.Velocity, Is.Zero);
        world.ZPhysics.Update(step);

        Assert.That(item.Physics.Velocity, Is.EqualTo(firstDropVelocity).Within(0.0001f),
            "the second drop must accelerate from rest at the same rate as the first");
    }

    [Test]
    public void FullyManualPresentationBodyNeverFallsOrSnaps()
    {
        var world = CreateWorld(2, floorLevels: [0], tickRate: 5, velocityLimit: 100f, recordEvents: true);
        var body = SpawnBody(world, 0, 0.4f, 0f, gravity: false);
        world.ZPhysics.SetGroundSupport((body.Uid, body.Physics), fallable: false, autoStep: false);

        RunFor(world, 2f, 5);
        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(0.4f).Within(0.0001f));
            Assert.That(body.Physics.Velocity, Is.Zero);
            Assert.That(world.ZPhysics.ActiveBodies, Does.Not.Contain(body.Uid));
            Assert.That(world.Events!.Events, Is.Empty);
        });

        world.ZPhysics.SetZPosition((body.Uid, body.Physics), -0.25f);
        world.ZPhysics.SetZVelocity((body.Uid, body.Physics), -10f);
        RunFor(world, 2f, 5);

        Assert.Multiple(() =>
        {
            Assert.That(world.Transform.GetMap(body.Uid), Is.EqualTo(world.Maps[0]));
            Assert.That(body.Presentation.LocalHeight, Is.EqualTo(-0.25f).Within(0.0001f));
            Assert.That(body.Physics.Velocity, Is.EqualTo(-10f));
            Assert.That(world.ZPhysics.ActiveBodies, Does.Not.Contain(body.Uid));
            Assert.That(world.Events!.Events, Is.Empty);
        });
    }

    private static TestWorld CreateWorld(
        int levels,
        IReadOnlyCollection<int>? floorLevels = null,
        int tickRate = 60,
        float velocityLimit = 20f,
        bool recordEvents = false)
    {
        var builder = RobustServerSimulation.NewSimulation();
        if (recordEvents)
            builder.RegisterEntitySystems(factory => factory.LoadExtraSystemType<ZPhysicsEventRecorderSystem>());

        var simulation = builder.InitializeInstance();
        var entities = simulation.Resolve<IEntityManager>();
        var config = simulation.Resolve<IConfigurationManager>();
        var map = entities.System<SharedMapSystem>();
        var maps = new EntityUid[levels];
        var mapIds = new MapId[levels];

        config.SetCVar(CVars.PhysicsZLevelTickRate, tickRate);
        config.SetCVar(CVars.PhysicsZLevelVelocityLimit, velocityLimit);

        for (var i = 0; i < levels; i++)
        {
            maps[i] = map.CreateMap(out mapIds[i]);
            if (floorLevels?.Contains(i) != true)
                continue;

            var grid = map.CreateGridEntity(mapIds[i]);
            map.SetTile(grid, Vector2i.Zero, new Tile(1));
        }

        var zLevels = entities.System<ZLevelSystem>();
        Assert.That(zLevels.TryCreateMapNetwork(maps, out _), Is.True);

        return new TestWorld(
            simulation,
            entities,
            map,
            entities.System<SharedPhysicsSystem>(),
            entities.System<SharedTransformSystem>(),
            zLevels,
            entities.System<ZLevelPhysicsSystem>(),
            maps,
            mapIds,
            recordEvents ? entities.System<ZPhysicsEventRecorderSystem>() : null);
    }

    private static TestBody SpawnBody(
        TestWorld world,
        int level,
        float localHeight,
        float velocity,
        bool gravity = true)
    {
        var uid = world.Entities.SpawnEntity(
            null,
            new MapCoordinates(new Vector2(0.5f), world.MapIds[level]));
        var body = world.Entities.AddComponent<PhysicsComponent>(uid);
        world.SharedPhysics.SetBodyType(uid, BodyType.Dynamic, body: body);
        var vertical = world.Entities.AddComponent<ZLevelPhysicsComponent>(uid);
        SetVelocityGravity(vertical, gravity);
        world.ZPhysics.SetZPosition((uid, vertical), localHeight);
        world.ZPhysics.SetZVelocity((uid, vertical), velocity);
        world.ZPhysics.RefreshGround((uid, vertical), true);
        world.ZPhysics.RefreshBody((uid, vertical));
        var presentation = world.Entities.GetComponent<ZLevelPresentationComponent>(uid);
        return new TestBody(uid, body, vertical, presentation);
    }

    private static void RunFor(TestWorld world, float seconds, int outerRate)
    {
        var steps = (int) MathF.Round(seconds * outerRate);
        for (var i = 0; i < steps; i++)
            world.ZPhysics.Update(1f / outerRate);
    }

    private static float GetAbsoluteZ(TestWorld world, TestBody body)
    {
        var map = world.Transform.GetMap(body.Uid);
        Assert.That(map, Is.Not.Null);
        Assert.That(world.ZLevels.TryGetMapDepth(map!.Value, out var depth), Is.True);
        return ZLevelProjection.GetAbsoluteZ(depth!.Value, body.Presentation.LocalHeight);
    }

    private static void SetVelocityGravity(ZLevelPhysicsComponent component, bool value)
    {
        typeof(ZLevelPhysicsComponent)
            .GetField(nameof(ZLevelPhysicsComponent.VelocityGravity))!
            .SetValue(component, value);
    }

    private sealed record TestWorld(
        ISimulation Simulation,
        IEntityManager Entities,
        SharedMapSystem Map,
        SharedPhysicsSystem SharedPhysics,
        SharedTransformSystem Transform,
        ZLevelSystem ZLevels,
        ZLevelPhysicsSystem ZPhysics,
        EntityUid[] Maps,
        MapId[] MapIds,
        ZPhysicsEventRecorderSystem? Events);

    private readonly record struct TestBody(
        EntityUid Uid,
        PhysicsComponent Body,
        ZLevelPhysicsComponent Physics,
        ZLevelPresentationComponent Presentation);
}

[Reflect(false)]
internal sealed partial class ZPhysicsEventRecorderSystem : EntitySystem
{
    public readonly List<string> Events = [];

    [SubscribeLocalEvent]
    private void OnTransition(Entity<ZLevelPhysicsComponent> entity, ref ZLevelMapMoveEvent args)
        => Events.Add($"transition:{args.Offset}");

    [SubscribeLocalEvent]
    private void OnImpact(Entity<ZLevelPhysicsComponent> entity, ref ZLevelImpactEvent args)
        => Events.Add($"impact:{args.Surface}");

    [SubscribeLocalEvent]
    private void OnLanding(Entity<ZLevelPhysicsComponent> entity, ref ZLevelLandingEvent args)
        => Events.Add($"landing:{args.Surface}");

    [SubscribeLocalEvent]
    private void OnBoundary(Entity<ZLevelPhysicsComponent> entity, ref ZLevelBoundaryEvent args)
        => Events.Add($"boundary:{args.Direction}");
}
