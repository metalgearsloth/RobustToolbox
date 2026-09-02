using System.Collections.Generic;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared;
using Robust.Shared.Configuration;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Components;
using Robust.Shared.Physics.Systems;
using Robust.UnitTesting.Server;

namespace Robust.UnitTesting.Shared.Map;

[Parallelizable(ParallelScope.All | ParallelScope.Fixtures)]
[TestFixture]
internal sealed class ZLevelTests
{
    [Test]
    public void MapNetworkQueriesAndInsertionStayOrdered()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entManager = sim.Resolve<IEntityManager>();
        var zLevels = entManager.System<ZLevelSystem>();

        var bottom = sim.CreateMap().Uid;
        var top = sim.CreateMap().Uid;
        var middle = sim.CreateMap().Uid;
        var network = zLevels.CreateMapNetwork();

        Assert.That(zLevels.TryAddMaps(network, new Dictionary<EntityUid, int>
        {
            [bottom] = -10,
            [top] = 20,
        }), Is.True);
        Assert.That(zLevels.TryInsertMap(network, 1, middle), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(zLevels.HasLinearDepths(network), Is.True);
            Assert.That(network.Comp.SortedZLevels, Is.EqualTo(new[] { bottom, middle, top }));
            Assert.That(zLevels.TryGetMapBelow(middle, out var below), Is.True);
            Assert.That(below, Is.EqualTo(bottom));
            Assert.That(zLevels.TryGetMapAbove(middle, out var above), Is.True);
            Assert.That(above, Is.EqualTo(top));
            Assert.That(zLevels.TryGetMapDepthOffset(bottom, top, out var offset), Is.True);
            Assert.That(offset, Is.EqualTo(2));
        });
    }

    [Test]
    public void MovingBetweenMapsPreservesCanonicalPose()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entManager = sim.Resolve<IEntityManager>();
        var mapSystem = entManager.System<SharedMapSystem>();
        var transform = entManager.System<SharedTransformSystem>();
        var zLevels = entManager.System<ZLevelSystem>();

        var lower = mapSystem.CreateMap(out _);
        var upper = mapSystem.CreateMap(out _);
        Assert.That(zLevels.TryCreateMapNetwork([lower, upper], out _), Is.True);

        var entity = entManager.SpawnAtPosition(null, new EntityCoordinates(lower, new Vector2(4.25f, -3.5f)));
        transform.SetWorldRotation(entity, Angle.FromDegrees(37f));
        Assert.That(zLevels.TryMoveEntityToMapOffset(entity, 1), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(transform.GetMap(entity), Is.EqualTo(upper));
            AssertVector(transform.GetWorldPosition(entity), new Vector2(4.25f, -3.5f));
            Assert.That(transform.GetWorldRotation(entity).Degrees, Is.EqualTo(37f).Within(0.001f));
        });
    }

    [Test]
    public void LinkedStaticGridBecomesTransitionParent()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transform = entities.System<SharedTransformSystem>();
        var zLevels = entities.System<ZLevelSystem>();

        var lowerMap = maps.CreateMap(out var lowerMapId);
        var upperMap = maps.CreateMap(out var upperMapId);
        var lowerGrid = maps.CreateGridEntity(lowerMapId);
        var upperGrid = maps.CreateGridEntity(upperMapId);
        maps.SetTile(lowerGrid, Vector2i.Zero, new Tile(1));
        maps.SetTile(upperGrid, Vector2i.Zero, new Tile(1));
        transform.SetWorldPosition(upperGrid, new Vector2(0.49f, 0.51f));
        transform.SetWorldRotation(upperGrid, Angle.FromDegrees(89f));
        Assert.That(zLevels.TryCreateMapNetwork([lowerMap, upperMap], out _), Is.True);
        Assert.That(zLevels.TryLinkGrids(lowerGrid, upperGrid), Is.True);

        Assert.Multiple(() =>
        {
            AssertVector(transform.GetWorldPosition(upperGrid), new Vector2(0f, 1f));
            Assert.That(
                transform.GetWorldRotation(upperGrid).Degrees,
                Is.EqualTo(90f).Within(0.001f),
                "linking should preserve the intended same-spot placement but snap position and rotation to tile-grid units");
        });

        var body = entities.SpawnAtPosition(null, new EntityCoordinates(lowerGrid, new Vector2(0.2f, 0.1f)));
        Assert.That(zLevels.TryMoveEntityToMapOffset(body, 1), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(transform.GetMap(body), Is.EqualTo(upperMap));
            Assert.That(transform.GetGrid(body), Is.EqualTo(upperGrid.Owner));
            AssertVector(transform.GetWorldPosition(body), new Vector2(0.2f, 0.1f));
        });
    }

    [Test]
    public void UnlinkedOverlappingGridIsNotImplicitlySelected()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transform = entities.System<SharedTransformSystem>();
        var zLevels = entities.System<ZLevelSystem>();

        var lowerMap = maps.CreateMap(out var lowerMapId);
        var upperMap = maps.CreateMap(out var upperMapId);
        var lowerGrid = maps.CreateGridEntity(lowerMapId);
        var unrelatedUpperGrid = maps.CreateGridEntity(upperMapId);
        maps.SetTile(lowerGrid, Vector2i.Zero, new Tile(1));
        maps.SetTile(unrelatedUpperGrid, Vector2i.Zero, new Tile(1));
        Assert.That(zLevels.TryCreateMapNetwork([lowerMap, upperMap], out _), Is.True);

        var body = entities.SpawnAtPosition(null, new EntityCoordinates(lowerGrid, Vector2.Zero));
        Assert.That(zLevels.TryMoveEntityToMapOffset(body, 1), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(transform.GetMap(body), Is.EqualTo(upperMap));
            Assert.That(transform.GetGrid(body), Is.Null);
            Assert.That(transform.GetParentUid(body), Is.EqualTo(upperMap));
        });
    }

    [Test]
    public void LinkedMovingRotatingGridsPreservePoseAndParent()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transform = entities.System<SharedTransformSystem>();
        var zLevels = entities.System<ZLevelSystem>();

        var lowerMap = maps.CreateMap(out var lowerMapId);
        var upperMap = maps.CreateMap(out var upperMapId);
        var lowerGrid = maps.CreateGridEntity(lowerMapId);
        var upperGrid = maps.CreateGridEntity(upperMapId);
        maps.SetTile(lowerGrid, Vector2i.Zero, new Tile(1));
        maps.SetTile(upperGrid, Vector2i.Zero, new Tile(1));
        Assert.That(zLevels.TryCreateMapNetwork([lowerMap, upperMap], out _), Is.True);
        Assert.That(zLevels.TryLinkGrids(lowerGrid, upperGrid), Is.True);
        Assert.That(zLevels.TryMoveLinkedGrids(lowerGrid, new Vector2(6f, -3f), Angle.FromDegrees(90f)), Is.True);

        var body = entities.SpawnAtPosition(null, new EntityCoordinates(lowerGrid, new Vector2(0.25f, 0.1f)));
        var beforePosition = transform.GetWorldPosition(body);
        var beforeRotation = transform.GetWorldRotation(body);
        Assert.That(zLevels.TryMoveEntityToMapOffset(body, 1), Is.True);

        Assert.Multiple(() =>
        {
            AssertVector(transform.GetWorldPosition(upperGrid), new Vector2(6f, -3f));
            Assert.That(transform.GetWorldRotation(upperGrid).Degrees, Is.EqualTo(90f).Within(0.001f));
            Assert.That(transform.GetGrid(body), Is.EqualTo(upperGrid.Owner));
            AssertVector(transform.GetWorldPosition(body), beforePosition);
            Assert.That(transform.GetWorldRotation(body).Degrees, Is.EqualTo(beforeRotation.Degrees).Within(0.001f));
        });
    }

    [Test]
    public void LinkedDynamicGridsReceiveSoftCorrectionAndSnapLargeDrift()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var physics = entities.System<SharedPhysicsSystem>();
        var transform = entities.System<SharedTransformSystem>();
        var sync = entities.System<ZLevelGridSyncSystem>();
        var zLevels = entities.System<ZLevelSystem>();
        var config = sim.Resolve<IConfigurationManager>();

        var lowerMap = maps.CreateMap(out var lowerMapId);
        var upperMap = maps.CreateMap(out var upperMapId);
        var lowerGrid = maps.CreateGridEntity(lowerMapId);
        var upperGrid = maps.CreateGridEntity(upperMapId);
        maps.SetTile(lowerGrid, Vector2i.Zero, new Tile(1));
        maps.SetTile(upperGrid, Vector2i.Zero, new Tile(1));
        maps.SetTile(upperGrid, new Vector2i(1, 0), new Tile(1));
        Assert.That(zLevels.TryCreateMapNetwork([lowerMap, upperMap], out _), Is.True);
        Assert.That(zLevels.TryLinkGrids(lowerGrid, upperGrid), Is.True);

        var lowerBody = entities.GetComponent<PhysicsComponent>(lowerGrid);
        var upperBody = entities.GetComponent<PhysicsComponent>(upperGrid);
        physics.SetBodyType(lowerGrid, BodyType.Dynamic, body: lowerBody);
        physics.SetBodyType(upperGrid, BodyType.Dynamic, body: upperBody);
        transform.SetWorldPosition(upperGrid, new Vector2(2f, 0f));

        config.SetCVar(CVars.PhysicsZLevelGridSync, true);
        sync.UpdateBeforeSolve(false, 1f / 60f);
        Assert.Multiple(() =>
        {
            Assert.That(lowerBody.LinearVelocity.X, Is.GreaterThan(0f));
            Assert.That(upperBody.LinearVelocity.X, Is.LessThan(0f));
        });

        physics.SetLinearVelocity(lowerGrid, Vector2.Zero, body: lowerBody);
        physics.SetLinearVelocity(upperGrid, Vector2.Zero, body: upperBody);
        transform.SetWorldPosition(upperGrid, new Vector2(32f, 0f));
        sync.UpdateBeforeSolve(false, 1f / 60f);
        Assert.Multiple(() =>
        {
            AssertVector(transform.GetWorldPosition(lowerGrid), Vector2.Zero);
            AssertVector(transform.GetWorldPosition(upperGrid), Vector2.Zero);
            Assert.That(lowerBody.LinearVelocity, Is.EqualTo(Vector2.Zero));
            Assert.That(upperBody.LinearVelocity, Is.EqualTo(Vector2.Zero));
        });
    }

    [Test]
    public void RelinkingAndRepeatedTransitionsRemainReciprocal()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var transform = entities.System<SharedTransformSystem>();
        var zLevels = entities.System<ZLevelSystem>();

        var bottom = maps.CreateMap(out var bottomId);
        var middle = maps.CreateMap(out var middleId);
        var top = maps.CreateMap(out var topId);
        var bottomGrid = maps.CreateGridEntity(bottomId);
        var middleGrid = maps.CreateGridEntity(middleId);
        var topGrid = maps.CreateGridEntity(topId);
        foreach (var grid in new[] { bottomGrid, middleGrid, topGrid })
            maps.SetTile(grid, Vector2i.Zero, new Tile(1));

        Assert.That(zLevels.TryCreateMapNetwork([bottom, top], out var maybeNetwork), Is.True);
        var network = maybeNetwork!.Value;
        Assert.That(zLevels.TryLinkGrids(bottomGrid, topGrid), Is.True);
        Assert.That(zLevels.TryInsertMap(network, 1, middle), Is.True);
        Assert.That(zLevels.TryGetGridAbove(bottomGrid, out _), Is.False);
        Assert.That(zLevels.TryGetGridBelow(topGrid, out _), Is.False);
        Assert.That(zLevels.TryLinkGrids(bottomGrid, middleGrid), Is.True);
        Assert.That(zLevels.TryLinkGrids(middleGrid, topGrid), Is.True);

        var body = entities.SpawnAtPosition(null, new EntityCoordinates(bottomGrid, Vector2.Zero));
        for (var repetition = 0; repetition < 3; repetition++)
        {
            Assert.That(zLevels.TryMoveEntityToMapOffset(body, 1), Is.True);
            Assert.That(transform.GetGrid(body), Is.EqualTo(middleGrid.Owner));
            Assert.That(zLevels.TryMoveEntityToMapOffset(body, 1), Is.True);
            Assert.That(transform.GetGrid(body), Is.EqualTo(topGrid.Owner));
            Assert.That(zLevels.TryMoveEntityToMapOffset(body, -1), Is.True);
            Assert.That(transform.GetGrid(body), Is.EqualTo(middleGrid.Owner));
            Assert.That(zLevels.TryMoveEntityToMapOffset(body, -1), Is.True);
            Assert.That(transform.GetGrid(body), Is.EqualTo(bottomGrid.Owner));
        }
    }

    [Test]
    public void RemovingMapPrunesLinksAndAllowsExplicitRelink()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var zLevels = entities.System<ZLevelSystem>();

        var bottom = maps.CreateMap(out var bottomId);
        var middle = maps.CreateMap(out var middleId);
        var top = maps.CreateMap(out var topId);
        var bottomGrid = maps.CreateGridEntity(bottomId);
        var middleGrid = maps.CreateGridEntity(middleId);
        var topGrid = maps.CreateGridEntity(topId);
        Assert.That(zLevels.TryCreateMapNetwork([bottom, middle, top], out _), Is.True);
        Assert.That(zLevels.TryLinkGrids(bottomGrid, middleGrid), Is.True);
        Assert.That(zLevels.TryLinkGrids(middleGrid, topGrid), Is.True);

        Assert.That(zLevels.TryRemoveMapFromNetwork(middle), Is.True);
        Assert.Multiple(() =>
        {
            Assert.That(zLevels.TryGetGridAbove(bottomGrid, out _), Is.False);
            Assert.That(zLevels.TryGetGridBelow(topGrid, out _), Is.False);
        });

        Assert.That(zLevels.TryLinkGrids(bottomGrid, topGrid), Is.True);
        Assert.That(zLevels.TryGetGridAbove(bottomGrid, out var relinked), Is.True);
        Assert.That(relinked, Is.EqualTo(topGrid.Owner));
    }

    [Test]
    public void RelinkingOneNetworkDoesNotPruneIndependentNetwork()
    {
        var sim = RobustServerSimulation.NewSimulation().InitializeInstance();
        var entities = sim.Resolve<IEntityManager>();
        var maps = entities.System<SharedMapSystem>();
        var zLevels = entities.System<ZLevelSystem>();

        var firstLower = maps.CreateMap(out var firstLowerId);
        var firstUpper = maps.CreateMap(out var firstUpperId);
        var firstLowerGrid = maps.CreateGridEntity(firstLowerId);
        var firstUpperGrid = maps.CreateGridEntity(firstUpperId);
        Assert.That(zLevels.TryCreateMapNetwork([firstLower, firstUpper], out _), Is.True);
        Assert.That(zLevels.TryLinkGrids(firstLowerGrid, firstUpperGrid), Is.True);

        var secondLower = maps.CreateMap(out _);
        var secondUpper = maps.CreateMap(out _);
        var inserted = maps.CreateMap(out _);
        Assert.That(zLevels.TryCreateMapNetwork([secondLower, secondUpper], out var maybeSecondNetwork), Is.True);
        Assert.That(zLevels.TryInsertMap(maybeSecondNetwork!.Value, 1, inserted), Is.True);

        Assert.Multiple(() =>
        {
            Assert.That(zLevels.TryGetGridAbove(firstLowerGrid, out var above), Is.True);
            Assert.That(above, Is.EqualTo(firstUpperGrid.Owner));
            Assert.That(zLevels.TryGetGridBelow(firstUpperGrid, out var below), Is.True);
            Assert.That(below, Is.EqualTo(firstLowerGrid.Owner));
        });
    }

    [TestCase(0f)]
    [TestCase(0.7f)]
    public void ProjectionRoundTripsWithoutChangingCanonicalCoordinates(float offset)
    {
        var canonical = new Vector2(11.5f, -2.25f);
        var projection = new Vector2(0.2f, offset);

        foreach (var absoluteZ in new[] { 0f, 0.00001f, 0.99999f, 1f, 1.00001f, 2.4f })
        {
            var projected = ZLevelProjection.Project(canonical, absoluteZ, 1, projection);
            var unprojected = ZLevelProjection.Unproject(projected, absoluteZ, 1, projection);
            AssertVector(unprojected, canonical);
        }
    }

    [TestCase(0.00001f, 0, 1f, 0, 0f)]
    [TestCase(0.25f, 0, 0.75f, 1, 0.25f)]
    [TestCase(0.99999f, 1, 1f, 1, 0f)]
    [TestCase(1f, 1, 1f, 1, 0f)]
    [TestCase(1.75f, 1, 0.25f, 2, 0.75f)]
    public void LayerWeightsAreComplementary(
        float absoluteZ,
        int lowerDepth,
        float lowerWeight,
        int upperDepth,
        float upperWeight)
    {
        var weights = ZLevelProjection.GetLayerWeights(absoluteZ);
        Assert.Multiple(() =>
        {
            Assert.That(weights.LowerDepth, Is.EqualTo(lowerDepth));
            Assert.That(weights.LowerWeight, Is.EqualTo(lowerWeight).Within(0.0001f));
            Assert.That(weights.UpperDepth, Is.EqualTo(upperDepth));
            Assert.That(weights.UpperWeight, Is.EqualTo(upperWeight).Within(0.0001f));
            Assert.That(weights.LowerWeight + weights.UpperWeight, Is.EqualTo(1f).Within(0.0001f));
        });
    }

    private static void AssertVector(Vector2 actual, Vector2 expected)
    {
        Assert.That(actual.X, Is.EqualTo(expected.X).Within(0.0001f));
        Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(0.0001f));
    }
}
