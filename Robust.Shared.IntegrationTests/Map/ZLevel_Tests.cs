using System.Collections.Generic;
using System.Numerics;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Maths;
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
