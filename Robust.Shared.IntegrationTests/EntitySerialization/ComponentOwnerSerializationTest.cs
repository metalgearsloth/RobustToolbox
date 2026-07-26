using System.Numerics;
using System.Threading.Tasks;
using NUnit.Framework;
using Robust.Shared.Containers;
using Robust.Shared.EntitySerialization.Systems;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Map.Components;
using Robust.Shared.Physics.Components;
using Robust.Shared.Utility;
using static Robust.UnitTesting.Shared.EntitySerialization.EntitySaveTestComponent;

namespace Robust.UnitTesting.Shared.EntitySerialization;

[TestFixture]
internal sealed class ComponentOwnerSerializationTest : RobustIntegrationTest
{
    private const string TestTileDefId = "a";
    private const string Prototype = $@"
- type: testTileDef
  id: space

- type: testTileDef
  id: {TestTileDefId}

- type: entity
  id: TestSerializedPhysicsOwner
  components:
  - type: Transform
    anchored: true
  - type: Physics
  - type: EntitySaveTest
  - type: ContainerContainer
    containers:
      test_slot: !type:ContainerSlot
        ent: null
";

    [Test]
    public async Task LoadedPhysicsComponentOwnsEntity()
    {
        var opts = new ServerIntegrationOptions
        {
            ExtraPrototypes = Prototype
        };

        var server = StartServer(opts);
        await server.WaitIdleAsync();

        var entMan = server.EntMan;
        var mapSys = server.System<SharedMapSystem>();
        var loader = server.System<MapLoaderSystem>();
        var tileMan = server.ResolveDependency<ITileDefinitionManager>();

        var path = new ResPath($"{nameof(LoadedPhysicsComponentOwnsEntity)}.yml");
        MapId mapId = default;

        SerializationTestHelper.LoadTileDefs(server.ProtoMan, tileMan, "space");
        var tDef = server.ProtoMan.Index<TileDef>(TestTileDefId);

        await server.WaitPost(() =>
        {
            mapSys.CreateMap(out mapId);
            var grid = mapSys.CreateGridEntity(mapId);
            var coords = new MapCoordinates(new Vector2(7, 7), mapId);
            mapSys.SetTile(grid, mapSys.TileIndicesFor(grid, coords), new Tile(tDef.TileId));

            for (var i = 0; i < 3; i++)
            {
                var ent = entMan.Spawn("TestSerializedPhysicsOwner", coords);
                entMan.GetComponent<EntitySaveTestComponent>(ent).Id = $"{nameof(LoadedPhysicsComponentOwnsEntity)}{i}";
            }
        });

        Assert.That(loader.TrySaveMap(mapId, path));

        await server.WaitPost(() => mapSys.DeleteMap(mapId));

        await server.WaitPost(() =>
        {
            Assert.That(loader.TryLoadMap(path, out var map, out _));
            mapId = map!.Value.Comp.MapId;
        });

        for (var i = 0; i < 3; i++)
        {
            var loaded = Find($"{nameof(LoadedPhysicsComponentOwnsEntity)}{i}", entMan);
            var physics = entMan.GetComponent<PhysicsComponent>(loaded.Owner);
            var containers = entMan.GetComponent<ContainerManagerComponent>(loaded.Owner);
            var container = containers.Containers["test_slot"];

#pragma warning disable CS0618
            Assert.That(physics.Owner, Is.EqualTo(loaded.Owner));
#pragma warning restore CS0618
            Assert.That(container.Owner, Is.EqualTo(loaded.Owner));
            Assert.That(container.Manager, Is.EqualTo(containers));
        }

        await server.WaitPost(() => Assert.DoesNotThrow(() => mapSys.DeleteMap(mapId)));
    }
}
