﻿using System.IO;
using NUnit.Framework;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Map;
using Robust.Shared.Maths;
using Robust.Shared.Prototypes;

// ReSharper disable InconsistentNaming
// ReSharper disable AccessToStaticMemberViaDerivedType
namespace Robust.UnitTesting.Shared.Map
{
    [TestFixture, Parallelizable, TestOf(typeof(EntityCoordinates))]
    public class EntityCoordinates_Tests : RobustUnitTest
    {
        private const string PROTOTYPES = @"
- type: entity
  name: dummy
  id: dummy
  components:
  - type: Transform";

        [OneTimeSetUp]
        public void Setup()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            entityManager.Initialize();

            var mapManager = IoCManager.Resolve<IMapManager>();

            mapManager.Initialize();
            mapManager.Startup();

            var prototypeManager = IoCManager.Resolve<IPrototypeManager>();
            prototypeManager.LoadFromStream(new StringReader(PROTOTYPES));
            prototypeManager.Resync();
        }

        /// <summary>
        ///     Passing an invalid entity ID into the constructor makes it invalid.
        /// </summary>
        [Test]
        public void IsValid_InvalidEntId_False()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();

            // Same as EntityCoordinates.Invalid
            var coords = new EntityCoordinates(EntityUid.Invalid, Vector2.Zero);

            Assert.That(coords.IsValid(entityManager), Is.False);
        }

        /// <summary>
        ///     Deleting the parent entity should make the coordinates invalid.
        /// </summary>
        [Test]
        public void IsValid_EntityDeleted_False()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEntity = mapManager.CreateNewMapEntity(mapId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new MapCoordinates(Vector2.Zero, mapId));

            var coords = newEnt.Transform.Coordinates;

            var result = coords.IsValid(entityManager);

            Assert.That(result, Is.True);

            mapEntity.Delete();

            result = coords.IsValid(entityManager);

            Assert.That(result, Is.False);
        }

        /// <summary>
        ///     Passing a valid entity ID into the constructor with infinite numbers makes it invalid.
        /// </summary>
        [TestCase(float.NaN, float.NaN)]
        [TestCase(0, float.NaN)]
        [TestCase(float.NaN, 0)]
        public void IsValid_NonFiniteVector_False(float x, float y)
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEntity = mapManager.CreateNewMapEntity(mapId);

            var newEnt = entityManager.CreateEntityUninitialized("dummy", new MapCoordinates(new Vector2(x, y), mapId));
            var coords = newEnt.Transform.Coordinates;

            Assert.That(coords.IsValid(entityManager), Is.False);
        }

        [Test]
        public void EntityCoordinates_Map()
        {
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEntity = mapManager.CreateNewMapEntity(mapId);

            Assert.That(mapEntity.Transform.ParentUid.IsValid(), Is.False);
            Assert.That(mapEntity.Transform.Coordinates.EntityId, Is.EqualTo(mapEntity.Uid));
        }

        /// <summary>
        ///     Even if we change the local position of an entity without a parent,
        ///     EntityCoordinates should still return offset 0.
        /// </summary>
        [Test]
        public void NoParent_OffsetZero()
        {
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEntity = mapManager.CreateNewMapEntity(mapId);
            Assert.That(mapEntity.Transform.Coordinates.Position, Is.EqualTo(Vector2.Zero));

            mapEntity.Transform.LocalPosition = Vector2.One;
            Assert.That(mapEntity.Transform.Coordinates.Position, Is.EqualTo(Vector2.Zero));
        }

        [Test]
        public void GetGridId_Map()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEnt = mapManager.CreateNewMapEntity(mapId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new MapCoordinates(Vector2.Zero, mapId));

            Assert.That(mapEnt.Transform.Coordinates.GetGridId(entityManager), Is.EqualTo(GridId.Invalid));
            Assert.That(newEnt.Transform.Coordinates.GetGridId(entityManager), Is.EqualTo(GridId.Invalid));
            Assert.That(newEnt.Transform.Coordinates.EntityId, Is.EqualTo(mapEnt.Uid));
        }

        [Test]
        public void GetGridId_Grid()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var grid = mapManager.CreateGrid(mapId);
            var gridEnt = entityManager.GetEntity(grid.GridEntityId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(grid.GridEntityId, Vector2.Zero));

            // Grids aren't parented to other grids.
            Assert.That(gridEnt.Transform.Coordinates.GetGridId(entityManager), Is.EqualTo(GridId.Invalid));
            Assert.That(newEnt.Transform.Coordinates.GetGridId(entityManager), Is.EqualTo(grid.Index));
            Assert.That(newEnt.Transform.Coordinates.EntityId, Is.EqualTo(gridEnt.Uid));
        }

        [Test]
        public void GetMapId_Map()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEnt = mapManager.CreateNewMapEntity(mapId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new MapCoordinates(Vector2.Zero, mapId));

            Assert.That(mapEnt.Transform.Coordinates.GetMapId(entityManager), Is.EqualTo(mapId));
            Assert.That(newEnt.Transform.Coordinates.GetMapId(entityManager), Is.EqualTo(mapId));
        }

        [Test]
        public void GetMapId_Grid()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var grid = mapManager.CreateGrid(mapId);
            var gridEnt = entityManager.GetEntity(grid.GridEntityId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(grid.GridEntityId, Vector2.Zero));

            Assert.That(gridEnt.Transform.Coordinates.GetMapId(entityManager), Is.EqualTo(mapId));
            Assert.That(newEnt.Transform.Coordinates.GetMapId(entityManager), Is.EqualTo(mapId));
        }

        [Test]
        public void GetParent()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var grid = mapManager.CreateGrid(mapId);
            var mapEnt = mapManager.GetMapEntity(mapId);
            var gridEnt = entityManager.GetEntity(grid.GridEntityId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(grid.GridEntityId, Vector2.Zero));

            Assert.That(mapEnt.Transform.Coordinates.GetParent(entityManager), Is.EqualTo(mapEnt));
            Assert.That(gridEnt.Transform.Coordinates.GetParent(entityManager), Is.EqualTo(mapEnt));
            Assert.That(newEnt.Transform.Coordinates.GetParent(entityManager), Is.EqualTo(gridEnt));

            // Reparenting the entity should return correct results.
            newEnt.Transform.AttachParent(mapEnt);

            Assert.That(newEnt.Transform.Coordinates.GetParent(entityManager), Is.EqualTo(mapEnt));
        }

        [Test]
        public void TryGetParent()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var grid = mapManager.CreateGrid(mapId);
            var mapEnt = mapManager.GetMapEntity(mapId);
            var gridEnt = entityManager.GetEntity(grid.GridEntityId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(grid.GridEntityId, Vector2.Zero));

            Assert.That(mapEnt.Transform.Coordinates.TryGetParent(entityManager, out var mapEntParent), Is.EqualTo(true));
            Assert.That(mapEntParent, Is.EqualTo(mapEnt));

            Assert.That(gridEnt.Transform.Coordinates.TryGetParent(entityManager, out var gridEntParent), Is.EqualTo(true));
            Assert.That(gridEntParent, Is.EqualTo(mapEnt));

            Assert.That(newEnt.Transform.Coordinates.TryGetParent(entityManager, out var newEntParent), Is.EqualTo(true));
            Assert.That(newEntParent, Is.EqualTo(gridEnt));

            // Reparenting the entity should return correct results.
            newEnt.Transform.AttachParent(mapEnt);

            Assert.That(newEnt.Transform.Coordinates.TryGetParent(entityManager, out newEntParent), Is.EqualTo(true));
            Assert.That(newEntParent, Is.EqualTo(mapEnt));

            // Deleting the parent should make TryGetParent return false.
            mapEnt.Delete();

            // These shouldn't be valid anymore.
            Assert.That(newEnt.Transform.Coordinates.IsValid(entityManager), Is.False);
            Assert.That(gridEnt.Transform.Coordinates.IsValid(entityManager), Is.False);

            Assert.That(newEnt.Transform.Coordinates.TryGetParent(entityManager, out newEntParent), Is.EqualTo(false));
            Assert.That(newEntParent, Is.Null);

            Assert.That(gridEnt.Transform.Coordinates.TryGetParent(entityManager, out gridEntParent), Is.EqualTo(false));
            Assert.That(gridEntParent, Is.Null);
        }

        [Test]
        [TestCase(0, 0, 0, 0)]
        [TestCase(5, 0, 0, 0)]
        [TestCase(5, 0, 0, 5)]
        [TestCase(-5, 5, 5, -5)]
        [TestCase(100, -500, -200, -300)]
        public void ToMap_MoveGrid(float x1, float y1, float x2, float y2)
        {
            var gridPos = new Vector2(x1, y1);
            var entPos = new Vector2(x2, y2);

            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var grid = mapManager.CreateGrid(mapId);
            var gridEnt = entityManager.GetEntity(grid.GridEntityId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(grid.GridEntityId, entPos));

            Assert.That(newEnt.Transform.Coordinates.ToMap(entityManager), Is.EqualTo(new MapCoordinates(entPos, mapId)));

            gridEnt.Transform.LocalPosition += gridPos;

            Assert.That(newEnt.Transform.Coordinates.ToMap(entityManager), Is.EqualTo(new MapCoordinates(entPos + gridPos, mapId)));
        }

        [Test]
        public void WithEntityId()
        {
            var entityManager = IoCManager.Resolve<IEntityManager>();
            var mapManager = IoCManager.Resolve<IMapManager>();

            var mapId = mapManager.CreateMap();
            var mapEnt = mapManager.GetMapEntity(mapId);
            var grid = mapManager.CreateGrid(mapId);
            var gridEnt = entityManager.GetEntity(grid.GridEntityId);
            var newEnt = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(grid.GridEntityId, Vector2.Zero));

            Assert.That(newEnt.Transform.Coordinates.WithEntityId(mapEnt).Position, Is.EqualTo(Vector2.Zero));

            newEnt.Transform.LocalPosition = Vector2.One;

            Assert.That(newEnt.Transform.Coordinates.Position, Is.EqualTo(Vector2.One));
            Assert.That(newEnt.Transform.Coordinates.WithEntityId(mapEnt).Position, Is.EqualTo(Vector2.One));

            gridEnt.Transform.LocalPosition = Vector2.One;

            Assert.That(newEnt.Transform.Coordinates.Position, Is.EqualTo(Vector2.One));
            Assert.That(newEnt.Transform.Coordinates.WithEntityId(mapEnt).Position, Is.EqualTo(new Vector2(2, 2)));

            var newEntTwo = entityManager.CreateEntityUninitialized("dummy", new EntityCoordinates(newEnt.Uid, Vector2.Zero));

            Assert.That(newEntTwo.Transform.Coordinates.Position, Is.EqualTo(Vector2.Zero));
            Assert.That(newEntTwo.Transform.Coordinates.WithEntityId(mapEnt).Position, Is.EqualTo(newEnt.Transform.Coordinates.WithEntityId(mapEnt).Position));
            Assert.That(newEntTwo.Transform.Coordinates.WithEntityId(gridEnt).Position, Is.EqualTo(newEnt.Transform.Coordinates.Position));

            newEntTwo.Transform.LocalPosition = -Vector2.One;

            Assert.That(newEntTwo.Transform.Coordinates.Position, Is.EqualTo(-Vector2.One));
            Assert.That(newEntTwo.Transform.Coordinates.WithEntityId(mapEnt).Position, Is.EqualTo(Vector2.One));
            Assert.That(newEntTwo.Transform.Coordinates.WithEntityId(gridEnt).Position, Is.EqualTo(Vector2.Zero));
        }
    }
}
