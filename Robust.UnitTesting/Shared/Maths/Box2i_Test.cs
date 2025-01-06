using System.Collections.Generic;
using NUnit.Framework;
using Robust.Shared.Maths;

namespace Robust.UnitTesting.Shared.Maths
{
    [TestFixture, Parallelizable, TestOf(typeof(Box2i))]
    sealed class Box2i_Test
    {
        private static IEnumerable<TestCaseData> ContainsSources()
        {
            // Empties
            yield return new TestCaseData(Box2i.Empty, Box2i.Empty, false);
            // Right square
            yield return new TestCaseData(new Box2i(0, 0, 1, 1), new Box2i(1, 0, 2, 1), false);
            // Left square
            yield return new TestCaseData(new Box2i(0, 0, 1, 1), new Box2i(-1, 0, 0, 1), false);
            // Top square
            yield return new TestCaseData(new Box2i(0, 0, 1, 1), new Box2i(0, 1, 1, 2), false);
            // Bottom square
            yield return new TestCaseData(new Box2i(0, 0, 1, 1), new Box2i(0, -1, 1, 0), false);
            // Encompassing
            yield return new TestCaseData(new Box2i(0, 0, 1, 1), new Box2i(-1, -1, 2, 2), true);
            yield return new TestCaseData(new Box2i(0, 0, 1, 1), new Box2i(0, 0, 1, 1), true);
        }

        [Test]
        public void Box2iUnion()
        {
            var boxOne = new Box2i(-1, -1, 1, 1);
            var boxTwo = new Box2i(0, 0, 2, 2);

            var result = boxOne.Union(boxTwo);

            Assert.That(result.Left, Is.EqualTo(-1));
            Assert.That(result.Bottom, Is.EqualTo(-1));
            Assert.That(result.Right, Is.EqualTo(2));
            Assert.That(result.Top, Is.EqualTo(2));
        }

        [Test]
        public void Box2iVector2iUnion()
        {
            var box = new Box2i();
            Assert.That(box, Is.EqualTo(Box2i.Empty));

            box = box.UnionTile(Vector2i.Zero);
            Assert.That(box.Right, Is.EqualTo(1));

            box = box.UnionTile(Vector2i.One);
            Assert.That(box.Top, Is.EqualTo(2));

            box = box.Union(new Vector2i(2, 0));
            Assert.That(box.Right, Is.EqualTo(2));
        }

        [Test, TestCaseSource(nameof(ContainsSources))]
        public void Intersects(Box2i source, Box2i target, bool result)
        {
            Assert.That(source.Intersects(target), Is.EqualTo(result));
        }
    }
}
