using System;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Components;

namespace Robust.Shared.Physics.Collision.Shapes;

/// <summary>
/// A primitive physical shape that is used by a <see cref="PhysicsComponent"/>.
/// </summary>
public interface IPhysShape : IEquatable<IPhysShape>;
