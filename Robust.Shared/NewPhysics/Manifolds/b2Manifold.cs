using System.Numerics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box2D uses speculative collision so some contact points may be separated.
public record struct b2Manifold
{
    /// The unit normal vector in world space, points from shape A to bodyB
    internal Vector2 normal;

    /// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
    internal float rollingImpulse;

    /// The manifold points, up to two are possible in 2D
    internal FixedArray2<b2ManifoldPoint> points;

    /// The number of contacts points, will be 0, 1, or 2
    internal int pointCount;

}
