using System.Numerics;

namespace Robust.Shared.NewPhysics;

/// Result of computing the distance between two line segments
internal record struct SegmentDistanceResult
{
    /// The closest point on the first segment
    public Vector2 closest1;

    /// The closest point on the second segment
    public Vector2 closest2;

    /// The barycentric coordinate on the first segment
    public float fraction1;

    /// The barycentric coordinate on the second segment
    public float fraction2;

    /// The squared distance between the closest points
    public float distanceSquared;
}
