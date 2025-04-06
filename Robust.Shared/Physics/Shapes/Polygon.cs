using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using Robust.Shared.Maths;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager.Attributes;
using Robust.Shared.Utility;

namespace Robust.Shared.Physics.Shapes;

[DataDefinition, Serializable, NetSerializable]
public partial record struct Polygon : IPhysShape
{
    [DataField]
    public byte VertexCount { get; internal set; }

    /// <summary>
    /// Vertices associated with this polygon. Will be sliced to <see cref="VertexCount"/>
    /// </summary>
    /// <remarks>
    /// Consider using _vertices if doing engine work.
    /// </remarks>
    [DataField]
    public Vector2[] Vertices
    {
        get => _vertices.AsSpan[..VertexCount].ToArray();
        set
        {
            // Mostly for serialization... but content may also want to do it.
            if (value.Length == 0)
            {
                VertexCount = 0;
                return;
            }

            // Validate the verts; engine can just manually handle this but public callers may not have done so.
            var hull = InternalPhysicsHull.ComputeHull(value.AsSpan(), VertexCount);
            VertexCount = (byte) hull.Count;

            hull.Points.CopyTo(_vertices.AsSpan);
            Centroid = ComputeCentroid(_vertices.AsSpan[..VertexCount]);
            CalculateNormals(_vertices.AsSpan, _normals.AsSpan, VertexCount);
        }
    }

    public Vector2[] Normals => _normals.AsSpan[..VertexCount].ToArray();

    [NonSerialized]
    internal FixedArray8<Vector2> _vertices;

    [NonSerialized]
    internal FixedArray8<Vector2> _normals;

    [field: NonSerialized]
    public Vector2 Centroid { get; internal set; }

    public int ChildCount => 1;
    public float Radius { get; set; } = PhysicsConstants.PolygonRadius;
    public ShapeType ShapeType => ShapeType.Polygon;

    // Hopefully this one is short-lived for a few months
    public Polygon(IPhysShape shape) : this((PolygonShape) shape)
    {

    }

    public Polygon(PolygonShape polyShape)
    {
        Unsafe.SkipInit(out this);
        Radius = polyShape.Radius;
        Centroid = polyShape.Centroid;
        VertexCount = (byte) polyShape.VertexCount;

        polyShape.Vertices.AsSpan()[..VertexCount].CopyTo(_vertices.AsSpan);
        polyShape.Normals.AsSpan()[..VertexCount].CopyTo(_normals.AsSpan);
    }

    public Polygon(Box2 box)
    {
        Unsafe.SkipInit(out this);
        Radius = 0f;
        VertexCount = 4;

        _vertices._00 = box.BottomLeft;
        _vertices._01 = box.BottomRight;
        _vertices._02 = box.TopRight;
        _vertices._03 = box.TopLeft;

        _normals._00 = new Vector2(0.0f, -1.0f);
        _normals._01 = new Vector2(1.0f, 0.0f);
        _normals._02 = new Vector2(0.0f, 1.0f);
        _normals._03 = new Vector2(-1.0f, 0.0f);

        Centroid = box.Center;
    }

    public Polygon(Box2Rotated bounds)
    {
        Unsafe.SkipInit(out this);
        Radius = 0f;
        VertexCount = 4;

        _vertices._00 = bounds.BottomLeft;
        _vertices._01 = bounds.BottomRight;
        _vertices._02 = bounds.TopRight;
        _vertices._03 = bounds.TopLeft;

        CalculateNormals(_vertices.AsSpan, _normals.AsSpan, 4);

        Centroid = bounds.Center;
    }

    /// <summary>
    /// Manually constructed polygon for internal use to take advantage of pooling.
    /// </summary>
    internal Polygon(ReadOnlySpan<Vector2> vertices, ReadOnlySpan<Vector2> normals, Vector2 centroid, byte count)
    {
        Unsafe.SkipInit(out this);
        vertices[..VertexCount].CopyTo(_vertices.AsSpan);
        normals[..VertexCount].CopyTo(_normals.AsSpan);
        Centroid = centroid;
        VertexCount = count;
        Radius = 0f;
    }

    public Polygon(Vector2[] vertices)
    {
        Unsafe.SkipInit(out this);
        var hull = InternalPhysicsHull.ComputeHull(vertices, vertices.Length);

        if (hull.Count < 3)
        {
            VertexCount = 0;
            return;
        }

        VertexCount = (byte) vertices.Length;
        var vertSpan = _vertices.AsSpan;

        vertices.AsSpan().CopyTo(vertSpan);
        Set(hull);
        Centroid = ComputeCentroid(vertSpan);
    }

    public static explicit operator Polygon(PolygonShape polyShape)
    {
        return new Polygon(polyShape);
    }

    internal void Set(InternalPhysicsHull hull)
    {
        DebugTools.Assert(hull.Count >= 3);
        var vertexCount = hull.Count;
        var verts = _vertices.AsSpan;
        var norms = _normals.AsSpan;

        for (var i = 0; i < vertexCount; i++)
        {
            verts[i] = hull.Points[i];
        }

        // Compute normals. Ensure the edges have non-zero length.
        CalculateNormals(verts, norms, vertexCount);
    }

    public static void CalculateNormals(ReadOnlySpan<Vector2> vertices, Span<Vector2> normals, int count)
    {
        for (var i = 0; i < count; i++)
        {
            var next = i + 1 < count ? i + 1 : 0;
            var edge = vertices[next] - vertices[i];
            DebugTools.Assert(edge.LengthSquared() > float.Epsilon * float.Epsilon);

            var temp = Vector2Helpers.Cross(edge, 1f);
            normals[i] = temp.Normalized();
        }
    }

    public static Vector2 ComputeCentroid(ReadOnlySpan<Vector2> vs)
    {
        var count = vs.Length;
        DebugTools.Assert(count >= 3);

        var c = new Vector2(0.0f, 0.0f);
        float area = 0.0f;

        // Get a reference point for forming triangles.
        // Use the first vertex to reduce round-off errors.
        var s = vs[0];

        const float inv3 = 1.0f / 3.0f;

        for (var i = 0; i < count; ++i)
        {
            // Triangle vertices.
            var p1 = vs[0] - s;
            var p2 = vs[i] - s;
            var p3 = i + 1 < count ? vs[i+1] - s : vs[0] - s;

            var e1 = p2 - p1;
            var e2 = p3 - p1;

            float D = Vector2Helpers.Cross(e1, e2);

            float triangleArea = 0.5f * D;
            area += triangleArea;

            // Area weighted centroid
            c += (p1 + p2 + p3) * triangleArea * inv3;
        }

        // Centroid
        DebugTools.Assert(area > float.Epsilon);
        c = c * (1.0f / area) + s;
        return c;
    }

    public Box2 ComputeAABB(Transform transform, int childIndex)
    {
        DebugTools.Assert(VertexCount > 0);
        DebugTools.Assert(childIndex == 0);
        var verts = _vertices.AsSpan;
        var lower = Transform.Mul(transform, verts[0]);
        var upper = lower;

        for (var i = 1; i < VertexCount; ++i)
        {
            var v = Transform.Mul(transform, verts[i]);
            lower = Vector2.Min(lower, v);
            upper = Vector2.Max(upper, v);
        }

        var r = new Vector2(Radius, Radius);
        return new Box2(lower - r, upper + r);
    }

    public void SetAsBox(Box2Rotated bounds)
    {
        VertexCount = 4;

        _vertices._00 = bounds.BottomLeft;
        _vertices._01 = bounds.BottomRight;
        _vertices._02 = bounds.TopRight;
        _vertices._03 = bounds.TopLeft;

        var hull = new InternalPhysicsHull(_vertices.AsSpan, VertexCount);
        Set(hull);
    }

    public void SetAsBox(Box2 box)
    {
        VertexCount = 4;

        _vertices._00 = box.BottomLeft;
        _vertices._01 = box.BottomRight;
        _vertices._02 = box.TopRight;
        _vertices._03 = box.TopLeft;

        _normals._00 = new Vector2(0.0f, -1.0f);
        _normals._01 = new Vector2(1.0f, 0.0f);
        _normals._02 = new Vector2(0.0f, 1.0f);
        _normals._03 = new Vector2(-1.0f, 0.0f);

        Centroid = box.Center;
    }

    public void SetAsBox(float halfWidth, float halfHeight)
    {
        VertexCount = 4;

        _vertices._00 = new Vector2(-halfWidth, -halfHeight);
        _vertices._01 = new Vector2(halfWidth, -halfHeight);
        _vertices._02 = new Vector2(halfWidth,  halfHeight);
        _vertices._03 = new Vector2(-halfWidth,  halfHeight);

        _normals._00 = new Vector2(0.0f, -1.0f);
        _normals._01 = new Vector2(1.0f, 0.0f);
        _normals._02 = new Vector2(0.0f, 1.0f);
        _normals._03 = new Vector2(-1.0f, 0.0f);

        Centroid = Vector2.Zero;
    }

    public void SetAsBox(float halfWidth, float halfHeight, Vector2 center, float angle)
    {
        VertexCount = 4;

        _vertices._00 = new Vector2(-halfWidth, -halfHeight);
        _vertices._01 = new Vector2(halfWidth, -halfHeight);
        _vertices._02 = new Vector2(halfWidth, halfHeight);
        _vertices._03 = new Vector2(-halfWidth, halfHeight);

        _normals._00 = new Vector2(0f, -1f);
        _normals._01 = new Vector2(1f, 0f);
        _normals._02 = new Vector2(0f, 1f);
        _normals._03 = new Vector2(-1f, 0f);

        Centroid = center;

        var xf = new Transform(center, angle);

        // Transform vertices and normals.
        for (var i = 0; i < VertexCount; ++i)
        {
            _vertices.AsSpan[i] = Transform.Mul(xf, _vertices.AsSpan[i]);
            _normals.AsSpan[i] = Transform.Mul(xf.Quaternion2D, _normals.AsSpan[i]);
        }
    }

    public bool Equals(IPhysShape? other)
    {
        if (other is SlimPolygon slim)
        {
            return Equals(slim);
        }

        return other is Polygon poly && Equals(poly);
    }

    public bool Equals(Polygon other)
    {
        if (VertexCount != other.VertexCount)
            return false;

        var ourVerts = _vertices.AsSpan;
        var otherVerts = other._vertices.AsSpan;

        for (var i = 0; i < VertexCount; i++)
        {
            var vert = ourVerts[i];
            if (!vert.Equals(otherVerts[i]))
                return false;
        }

        return true;
    }

    internal bool Equals(SlimPolygon other)
    {
        if (VertexCount != other.VertexCount) return false;

        var ourVerts = _vertices.AsSpan;
        var otherVerts = other._vertices.AsSpan;

        for (var i = 0; i < VertexCount; i++)
        {
            var vert = ourVerts[i];
            if (!vert.Equals(otherVerts[i]))
                return false;
        }

        return true;
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(VertexCount, _vertices.AsSpan.ToArray(), Radius);
    }
}
