using System;
using System.Numerics;
using Robust.Shared.Maths;
using Robust.Shared.NewPhysics.Shapes;
using Robust.Shared.Physics;
using Robust.Shared.Physics.Collision;
using Robust.Shared.Physics.Collision.Shapes;
using Robust.Shared.Physics.Shapes;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private ushort MakeId(int a, int b)
    {
        return (ushort) ((byte) a << 8 | (byte) b);
    }

    private b2Manifold GetManifold(
        IPhysShape shapeA,
        in Transform transformA,
        IPhysShape shapeB,
        in Transform transformB,
        SimplexCache cache)
    {
        switch (shapeA)
        {
            case Capsule capsuleA:
                switch (shapeB)
                {
                    case Capsule capsuleB:
                        return CollideCapsules(capsuleA, transformA, capsuleB, transformB);
                    case PhysShapeCircle circleB:
                        return CollideCapsuleAndCircle(capsuleA, transformA, circleB, transformB);
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            case PhysShapeCircle circleA:
                switch (shapeB)
                {
                    case PhysShapeCircle circleB:
                        return CollideCircles(circleA, transformA, circleB, transformB);
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            case Polygon polyA:
            {
                switch (shapeB)
                {
                    case PhysShapeCircle circleB:
                        return CollidePolygonAndCircle(polyA, transformA, circleB, transformB);
                    case Polygon polyB:
                        return CollidePolygons(polyA, transformA, polyB, transformB, cache);
                    default:
                        throw new ArgumentOutOfRangeException();
                }
            }
            default:
                throw new ArgumentOutOfRangeException();
        }
    }

    #region Capsules

    // Follows Ericson 5.1.9 Closest Points of Two Line Segments
    // Adds some logic to support clipping to get two contact points
    private b2Manifold CollideCapsules(in Capsule capsuleA, in Transform xfA, in Capsule capsuleB, in Transform xfB)
    {
	    var origin = capsuleA.center1;

	    // Shift polyA to origin
	    // pw = q * pb + p
	    // pw = q * (pbs + origin) + p
	    // pw = q * pbs + (p + q * origin)
	    b2Transform sfA = { b2Add( xfA.p, b2RotateVector( xfA.q, origin ) ), xfA.q };
	    b2Transform xf = b2InvMulTransforms( sfA, xfB );

	    b2Vec2 p1 = b2Vec2_zero;
	    b2Vec2 q1 = b2Sub( capsuleA->center2, origin );

	    b2Vec2 p2 = b2TransformPoint( xf, capsuleB->center1 );
	    b2Vec2 q2 = b2TransformPoint( xf, capsuleB->center2 );

	    b2Vec2 d1 = b2Sub( q1, p1 );
	    b2Vec2 d2 = b2Sub( q2, p2 );

	    float dd1 = b2Dot( d1, d1 );
	    float dd2 = b2Dot( d2, d2 );

	    const float epsSqr = FLT_EPSILON * FLT_EPSILON;
	    B2_ASSERT( dd1 > epsSqr && dd2 > epsSqr );

	    b2Vec2 r = b2Sub( p1, p2 );
	    float rd1 = b2Dot( r, d1 );
	    float rd2 = b2Dot( r, d2 );

	    float d12 = b2Dot( d1, d2 );

	    float denom = dd1 * dd2 - d12 * d12;

	    // Fraction on segment 1
	    float f1 = 0.0f;
	    if ( denom != 0.0f )
	    {
		    // not parallel
		    f1 = b2ClampFloat( ( d12 * rd2 - rd1 * dd2 ) / denom, 0.0f, 1.0f );
	    }

	    // Compute point on segment 2 closest to p1 + f1 * d1
	    float f2 = ( d12 * f1 + rd2 ) / dd2;

	    // Clamping of segment 2 requires a do over on segment 1
	    if ( f2 < 0.0f )
	    {
		    f2 = 0.0f;
		    f1 = b2ClampFloat( -rd1 / dd1, 0.0f, 1.0f );
	    }
	    else if ( f2 > 1.0f )
	    {
		    f2 = 1.0f;
		    f1 = b2ClampFloat( ( d12 - rd1 ) / dd1, 0.0f, 1.0f );
	    }

	    b2Vec2 closest1 = b2MulAdd( p1, f1, d1 );
	    b2Vec2 closest2 = b2MulAdd( p2, f2, d2 );
	    float distanceSquared = b2DistanceSquared( closest1, closest2 );

	    b2Manifold manifold = { 0 };
	    float radiusA = capsuleA->radius;
	    float radiusB = capsuleB->radius;
	    float radius = radiusA + radiusB;
	    float maxDistance = radius + B2_SPECULATIVE_DISTANCE;

	    if ( distanceSquared > maxDistance * maxDistance )
	    {
		    return manifold;
	    }

	    float distance = sqrtf( distanceSquared );

	    float length1, length2;
	    b2Vec2 u1 = b2GetLengthAndNormalize( &length1, d1 );
	    b2Vec2 u2 = b2GetLengthAndNormalize( &length2, d2 );

	    // Does segment B project outside segment A?
	    float fp2 = b2Dot( b2Sub( p2, p1 ), u1 );
	    float fq2 = b2Dot( b2Sub( q2, p1 ), u1 );
	    bool outsideA = ( fp2 <= 0.0f && fq2 <= 0.0f ) || ( fp2 >= length1 && fq2 >= length1 );

	    // Does segment A project outside segment B?
	    float fp1 = b2Dot( b2Sub( p1, p2 ), u2 );
	    float fq1 = b2Dot( b2Sub( q1, p2 ), u2 );
	    bool outsideB = ( fp1 <= 0.0f && fq1 <= 0.0f ) || ( fp1 >= length2 && fq1 >= length2 );

	    if ( outsideA == false && outsideB == false )
	    {
		    // attempt to clip
		    // this may yield contact points with excessive separation
		    // in that case the algorithm falls back to single point collision

		    // find reference edge using SAT
		    b2Vec2 normalA;
		    float separationA;

		    {
			    normalA = b2LeftPerp( u1 );
			    float ss1 = b2Dot( b2Sub( p2, p1 ), normalA );
			    float ss2 = b2Dot( b2Sub( q2, p1 ), normalA );
			    float s1p = ss1 < ss2 ? ss1 : ss2;
			    float s1n = -ss1 < -ss2 ? -ss1 : -ss2;

			    if ( s1p > s1n )
			    {
				    separationA = s1p;
			    }
			    else
			    {
				    separationA = s1n;
				    normalA = b2Neg( normalA );
			    }
		    }

		    b2Vec2 normalB;
		    float separationB;
		    {
			    normalB = b2LeftPerp( u2 );
			    float ss1 = b2Dot( b2Sub( p1, p2 ), normalB );
			    float ss2 = b2Dot( b2Sub( q1, p2 ), normalB );
			    float s1p = ss1 < ss2 ? ss1 : ss2;
			    float s1n = -ss1 < -ss2 ? -ss1 : -ss2;

			    if ( s1p > s1n )
			    {
				    separationB = s1p;
			    }
			    else
			    {
				    separationB = s1n;
				    normalB = b2Neg( normalB );
			    }
		    }

		    // biased to avoid feature flip-flop
		    // todo more testing?
		    if ( separationA + 0.1f * PhysicsConstants.LinearSlop >= separationB )
		    {
			    manifold.normal = normalA;

			    var cp = p2;
			    var cq = q2;

			    // clip to p1
			    if ( fp2 < 0.0f && fq2 > 0.0f )
			    {
				    cp = Vector2.Lerp( p2, q2, ( 0.0f - fp2 ) / ( fq2 - fp2 ) );
			    }
			    else if ( fq2 < 0.0f && fp2 > 0.0f )
			    {
				    cq = b2Lerp( q2, p2, ( 0.0f - fq2 ) / ( fp2 - fq2 ) );
			    }

			    // clip to q1
			    if ( fp2 > length1 && fq2 < length1 )
			    {
				    cp = b2Lerp( p2, q2, ( fp2 - length1 ) / ( fp2 - fq2 ) );
			    }
			    else if ( fq2 > length1 && fp2 < length1 )
			    {
				    cq = b2Lerp( q2, p2, ( fq2 - length1 ) / ( fq2 - fp2 ) );
			    }

			    float sp = b2Dot( b2Sub( cp, p1 ), normalA );
			    float sq = b2Dot( b2Sub( cq, p1 ), normalA );

			    if ( sp <= distance + B2_LINEAR_SLOP || sq <= distance + B2_LINEAR_SLOP )
			    {
				    b2ManifoldPoint* mp;
				    mp = manifold.points + 0;
				    mp->anchorA = b2MulAdd( cp, 0.5f * ( radiusA - radiusB - sp ), normalA );
				    mp->separation = sp - radius;
				    mp->id = B2_MAKE_ID( 0, 0 );

				    mp = manifold.points + 1;
				    mp->anchorA = b2MulAdd( cq, 0.5f * ( radiusA - radiusB - sq ), normalA );
				    mp->separation = sq - radius;
				    mp->id = B2_MAKE_ID( 0, 1 );
				    manifold.pointCount = 2;
			    }
		    }
		    else
		    {
			    // normal always points from A to B
			    manifold.normal = -normalB;

			    b2Vec2 cp = p1;
			    b2Vec2 cq = q1;

			    // clip to p2
			    if ( fp1 < 0.0f && fq1 > 0.0f )
			    {
				    cp = b2Lerp( p1, q1, ( 0.0f - fp1 ) / ( fq1 - fp1 ) );
			    }
			    else if ( fq1 < 0.0f && fp1 > 0.0f )
			    {
				    cq = b2Lerp( q1, p1, ( 0.0f - fq1 ) / ( fp1 - fq1 ) );
			    }

			    // clip to q2
			    if ( fp1 > length2 && fq1 < length2 )
			    {
				    cp = b2Lerp( p1, q1, ( fp1 - length2 ) / ( fp1 - fq1 ) );
			    }
			    else if ( fq1 > length2 && fp1 < length2 )
			    {
				    cq = b2Lerp( q1, p1, ( fq1 - length2 ) / ( fq1 - fp1 ) );
			    }

			    float sp = b2Dot( b2Sub( cp, p2 ), normalB );
			    float sq = b2Dot( b2Sub( cq, p2 ), normalB );

			    if ( sp <= distance + B2_LINEAR_SLOP || sq <= distance + B2_LINEAR_SLOP )
			    {
				    b2ManifoldPoint* mp;
				    mp = manifold.points + 0;
				    mp->anchorA = b2MulAdd( cp, 0.5f * ( radiusB - radiusA - sp ), normalB );
				    mp->separation = sp - radius;
				    mp->id = B2_MAKE_ID( 0, 0 );
				    mp = manifold.points + 1;
				    mp->anchorA = b2MulAdd( cq, 0.5f * ( radiusB - radiusA - sq ), normalB );
				    mp->separation = sq - radius;
				    mp->id = B2_MAKE_ID( 1, 0 );
				    manifold.pointCount = 2;
			    }
		    }
	    }

	    if ( manifold.pointCount == 0 )
	    {
		    // single point collision
		    b2Vec2 normal = b2Sub( closest2, closest1 );
		    if ( b2Dot( normal, normal ) > epsSqr )
		    {
			    normal = b2Normalize( normal );
		    }
		    else
		    {
			    normal = b2LeftPerp( u1 );
		    }

		    b2Vec2 c1 = b2MulAdd( closest1, radiusA, normal );
		    b2Vec2 c2 = b2MulAdd( closest2, -radiusB, normal );

		    int i1 = f1 == 0.0f ? 0 : 1;
		    int i2 = f2 == 0.0f ? 0 : 1;

		    manifold.normal = normal;
		    manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
		    manifold.points[0].separation = sqrtf( distanceSquared ) - radius;
		    manifold.points[0].id = B2_MAKE_ID( i1, i2 );
		    manifold.pointCount = 1;
	    }

	    // Convert manifold to world space
	    manifold.normal = b2RotateVector( xfA.q, manifold.normal );
	    for ( int i = 0; i < manifold.pointCount; ++i )
	    {
		    b2ManifoldPoint* mp = manifold.points + i;

		    // anchor points relative to shape origin in world space
		    mp->anchorA = b2RotateVector( xfA.q, b2Add( mp->anchorA, origin ) );
		    mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		    mp->point = b2Add( xfA.p, mp->anchorA );
	    }

	    return manifold;
    }

    /// Compute the collision manifold between a capsule and circle
    private b2Manifold CollideCapsuleAndCircle(in Capsule capsuleA, in Transform xfA, PhysShapeCircle circleB, in Transform xfB)
    {
        b2Manifold manifold = new();

        var xf = Physics.Transform.InvMulTransforms( xfA, xfB );

        // Compute circle position in the frame of the capsule.
        var pB = Physics.Transform.TransformPoint(xf, circleB.Position);

        // Compute closest point
        var p1 = capsuleA.center1;
        var p2 = capsuleA.center2;

        var e = p2 - p1;

        // dot(p - pA, e) = 0
        // dot(p - (p1 + s1 * e), e) = 0
        // s1 = dot(p - p1, e)
        Vector2 pA;
        float s1 = Vector2.Dot(pB - p1, e);
        float s2 = Vector2.Dot(p2 - pB, e);
        if ( s1 < 0.0f )
        {
            // p1 region
            pA = p1;
        }
        else if ( s2 < 0.0f )
        {
            // p2 region
            pA = p2;
        }
        else
        {
            // circle colliding with segment interior
            float s = s1 / Vector2.Dot( e, e );
            pA = Vector2Helpers.MulAdd( p1, s, e );
        }

        float distance;
        dis
        var normal = b2GetLengthAndNormalize( &distance, b2Sub( pB, pA ) );

        float radiusA = capsuleA.Radius;
        float radiusB = circleB.Radius;
        float separation = distance - radiusA - radiusB;
        if ( separation > PhysicsConstants.SpeculativeDistance )
        {
            return manifold;
        }

        var cA = Vector2Helpers.MulAdd( pA, radiusA, normal );
        var cB = Vector2Helpers.MulAdd( pB, -radiusB, normal );
        var contactPointA = Vector2.Lerp( cA, cB, 0.5f );

        manifold.normal = Quaternion2D.RotateVector(xfA.Quaternion2D, normal);
        ref var mp = ref manifold.points._00;
        mp.anchorA = Quaternion2D.RotateVector(xfA.Quaternion2D, contactPointA);
        mp.anchorB = mp.anchorA + xfA.Position - xfB.Position;
        mp.point = xfA.Position + mp.anchorA;
        mp.separation = separation;
        mp.id = 0;
        manifold.pointCount = 1;
        return manifold;
    }

    #endregion

    #region Circles

    // point = qA * localAnchorA + pA
    // localAnchorB = qBc * (point - pB)
    // anchorB = point - pB = qA * localAnchorA + pA - pB
    //         = anchorA + (pA - pB)
    private b2Manifold CollideCircles(PhysShapeCircle circleA, Transform xfA, PhysShapeCircle circleB, Transform xfB)
    {
        b2Manifold manifold = new();

        var xf = Physics.Transform.InvMulTransforms( xfA, xfB );

        var pointA = circleA.Position;
        var pointB = Physics.Transform.TransformPoint(xf, circleB.Position);

        float distance;
        var normal = b2GetLengthAndNormalize( &distance, b2Sub( pointB, pointA ) );

        float radiusA = circleA.Radius;
        float radiusB = circleB.Radius;

        float separation = distance - radiusA - radiusB;
        if (separation > PhysicsConstants.SpeculativeDistance)
        {
            return manifold;
        }

        var cA = Vector2Helpers.MulAdd( pointA, radiusA, normal );
        var cB = Vector2Helpers.MulAdd( pointB, -radiusB, normal );
        var contactPointA = Vector2.Lerp( cA, cB, 0.5f );

        manifold.normal = Quaternion2D.RotateVector( xfA.Quaternion2D, normal );
        var mp = manifold.points._00;
        mp.anchorA = Quaternion2D.RotateVector( xfA.Quaternion2D, contactPointA );
        mp.anchorB = mp.anchorA + xfA.Position - xfB.Position;
        mp.point = mp.anchorA + xfA.Position;
        mp.separation = separation;
        mp.id = 0;
        manifold.pointCount = 1;
        return manifold;
    }

    #endregion

    #region Polygons

    private b2Manifold CollidePolygonAndCircle(in Polygon polygonA, Transform xfA, PhysShapeCircle circleB, Transform xfB)
    {
	    b2Manifold manifold = new();
	    const float speculativeDistance = PhysicsConstants.SpeculativeDistance;

	    var xf = Physics.Transform.InvMulTransforms( xfA, xfB );

	    // Compute circle position in the frame of the polygon.
	    var center = Physics.Transform.TransformPoint(xf, circleB.Position);
	    float radiusA = polygonA.Radius;
	    float radiusB = circleB.Radius;
	    float radius = radiusA + radiusB;

	    // Find the min separating edge.
	    int normalIndex = 0;
	    float separation = float.MinValue;
	    int vertexCount = polygonA.VertexCount;
	    var vertices = polygonA._vertices.AsSpan;
	    var normals = polygonA._normals.AsSpan;

	    for ( int i = 0; i < vertexCount; ++i )
	    {
		    float s = Vector2.Dot(normals[i], center - vertices[i]);
		    if ( s > separation )
		    {
			    separation = s;
			    normalIndex = i;
		    }
	    }

	    if ( separation > radius + speculativeDistance )
	    {
		    return manifold;
	    }

	    // Vertices of the reference edge.
	    int vertIndex1 = normalIndex;
	    int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	    var v1 = vertices[vertIndex1];
	    var v2 = vertices[vertIndex2];

	    // Compute barycentric coordinates
	    float u1 = Vector2.Dot(center - v1, v2 - v1);
	    float u2 = Vector2.Dot(center - v2, v1 - v2);

	    if ( u1 < 0.0f && separation > float.Epsilon )
	    {
		    // Circle center is closest to v1 and safely outside the polygon
		    var normal = b2Normalize(center - v1);
		    separation = Vector2.Dot(center - v1, normal);
		    if ( separation > radius + speculativeDistance )
		    {
			    return manifold;
		    }

		    b2Vec2 cA = Vector2Helpers.MulAdd( v1, radiusA, normal );
		    b2Vec2 cB = Vector2Helpers.MulSub( center, radiusB, normal );
		    b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

		    manifold.normal = b2RotateVector( xfA.q, normal );
		    b2ManifoldPoint* mp = manifold.points + 0;
		    mp->anchorA = b2RotateVector( xfA.q, contactPointA );
		    mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		    mp->point = b2Add( xfA.p, mp->anchorA );
		    mp->separation = b2Dot( b2Sub( cB, cA ), normal );
		    mp->id = 0;
		    manifold.pointCount = 1;
	    }
	    else if ( u2 < 0.0f && separation > FLT_EPSILON )
	    {
		    // Circle center is closest to v2 and safely outside the polygon
		    b2Vec2 normal = b2Normalize( b2Sub( center, v2 ) );
		    separation = b2Dot( b2Sub( center, v2 ), normal );
		    if ( separation > radius + speculativeDistance )
		    {
			    return manifold;
		    }

		    b2Vec2 cA = b2MulAdd( v2, radiusA, normal );
		    b2Vec2 cB = b2MulSub( center, radiusB, normal );
		    b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

		    manifold.normal = b2RotateVector( xfA.q, normal );
		    b2ManifoldPoint* mp = manifold.points + 0;
		    mp->anchorA = b2RotateVector( xfA.q, contactPointA );
		    mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		    mp->point = b2Add( xfA.p, mp->anchorA );
		    mp->separation = b2Dot( b2Sub( cB, cA ), normal );
		    mp->id = 0;
		    manifold.pointCount = 1;
	    }
	    else
	    {
		    // Circle center is between v1 and v2. Center may be inside polygon
		    b2Vec2 normal = normals[normalIndex];
		    manifold.normal = b2RotateVector( xfA.q, normal );

		    // cA is the projection of the circle center onto to the reference edge
		    b2Vec2 cA = b2MulAdd( center, radiusA - b2Dot( b2Sub( center, v1 ), normal ), normal );

		    // cB is the deepest point on the circle with respect to the reference edge
		    b2Vec2 cB = b2MulSub( center, radiusB, normal );

		    b2Vec2 contactPointA = b2Lerp( cA, cB, 0.5f );

		    // The contact point is the midpoint in world space
		    b2ManifoldPoint* mp = manifold.points + 0;
		    mp->anchorA = b2RotateVector( xfA.q, contactPointA );
		    mp->anchorB = b2Add( mp->anchorA, b2Sub( xfA.p, xfB.p ) );
		    mp->point = b2Add( xfA.p, mp->anchorA );
		    mp->separation = separation - radius;
		    mp->id = 0;
		    manifold.pointCount = 1;
	    }

	    return manifold;
    }

    // Due to speculation, every polygon is rounded
    // Algorithm:
    //
    // compute edge separation using the separating axis test (SAT)
    // if (separation > speculation_distance)
    //   return
    // find reference and incident edge
    // if separation >= 0.1f * B2_LINEAR_SLOP
    //   compute closest points between reference and incident edge
    //   if vertices are closest
    //      single vertex-vertex contact
    //   else
    //      clip edges
    //   end
    // else
    //   clip edges
    // end
    private b2Manifold CollidePolygons(in Polygon polygonA, in Transform xfA, in Polygon polygonB, in Transform xfB, SimplexCache cache)
    {
        var origin = polygonA._vertices._00;
	    float linearSlop = PhysicsConstants.LinearSlop;
	    float speculativeDistance = PhysicsConstants.SpeculativeDistance;

	    // Shift polyA to origin
	    // pw = q * pb + p
	    // pw = q * (pbs + origin) + p
	    // pw = q * pbs + (p + q * origin)
	    var sfA = new Transform(xfA.Position + Quaternion2D.RotateVector(xfA.Quaternion2D, origin ), xfA.Quaternion2D);
	    var xf = Physics.Transform.InvMulTransforms(sfA, xfB);

	    Polygon localPolyA = new();

        localPolyA.Radius = polygonA.Radius;
        localPolyA._vertices._00 = Vector2.Zero;
        localPolyA._normals._00 = polygonA._normals._00;
        localPolyA.VertexCount = polygonA.VertexCount;

        var localVertsA = localPolyA._vertices.AsSpan;
        var localNormalsA = localPolyA._normals.AsSpan;

        var polyAVerts = polygonA._vertices.AsSpan;
        var polyANorms = polygonA._normals.AsSpan;

        for (var i = 1; i < localPolyA.VertexCount; i++)
        {
            localVertsA[i] = polyAVerts[i] - origin;
            localNormalsA[i] = polyANorms[i];
        }

	    // Put polyB in polyA's frame to reduce round-off error
        Polygon localPolyB = new();
        localPolyB.Radius = polygonB.Radius;
        localPolyB.VertexCount = polygonB.VertexCount;

        var localVertsB = localPolyB._vertices.AsSpan;
        var localNormalsB = localPolyB._normals.AsSpan;

        var polyBVerts = polygonB._vertices.AsSpan;
        var polyBNorms = polygonB._normals.AsSpan;

	    for ( int i = 0; i < localPolyB.VertexCount; ++i )
	    {
		    localVertsB[i] = Physics.Transform.TransformPoint(xf, polyBVerts[i]);
		    localNormalsB[i] = Quaternion2D.RotateVector(xf.Quaternion2D, polyBNorms[i]);
	    }

	    int edgeA = 0;
	    float separationA = FindMaxSeparation(ref edgeA, ref localPolyA, ref localPolyB);

	    int edgeB = 0;
	    float separationB = FindMaxSeparation(ref edgeB, ref localPolyB, ref localPolyA);

	    float radius = localPolyA.Radius + localPolyB.Radius;

	    if ( separationA > speculativeDistance + radius || separationB > speculativeDistance + radius )
	    {
		    return new b2Manifold();
	    }

	    // Find incident edge
	    bool flip;
	    if ( separationA >= separationB )
	    {
		    flip = false;

		    var searchDirection = localNormalsB[edgeA];

		    // Find the incident edge on polyB
		    int count = localPolyB.VertexCount;
            edgeB = 0;
		    float minDot = float.MaxValue;
		    for ( int i = 0; i < count; ++i )
		    {
			    float dot = Vector2.Dot(searchDirection, localNormalsB[i]);
			    if (dot < minDot)
			    {
				    minDot = dot;
				    edgeB = i;
			    }
		    }
	    }
	    else
	    {
		    flip = true;

		    var searchDirection = localNormalsB[edgeB];

		    // Find the incident edge on polyA
		    int count = localPolyA.VertexCount;
            edgeA = 0;
		    float minDot = float.MaxValue;
		    for ( int i = 0; i < count; ++i )
		    {
			    float dot = Vector2.Dot( searchDirection, localNormalsA[i] );
			    if ( dot < minDot )
			    {
				    minDot = dot;
				    edgeA = i;
			    }
		    }
	    }

	    b2Manifold manifold = new();
        var manPoints = manifold.points.AsSpan;

	    // Using slop here to ensure vertex-vertex normal vectors can be safely normalized
	    // todo this means edge clipping needs to handle slightly non-overlapping edges.
	    if ( separationA > 0.1f * linearSlop || separationB > 0.1f * linearSlop )
	    {
    #if true
		    // Edges are disjoint. Find closest points between reference edge and incident edge
		    // Reference edge on polygon A
		    int i11 = edgeA;
		    int i12 = edgeA + 1 < localPolyA.VertexCount ? edgeA + 1 : 0;
		    int i21 = edgeB;
		    int i22 = edgeB + 1 < localPolyB.VertexCount ? edgeB + 1 : 0;

		    var v11 = localVertsA[i11];
		    var v12 = localVertsA[i12];
		    var v21 = localVertsB[i21];
		    var v22 = localVertsB[i22];

		    SegmentDistanceResult result = b2SegmentDistance( v11, v12, v21, v22 );
		    DebugTools.Assert( result.distanceSquared > 0.0f );
		    float distance = MathF.Sqrt( result.distanceSquared );
		    float separation = distance - radius;

		    if ( distance - radius > speculativeDistance )
		    {
			    // This can happen in the vertex-vertex case
			    return manifold;
		    }

		    // Attempt to clip edges
		    manifold = ClipPolygons(in localPolyA, in localPolyB, edgeA, edgeB, flip );

		    float minSeparation = float.MaxValue;

		    for ( int i = 0; i < manifold.pointCount; ++i )
		    {
			    minSeparation = MathF.Min( minSeparation, manPoints[i].separation );
		    }

		    // Does vertex-vertex have substantially larger separation?
		    if ( separation + 0.1f * linearSlop < minSeparation )
		    {
			    if ( result.fraction1 == 0.0f && result.fraction2 == 0.0f )
			    {
				    // v11 - v21
				    var normal = v21 - v11;
				    float invDistance = 1.0f / distance;
				    normal.X *= invDistance;
				    normal.Y *= invDistance;

				    var c1 = Vector2Helpers.MulAdd( v11, localPolyA.Radius, normal );
				    var c2 = Vector2Helpers.MulAdd( v21, -localPolyB.Radius, normal );

				    manifold.normal = normal;
				    manifold.points._00.anchorA = Vector2.Lerp( c1, c2, 0.5f );
				    manifold.points._00.separation = distance - radius;
				    manifold.points._00.id = MakeId(i11, i21);
				    manifold.pointCount = 1;
			    }
			    else if ( result.fraction1 == 0.0f && result.fraction2 == 1.0f )
			    {
				    // v11 - v22
				    var normal = v22 - v11;
				    float invDistance = 1.0f / distance;
				    normal.X *= invDistance;
				    normal.Y *= invDistance;

				    var c1 = Vector2Helpers.MulAdd( v11, localPolyA.Radius, normal );
				    var c2 = Vector2Helpers.MulAdd( v22, -localPolyB.Radius, normal );

				    manifold.normal = normal;
				    manifold.points._00.anchorA = Vector2.Lerp( c1, c2, 0.5f );
				    manifold.points._00.separation = distance - radius;
				    manifold.points._00.id = MakeId( i11, i22 );
				    manifold.pointCount = 1;
			    }
			    else if ( result.fraction1 == 1.0f && result.fraction2 == 0.0f )
			    {
				    // v12 - v21
				    var normal = v21 - v12;
				    float invDistance = 1.0f / distance;
				    normal.X *= invDistance;
				    normal.Y *= invDistance;

				    var c1 = Vector2Helpers.MulAdd( v12, localPolyA.Radius, normal );
				    var c2 = Vector2Helpers.MulAdd( v21, -localPolyB.Radius, normal );

				    manifold.normal = normal;
				    manifold.points._00.anchorA = Vector2.Lerp( c1, c2, 0.5f );
				    manifold.points._00.separation = distance - radius;
				    manifold.points._00.id = MakeId( i12, i21 );
				    manifold.pointCount = 1;
			    }
			    else if ( result.fraction1 == 1.0f && result.fraction2 == 1.0f )
			    {
				    // v12 - v22
				    var normal = v22 - v12;
				    float invDistance = 1.0f / distance;
				    normal.X *= invDistance;
				    normal.Y *= invDistance;

				    var c1 = Vector2Helpers.MulAdd( v12, localPolyA.Radius, normal );
				    var c2 = Vector2Helpers.MulAdd( v22, -localPolyB.Radius, normal );

				    manifold.normal = normal;
				    manifold.points._00.anchorA = Vector2.Lerp( c1, c2, 0.5f );
				    manifold.points._00.separation = distance - radius;
				    manifold.points._00.id = MakeId( i12, i22 );
				    manifold.pointCount = 1;
			    }
		    }
    #else
		    // Polygons are disjoint. Find closest points between reference edge and incident edge
		    // Reference edge on polygon A
		    int i11 = edgeA;
		    int i12 = edgeA + 1 < localPolyA.count ? edgeA + 1 : 0;
		    int i21 = edgeB;
		    int i22 = edgeB + 1 < localPolyB.count ? edgeB + 1 : 0;

		    b2Vec2 v11 = localPolyA.vertices[i11];
		    b2Vec2 v12 = localPolyA.vertices[i12];
		    b2Vec2 v21 = localPolyB.vertices[i21];
		    b2Vec2 v22 = localPolyB.vertices[i22];

		    b2SegmentDistanceResult result = b2SegmentDistance( v11, v12, v21, v22 );

		    if ( result.fraction1 == 0.0f && result.fraction2 == 0.0f )
		    {
			    // v11 - v21
			    b2Vec2 normal = b2Sub( v21, v11 );
			    B2_ASSERT( result.distanceSquared > 0.0f );
			    float distance = sqrtf( result.distanceSquared );
			    if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			    {
				    return manifold;
			    }
			    float invDistance = 1.0f / distance;
			    normal.x *= invDistance;
			    normal.y *= invDistance;

			    b2Vec2 c1 = b2MulAdd( v11, localPolyA.radius, normal );
			    b2Vec2 c2 = b2MulAdd( v21, -localPolyB.radius, normal );

			    manifold.normal = normal;
			    manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			    manifold.points[0].separation = distance - radius;
			    manifold.points[0].id = B2_MAKE_ID( i11, i21 );
			    manifold.pointCount = 1;
		    }
		    else if ( result.fraction1 == 0.0f && result.fraction2 == 1.0f )
		    {
			    // v11 - v22
			    b2Vec2 normal = b2Sub( v22, v11 );
			    B2_ASSERT( result.distanceSquared > 0.0f );
			    float distance = sqrtf( result.distanceSquared );
			    if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			    {
				    return manifold;
			    }
			    float invDistance = 1.0f / distance;
			    normal.x *= invDistance;
			    normal.y *= invDistance;

			    b2Vec2 c1 = b2MulAdd( v11, localPolyA.radius, normal );
			    b2Vec2 c2 = b2MulAdd( v22, -localPolyB.radius, normal );

			    manifold.normal = normal;
			    manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			    manifold.points[0].separation = distance - radius;
			    manifold.points[0].id = B2_MAKE_ID( i11, i22 );
			    manifold.pointCount = 1;
		    }
		    else if ( result.fraction1 == 1.0f && result.fraction2 == 0.0f )
		    {
			    // v12 - v21
			    b2Vec2 normal = b2Sub( v21, v12 );
			    B2_ASSERT( result.distanceSquared > 0.0f );
			    float distance = sqrtf( result.distanceSquared );
			    if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			    {
				    return manifold;
			    }
			    float invDistance = 1.0f / distance;
			    normal.x *= invDistance;
			    normal.y *= invDistance;

			    b2Vec2 c1 = b2MulAdd( v12, localPolyA.radius, normal );
			    b2Vec2 c2 = b2MulAdd( v21, -localPolyB.radius, normal );

			    manifold.normal = normal;
			    manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			    manifold.points[0].separation = distance - radius;
			    manifold.points[0].id = B2_MAKE_ID( i12, i21 );
			    manifold.pointCount = 1;
		    }
		    else if ( result.fraction1 == 1.0f && result.fraction2 == 1.0f )
		    {
			    // v12 - v22
			    b2Vec2 normal = b2Sub( v22, v12 );
			    B2_ASSERT( result.distanceSquared > 0.0f );
			    float distance = sqrtf( result.distanceSquared );
			    if ( distance > B2_SPECULATIVE_DISTANCE + radius )
			    {
				    return manifold;
			    }
			    float invDistance = 1.0f / distance;
			    normal.x *= invDistance;
			    normal.y *= invDistance;

			    b2Vec2 c1 = b2MulAdd( v12, localPolyA.radius, normal );
			    b2Vec2 c2 = b2MulAdd( v22, -localPolyB.radius, normal );

			    manifold.normal = normal;
			    manifold.points[0].anchorA = b2Lerp( c1, c2, 0.5f );
			    manifold.points[0].separation = distance - radius;
			    manifold.points[0].id = B2_MAKE_ID( i12, i22 );
			    manifold.pointCount = 1;
		    }
		    else
		    {
			    // Edge region
			    manifold = b2ClipPolygons( &localPolyA, &localPolyB, edgeA, edgeB, flip );
		    }
    #endif
	    }
	    else
	    {
		    // Polygons overlap
		    manifold = ClipPolygons(in localPolyA, in localPolyB, edgeA, edgeB, flip);
	    }

	    // Convert manifold to world space
	    if ( manifold.pointCount > 0 )
	    {
		    manifold.normal = Quaternion2D.RotateVector(xfA.Quaternion2D, manifold.normal);
		    for ( int i = 0; i < manifold.pointCount; ++i )
		    {
			    ref var mp = ref manPoints[i];

			    // anchor points relative to shape origin in world space
			    mp.anchorA = Quaternion2D.RotateVector(xfA.Quaternion2D, mp.anchorA + origin);
			    mp.anchorB = mp.anchorA + xfA.Position - xfB.Position;
			    mp.point = xfA.Position + mp.anchorA;
		    }
	    }

	    return manifold;
    }

    // Polygon clipper used to compute contact points when there are potentially two contact points.
    private b2Manifold ClipPolygons(in Polygon polyA, in Polygon polyB, int edgeA, int edgeB, bool flip )
    {
        b2Manifold manifold = new();

	    // reference polygon
	    Polygon poly1;
	    int i11, i12;

	    // incident polygon
        Polygon poly2;
	    int i21, i22;

	    if ( flip )
	    {
		    poly1 = polyB;
		    poly2 = polyA;
		    i11 = edgeB;
		    i12 = edgeB + 1 < polyB.VertexCount ? edgeB + 1 : 0;
		    i21 = edgeA;
		    i22 = edgeA + 1 < polyA.VertexCount ? edgeA + 1 : 0;
	    }
	    else
	    {
		    poly1 = polyA;
		    poly2 = polyB;
		    i11 = edgeA;
		    i12 = edgeA + 1 < polyA.VertexCount ? edgeA + 1 : 0;
		    i21 = edgeB;
		    i22 = edgeB + 1 < polyB.VertexCount ? edgeB + 1 : 0;
	    }

        var poly1Verts = poly1._vertices.AsSpan;
        var poly2Verts = poly2._vertices.AsSpan;
        var poly1Norms = poly1._normals.AsSpan;
        var poly2Norms = poly2._normals.AsSpan;

	    var normal = poly1Norms[i11];

	    // Reference edge vertices
	    var v11 = poly1Verts[i11];
	    var v12 = poly1Verts[i12];

	    // Incident edge vertices
	    var v21 = poly2Verts[i21];
	    var v22 = poly2Verts[i22];

	    var tangent = Vector2Helpers.Cross( 1.0f, normal );

	    float lower1 = 0.0f;
	    float upper1 = Vector2.Dot(v12 - v11, tangent);

	    // Incident edge points opposite of tangent due to CCW winding
	    float upper2 = Vector2.Dot(v21 - v11, tangent);
	    float lower2 = Vector2.Dot(v22 - v11, tangent);

	    // Are the segments disjoint?
	    if ( upper2 < lower1 || upper1 < lower2 )
	    {
		    return manifold;
	    }

	    Vector2 vLower;
	    if ( lower2 < lower1 && upper2 - lower2 > float.Epsilon )
	    {
		    vLower = Vector2.Lerp( v22, v21, ( lower1 - lower2 ) / ( upper2 - lower2 ) );
	    }
	    else
	    {
		    vLower = v22;
	    }

	    Vector2 vUpper;
	    if ( upper2 > upper1 && upper2 - lower2 > float.Epsilon )
	    {
		    vUpper = Vector2.Lerp( v22, v21, ( upper1 - lower2 ) / ( upper2 - lower2 ) );
	    }
	    else
	    {
		    vUpper = v21;
	    }

	    // todo vLower can be very close to vUpper, reduce to one point?

	    float separationLower = Vector2.Dot(vLower - v11, normal);
	    float separationUpper = Vector2.Dot(vUpper - v11, normal);

	    float r1 = poly1.Radius;
	    float r2 = poly2.Radius;

	    // Put contact points at midpoint, accounting for radii
	    vLower = Vector2Helpers.MulAdd( vLower, 0.5f * ( r1 - r2 - separationLower ), normal );
	    vUpper = Vector2Helpers.MulAdd( vUpper, 0.5f * ( r1 - r2 - separationUpper ), normal );

	    float radius = r1 + r2;

	    if ( flip == false )
	    {
		    manifold.normal = normal;
		    ref var cp = ref manifold.points._00;

		    {
			    cp.anchorA = vLower;
			    cp.separation = separationLower - radius;
			    cp.id = MakeId( i11, i22 );
			    manifold.pointCount += 1;
		    }

            ref var cp2 = ref manifold.points._01;
		    {

			    cp2.anchorA = vUpper;
			    cp.separation = separationUpper - radius;
			    cp.id = MakeId( i12, i21 );
			    manifold.pointCount += 1;
		    }
	    }
	    else
	    {
		    manifold.normal = -normal;
		    ref var cp = ref manifold.points._00;

		    {
			    cp.anchorA = vUpper;
			    cp.separation = separationUpper - radius;
			    cp.id = MakeId( i21, i12 );
			    manifold.pointCount += 1;
		    }

            ref var cp2 = ref manifold.points._01;
		    {
			    cp2.anchorA = vLower;
			    cp2.separation = separationLower - radius;
			    cp2.id = MakeId( i22, i11 );
			    manifold.pointCount += 1;
		    }
	    }

	    return manifold;
    }

    // Find the max separation between poly1 and poly2 using edge normals from poly1.
    private float FindMaxSeparation(ref int edgeIndex, ref Polygon poly1, ref Polygon poly2)
    {
        int count1 = poly1.VertexCount;
        int count2 = poly2.VertexCount;
        var n1s = poly1._normals.AsSpan;
        var v1s = poly1._vertices.AsSpan;
        var v2s = poly2._vertices.AsSpan;

        int bestIndex = 0;
        float maxSeparation = float.MinValue;
        for ( int i = 0; i < count1; ++i )
        {
            // Get poly1 normal in frame2.
            var n = n1s[i];
            var v1 = v1s[i];

            // Find the deepest point for normal i.
            float si = float.MaxValue;
            for ( int j = 0; j < count2; ++j )
            {
                float sij = Vector2.Dot( n, v2s[j] - v1);
                if ( sij < si )
                {
                    si = sij;
                }
            }

            if ( si > maxSeparation )
            {
                maxSeparation = si;
                bestIndex = i;
            }
        }

        edgeIndex = bestIndex;
        return maxSeparation;
    }

    /// Follows Ericson 5.1.9 Closest Points of Two Line Segments
    private SegmentDistanceResult b2SegmentDistance(Vector2 p1, Vector2 q1, Vector2 p2, Vector2 q2)
    {
        SegmentDistanceResult result = new();

        var d1 = q1 - p1;
        var d2 = q2 - p2;
        var r = p1 - p2;
        float dd1 = Vector2.Dot(d1, d1);
        float dd2 = Vector2.Dot(d2, d2);
        float rd1 = Vector2.Dot(r, d1);
        float rd2 = Vector2.Dot(r, d2);

        const float epsSqr = float.Epsilon * float.Epsilon;

        if ( dd1 < epsSqr || dd2 < epsSqr )
        {
            // Handle all degeneracies
            if ( dd1 >= epsSqr )
            {
                // Segment 2 is degenerate
                result.fraction1 = Math.Clamp(-rd1 / dd1, 0.0f, 1.0f);
                result.fraction2 = 0.0f;
            }
            else if ( dd2 >= epsSqr )
            {
                // Segment 1 is degenerate
                result.fraction1 = 0.0f;
                result.fraction2 = Math.Clamp(rd2 / dd2, 0.0f, 1.0f);
            }
            else
            {
                result.fraction1 = 0.0f;
                result.fraction2 = 0.0f;
            }
        }
        else
        {
            // Non-degenerate segments
            float d12 = Vector2.Dot(d1, d2);

            float denominator = dd1 * dd2 - d12 * d12;

            // Fraction on segment 1
            float f1 = 0.0f;
            if ( denominator != 0.0f )
            {
                // not parallel
                f1 = Math.Clamp( ( d12 * rd2 - rd1 * dd2 ) / denominator, 0.0f, 1.0f );
            }

            // Compute point on segment 2 closest to p1 + f1 * d1
            float f2 = ( d12 * f1 + rd2 ) / dd2;

            // Clamping of segment 2 requires a do over on segment 1
            if ( f2 < 0.0f )
            {
                f2 = 0.0f;
                f1 = Math.Clamp( -rd1 / dd1, 0.0f, 1.0f );
            }
            else if ( f2 > 1.0f )
            {
                f2 = 1.0f;
                f1 = Math.Clamp( ( d12 - rd1 ) / dd1, 0.0f, 1.0f );
            }

            result.fraction1 = f1;
            result.fraction2 = f2;
        }

        result.closest1 = Vector2Helpers.MulAdd(p1, result.fraction1, d1);
        result.closest2 = Vector2Helpers.MulAdd(p2, result.fraction2, d2);
        result.distanceSquared = (result.closest1 - result.closest2).LengthSquared();
        return result;
    }

    #endregion
}
