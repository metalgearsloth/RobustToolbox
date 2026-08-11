using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Lighting;

/// <summary>
/// Deterministic CPU reference helpers for the experimental 2D GI paths.
/// The radiance-cascade implementation intentionally models directional interval samples
/// instead of mirroring incidental GPU texture behaviour.
/// </summary>
internal static class GlobalIlluminationReference
{
    public const int RadianceCascadeSpatialBranchFactor = 2;
    public const int RadianceCascadeAngularBranchFactor = 4;

    public readonly record struct RadianceSample(Vector3 Radiance, float Transmittance)
    {
        public static RadianceSample Transparent => new(Vector3.Zero, 1f);

        public static RadianceSample Merge(RadianceSample near, RadianceSample far)
        {
            return new RadianceSample(
                near.Radiance + near.Transmittance * far.Radiance,
                near.Transmittance * far.Transmittance);
        }
    }

    public readonly record struct TraceResult(bool Hit, Vector2i Cell, Vector3 Radiance, float Transmittance, int Steps);

    public readonly record struct ErrorMetrics(float MeanAbsoluteError, float RootMeanSquaredError, float MaxAbsoluteError, float MeanReferenceMagnitude);

    public sealed class RadianceCascadeResult
    {
        public RadianceCascadeResult(
            Vector3[] current,
            RadianceSample[][] cascades,
            Vector2i[] probeSizes,
            Vector2i[] atlasSizes,
            int[] directionCounts,
            float[] intervalStarts,
            float[] intervalEnds)
        {
            Current = current;
            Cascades = cascades;
            ProbeSizes = probeSizes;
            AtlasSizes = atlasSizes;
            DirectionCounts = directionCounts;
            IntervalStarts = intervalStarts;
            IntervalEnds = intervalEnds;
        }

        public Vector3[] Current { get; }
        public RadianceSample[][] Cascades { get; }
        public Vector2i[] ProbeSizes { get; }
        public Vector2i[] AtlasSizes { get; }
        public int[] DirectionCounts { get; }
        public float[] IntervalStarts { get; }
        public float[] IntervalEnds { get; }
    }

    public static Vector2i ScaledTargetSize(Vector2i viewportSize, float scale)
    {
        var clampedScale = Math.Clamp(scale, 0.05f, 1f);
        return new Vector2i(
            Math.Max(1, (int)Math.Ceiling(viewportSize.X * clampedScale)),
            Math.Max(1, (int)Math.Ceiling(viewportSize.Y * clampedScale)));
    }

    public static Matrix3x2 CreateScreenUvToWorldMatrix(
        Vector2 worldTopLeft,
        Vector2 worldTopRight,
        Vector2 worldBottomLeft)
    {
        // Clyde's fullscreen passes use pos = (clip + 1) / 2, so uv.y = 1 is the top of the
        // viewport and uv.y = 0 is the bottom. Keep this convention explicit so GI buffers line
        // up with the normal light-map sampling path.
        var worldX = worldTopRight - worldTopLeft;
        var worldY = worldTopLeft - worldBottomLeft;

        return new Matrix3x2(
            worldX.X,
            worldX.Y,
            worldY.X,
            worldY.Y,
            worldBottomLeft.X,
            worldBottomLeft.Y);
    }

    public static Vector2i RadianceCascadeProbeSize(Vector2i baseSize, int cascade)
    {
        var divisor = 1 << Math.Clamp(cascade, 0, 12);
        return new Vector2i(
            Math.Max(1, (baseSize.X + divisor - 1) / divisor),
            Math.Max(1, (baseSize.Y + divisor - 1) / divisor));
    }

    public static Vector2i RadianceCascadeDirectionGrid(int directionCount)
    {
        directionCount = Math.Max(1, directionCount);
        var columns = (int)Math.Ceiling(Math.Sqrt(directionCount));
        var rows = (directionCount + columns - 1) / columns;
        return new Vector2i(columns, rows);
    }

    public static Vector2i RadianceCascadeAtlasSize(Vector2i baseSize, int cascade, int baseRays)
    {
        var probeSize = RadianceCascadeProbeSize(baseSize, cascade);
        var directionGrid = RadianceCascadeDirectionGrid(RadianceCascadeRayCount(baseRays, cascade));
        return new Vector2i(probeSize.X * directionGrid.X, probeSize.Y * directionGrid.Y);
    }

    public static int RadianceCascadeRayCount(int baseRays, int cascade)
    {
        if (baseRays <= 0)
            throw new ArgumentOutOfRangeException(nameof(baseRays));

        var result = baseRays;
        for (var i = 0; i < cascade; i++)
            result *= RadianceCascadeAngularBranchFactor;

        return Math.Clamp(result, 1, 4096);
    }

    public static int RadianceCascadeChildDirectionIndex(int parentDirectionIndex, int child)
    {
        if (parentDirectionIndex < 0)
            throw new ArgumentOutOfRangeException(nameof(parentDirectionIndex));

        if (child < 0 || child >= RadianceCascadeAngularBranchFactor)
            throw new ArgumentOutOfRangeException(nameof(child));

        return parentDirectionIndex * RadianceCascadeAngularBranchFactor + child;
    }

    public static float RadianceCascadeDirectionAngle(int directionIndex, int directionCount)
    {
        if (directionCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(directionCount));

        if (directionIndex < 0 || directionIndex >= directionCount)
            throw new ArgumentOutOfRangeException(nameof(directionIndex));

        return (directionIndex + 0.5f) / directionCount * MathF.Tau;
    }

    public static Vector2 RadianceCascadeDirection(int directionIndex, int directionCount)
    {
        var angle = RadianceCascadeDirectionAngle(directionIndex, directionCount);
        return new Vector2(MathF.Cos(angle), MathF.Sin(angle));
    }

    public static float RadianceCascadeIntervalStart(float baseIntervalLength, int cascade)
    {
        if (baseIntervalLength <= 0f)
            throw new ArgumentOutOfRangeException(nameof(baseIntervalLength));

        if (cascade < 0)
            throw new ArgumentOutOfRangeException(nameof(cascade));

        return baseIntervalLength * (MathF.Pow(RadianceCascadeAngularBranchFactor, cascade) - 1f) /
               (RadianceCascadeAngularBranchFactor - 1f);
    }

    public static float RadianceCascadeIntervalEnd(float baseIntervalLength, int cascade)
    {
        return RadianceCascadeIntervalStart(baseIntervalLength, cascade + 1);
    }

    public static Vector2i?[] ExactNearestField(
        ReadOnlySpan<bool> occlusionMask,
        int width,
        int height,
        Vector2? cellWorldSize = null)
    {
        ValidateMask(occlusionMask, width, height);
        var cellSize = cellWorldSize ?? Vector2.One;
        var result = new Vector2i?[width * height];

        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                var bestDistance = float.PositiveInfinity;
                Vector2i? best = null;

                for (var sy = 0; sy < height; sy++)
                {
                    for (var sx = 0; sx < width; sx++)
                    {
                        if (!occlusionMask[Index(sx, sy, width)])
                            continue;

                        var distance = AspectDistanceSquared(new Vector2i(sx, sy), x, y, cellSize);
                        if (distance >= bestDistance)
                            continue;

                        bestDistance = distance;
                        best = new Vector2i(sx, sy);
                    }
                }

                result[Index(x, y, width)] = best;
            }
        }

        return result;
    }

    public static Vector2i?[] JumpFloodNearestField(
        ReadOnlySpan<bool> occlusionMask,
        int width,
        int height,
        Vector2? cellWorldSize = null)
    {
        ValidateMask(occlusionMask, width, height);
        var cellSize = cellWorldSize ?? Vector2.One;
        var source = new Vector2i?[width * height];
        var destination = new Vector2i?[width * height];

        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                if (occlusionMask[Index(x, y, width)])
                    source[Index(x, y, width)] = new Vector2i(x, y);
            }
        }

        for (var step = InitialJumpFloodStep(Math.Max(width, height)); step > 0; step /= 2)
        {
            for (var y = 0; y < height; y++)
            {
                for (var x = 0; x < width; x++)
                {
                    var best = source[Index(x, y, width)];
                    var bestDistance = best is { } seed
                        ? AspectDistanceSquared(seed, x, y, cellSize)
                        : float.PositiveInfinity;

                    for (var oy = -1; oy <= 1; oy++)
                    {
                        for (var ox = -1; ox <= 1; ox++)
                        {
                            var sx = x + ox * step;
                            var sy = y + oy * step;
                            if (!Inside(sx, sy, width, height))
                                continue;

                            var candidate = source[Index(sx, sy, width)];
                            if (candidate == null)
                                continue;

                            var distance = AspectDistanceSquared(candidate.Value, x, y, cellSize);
                            if (distance >= bestDistance)
                                continue;

                            bestDistance = distance;
                            best = candidate;
                        }
                    }

                    destination[Index(x, y, width)] = best;
                }
            }

            (source, destination) = (destination, source);
            Array.Clear(destination);
        }

        return source;
    }

    public static float[] DistanceField(
        ReadOnlySpan<Vector2i?> nearestField,
        int width,
        int height,
        Vector2? cellWorldSize = null)
    {
        if (nearestField.Length != width * height)
            throw new ArgumentException("Nearest field dimensions do not match.", nameof(nearestField));

        var cellSize = cellWorldSize ?? Vector2.One;
        var result = new float[width * height];
        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                var nearest = nearestField[Index(x, y, width)];
                result[Index(x, y, width)] = nearest == null
                    ? float.PositiveInfinity
                    : MathF.Sqrt(AspectDistanceSquared(nearest.Value, x, y, cellSize));
            }
        }

        return result;
    }

    public static TraceResult TraceRay(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        Vector2 start,
        Vector2 direction,
        int maxSteps,
        float bounceDecay)
    {
        ValidateMask(occlusionMask, width, height);
        ValidateRadiance(directRadiance, width, height, nameof(directRadiance));
        ValidateRadiance(previousGi, width, height, nameof(previousGi));
        ValidateFov(fovMask, width, height);

        var source = new Vector3[width * height];
        for (var i = 0; i < source.Length; i++)
            source[i] = directRadiance[i] + previousGi[i];

        var sample = TraceDirectInterval(
            occlusionMask,
            source,
            fovMask,
            width,
            height,
            start,
            Vector2.Normalize(direction),
            0.25f,
            maxSteps,
            0.25f,
            out var hit,
            out var hitCell,
            out var steps);

        return new TraceResult(hit, hitCell, sample.Radiance * bounceDecay, sample.Transmittance, steps);
    }

    public static Vector3 TracePixel(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        Vector2 pixel,
        int rays,
        int maxSteps,
        float bounceDecay)
    {
        if (rays <= 0)
            throw new ArgumentOutOfRangeException(nameof(rays));

        ValidateMask(occlusionMask, width, height);
        ValidateRadiance(directRadiance, width, height, nameof(directRadiance));
        ValidateRadiance(previousGi, width, height, nameof(previousGi));
        ValidateFov(fovMask, width, height);

        var gathered = Vector3.Zero;
        for (var i = 0; i < rays; i++)
        {
            var direction = RadianceCascadeDirection(i, rays);
            gathered += TraceRay(
                occlusionMask,
                directRadiance,
                previousGi,
                fovMask,
                width,
                height,
                pixel,
                direction,
                maxSteps,
                bounceDecay).Radiance;
        }

        return gathered / rays;
    }

    public static Vector3[] TraceImage(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        int rays,
        int maxSteps,
        float bounceDecay,
        bool useJumpFloodNearest = true)
    {
        var result = new Vector3[width * height];
        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                if (occlusionMask[Index(x, y, width)])
                    continue;

                result[Index(x, y, width)] = TracePixel(
                    occlusionMask,
                    directRadiance,
                    previousGi,
                    fovMask,
                    width,
                    height,
                    new Vector2(x + 0.5f, y + 0.5f),
                    rays,
                    maxSteps,
                    bounceDecay);
            }
        }

        return result;
    }

    public static Vector3[] TraceBruteForceImage(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        int rays,
        float maxDistance,
        float stepLength,
        float bounceDecay)
    {
        if (rays <= 0)
            throw new ArgumentOutOfRangeException(nameof(rays));

        ValidateMask(occlusionMask, width, height);
        ValidateRadiance(directRadiance, width, height, nameof(directRadiance));
        ValidateFov(fovMask, width, height);

        var result = new Vector3[width * height];
        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                if (occlusionMask[Index(x, y, width)])
                    continue;

                var start = new Vector2(x + 0.5f, y + 0.5f);
                var gathered = Vector3.Zero;
                for (var directionIndex = 0; directionIndex < rays; directionIndex++)
                {
                    var direction = RadianceCascadeDirection(directionIndex, rays);
                    var sample = TraceDirectInterval(
                        occlusionMask,
                        directRadiance,
                        fovMask,
                        width,
                        height,
                        start,
                        direction,
                        stepLength,
                        maxDistance,
                        stepLength,
                        out _,
                        out _,
                        out _);
                    gathered += sample.Radiance;
                }

                result[Index(x, y, width)] = gathered / rays * bounceDecay;
            }
        }

        return result;
    }

    public static RadianceCascadeResult TraceRadianceCascades(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        int cascadeCount,
        int baseRays,
        float baseIntervalLength,
        float traceStepLength,
        float bounceDecay)
    {
        if (cascadeCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(cascadeCount));

        if (baseRays <= 0)
            throw new ArgumentOutOfRangeException(nameof(baseRays));

        if (baseIntervalLength <= 0f)
            throw new ArgumentOutOfRangeException(nameof(baseIntervalLength));

        if (traceStepLength <= 0f)
            throw new ArgumentOutOfRangeException(nameof(traceStepLength));

        ValidateMask(occlusionMask, width, height);
        ValidateRadiance(directRadiance, width, height, nameof(directRadiance));
        ValidateRadiance(previousGi, width, height, nameof(previousGi));
        ValidateFov(fovMask, width, height);

        cascadeCount = Math.Clamp(cascadeCount, 1, 5);
        var cascades = new RadianceSample[cascadeCount][];
        var probeSizes = new Vector2i[cascadeCount];
        var atlasSizes = new Vector2i[cascadeCount];
        var directionCounts = new int[cascadeCount];
        var intervalStarts = new float[cascadeCount];
        var intervalEnds = new float[cascadeCount];
        var baseSize = new Vector2i(width, height);

        for (var cascade = 0; cascade < cascadeCount; cascade++)
        {
            probeSizes[cascade] = RadianceCascadeProbeSize(baseSize, cascade);
            directionCounts[cascade] = RadianceCascadeRayCount(baseRays, cascade);
            atlasSizes[cascade] = RadianceCascadeAtlasSize(baseSize, cascade, baseRays);
            intervalStarts[cascade] = RadianceCascadeIntervalStart(baseIntervalLength, cascade);
            intervalEnds[cascade] = RadianceCascadeIntervalEnd(baseIntervalLength, cascade);
            cascades[cascade] = new RadianceSample[probeSizes[cascade].X * probeSizes[cascade].Y * directionCounts[cascade]];
        }

        for (var cascade = cascadeCount - 1; cascade >= 0; cascade--)
        {
            var probeSize = probeSizes[cascade];
            var directionCount = directionCounts[cascade];
            var intervalStart = intervalStarts[cascade];
            var intervalEnd = intervalEnds[cascade];

            for (var py = 0; py < probeSize.Y; py++)
            {
                for (var px = 0; px < probeSize.X; px++)
                {
                    var world = ProbeWorldPosition(px, py, probeSize, width, height);
                    var worldCell = ToCell(world, width, height);
                    if (worldCell != null && occlusionMask[Index(worldCell.Value.X, worldCell.Value.Y, width)])
                        continue;

                    for (var directionIndex = 0; directionIndex < directionCount; directionIndex++)
                    {
                        var direction = RadianceCascadeDirection(directionIndex, directionCount);
                        var near = TraceDirectInterval(
                            occlusionMask,
                            directRadiance,
                            fovMask,
                            width,
                            height,
                            world,
                            direction,
                            intervalStart,
                            intervalEnd,
                            traceStepLength,
                            out _,
                            out _,
                            out _);

                        var merged = near;
                        if (cascade + 1 < cascadeCount)
                        {
                            var farWorld = world + direction * intervalEnd;
                            var far = new RadianceSample(Vector3.Zero, 0f);
                            for (var child = 0; child < RadianceCascadeAngularBranchFactor; child++)
                            {
                                var childDirection = RadianceCascadeChildDirectionIndex(directionIndex, child);
                                far = Add(far, SampleCascadeBilinear(
                                    cascades[cascade + 1],
                                    probeSizes[cascade + 1],
                                    directionCounts[cascade + 1],
                                    occlusionMask,
                                    width,
                                    height,
                                    farWorld,
                                    childDirection));
                            }

                            far = Scale(far, 1f / RadianceCascadeAngularBranchFactor);
                            merged = RadianceSample.Merge(near, far);
                        }

                        cascades[cascade][DirectionalIndex(px, py, directionIndex, probeSize, directionCount)] = merged;
                    }
                }
            }
        }

        var current = ResolveCascadeZero(cascades[0], probeSizes[0], directionCounts[0], bounceDecay);
        return new RadianceCascadeResult(current, cascades, probeSizes, atlasSizes, directionCounts, intervalStarts, intervalEnds);
    }

    public static ErrorMetrics CompareImages(ReadOnlySpan<Vector3> actual, ReadOnlySpan<Vector3> reference, ReadOnlySpan<bool> occlusionMask)
    {
        if (actual.Length != reference.Length)
            throw new ArgumentException("Image dimensions do not match.", nameof(reference));

        if (occlusionMask.Length != 0 && occlusionMask.Length != actual.Length)
            throw new ArgumentException("Occlusion mask dimensions do not match.", nameof(occlusionMask));

        var count = 0;
        var absoluteSum = 0f;
        var squaredSum = 0f;
        var max = 0f;
        var referenceSum = 0f;

        for (var i = 0; i < actual.Length; i++)
        {
            if (occlusionMask.Length != 0 && occlusionMask[i])
                continue;

            var diff = actual[i] - reference[i];
            var abs = (MathF.Abs(diff.X) + MathF.Abs(diff.Y) + MathF.Abs(diff.Z)) / 3f;
            absoluteSum += abs;
            squaredSum += diff.LengthSquared() / 3f;
            max = MathF.Max(max, MathF.Max(MathF.Abs(diff.X), MathF.Max(MathF.Abs(diff.Y), MathF.Abs(diff.Z))));
            referenceSum += reference[i].Length();
            count++;
        }

        if (count == 0)
            return new ErrorMetrics(0f, 0f, 0f, 0f);

        return new ErrorMetrics(
            absoluteSum / count,
            MathF.Sqrt(squaredSum / count),
            max,
            referenceSum / count);
    }

    public static bool ShouldRejectHistory(
        Vector2 previousEyePosition,
        Vector2 currentEyePosition,
        Vector2 previousZoom,
        Vector2 currentZoom,
        Angle previousRotation,
        Angle currentRotation,
        int previousSettingsVersion,
        int currentSettingsVersion,
        ulong previousOcclusionHash,
        ulong currentOcclusionHash,
        float maxCameraDelta = 4f,
        float maxZoomDeltaSquared = 0.01f,
        double maxRotationDelta = Math.PI / 18.0)
    {
        if ((previousEyePosition - currentEyePosition).LengthSquared() > maxCameraDelta * maxCameraDelta)
            return true;

        if ((previousZoom - currentZoom).LengthSquared() > maxZoomDeltaSquared)
            return true;

        if (Math.Abs(Angle.ShortestDistance(previousRotation, currentRotation).Theta) > maxRotationDelta)
            return true;

        if (previousSettingsVersion != currentSettingsVersion)
            return true;

        return previousOcclusionHash != currentOcclusionHash;
    }

    public static Vector2 ProbeWorldPosition(int probeX, int probeY, Vector2i probeSize, int width, int height)
    {
        return new Vector2(
            (probeX + 0.5f) / probeSize.X * width,
            (probeY + 0.5f) / probeSize.Y * height);
    }

    public static int DirectionalIndex(int probeX, int probeY, int directionIndex, Vector2i probeSize, int directionCount)
    {
        return (probeY * probeSize.X + probeX) * directionCount + directionIndex;
    }

    private static Vector3[] ResolveCascadeZero(ReadOnlySpan<RadianceSample> cascade, Vector2i probeSize, int directionCount, float bounceDecay)
    {
        var result = new Vector3[probeSize.X * probeSize.Y];
        for (var y = 0; y < probeSize.Y; y++)
        {
            for (var x = 0; x < probeSize.X; x++)
            {
                var radiance = Vector3.Zero;
                for (var d = 0; d < directionCount; d++)
                    radiance += cascade[DirectionalIndex(x, y, d, probeSize, directionCount)].Radiance;

                result[Index(x, y, probeSize.X)] = radiance / directionCount * bounceDecay;
            }
        }

        return result;
    }

    private static RadianceSample TraceDirectInterval(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        Vector2 start,
        Vector2 direction,
        float intervalStart,
        float intervalEnd,
        float stepLength,
        out bool hit,
        out Vector2i hitCell,
        out int steps)
    {
        hit = false;
        hitCell = default;
        steps = 0;
        var radiance = Vector3.Zero;

        for (var traveled = stepLength; traveled < intervalEnd; traveled += stepLength)
        {
            steps++;
            var position = start + direction * traveled;
            var cell = ToCell(position, width, height);
            if (cell == null)
                return new RadianceSample(radiance, 1f);

            var idx = Index(cell.Value.X, cell.Value.Y, width);
            if (occlusionMask[idx])
            {
                hit = true;
                hitCell = cell.Value;
                return new RadianceSample(radiance, 0f);
            }

            if (traveled >= intervalStart && (fovMask.Length == 0 || fovMask[idx]))
                radiance += directRadiance[idx] * (stepLength / (1f + traveled * 0.35f));
        }

        return new RadianceSample(radiance, 1f);
    }

    private static RadianceSample SampleCascadeBilinear(
        ReadOnlySpan<RadianceSample> cascade,
        Vector2i probeSize,
        int directionCount,
        ReadOnlySpan<bool> occlusionMask,
        int width,
        int height,
        Vector2 world,
        int directionIndex)
    {
        if (directionIndex < 0 || directionIndex >= directionCount)
            return RadianceSample.Transparent;

        var uv = new Vector2(world.X / width, world.Y / height);
        if (uv.X < 0f || uv.Y < 0f || uv.X > 1f || uv.Y > 1f)
            return RadianceSample.Transparent;

        var probeCoord = uv * new Vector2(probeSize.X, probeSize.Y) - new Vector2(0.5f);
        var baseX = MathF.Floor(probeCoord.X);
        var baseY = MathF.Floor(probeCoord.Y);
        var fracX = probeCoord.X - baseX;
        var fracY = probeCoord.Y - baseY;
        var total = new RadianceSample(Vector3.Zero, 0f);

        for (var oy = 0; oy <= 1; oy++)
        {
            for (var ox = 0; ox <= 1; ox++)
            {
                var x = Math.Clamp((int)baseX + ox, 0, probeSize.X - 1);
                var y = Math.Clamp((int)baseY + oy, 0, probeSize.Y - 1);
                var wx = ox == 0 ? 1f - fracX : fracX;
                var wy = oy == 0 ? 1f - fracY : fracY;
                var weight = wx * wy;
                var probeWorld = ProbeWorldPosition(x, y, probeSize, width, height);
                var sample = SegmentBlocked(occlusionMask, width, height, world, probeWorld, 0.25f)
                    ? new RadianceSample(Vector3.Zero, 0f)
                    : cascade[DirectionalIndex(x, y, directionIndex, probeSize, directionCount)];
                total = Add(total, Scale(sample, weight));
            }
        }

        return total;
    }

    private static bool SegmentBlocked(
        ReadOnlySpan<bool> occlusionMask,
        int width,
        int height,
        Vector2 start,
        Vector2 end,
        float stepLength)
    {
        var delta = end - start;
        var length = delta.Length();
        if (length <= 0.0001f)
            return false;

        var direction = delta / length;
        for (var traveled = stepLength; traveled < length; traveled += stepLength)
        {
            var cell = ToCell(start + direction * traveled, width, height);
            if (cell == null)
                return true;

            if (occlusionMask[Index(cell.Value.X, cell.Value.Y, width)])
                return true;
        }

        return false;
    }

    private static RadianceSample Add(RadianceSample a, RadianceSample b)
    {
        return new RadianceSample(a.Radiance + b.Radiance, a.Transmittance + b.Transmittance);
    }

    private static RadianceSample Scale(RadianceSample sample, float scale)
    {
        return new RadianceSample(sample.Radiance * scale, sample.Transmittance * scale);
    }

    private static Vector2i? ToCell(Vector2 position, int width, int height)
    {
        var x = (int)MathF.Floor(position.X);
        var y = (int)MathF.Floor(position.Y);
        if (!Inside(x, y, width, height))
            return null;

        return new Vector2i(x, y);
    }

    private static int InitialJumpFloodStep(int size)
    {
        var step = 1;
        while (step < size)
            step <<= 1;

        return step >> 1;
    }

    private static float AspectDistanceSquared(Vector2i seed, int x, int y, Vector2 cellSize)
    {
        var dx = (seed.X - x) * cellSize.X;
        var dy = (seed.Y - y) * cellSize.Y;
        return dx * dx + dy * dy;
    }

    private static bool Inside(int x, int y, int width, int height)
    {
        return x >= 0 && y >= 0 && x < width && y < height;
    }

    private static int Index(int x, int y, int width)
    {
        return y * width + x;
    }

    private static void ValidateMask(ReadOnlySpan<bool> mask, int width, int height)
    {
        if (width <= 0)
            throw new ArgumentOutOfRangeException(nameof(width));

        if (height <= 0)
            throw new ArgumentOutOfRangeException(nameof(height));

        if (mask.Length != width * height)
            throw new ArgumentException("Mask dimensions do not match.", nameof(mask));
    }

    private static void ValidateRadiance(ReadOnlySpan<Vector3> radiance, int width, int height, string argumentName)
    {
        if (radiance.Length != width * height)
            throw new ArgumentException("Radiance dimensions do not match.", argumentName);
    }

    private static void ValidateFov(ReadOnlySpan<bool> fovMask, int width, int height)
    {
        if (fovMask.Length != 0 && fovMask.Length != width * height)
            throw new ArgumentException("FOV mask dimensions do not match.", nameof(fovMask));
    }
}
