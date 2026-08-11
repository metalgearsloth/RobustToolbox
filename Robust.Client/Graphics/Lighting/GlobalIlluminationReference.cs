using System;
using System.Numerics;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Lighting;

/// <summary>
/// Deterministic CPU reference helpers for the experimental SDF/JFA GI path.
/// These are intentionally simple and mirror the GPU algorithm closely enough for unit tests.
/// </summary>
internal static class GlobalIlluminationReference
{
    public readonly record struct TraceResult(bool Hit, Vector2i Cell, Vector3 Radiance, int Steps);

    public sealed class RadianceCascadeResult
    {
        public RadianceCascadeResult(Vector3[] current, Vector3[][] cascades, Vector2i[] cascadeSizes)
        {
            Current = current;
            Cascades = cascades;
            CascadeSizes = cascadeSizes;
        }

        public Vector3[] Current { get; }
        public Vector3[][] Cascades { get; }
        public Vector2i[] CascadeSizes { get; }
    }

    public static Vector2i ScaledTargetSize(Vector2i viewportSize, float scale)
    {
        var clampedScale = Math.Clamp(scale, 0.05f, 1f);
        return new Vector2i(
            Math.Max(1, (int)Math.Ceiling(viewportSize.X * clampedScale)),
            Math.Max(1, (int)Math.Ceiling(viewportSize.Y * clampedScale)));
    }

    public static Vector2i RadianceCascadeTargetSize(Vector2i baseSize, int cascade)
    {
        var divisor = 1 << Math.Clamp(cascade, 0, 8);
        return new Vector2i(
            Math.Max(1, (baseSize.X + divisor - 1) / divisor),
            Math.Max(1, (baseSize.Y + divisor - 1) / divisor));
    }

    public static Vector2i?[] ExactNearestField(ReadOnlySpan<bool> occlusionMask, int width, int height)
    {
        ValidateMask(occlusionMask, width, height);

        var result = new Vector2i?[width * height];

        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                var bestDistance = int.MaxValue;
                Vector2i? best = null;

                for (var sy = 0; sy < height; sy++)
                {
                    for (var sx = 0; sx < width; sx++)
                    {
                        if (!occlusionMask[Index(sx, sy, width)])
                            continue;

                        var dx = sx - x;
                        var dy = sy - y;
                        var distance = dx * dx + dy * dy;
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

    public static Vector2i?[] JumpFloodNearestField(ReadOnlySpan<bool> occlusionMask, int width, int height)
    {
        ValidateMask(occlusionMask, width, height);

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
                        ? DistanceSquared(seed, x, y)
                        : int.MaxValue;

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

                            var distance = DistanceSquared(candidate.Value, x, y);
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

    public static float[] DistanceField(ReadOnlySpan<Vector2i?> nearestField, int width, int height)
    {
        if (nearestField.Length != width * height)
            throw new ArgumentException("Nearest field dimensions do not match.", nameof(nearestField));

        var result = new float[width * height];
        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                var nearest = nearestField[Index(x, y, width)];
                result[Index(x, y, width)] = nearest == null
                    ? float.PositiveInfinity
                    : MathF.Sqrt(DistanceSquared(nearest.Value, x, y));
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

        if (fovMask.Length != 0 && fovMask.Length != width * height)
            throw new ArgumentException("FOV mask dimensions do not match.", nameof(fovMask));

        if (direction.LengthSquared() <= 0.000001f)
            throw new ArgumentException("Direction must be non-zero.", nameof(direction));

        var nearest = ExactNearestField(occlusionMask, width, height);
        return TraceRay(occlusionMask, directRadiance, previousGi, fovMask, nearest, width, height, start, direction, maxSteps, bounceDecay);
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

        if (fovMask.Length != 0 && fovMask.Length != width * height)
            throw new ArgumentException("FOV mask dimensions do not match.", nameof(fovMask));

        var nearest = ExactNearestField(occlusionMask, width, height);
        return TracePixel(
            occlusionMask,
            directRadiance,
            previousGi,
            fovMask,
            nearest,
            width,
            height,
            pixel,
            rays,
            maxSteps,
            bounceDecay);
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
        if (rays <= 0)
            throw new ArgumentOutOfRangeException(nameof(rays));

        ValidateMask(occlusionMask, width, height);
        ValidateRadiance(directRadiance, width, height, nameof(directRadiance));
        ValidateRadiance(previousGi, width, height, nameof(previousGi));

        if (fovMask.Length != 0 && fovMask.Length != width * height)
            throw new ArgumentException("FOV mask dimensions do not match.", nameof(fovMask));

        var nearest = useJumpFloodNearest
            ? JumpFloodNearestField(occlusionMask, width, height)
            : ExactNearestField(occlusionMask, width, height);

        var result = new Vector3[width * height];

        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                var idx = Index(x, y, width);
                if (fovMask.Length != 0 && !fovMask[idx])
                    continue;

                result[idx] = TracePixel(
                    occlusionMask,
                    directRadiance,
                    previousGi,
                    fovMask,
                    nearest,
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

    public static RadianceCascadeResult TraceRadianceCascades(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        int cascadeCount,
        int baseRays,
        int maxSteps,
        float bounceDecay,
        bool useJumpFloodNearest = true)
    {
        if (cascadeCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(cascadeCount));

        if (baseRays <= 0)
            throw new ArgumentOutOfRangeException(nameof(baseRays));

        ValidateMask(occlusionMask, width, height);
        ValidateRadiance(directRadiance, width, height, nameof(directRadiance));
        ValidateRadiance(previousGi, width, height, nameof(previousGi));

        if (fovMask.Length != 0 && fovMask.Length != width * height)
            throw new ArgumentException("FOV mask dimensions do not match.", nameof(fovMask));

        var nearest = useJumpFloodNearest
            ? JumpFloodNearestField(occlusionMask, width, height)
            : ExactNearestField(occlusionMask, width, height);

        cascadeCount = Math.Clamp(cascadeCount, 1, 4);
        var cascades = new Vector3[cascadeCount][];
        var cascadeSizes = new Vector2i[cascadeCount];
        var baseSize = new Vector2i(width, height);

        Vector3[]? previousCascade = null;
        Vector2i previousCascadeSize = default;

        for (var cascade = cascadeCount - 1; cascade >= 0; cascade--)
        {
            var size = RadianceCascadeTargetSize(baseSize, cascade);
            var result = new Vector3[size.X * size.Y];
            var rays = RadianceCascadeRayCount(baseRays, cascade);
            var intervalStart = RadianceCascadeIntervalStart(maxSteps, cascade);
            var intervalEnd = RadianceCascadeIntervalEnd(maxSteps, cascade);

            for (var y = 0; y < size.Y; y++)
            {
                for (var x = 0; x < size.X; x++)
                {
                    var uv = new Vector2((x + 0.5f) / size.X, (y + 0.5f) / size.Y);
                    var basePixel = new Vector2(uv.X * width, uv.Y * height);
                    var baseCell = new Vector2i(
                        Math.Clamp((int)MathF.Floor(basePixel.X), 0, width - 1),
                        Math.Clamp((int)MathF.Floor(basePixel.Y), 0, height - 1));
                    var baseIdx = Index(baseCell.X, baseCell.Y, width);

                    if (fovMask.Length != 0 && !fovMask[baseIdx])
                        continue;

                    var gathered = Vector3.Zero;
                    for (var i = 0; i < rays; i++)
                    {
                        var angle = i / (float)rays * MathF.PI * 2f;
                        var direction = new Vector2(MathF.Cos(angle), MathF.Sin(angle));
                        gathered += TraceRadianceCascadeInterval(
                            occlusionMask,
                            directRadiance,
                            previousGi,
                            fovMask,
                            nearest,
                            previousCascade ?? Array.Empty<Vector3>(),
                            previousCascadeSize,
                            width,
                            height,
                            basePixel,
                            direction,
                            maxSteps,
                            intervalStart,
                            intervalEnd,
                            bounceDecay);
                    }

                    result[Index(x, y, size.X)] = gathered / rays;
                }
            }

            cascades[cascade] = result;
            cascadeSizes[cascade] = size;
            previousCascade = result;
            previousCascadeSize = size;
        }

        return new RadianceCascadeResult(cascades[0], cascades, cascadeSizes);
    }

    public static int RadianceCascadeRayCount(int baseRays, int cascade)
    {
        var multiplier = 1;
        for (var i = 0; i < cascade; i++)
            multiplier *= 4;

        return Math.Clamp(baseRays * multiplier, 1, 256);
    }

    public static float RadianceCascadeIntervalStart(int maxSteps, int cascade)
    {
        return 0f;
    }

    public static float RadianceCascadeIntervalEnd(int maxSteps, int cascade)
    {
        return maxSteps * (MathF.Pow(2f, cascade + 1) - 1f);
    }

    private static Vector3 TracePixel(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        ReadOnlySpan<Vector2i?> nearest,
        int width,
        int height,
        Vector2 pixel,
        int rays,
        int maxSteps,
        float bounceDecay)
    {
        var gathered = Vector3.Zero;

        for (var i = 0; i < rays; i++)
        {
            var angle = i / (float)rays * MathF.PI * 2f;
            var direction = new Vector2(MathF.Cos(angle), MathF.Sin(angle));
            gathered += TraceRay(
                occlusionMask,
                directRadiance,
                previousGi,
                fovMask,
                nearest,
                width,
                height,
                pixel,
                direction,
                maxSteps,
                bounceDecay).Radiance;
        }

        return gathered / rays;
    }

    private static Vector3 TraceRadianceCascadeInterval(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        ReadOnlySpan<Vector2i?> nearest,
        ReadOnlySpan<Vector3> previousCascade,
        Vector2i previousCascadeSize,
        int width,
        int height,
        Vector2 start,
        Vector2 direction,
        int maxSteps,
        float intervalStart,
        float intervalEnd,
        float bounceDecay)
    {
        var rayDirection = Vector2.Normalize(direction);
        var startDistance = 1f;
        var position = start + rayDirection * startDistance;
        var traveled = startDistance;
        var bestRadiance = Vector3.Zero;
        var bestScore = 0f;

        for (var step = 0; step < maxSteps; step++)
        {
            var cell = new Vector2i((int)MathF.Floor(position.X), (int)MathF.Floor(position.Y));
            if (!Inside(cell.X, cell.Y, width, height))
                break;

            var idx = Index(cell.X, cell.Y, width);
            var nearestCell = nearest[idx];
            if (nearestCell == null)
                break;

            if (traveled >= intervalStart)
            {
                var rayRadiance = SampleRadianceAtPosition(directRadiance, previousGi, fovMask, width, height, position);
                var rayScore = Luminance(rayRadiance) / (1f + traveled * 0.35f);
                if (rayScore > bestScore)
                {
                    bestScore = rayScore;
                    bestRadiance = rayRadiance / (1f + traveled * 0.35f);
                }
            }

            var distance = DistanceToCellCenter(nearestCell.Value, position);
            if (occlusionMask[idx] || distance <= 0.75f)
            {
                if (traveled < intervalStart)
                    return bestRadiance;

                var hitCell = occlusionMask[idx] ? cell : nearestCell.Value;
                var hitIdx = Index(hitCell.X, hitCell.Y, width);

                if (fovMask.Length != 0 && !fovMask[hitIdx])
                    return Vector3.Zero;

                var surfaceRadiance = SampleRadianceAtPosition(
                    directRadiance,
                    previousGi,
                    fovMask,
                    width,
                    height,
                    position - rayDirection * 0.5f);
                return ComponentMax(bestRadiance, surfaceRadiance) * bounceDecay;
            }

            if (traveled >= intervalEnd)
            {
                if (previousCascade.Length != 0)
                    return ComponentMax(bestRadiance, SampleRadiance(previousCascade, previousCascadeSize, position, width, height));

                break;
            }

            var stepLength = Math.Max(distance - 0.75f, 0.25f);
            position += rayDirection * stepLength;
            traveled += stepLength;
        }

        return bestRadiance * bounceDecay;
    }

    private static Vector3 SampleRadiance(ReadOnlySpan<Vector3> radiance, Vector2i radianceSize, Vector2 position, int width, int height)
    {
        if (radiance.Length == 0)
            return Vector3.Zero;

        var x = Math.Clamp((int)MathF.Floor(position.X / width * radianceSize.X), 0, radianceSize.X - 1);
        var y = Math.Clamp((int)MathF.Floor(position.Y / height * radianceSize.Y), 0, radianceSize.Y - 1);
        return radiance[Index(x, y, radianceSize.X)];
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

    private static TraceResult TraceRay(
        ReadOnlySpan<bool> occlusionMask,
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        ReadOnlySpan<Vector2i?> nearest,
        int width,
        int height,
        Vector2 start,
        Vector2 direction,
        int maxSteps,
        float bounceDecay)
    {
        var rayDirection = Vector2.Normalize(direction);
        var position = start + rayDirection * 0.5f;
        var traveled = 0.5f;
        var bestRadiance = Vector3.Zero;
        var bestScore = 0f;

        for (var step = 0; step < maxSteps; step++)
        {
            var cell = new Vector2i((int)MathF.Floor(position.X), (int)MathF.Floor(position.Y));
            if (!Inside(cell.X, cell.Y, width, height))
                break;

            var idx = Index(cell.X, cell.Y, width);
            var nearestCell = nearest[idx];
            if (nearestCell == null)
                break;

            var rayRadiance = SampleRadianceAtPosition(directRadiance, previousGi, fovMask, width, height, position);
            var rayScore = Luminance(rayRadiance) / (1f + traveled * 0.35f);
            if (rayScore > bestScore)
            {
                bestScore = rayScore;
                bestRadiance = rayRadiance / (1f + traveled * 0.35f);
            }

            var distance = DistanceToCellCenter(nearestCell.Value, position);
            if (occlusionMask[idx] || distance <= 0.75f)
            {
                var hitCell = occlusionMask[idx] ? cell : nearestCell.Value;
                var hitIdx = Index(hitCell.X, hitCell.Y, width);

                if (fovMask.Length != 0 && !fovMask[hitIdx])
                    return new TraceResult(true, hitCell, Vector3.Zero, step + 1);

                var surfaceRadiance = SampleRadianceAtPosition(
                    directRadiance,
                    previousGi,
                    fovMask,
                    width,
                    height,
                    position - rayDirection * 0.5f);
                var radiance = ComponentMax(bestRadiance, surfaceRadiance) * bounceDecay;
                return new TraceResult(true, hitCell, radiance, step + 1);
            }

            var stepLength = Math.Max(distance - 0.75f, 0.25f);
            position += rayDirection * stepLength;
            traveled += stepLength;
        }

        return new TraceResult(false, default, bestRadiance * bounceDecay, maxSteps);
    }

    private static Vector3 SampleRadianceAtPosition(
        ReadOnlySpan<Vector3> directRadiance,
        ReadOnlySpan<Vector3> previousGi,
        ReadOnlySpan<bool> fovMask,
        int width,
        int height,
        Vector2 position)
    {
        var cell = new Vector2i((int)MathF.Floor(position.X), (int)MathF.Floor(position.Y));
        if (!Inside(cell.X, cell.Y, width, height))
            return Vector3.Zero;

        var idx = Index(cell.X, cell.Y, width);
        if (fovMask.Length != 0 && !fovMask[idx])
            return Vector3.Zero;

        return directRadiance[idx] + previousGi[idx];
    }

    private static Vector3 ComponentMax(Vector3 a, Vector3 b)
    {
        return new Vector3(
            MathF.Max(a.X, b.X),
            MathF.Max(a.Y, b.Y),
            MathF.Max(a.Z, b.Z));
    }

    private static float Luminance(Vector3 value)
    {
        return value.X * 0.2126f + value.Y * 0.7152f + value.Z * 0.0722f;
    }

    private static int InitialJumpFloodStep(int size)
    {
        var step = 1;
        while (step < size)
            step <<= 1;

        return step >> 1;
    }

    private static int DistanceSquared(Vector2i seed, int x, int y)
    {
        var dx = seed.X - x;
        var dy = seed.Y - y;
        return dx * dx + dy * dy;
    }

    private static float DistanceToCellCenter(Vector2i seed, Vector2 position)
    {
        var center = new Vector2(seed.X + 0.5f, seed.Y + 0.5f);
        return (center - position).Length();
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
}
