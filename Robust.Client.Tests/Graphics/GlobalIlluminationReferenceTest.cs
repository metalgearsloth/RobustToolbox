using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Diagnostics.CodeAnalysis;
using System.Numerics;
using NUnit.Framework;
using Robust.Client.Graphics;
using Robust.Client.Graphics.Lighting;
using Robust.Shared;
using Robust.Shared.ContentPack;
using Robust.Shared.Maths;
using Robust.Shared.Utility;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
public sealed class GlobalIlluminationReferenceTest
{
    private static readonly string[] GiShaderPaths =
    {
        "/Shaders/Internal/gi-occlusion-mask.swsl",
        "/Shaders/Internal/gi-jfa-seed.swsl",
        "/Shaders/Internal/gi-jfa-jump.swsl",
        "/Shaders/Internal/gi-trace.swsl",
        "/Shaders/Internal/gi-radiance-cascade.swsl",
        "/Shaders/Internal/gi-combine.swsl",
        "/Shaders/Internal/gi-debug.swsl"
    };

    [Test]
    public void GiEnabledByDefaultForIteration()
    {
        Assert.Multiple(() =>
        {
            Assert.That(CVars.DisplayGiEnabled.Name, Is.EqualTo("display.gi_enabled"));
            Assert.That(CVars.DisplayGiEnabled.DefaultValue, Is.True);
            Assert.That(CVars.DisplayGiBackend.Name, Is.EqualTo("display.gi_backend"));
            Assert.That(CVars.DisplayGiBackend.DefaultValue, Is.EqualTo(0));
            Assert.That(CVars.DisplayGiIntensity.Name, Is.EqualTo("display.gi_intensity"));
            Assert.That(CVars.DisplayGiIntensity.DefaultValue, Is.EqualTo(2.0f));
            Assert.That(CVars.DisplayGiTemporalJitter.Name, Is.EqualTo("display.gi_temporal_jitter"));
            Assert.That(CVars.DisplayGiTemporalJitter.DefaultValue, Is.EqualTo(0.0f));
        });
    }

    [Test]
    public void GiShadersParse()
    {
        var resourceManager = new ShaderResourceManagerStub();

        Assert.Multiple(() =>
        {
            foreach (var shaderPath in GiShaderPaths)
            {
                using var stream = resourceManager.ContentFileRead(shaderPath);
                using var reader = new StreamReader(stream);
                Assert.DoesNotThrow(() => ShaderParser.Parse(reader, resourceManager), shaderPath);
            }
        });
    }

    [Test]
    public void JfaProducesSensibleNearestObstacleResults()
    {
        const int width = 8;
        const int height = 8;
        var mask = new bool[width * height];
        mask[Index(2, 2, width)] = true;
        mask[Index(6, 5, width)] = true;

        var field = GlobalIlluminationReference.JumpFloodNearestField(mask, width, height);
        var distances = GlobalIlluminationReference.DistanceField(field, width, height);

        Assert.Multiple(() =>
        {
            Assert.That(field[Index(2, 2, width)], Is.EqualTo(new Vector2i(2, 2)));
            Assert.That(field[Index(6, 5, width)], Is.EqualTo(new Vector2i(6, 5)));
            Assert.That(field[Index(0, 0, width)], Is.EqualTo(new Vector2i(2, 2)));
            Assert.That(field[Index(7, 7, width)], Is.EqualTo(new Vector2i(6, 5)));
            Assert.That(distances[Index(2, 2, width)], Is.EqualTo(0f));
            Assert.That(distances.All(float.IsFinite), Is.True);
        });
    }

    [Test]
    public void EmptyMaskDoesNotGenerateIntersections()
    {
        const int width = 6;
        const int height = 4;
        var mask = new bool[width * height];
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];

        var field = GlobalIlluminationReference.JumpFloodNearestField(mask, width, height);
        var distances = GlobalIlluminationReference.DistanceField(field, width, height);
        var trace = GlobalIlluminationReference.TraceRay(
            mask,
            direct,
            previous,
            Array.Empty<bool>(),
            width,
            height,
            new Vector2(3.5f, 1.5f),
            -Vector2.UnitX,
            16,
            1f);

        Assert.Multiple(() =>
        {
            Assert.That(field.All(x => x == null), Is.True);
            Assert.That(distances.All(float.IsPositiveInfinity), Is.True);
            Assert.That(trace.Hit, Is.False);
            Assert.That(trace.Radiance, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void SingleWallBlocksGiRays()
    {
        const int width = 7;
        const int height = 3;
        var mask = new bool[width * height];
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];

        for (var y = 0; y < height; y++)
            mask[Index(3, y, width)] = true;

        mask[Index(0, 1, width)] = true;
        direct[Index(0, 1, width)] = new Vector3(10f, 10f, 10f);

        var trace = GlobalIlluminationReference.TraceRay(
            mask,
            direct,
            previous,
            Array.Empty<bool>(),
            width,
            height,
            new Vector2(5.5f, 1.5f),
            -Vector2.UnitX,
            32,
            1f);

        Assert.Multiple(() =>
        {
            Assert.That(trace.Hit, Is.True);
            Assert.That(trace.Cell, Is.EqualTo(new Vector2i(3, 1)));
            Assert.That(trace.Radiance, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void DoorOpeningAllowsIndirectLightThrough()
    {
        const int width = 9;
        const int height = 5;
        var mask = new bool[width * height];
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];

        mask = BuildTwoRoomScene(width, height, doorwayOpen: true);
        direct[Index(2, 2, width)] = new Vector3(4f, 2f, 1f);

        var trace = GlobalIlluminationReference.TraceRay(
            mask,
            direct,
            previous,
            Array.Empty<bool>(),
            width,
            height,
            new Vector2(7.5f, 2.5f),
            -Vector2.UnitX,
            32,
            0.5f);

        Assert.Multiple(() =>
        {
            Assert.That(trace.Hit, Is.True);
            Assert.That(trace.Cell, Is.EqualTo(new Vector2i(0, 2)));
            Assert.That(trace.Radiance.Length(), Is.GreaterThan(0.1f));
        });
    }

    [Test]
    public void DoorOpeningAllowsFloorRadiancePropagationWithoutWallLeak()
    {
        const int width = 9;
        const int height = 5;
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();
        var openDoorMask = BuildTwoRoomScene(width, height, doorwayOpen: true);
        var sealedWallMask = BuildTwoRoomScene(width, height, doorwayOpen: false);

        direct[Index(2, 2, width)] = new Vector3(6f, 4f, 2f);

        var openDoorGi = GlobalIlluminationReference.TraceRay(
            openDoorMask,
            direct,
            previous,
            fov,
            width,
            height,
            new Vector2(7.5f, 2.5f),
            -Vector2.UnitX,
            32,
            0.5f);

        var sealedWallGi = GlobalIlluminationReference.TraceRay(
            sealedWallMask,
            direct,
            previous,
            fov,
            width,
            height,
            new Vector2(7.5f, 2.5f),
            -Vector2.UnitX,
            32,
            0.5f);

        Assert.Multiple(() =>
        {
            Assert.That(openDoorGi.Radiance.Length(), Is.GreaterThan(0.1f));
            Assert.That(sealedWallGi.Radiance, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void TwoRoomDoorwayDebugSceneProducesIndirectLightWithoutWallLeak()
    {
        const int width = 10;
        const int height = 5;
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();
        var openDoorMask = BuildTwoRoomScene(width, height, doorwayOpen: true);
        var sealedWallMask = BuildTwoRoomScene(width, height, doorwayOpen: false);

        direct[Index(0, 2, width)] = new Vector3(8f, 6f, 4f);

        var openDoorGi = GlobalIlluminationReference.TracePixel(
            openDoorMask,
            direct,
            previous,
            fov,
            width,
            height,
            new Vector2(7.5f, 2.5f),
            4,
            64,
            0.5f);

        var sealedWallGi = GlobalIlluminationReference.TracePixel(
            sealedWallMask,
            direct,
            previous,
            fov,
            width,
            height,
            new Vector2(7.5f, 2.5f),
            4,
            64,
            0.5f);

        Assert.Multiple(() =>
        {
            Assert.That(openDoorGi.Length(), Is.GreaterThan(0.1f));
            Assert.That(sealedWallGi, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void RadianceCascadeTargetsAndIntervalsAreDeterministic()
    {
        Assert.Multiple(() =>
        {
            Assert.That(GlobalIlluminationReference.RadianceCascadeTargetSize(new Vector2i(96, 48), 0), Is.EqualTo(new Vector2i(96, 48)));
            Assert.That(GlobalIlluminationReference.RadianceCascadeTargetSize(new Vector2i(96, 48), 1), Is.EqualTo(new Vector2i(48, 24)));
            Assert.That(GlobalIlluminationReference.RadianceCascadeTargetSize(new Vector2i(95, 47), 2), Is.EqualTo(new Vector2i(24, 12)));
            Assert.That(GlobalIlluminationReference.RadianceCascadeRayCount(4, 0), Is.EqualTo(4));
            Assert.That(GlobalIlluminationReference.RadianceCascadeRayCount(4, 1), Is.EqualTo(16));
            Assert.That(GlobalIlluminationReference.RadianceCascadeRayCount(4, 2), Is.EqualTo(64));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalStart(8, 0), Is.EqualTo(0f));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalEnd(8, 2), Is.EqualTo(56f));
        });
    }

    [Test]
    public void RadianceCascadesAllowIndirectLightThroughDoorwayWithoutWallLeak()
    {
        const int width = 48;
        const int height = 24;
        var previous = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();
        var openDoorMask = BuildTwoRoomDebugDumpScene(width, height, doorwayOpen: true);
        var sealedWallMask = BuildTwoRoomDebugDumpScene(width, height, doorwayOpen: false);
        var direct = BuildDebugDirectRadiance(openDoorMask, width, height);

        var openDoorResult = GlobalIlluminationReference.TraceRadianceCascades(
            openDoorMask,
            direct,
            previous,
            fov,
            width,
            height,
            cascadeCount: 3,
            baseRays: 4,
            maxSteps: 8,
            bounceDecay: 0.55f);

        var sealedWallResult = GlobalIlluminationReference.TraceRadianceCascades(
            sealedWallMask,
            direct,
            previous,
            fov,
            width,
            height,
            cascadeCount: 3,
            baseRays: 4,
            maxSteps: 8,
            bounceDecay: 0.55f);

        var rightRoomNearDoor = Index(width / 2 + 3, height / 2, width);
        var farCorner = Index(width - 5, height / 2, width);

        Assert.Multiple(() =>
        {
            Assert.That(openDoorResult.Current[rightRoomNearDoor].Length(), Is.GreaterThan(0.05f));
            Assert.That(openDoorResult.Current[farCorner].Length(), Is.GreaterThan(0.001f));
            Assert.That(sealedWallResult.Current[rightRoomNearDoor], Is.EqualTo(Vector3.Zero));
            Assert.That(sealedWallResult.Current[farCorner], Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void GiDebugDumpWritesArtifactsWhenRequested()
    {
        if (!IsTruthy(Environment.GetEnvironmentVariable("ROBUST_GI_DUMP_DEBUG")))
            return;

        const int width = 96;
        const int height = 48;
        const int rays = 16;
        const int steps = 96;
        const float bounceDecay = 0.55f;
        var fov = Enumerable.Repeat(true, width * height).ToArray();
        var mask = BuildTwoRoomDebugDumpScene(width, height, doorwayOpen: true);
        var direct = BuildDebugDirectRadiance(mask, width, height);
        var nearest = GlobalIlluminationReference.JumpFloodNearestField(mask, width, height);
        var distances = GlobalIlluminationReference.DistanceField(nearest, width, height);
        var previous = new Vector3[width * height];
        var firstBounce = GlobalIlluminationReference.TraceImage(mask, direct, previous, fov, width, height, rays, steps, bounceDecay);
        var accumulated = firstBounce;

        for (var i = 1; i < 4; i++)
            accumulated = GlobalIlluminationReference.TraceImage(mask, direct, accumulated, fov, width, height, rays, steps, bounceDecay);

        var combined = CombineDebugLighting(direct, accumulated, width, height);
        var rcFirst = GlobalIlluminationReference.TraceRadianceCascades(
            mask,
            direct,
            previous,
            fov,
            width,
            height,
            cascadeCount: 3,
            baseRays: 4,
            maxSteps: 12,
            bounceDecay: bounceDecay);
        var rcAccumulated = rcFirst;

        for (var i = 1; i < 4; i++)
        {
            rcAccumulated = GlobalIlluminationReference.TraceRadianceCascades(
                mask,
                direct,
                rcAccumulated.Current,
                fov,
                width,
                height,
                cascadeCount: 3,
                baseRays: 4,
                maxSteps: 12,
                bounceDecay: bounceDecay);
        }

        var rcCombined = CombineDebugLighting(direct, rcAccumulated.Current, width, height);
        var outputDir = GetGiDumpDirectory();
        Directory.CreateDirectory(outputDir);

        WriteImage(Path.Combine(outputDir, "01-occlusion-mask.png"), width, height, (x, y) =>
            mask[Index(x, y, width)] ? new Rgba32(255, 255, 255, 255) : new Rgba32(0, 0, 0, 255));

        WriteImage(Path.Combine(outputDir, "02-jfa-nearest-seed.png"), width, height, (x, y) =>
        {
            var seed = nearest[Index(x, y, width)];
            if (seed == null)
                return new Rgba32(0, 0, 0, 255);

            return new Rgba32(
                ToByte(seed.Value.X / (float)Math.Max(1, width - 1)),
                ToByte(seed.Value.Y / (float)Math.Max(1, height - 1)),
                255,
                255);
        });

        var maxDistance = distances.Where(float.IsFinite).DefaultIfEmpty(1f).Max();
        WriteImage(Path.Combine(outputDir, "03-sdf-distance.png"), width, height, (x, y) =>
        {
            var distance = distances[Index(x, y, width)];
            var value = float.IsFinite(distance)
                ? Math.Clamp(distance / Math.Max(1f, maxDistance), 0f, 1f)
                : 0f;

            var byteValue = ToByte(value);
            return new Rgba32(byteValue, byteValue, byteValue, 255);
        });

        WriteImage(Path.Combine(outputDir, "04-direct-radiance.png"), width, height, (x, y) =>
            EncodeRadiance(direct[Index(x, y, width)], 0.25f));

        WriteImage(Path.Combine(outputDir, "05-gi-first-bounce.png"), width, height, (x, y) =>
            EncodeRadiance(firstBounce[Index(x, y, width)], 1f));

        WriteImage(Path.Combine(outputDir, "06-gi-accumulated.png"), width, height, (x, y) =>
            EncodeRadiance(accumulated[Index(x, y, width)], 1f));

        WriteImage(Path.Combine(outputDir, "07-final-combined-reference.png"), width, height, (x, y) =>
            EncodeRadiance(combined[Index(x, y, width)], 0.4f));

        for (var cascade = rcAccumulated.Cascades.Length - 1; cascade >= 0; cascade--)
        {
            var cascadeSize = rcAccumulated.CascadeSizes[cascade];
            var cascadeImage = rcAccumulated.Cascades[cascade];
            WriteImage(Path.Combine(outputDir, $"08-rc-cascade-{cascade}.png"), cascadeSize.X, cascadeSize.Y, (x, y) =>
                EncodeRadiance(cascadeImage[Index(x, y, cascadeSize.X)], 1f));
        }

        WriteImage(Path.Combine(outputDir, "09-rc-final-current.png"), width, height, (x, y) =>
            EncodeRadiance(rcAccumulated.Current[Index(x, y, width)], 1f));

        WriteImage(Path.Combine(outputDir, "10-rc-final-combined-reference.png"), width, height, (x, y) =>
            EncodeRadiance(rcCombined[Index(x, y, width)], 0.4f));

        TestContext.Out.WriteLine($"Wrote GI debug dump images to: {outputDir}");
    }

    [Test]
    public void GiTargetScaleResizesDeterministically()
    {
        Assert.Multiple(() =>
        {
            Assert.That(GlobalIlluminationReference.ScaledTargetSize(new Vector2i(1920, 1080), 0.25f), Is.EqualTo(new Vector2i(480, 270)));
            Assert.That(GlobalIlluminationReference.ScaledTargetSize(new Vector2i(100, 100), 0f), Is.EqualTo(new Vector2i(5, 5)));
            Assert.That(GlobalIlluminationReference.ScaledTargetSize(new Vector2i(100, 50), 2f), Is.EqualTo(new Vector2i(100, 50)));
        });
    }

    [Test]
    public void HistoryRejectedWhenSettingsOrOcclusionChange()
    {
        Assert.Multiple(() =>
        {
            Assert.That(GlobalIlluminationReference.ShouldRejectHistory(
                Vector2.Zero,
                Vector2.Zero,
                Vector2.One,
                Vector2.One,
                Angle.Zero,
                Angle.Zero,
                1,
                2,
                10,
                10), Is.True);

            Assert.That(GlobalIlluminationReference.ShouldRejectHistory(
                Vector2.Zero,
                Vector2.Zero,
                Vector2.One,
                Vector2.One,
                Angle.Zero,
                Angle.Zero,
                1,
                1,
                10,
                11), Is.True);
        });
    }

    [Test]
    public void CameraMovementReprojectsHistoryUnlessJumping()
    {
        Assert.Multiple(() =>
        {
            Assert.That(GlobalIlluminationReference.ShouldRejectHistory(
                Vector2.Zero,
                new Vector2(0.01f, 0f),
                Vector2.One,
                Vector2.One,
                Angle.Zero,
                Angle.Zero,
                1,
                1,
                10,
                10), Is.False);

            Assert.That(GlobalIlluminationReference.ShouldRejectHistory(
                Vector2.Zero,
                new Vector2(5f, 0f),
                Vector2.One,
                Vector2.One,
                Angle.Zero,
                Angle.Zero,
                1,
                1,
                10,
                10), Is.True);

            Assert.That(GlobalIlluminationReference.ShouldRejectHistory(
                Vector2.Zero,
                Vector2.Zero,
                Vector2.One,
                Vector2.One,
                Angle.Zero,
                Angle.Zero,
                1,
                1,
                10,
                10), Is.False);
        });
    }

    [Test]
    public void FovMaskPreventsGiLeakFromHiddenLight()
    {
        const int width = 5;
        const int height = 1;
        var mask = new bool[width * height];
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        mask[Index(0, 0, width)] = true;
        direct[Index(0, 0, width)] = new Vector3(5f, 5f, 5f);
        fov[Index(0, 0, width)] = false;

        var trace = GlobalIlluminationReference.TraceRay(
            mask,
            direct,
            previous,
            fov,
            width,
            height,
            new Vector2(3.5f, 0.5f),
            -Vector2.UnitX,
            16,
            1f);

        Assert.Multiple(() =>
        {
            Assert.That(trace.Hit, Is.True);
            Assert.That(trace.Radiance, Is.EqualTo(Vector3.Zero));
        });
    }

    private static int Index(int x, int y, int width)
    {
        return y * width + x;
    }

    private static bool[] BuildTwoRoomScene(int width, int height, bool doorwayOpen)
    {
        var mask = new bool[width * height];

        for (var x = 0; x < width; x++)
        {
            mask[Index(x, 0, width)] = true;
            mask[Index(x, height - 1, width)] = true;
        }

        for (var y = 0; y < height; y++)
        {
            mask[Index(0, y, width)] = true;
            mask[Index(width - 1, y, width)] = true;

            if (!doorwayOpen || y != 2)
                mask[Index(4, y, width)] = true;
        }

        return mask;
    }

    private static bool[] BuildTwoRoomDebugDumpScene(int width, int height, bool doorwayOpen)
    {
        var mask = new bool[width * height];
        var splitX = width / 2;
        var doorCenter = height / 2;
        var doorHalfHeight = Math.Max(2, height / 10);

        for (var x = 0; x < width; x++)
        {
            mask[Index(x, 0, width)] = true;
            mask[Index(x, height - 1, width)] = true;
        }

        for (var y = 0; y < height; y++)
        {
            mask[Index(0, y, width)] = true;
            mask[Index(width - 1, y, width)] = true;

            var inDoorway = y >= doorCenter - doorHalfHeight && y <= doorCenter + doorHalfHeight;
            if (!doorwayOpen || !inDoorway)
                mask[Index(splitX, y, width)] = true;
        }

        return mask;
    }

    private static Vector3[] BuildDebugDirectRadiance(ReadOnlySpan<bool> mask, int width, int height)
    {
        var radiance = new Vector3[width * height];
        var splitX = width / 2;
        var lightPosition = new Vector2(width * 0.18f, height * 0.50f);
        var lightColor = new Vector3(8f, 5.5f, 3.5f);

        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                if (x >= splitX - 1)
                    continue;

                var idx = Index(x, y, width);
                var delta = new Vector2(x + 0.5f, y + 0.5f) - lightPosition;
                var attenuation = 1f / (1f + delta.LengthSquared() * 0.015f);

                // Keep non-occluder direct lighting dimmer than wall/surface lighting so the GI buffers
                // still show what rays collect from hit surfaces.
                radiance[idx] = lightColor * attenuation * (mask[idx] ? 1f : 0.35f);
            }
        }

        return radiance;
    }

    private static Vector3[] CombineDebugLighting(ReadOnlySpan<Vector3> direct, ReadOnlySpan<Vector3> gi, int width, int height)
    {
        var result = new Vector3[width * height];
        var ambient = new Vector3(0.03f, 0.03f, 0.04f);

        for (var i = 0; i < result.Length; i++)
            result[i] = ambient + direct[i] + gi[i];

        return result;
    }

    private static string GetGiDumpDirectory()
    {
        var configured = Environment.GetEnvironmentVariable("ROBUST_GI_DUMP_DIR");
        return string.IsNullOrWhiteSpace(configured)
            ? Path.Combine(TestContext.CurrentContext.WorkDirectory, "GiDebugDumps", "two-room-reference")
            : configured;
    }

    private static bool IsTruthy(string? value)
    {
        return value != null
            && (value.Equals("1", StringComparison.OrdinalIgnoreCase)
                || value.Equals("true", StringComparison.OrdinalIgnoreCase)
                || value.Equals("yes", StringComparison.OrdinalIgnoreCase));
    }

    private static void WriteImage(string path, int width, int height, Func<int, int, Rgba32> getPixel)
    {
        const int scale = 4;
        using var image = new Image<Rgba32>(width * scale, height * scale);

        for (var y = 0; y < height; y++)
        {
            for (var x = 0; x < width; x++)
            {
                var color = getPixel(x, y);
                for (var oy = 0; oy < scale; oy++)
                {
                    for (var ox = 0; ox < scale; ox++)
                    {
                        image[x * scale + ox, y * scale + oy] = color;
                    }
                }
            }
        }

        image.Save(path);
    }

    private static Rgba32 EncodeRadiance(Vector3 radiance, float exposure)
    {
        return new Rgba32(
            ToByte(ToneMap(radiance.X * exposure)),
            ToByte(ToneMap(radiance.Y * exposure)),
            ToByte(ToneMap(radiance.Z * exposure)),
            255);
    }

    private static float ToneMap(float value)
    {
        value = Math.Max(0f, value);
        return value / (1f + value);
    }

    private static byte ToByte(float value)
    {
        return (byte)Math.Round(Math.Clamp(value, 0f, 1f) * 255f);
    }

    private sealed class ShaderResourceManagerStub : IResourceManager
    {
        private readonly string _resourcesRoot;

        public ShaderResourceManagerStub()
        {
            _resourcesRoot = FindResourcesRoot();
        }

        public IWritableDirProvider UserData => throw new NotSupportedException();

        public void AddRoot(ResPath prefix, IContentRoot loader)
        {
            throw new NotSupportedException();
        }

        public Stream ContentFileRead(ResPath path)
        {
            return File.OpenRead(ToLocalPath(path));
        }

        public Stream ContentFileRead(string path)
        {
            return ContentFileRead(new ResPath(path));
        }

        public bool ContentFileExists(ResPath path)
        {
            return File.Exists(ToLocalPath(path));
        }

        public bool ContentFileExists(string path)
        {
            return ContentFileExists(new ResPath(path));
        }

        public bool TryContentFileRead(ResPath? path, [NotNullWhen(true)] out Stream? fileStream)
        {
            if (path == null || !ContentFileExists(path.Value))
            {
                fileStream = null;
                return false;
            }

            fileStream = ContentFileRead(path.Value);
            return true;
        }

        public bool TryContentFileRead(string path, [NotNullWhen(true)] out Stream? fileStream)
        {
            return TryContentFileRead(new ResPath(path), out fileStream);
        }

        public IEnumerable<ResPath> ContentFindFiles(ResPath? path)
        {
            return Array.Empty<ResPath>();
        }

        public IEnumerable<ResPath> ContentFindFiles(string path)
        {
            return Array.Empty<ResPath>();
        }

        public IEnumerable<string> ContentGetDirectoryEntries(ResPath path)
        {
            return Array.Empty<string>();
        }

        public IEnumerable<ResPath> GetContentRoots()
        {
            return Array.Empty<ResPath>();
        }

        private string ToLocalPath(ResPath path)
        {
            var relative = path.ToString().TrimStart('/').Replace('/', Path.DirectorySeparatorChar);
            return Path.Combine(_resourcesRoot, relative);
        }

        private static string FindResourcesRoot()
        {
            var directory = new DirectoryInfo(TestContext.CurrentContext.TestDirectory);
            while (directory != null)
            {
                var candidate = Path.Combine(directory.FullName, "Resources", "Shaders", "Internal");
                if (Directory.Exists(candidate))
                    return Path.Combine(directory.FullName, "Resources");

                directory = directory.Parent;
            }

            throw new DirectoryNotFoundException("Could not locate RobustToolbox Resources directory.");
        }
    }
}
