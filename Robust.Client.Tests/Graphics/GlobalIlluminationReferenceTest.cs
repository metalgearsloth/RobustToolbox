using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
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
    private const int RcCascadeCount = 3;
    private const int RcBaseRays = 32;
    private const float RcBaseInterval = 2f;
    private const float RcStep = 0.25f;
    private const float RcBounceDecay = 0.65f;
    private const int BruteForceRays = 256;
    private const float BruteForceStep = 0.125f;

    private static readonly string[] GiShaderPaths =
    {
        "/Shaders/Internal/gi-occlusion-mask.swsl",
        "/Shaders/Internal/gi-jfa-seed.swsl",
        "/Shaders/Internal/gi-jfa-jump.swsl",
        "/Shaders/Internal/gi-trace.swsl",
        "/Shaders/Internal/gi-radiance-cascade.swsl",
        "/Shaders/Internal/gi-radiance-resolve.swsl",
        "/Shaders/Internal/gi-combine.swsl",
        "/Shaders/Internal/gi-debug.swsl"
    };

    [Test]
    public void GiIsExperimentalAndDisabledByDefault()
    {
        Assert.Multiple(() =>
        {
            Assert.That(CVars.DisplayGiEnabled.Name, Is.EqualTo("display.gi_enabled"));
            Assert.That(CVars.DisplayGiEnabled.DefaultValue, Is.False);
            Assert.That(CVars.DisplayGiHistoryWeight.DefaultValue, Is.EqualTo(0.0f));
            Assert.That(CVars.DisplayGiIntensity.DefaultValue, Is.EqualTo(1.0f));
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
    public void JfaNearestFieldUsesAspectCorrectDistance()
    {
        const int width = 6;
        const int height = 4;
        var mask = new bool[width * height];
        mask[Index(4, 2, width)] = true;
        mask[Index(1, 0, width)] = true;

        var nearest = GlobalIlluminationReference.ExactNearestField(mask, width, height, new Vector2(0.25f, 2f));
        var jfa = GlobalIlluminationReference.JumpFloodNearestField(mask, width, height, new Vector2(0.25f, 2f));
        var query = Index(1, 2, width);

        Assert.Multiple(() =>
        {
            Assert.That(nearest[query], Is.EqualTo(new Vector2i(4, 2)));
            Assert.That(jfa[query], Is.EqualTo(new Vector2i(4, 2)));
        });
    }

    [Test]
    public void EmptyMaskDoesNotGenerateNearestSeedsOrIntersections()
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
            Assert.That(trace.Transmittance, Is.EqualTo(1f));
            Assert.That(trace.Radiance, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void SingleWallBlocksGiRaysWithZeroTransmittance()
    {
        const int width = 7;
        const int height = 3;
        var mask = new bool[width * height];
        var direct = new Vector3[width * height];
        var previous = new Vector3[width * height];

        for (var y = 0; y < height; y++)
            mask[Index(3, y, width)] = true;

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
            Assert.That(trace.Transmittance, Is.EqualTo(0f));
            Assert.That(trace.Radiance, Is.EqualTo(Vector3.Zero));
        });
    }

    [Test]
    public void RadianceCascadeIntervalsAreContiguousAndGeometric()
    {
        Assert.Multiple(() =>
        {
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalStart(2f, 0), Is.EqualTo(0f));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalEnd(2f, 0), Is.EqualTo(2f));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalStart(2f, 1), Is.EqualTo(2f));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalEnd(2f, 1), Is.EqualTo(10f));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalStart(2f, 2), Is.EqualTo(10f));
            Assert.That(GlobalIlluminationReference.RadianceCascadeIntervalEnd(2f, 2), Is.EqualTo(42f));
        });
    }

    [Test]
    public void RadianceCascadeLayoutRetainsAngularSamples()
    {
        var baseSize = new Vector2i(64, 32);

        Assert.Multiple(() =>
        {
            Assert.That(GlobalIlluminationReference.RadianceCascadeProbeSize(baseSize, 0), Is.EqualTo(new Vector2i(64, 32)));
            Assert.That(GlobalIlluminationReference.RadianceCascadeProbeSize(baseSize, 1), Is.EqualTo(new Vector2i(32, 16)));
            Assert.That(GlobalIlluminationReference.RadianceCascadeRayCount(8, 0), Is.EqualTo(8));
            Assert.That(GlobalIlluminationReference.RadianceCascadeRayCount(8, 1), Is.EqualTo(32));
            Assert.That(GlobalIlluminationReference.RadianceCascadeRayCount(8, 2), Is.EqualTo(128));
            Assert.That(GlobalIlluminationReference.RadianceCascadeDirectionGrid(32), Is.EqualTo(new Vector2i(6, 6)));
            Assert.That(GlobalIlluminationReference.RadianceCascadeAtlasSize(baseSize, 1, 8), Is.EqualTo(new Vector2i(192, 96)));
        });
    }

    [Test]
    public void AngularDirectionChildrenStayInsideParentCone()
    {
        const int parentDirectionCount = 8;
        var parentDirectionIndex = 3;
        var parentStart = parentDirectionIndex / (float)parentDirectionCount * MathF.Tau;
        var parentEnd = (parentDirectionIndex + 1f) / parentDirectionCount * MathF.Tau;
        var childDirectionCount = parentDirectionCount * GlobalIlluminationReference.RadianceCascadeAngularBranchFactor;
        var average = Vector2.Zero;

        for (var child = 0; child < GlobalIlluminationReference.RadianceCascadeAngularBranchFactor; child++)
        {
            var childIndex = GlobalIlluminationReference.RadianceCascadeChildDirectionIndex(parentDirectionIndex, child);
            var childAngle = GlobalIlluminationReference.RadianceCascadeDirectionAngle(childIndex, childDirectionCount);
            Assert.That(childAngle, Is.InRange(parentStart, parentEnd));
            average += GlobalIlluminationReference.RadianceCascadeDirection(childIndex, childDirectionCount);
        }

        average = Vector2.Normalize(average);
        var parent = GlobalIlluminationReference.RadianceCascadeDirection(parentDirectionIndex, parentDirectionCount);
        Assert.That(Vector2.Dot(average, parent), Is.GreaterThan(0.999f));
    }

    [Test]
    public void TransmittanceAwareMergePreservesOcclusion()
    {
        var blockedNear = new GlobalIlluminationReference.RadianceSample(new Vector3(1f, 0f, 0f), 0f);
        var partiallyOpenNear = new GlobalIlluminationReference.RadianceSample(new Vector3(1f, 0f, 0f), 0.25f);
        var far = new GlobalIlluminationReference.RadianceSample(new Vector3(0f, 4f, 0f), 0.5f);

        var blocked = GlobalIlluminationReference.RadianceSample.Merge(blockedNear, far);
        var open = GlobalIlluminationReference.RadianceSample.Merge(partiallyOpenNear, far);

        Assert.Multiple(() =>
        {
            Assert.That(blocked.Radiance, Is.EqualTo(new Vector3(1f, 0f, 0f)));
            Assert.That(blocked.Transmittance, Is.EqualTo(0f));
            Assert.That(open.Radiance, Is.EqualTo(new Vector3(1f, 1f, 0f)));
            Assert.That(open.Transmittance, Is.EqualTo(0.125f));
        });
    }

    [Test]
    public void RadianceCascadesApproximatePointEmitterWithBlocker()
    {
        const int width = 16;
        const int height = 16;
        var mask = BuildBorderedRoom(width, height);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        for (var y = 5; y <= 11; y++)
            mask[Index(8, y, width)] = true;

        direct[Index(4, 8, width)] = new Vector3(8f, 6f, 3f);

        AssertRcCloseToBrute("point-emitter-blocker", mask, direct, fov, width, height, maxMae: 0.08f, maxRmse: 0.18f, maxAbs: 1.35f);
    }

    [Test]
    public void RadianceCascadesApproximateDoorwayTransfer()
    {
        const int width = 20;
        const int height = 12;
        var mask = BuildTwoRoomScene(width, height, doorwayOpen: true);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        direct[Index(5, 6, width)] = new Vector3(7f, 4f, 2f);
        direct[Index(6, 5, width)] = new Vector3(4f, 2f, 1f);

        var comparison = AssertRcCloseToBrute("doorway", mask, direct, fov, width, height, maxMae: 0.10f, maxRmse: 0.22f, maxAbs: 1.6f);
        var rightRoomNearDoor = Index(12, 6, width);

        Assert.That(comparison.Rc.Current[rightRoomNearDoor].Length(), Is.GreaterThan(0.01f));
    }

    [Test]
    public void RadianceCascadesApproximateTwoColoredEmitters()
    {
        const int width = 16;
        const int height = 16;
        var mask = BuildBorderedRoom(width, height);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        direct[Index(4, 8, width)] = new Vector3(7f, 0f, 0f);
        direct[Index(11, 8, width)] = new Vector3(0f, 0f, 7f);

        var comparison = AssertRcCloseToBrute("two-colored-emitters", mask, direct, fov, width, height, maxMae: 0.09f, maxRmse: 0.20f, maxAbs: 1.45f);
        Assert.Multiple(() =>
        {
            Assert.That(comparison.Rc.Current[Index(5, 8, width)].X, Is.GreaterThan(comparison.Rc.Current[Index(5, 8, width)].Z));
            Assert.That(comparison.Rc.Current[Index(10, 8, width)].Z, Is.GreaterThan(comparison.Rc.Current[Index(10, 8, width)].X));
        });
    }

    [Test]
    public void RadianceCascadesPreserveSymmetry()
    {
        const int width = 18;
        const int height = 14;
        var mask = BuildBorderedRoom(width, height);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        direct[Index(5, 7, width)] = new Vector3(5f, 4f, 3f);
        direct[Index(12, 7, width)] = new Vector3(5f, 4f, 3f);

        var comparison = AssertRcCloseToBrute("symmetry", mask, direct, fov, width, height, maxMae: 0.07f, maxRmse: 0.16f, maxAbs: 1.2f);

        for (var y = 1; y < height - 1; y++)
        {
            for (var x = 1; x < width / 2; x++)
            {
                var left = comparison.Rc.Current[Index(x, y, width)];
                var right = comparison.Rc.Current[Index(width - 1 - x, y, width)];
                Assert.That((left - right).Length(), Is.LessThan(0.08f), $"Symmetry mismatch at {x},{y}");
            }
        }
    }

    [Test]
    public void SealedRoomDoesNotLeakLight()
    {
        const int width = 20;
        const int height = 12;
        var mask = BuildTwoRoomScene(width, height, doorwayOpen: false);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        direct[Index(5, 6, width)] = new Vector3(7f, 5f, 3f);
        var comparison = AssertRcCloseToBrute("sealed-room", mask, direct, fov, width, height, maxMae: 0.08f, maxRmse: 0.18f, maxAbs: 1.2f);

        var rightRoomMax = 0f;
        for (var y = 1; y < height - 1; y++)
        {
            for (var x = width / 2 + 1; x < width - 1; x++)
                rightRoomMax = MathF.Max(rightRoomMax, comparison.Rc.Current[Index(x, y, width)].Length());
        }

        Assert.That(rightRoomMax, Is.LessThan(0.01f));
    }

    [Test]
    public void FovMaskPreventsGiLeakFromHiddenLight()
    {
        const int width = 8;
        const int height = 5;
        var mask = BuildBorderedRoom(width, height);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();

        direct[Index(2, 2, width)] = new Vector3(8f, 8f, 8f);
        fov[Index(2, 2, width)] = false;

        var rc = GlobalIlluminationReference.TraceRadianceCascades(
            mask,
            direct,
            new Vector3[width * height],
            fov,
            width,
            height,
            RcCascadeCount,
            RcBaseRays,
            RcBaseInterval,
            RcStep,
            RcBounceDecay);

        Assert.That(rc.Current.Max(x => x.Length()), Is.LessThan(0.001f));
    }

    [Test]
    public void GiDebugDumpWritesArtifactsAndMetricsWhenRequested()
    {
        if (!IsTruthy(Environment.GetEnvironmentVariable("ROBUST_GI_DUMP_DEBUG")))
            return;

        const int width = 32;
        const int height = 18;
        var mask = BuildTwoRoomScene(width, height, doorwayOpen: true);
        var direct = new Vector3[width * height];
        var fov = Enumerable.Repeat(true, width * height).ToArray();
        direct[Index(7, 9, width)] = new Vector3(8f, 5f, 2f);
        direct[Index(9, 8, width)] = new Vector3(4f, 1f, 0.5f);

        var comparison = CompareRcToBrute(mask, direct, fov, width, height);
        var nearest = GlobalIlluminationReference.JumpFloodNearestField(mask, width, height, new Vector2(1f, 1f));
        var distances = GlobalIlluminationReference.DistanceField(nearest, width, height);
        var outputDir = GetGiDumpDirectory();
        Directory.CreateDirectory(outputDir);

        WriteImage(Path.Combine(outputDir, "01-occlusion-mask.png"), width, height, (x, y) =>
            mask[Index(x, y, width)] ? new Rgba32(255, 255, 255, 255) : new Rgba32(0, 0, 0, 255));

        WriteImage(Path.Combine(outputDir, "02-direct-radiance.png"), width, height, (x, y) =>
            EncodeRadiance(direct[Index(x, y, width)], 0.25f));

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

        WriteImage(Path.Combine(outputDir, "04-bruteforce-reference.png"), width, height, (x, y) =>
            EncodeRadiance(comparison.Brute[Index(x, y, width)], 1f));

        WriteImage(Path.Combine(outputDir, "05-rc-current.png"), width, height, (x, y) =>
            EncodeRadiance(comparison.Rc.Current[Index(x, y, width)], 1f));

        WriteImage(Path.Combine(outputDir, "06-rc-error-heatmap.png"), width, height, (x, y) =>
        {
            var diff = (comparison.Rc.Current[Index(x, y, width)] - comparison.Brute[Index(x, y, width)]).Length();
            return Heat(diff * 2f);
        });

        for (var cascade = 0; cascade < comparison.Rc.Cascades.Length; cascade++)
        {
            WriteCascadeAtlasImage(Path.Combine(outputDir, $"07-rc-cascade-{cascade}-radiance.png"), comparison.Rc, cascade, sample =>
                EncodeRadiance(sample.Radiance, 0.6f));
            WriteCascadeAtlasImage(Path.Combine(outputDir, $"08-rc-cascade-{cascade}-transmittance.png"), comparison.Rc, cascade, sample =>
            {
                var value = ToByte(sample.Transmittance);
                return new Rgba32(value, value, value, 255);
            });
        }

        File.WriteAllText(
            Path.Combine(outputDir, "metrics.txt"),
            $"MAE={comparison.Metrics.MeanAbsoluteError}\nRMSE={comparison.Metrics.RootMeanSquaredError}\nMAX={comparison.Metrics.MaxAbsoluteError}\nMeanReference={comparison.Metrics.MeanReferenceMagnitude}\n");

        TestContext.Out.WriteLine($"Wrote GI debug dump images and metrics to: {outputDir}");
    }

    private static RcComparison AssertRcCloseToBrute(
        string sceneName,
        bool[] mask,
        Vector3[] direct,
        bool[] fov,
        int width,
        int height,
        float maxMae,
        float maxRmse,
        float maxAbs)
    {
        var comparison = CompareRcToBrute(mask, direct, fov, width, height);
        TestContext.Out.WriteLine($"{sceneName}: MAE={comparison.Metrics.MeanAbsoluteError:0.0000}, RMSE={comparison.Metrics.RootMeanSquaredError:0.0000}, MAX={comparison.Metrics.MaxAbsoluteError:0.0000}, MeanRef={comparison.Metrics.MeanReferenceMagnitude:0.0000}");

        Assert.Multiple(() =>
        {
            Assert.That(comparison.Metrics.MeanAbsoluteError, Is.LessThan(maxMae), sceneName);
            Assert.That(comparison.Metrics.RootMeanSquaredError, Is.LessThan(maxRmse), sceneName);
            Assert.That(comparison.Metrics.MaxAbsoluteError, Is.LessThan(maxAbs), sceneName);
        });

        return comparison;
    }

    private static RcComparison CompareRcToBrute(bool[] mask, Vector3[] direct, bool[] fov, int width, int height)
    {
        var maxDistance = GlobalIlluminationReference.RadianceCascadeIntervalEnd(RcBaseInterval, RcCascadeCount - 1);
        var brute = GlobalIlluminationReference.TraceBruteForceImage(
            mask,
            direct,
            fov,
            width,
            height,
            BruteForceRays,
            maxDistance,
            BruteForceStep,
            RcBounceDecay);
        var rc = GlobalIlluminationReference.TraceRadianceCascades(
            mask,
            direct,
            new Vector3[width * height],
            fov,
            width,
            height,
            RcCascadeCount,
            RcBaseRays,
            RcBaseInterval,
            RcStep,
            RcBounceDecay);
        var metrics = GlobalIlluminationReference.CompareImages(rc.Current, brute, mask);
        return new RcComparison(rc, brute, metrics);
    }

    private static bool[] BuildBorderedRoom(int width, int height)
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
        }

        return mask;
    }

    private static bool[] BuildTwoRoomScene(int width, int height, bool doorwayOpen)
    {
        var mask = BuildBorderedRoom(width, height);
        var splitX = width / 2;
        var doorCenter = height / 2;
        var doorHalfHeight = Math.Max(1, height / 8);

        for (var y = 1; y < height - 1; y++)
        {
            var inDoorway = y >= doorCenter - doorHalfHeight && y <= doorCenter + doorHalfHeight;
            if (!doorwayOpen || !inDoorway)
                mask[Index(splitX, y, width)] = true;
        }

        return mask;
    }

    private static int Index(int x, int y, int width)
    {
        return y * width + x;
    }

    private static string GetGiDumpDirectory()
    {
        var configured = Environment.GetEnvironmentVariable("ROBUST_GI_DUMP_DIR");
        return string.IsNullOrWhiteSpace(configured)
            ? Path.Combine(TestContext.CurrentContext.WorkDirectory, "GiDebugDumps", "radiance-cascade-reference")
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
                        image[x * scale + ox, y * scale + oy] = color;
                }
            }
        }

        image.Save(path);
    }

    private static void WriteCascadeAtlasImage(string path, GlobalIlluminationReference.RadianceCascadeResult rc, int cascade, Func<GlobalIlluminationReference.RadianceSample, Rgba32> encode)
    {
        var probeSize = rc.ProbeSizes[cascade];
        var atlasSize = rc.AtlasSizes[cascade];
        var directionCount = rc.DirectionCounts[cascade];
        var directionGrid = GlobalIlluminationReference.RadianceCascadeDirectionGrid(directionCount);
        var samples = rc.Cascades[cascade];

        WriteImage(path, atlasSize.X, atlasSize.Y, (x, y) =>
        {
            var probeX = x % probeSize.X;
            var probeY = y % probeSize.Y;
            var tileX = x / probeSize.X;
            var tileY = y / probeSize.Y;
            var directionIndex = tileY * directionGrid.X + tileX;
            if (directionIndex >= directionCount)
                return new Rgba32(0, 0, 0, 255);

            return encode(samples[GlobalIlluminationReference.DirectionalIndex(probeX, probeY, directionIndex, probeSize, directionCount)]);
        });
    }

    private static Rgba32 EncodeRadiance(Vector3 radiance, float exposure)
    {
        return new Rgba32(
            ToByte(ToneMap(radiance.X * exposure)),
            ToByte(ToneMap(radiance.Y * exposure)),
            ToByte(ToneMap(radiance.Z * exposure)),
            255);
    }

    private static Rgba32 Heat(float value)
    {
        value = Math.Clamp(value, 0f, 1f);
        return new Rgba32(
            ToByte(value),
            ToByte(MathF.Max(0f, 1f - MathF.Abs(value - 0.5f) * 2f)),
            ToByte(1f - value),
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

    private readonly record struct RcComparison(
        GlobalIlluminationReference.RadianceCascadeResult Rc,
        Vector3[] Brute,
        GlobalIlluminationReference.ErrorMetrics Metrics);

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
