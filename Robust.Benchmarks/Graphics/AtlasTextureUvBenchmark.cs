using BenchmarkDotNet.Attributes;
using Robust.Client.Graphics;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;

namespace Robust.Benchmarks.Graphics;

[MemoryDiagnoser]
public class AtlasTextureUvBenchmark
{
    private AtlasTexture _atlas = default!;

    [GlobalSetup]
    public void Setup()
    {
        _atlas = new AtlasTexture(new TestTexture((256, 256)), UIBox2.FromDimensions(48, 80, 16, 16));
    }

    [Benchmark(Baseline = true)]
    public Box2 CalculateUvsPerDraw()
    {
        var region = _atlas.SubRegion;
        var (width, height) = _atlas.SourceTexture.Size;
        return new Box2(
            region.Left / width,
            (height - region.Bottom) / height,
            region.Right / width,
            (height - region.Top) / height);
    }

    [Benchmark]
    public Box2 UseCachedUvs() => _atlas.NormalizedSubRegion;

    private sealed class TestTexture(Vector2i size) : Texture(size)
    {
        public override Color GetPixel(int x, int y) => default;
    }
}
