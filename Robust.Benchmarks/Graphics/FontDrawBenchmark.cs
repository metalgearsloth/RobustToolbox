using System;
using System.Numerics;
using System.Text;
using BenchmarkDotNet.Attributes;
using Robust.Client.GameObjects;
using Robust.Client.Graphics;
using Robust.Shared.GameObjects;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;

namespace Robust.Benchmarks.Graphics;

[MemoryDiagnoser]
public class FontDrawBenchmark
{
    private readonly TextOutline _outline = new(1, Color.Black);
    private readonly string _text = new('A', 256);
    private Font _font = default!;
    private Font _legacyFont = default!;
    private TestDrawingHandle _handle = default!;

    [GlobalSetup]
    public void Setup()
    {
        var texture = new TestTexture(new Vector2i(16, 16));
        var fontHandle = new TestFontInstanceHandle(texture);
        _font = new VectorFont(fontHandle, 16);
        _legacyFont = new LegacyVectorFont(fontHandle);
        _handle = new TestDrawingHandle(texture);
    }

    [Benchmark(Baseline = true)]
    public Vector2 LegacyDrawString() => _handle.DrawString(_legacyFont, Vector2.Zero, _text, 1, Color.White);

    [Benchmark]
    public Vector2 DrawString() => _handle.DrawString(_font, Vector2.Zero, _text, 1, Color.White);

    [Benchmark]
    public Vector2 LegacyDrawOutlinedString() => _handle.DrawString(_legacyFont, Vector2.Zero, _text, 1, Color.White, _outline);

    [Benchmark]
    public Vector2 DrawOutlinedString() => _handle.DrawString(_font, Vector2.Zero, _text, 1, Color.White, _outline);

    private sealed class LegacyVectorFont(IFontInstanceHandle fontHandle) : Font
    {
        public override int GetAscent(float scale) => fontHandle.GetAscent(scale);
        public override int GetHeight(float scale) => fontHandle.GetHeight(scale);
        public override int GetDescent(float scale) => fontHandle.GetDescent(scale);
        public override int GetLineHeight(float scale) => fontHandle.GetLineHeight(scale);

        public override float DrawChar(DrawingHandleBase handle, Rune rune, Vector2 baseline, float scale, Color color,
            bool fallback = true)
        {
            var metrics = fontHandle.GetCharMetrics(rune, scale);
            if (metrics == null)
                return 0;

            var texture = fontHandle.GetCharTexture(rune, scale);
            if (texture == null)
                return metrics.Value.Advance;

            handle.DrawTexture(texture, baseline + new Vector2(metrics.Value.BearingX, -metrics.Value.BearingY), color);
            return metrics.Value.Advance;
        }

        public override float DrawChar(DrawingHandleBase handle, Rune rune, Vector2 baseline, float scale, Color color,
            TextOutline? outline, bool fallback = true)
        {
            var metrics = fontHandle.GetCharMetrics(rune, scale);
            if (metrics == null)
                return 0;

            var texture = fontHandle.GetCharTexture(rune, scale);
            if (texture == null)
                return metrics.Value.Advance;

            if (outline is { Thickness: > 0 } settings &&
                fontHandle.GetOutlinedChar(rune, scale, settings.Thickness) is { } outlinedGlyph)
            {
                handle.DrawTexture(outlinedGlyph.Texture,
                    baseline + new Vector2(outlinedGlyph.Left, -outlinedGlyph.Top), settings.Color);
            }

            handle.DrawTexture(texture, baseline + new Vector2(metrics.Value.BearingX, -metrics.Value.BearingY), color);
            return metrics.Value.Advance;
        }

        public override float DrawCharOutline(DrawingHandleBase handle, Rune rune, Vector2 baseline, float scale,
            TextOutline outline, bool fallback = true)
        {
            var metrics = fontHandle.GetCharMetrics(rune, scale);
            if (metrics == null)
                return 0;

            if (outline.Thickness > 0 &&
                fontHandle.GetOutlinedChar(rune, scale, outline.Thickness) is { } outlinedGlyph)
            {
                handle.DrawTexture(outlinedGlyph.Texture,
                    baseline + new Vector2(outlinedGlyph.Left, -outlinedGlyph.Top), outline.Color);
            }

            return metrics.Value.Advance;
        }

        public override CharMetrics? GetCharMetrics(Rune rune, float scale, bool fallback = true)
            => fontHandle.GetCharMetrics(rune, scale);
    }

    private sealed class TestFontInstanceHandle(Texture texture) : IFontInstanceHandle
    {
        private static readonly CharMetrics Metrics = new(0, 16, 16, 16, 16);

        public bool TryGetGlyph(
            Rune codePoint,
            float scale,
            float outlineThickness,
            out CharMetrics metrics,
            out Texture? glyphTexture,
            out OutlinedGlyph? outlinedGlyph)
        {
            metrics = Metrics;
            glyphTexture = texture;
            outlinedGlyph = outlineThickness > 0 ? new OutlinedGlyph(texture, 0, 16) : null;
            return true;
        }

        public Texture? GetCharTexture(Rune codePoint, float scale) => texture;
        public OutlinedGlyph? GetOutlinedChar(Rune codePoint, float scale, float thickness)
            => thickness > 0 ? new OutlinedGlyph(texture, 0, 16) : null;
        public CharMetrics? GetCharMetrics(Rune codePoint, float scale) => Metrics;
        public int GetAscent(float scale) => 16;
        public int GetDescent(float scale) => 4;
        public int GetHeight(float scale) => 20;
        public int GetLineHeight(float scale) => 20;
    }

    private sealed class TestTexture(Vector2i size) : Texture(size)
    {
        public override Color GetPixel(int x, int y) => default;
    }

    private sealed class TestDrawingHandle(Texture white) : DrawingHandleScreen(white)
    {
        public override void SetTransform(in Matrix3x2 matrix) { }
        public override Matrix3x2 GetTransform() => Matrix3x2.Identity;
        public override void UseShader(ShaderInstance? shader) { }
        public override ShaderInstance? GetShader() => null;
        public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
            ReadOnlySpan<DrawVertexUV2DColor> vertices) { }
        public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
            ReadOnlySpan<ushort> indices, ReadOnlySpan<DrawVertexUV2DColor> vertices) { }
        public override void DrawCircle(Vector2 position, float radius, Color color, bool filled = true) { }
        public override void DrawLine(Vector2 from, Vector2 to, Color color) { }
        public override void RenderInRenderTarget(IRenderTarget target, Action a, Color? clearColor) { }
        public override void DrawTexture(Texture texture, Vector2 position, Color? modulate = null) { }
        public override void DrawRect(UIBox2 rect, Color color, bool filled = true) { }
        public override void DrawTextureRectRegion(Texture texture, UIBox2 rect, UIBox2? subRegion = null,
            Color? modulate = null) { }
        public override void DrawEntity(EntityUid entity, Vector2 position, Vector2 scale, Angle? worldRot,
            Angle eyeRotation = default, Direction? overrideDirection = null, SpriteComponent? sprite = null,
            TransformComponent? xform = null, SharedTransformSystem? xformSystem = null) { }
    }
}
