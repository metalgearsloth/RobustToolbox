using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using NUnit.Framework;
using Robust.Client.GameObjects;
using Robust.Client.Graphics;
using Robust.Shared.GameObjects;
using Robust.Shared.Graphics;
using Robust.Shared.Maths;

namespace Robust.Client.Tests.Graphics;

[TestFixture]
internal sealed class FontOutlineDrawOrderTest
{
    [Test]
    public void DrawStringDrawsTheCompleteOutlinePassBeforeTheFillPass()
    {
        var font = new RecordingFont();
        var texture = new TestTexture(new Vector2i(1, 1));
        var handle = new TestDrawingHandle(texture);

        handle.DrawString(font, Vector2.Zero, "ab", 1, Color.White, TextOutline.Default);

        Assert.That(font.DrawCalls, Is.EqualTo(new[] { "outline:a", "outline:b", "fill:a", "fill:b" }));
    }

    private sealed class RecordingFont : Font
    {
        public List<string> DrawCalls { get; } = new();

        public override int GetAscent(float scale) => 1;
        public override int GetHeight(float scale) => 1;
        public override int GetDescent(float scale) => 0;
        public override int GetLineHeight(float scale) => 1;

        public override float DrawChar(DrawingHandleBase handle, Rune rune, Vector2 baseline, float scale,
            Color color, bool fallback = true)
        {
            DrawCalls.Add($"fill:{rune}");
            return 1;
        }

        public override float DrawCharOutline(DrawingHandleBase handle, Rune rune, Vector2 baseline, float scale,
            TextOutline outline, bool fallback = true)
        {
            DrawCalls.Add($"outline:{rune}");
            return 1;
        }

        public override CharMetrics? GetCharMetrics(Rune rune, float scale, bool fallback = true)
            => new CharMetrics(0, 1, 1, 1, 1);
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
