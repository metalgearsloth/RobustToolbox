﻿using System;
using System.Runtime.InteropServices;
using Robust.Client.GameObjects;
using Robust.Shared.GameObjects;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics.Clyde
{
    // RenderHandle contains the public/internal API surface to control actual rendering operations in Clyde.

    internal partial class Clyde
    {
        private RenderHandle _renderHandle = default!;

        private sealed class RenderHandle : IRenderHandle
        {
            private readonly Clyde _clyde;

            public DrawingHandleScreen DrawingHandleScreen { get; }
            public DrawingHandleWorld DrawingHandleWorld { get; }

            public RenderHandle(Clyde clyde)
            {
                _clyde = clyde;

                DrawingHandleScreen = new DrawingHandleScreenImpl(this);
                DrawingHandleWorld = new DrawingHandleWorldImpl(this);
            }

            public void SetModelTransform(in Matrix3 matrix)
            {
                _clyde.DrawSetModelTransform(matrix);
            }

            public void SetProjView(in Matrix3 proj, in Matrix3 view)
            {
                _clyde.DrawSetProjViewTransform(proj, view);
            }

            public void DrawTextureScreen(Texture texture, Vector2 bl, Vector2 br, Vector2 tl, Vector2 tr,
                in Color modulate, in UIBox2? subRegion)
            {
                var clydeTexture = ExtractTexture(texture, subRegion, out var csr);

                var (w, h) = clydeTexture.Size;
                var sr = new Box2(csr.Left / w, (h - csr.Top) / h, csr.Right / w, (h - csr.Bottom) / h);

                _clyde.DrawTexture(clydeTexture.TextureId, bl, br, tl, tr, modulate, sr);
            }

            public void DrawTextureWorld(Texture texture, Vector2 bl, Vector2 br, Vector2 tl, Vector2 tr,
                Color modulate, in UIBox2? subRegion)
            {
                var clydeTexture = ExtractTexture(texture, subRegion, out var csr);

                var (w, h) = clydeTexture.Size;
                var sr = new Box2(csr.Left / w, (h - csr.Bottom) / h, csr.Right / w, (h - csr.Top) / h);

                _clyde.DrawTexture(clydeTexture.TextureId, bl, br, tl, tr, modulate, sr);
            }

            private static ClydeTexture ExtractTexture(Texture texture, in UIBox2? subRegion, out UIBox2 sr)
            {
                if (texture is AtlasTexture atlas)
                {
                    texture = atlas.SourceTexture;
                    if (subRegion.HasValue)
                    {
                        var offset = atlas.SubRegion.TopLeft;
                        sr = new UIBox2(
                            subRegion.Value.TopLeft + offset,
                            subRegion.Value.BottomRight + offset);
                    }
                    else
                    {
                        sr = atlas.SubRegion;
                    }
                }
                else
                {
                    sr = subRegion ?? new UIBox2(0, 0, texture.Width, texture.Height);
                }

                var clydeTexture = (ClydeTexture) texture;
                return clydeTexture;
            }

            public void SetScissor(UIBox2i? scissorBox)
            {
                _clyde.DrawSetScissor(scissorBox);
            }

            public void DrawEntity(IEntity entity, Vector2 position, Vector2 scale, Direction? overrideDirection)
            {
                if (entity.Deleted)
                {
                    throw new ArgumentException("Tried to draw an entity has been deleted.", nameof(entity));
                }

                var sprite = entity.GetComponent<SpriteComponent>();

                var oldProj = _clyde._currentMatrixProj;
                var oldView = _clyde._currentMatrixView;

                // Switch rendering to pseudo-world space.
                {
                    CalcWorldProjMatrix(_clyde._currentRenderTarget.Size, out var proj);

                    var ofsX = position.X - _clyde.ScreenSize.X / 2f;
                    var ofsY = position.Y - _clyde.ScreenSize.Y / 2f;

                    var view = Matrix3.Identity;
                    view.R0C0 = scale.X;
                    view.R1C1 = scale.Y;
                    view.R0C2 = ofsX / EyeManager.PixelsPerMeter;
                    view.R1C2 = -ofsY / EyeManager.PixelsPerMeter;

                    SetProjView(proj, view);
                }

                // Draw the entity.
                sprite.Render(
                    DrawingHandleWorld,
                    overrideDirection == null
                        ? entity.Transform.WorldRotation
                        : Angle.Zero,
                    overrideDirection);

                // Reset to screen space
                SetProjView(oldProj, oldView);
            }

            public void DrawLine(Vector2 a, Vector2 b, Color color)
            {
                _clyde.DrawLine(a, b, color);
            }

            public void UseShader(ShaderInstance? shader)
            {
                if (shader != null && shader.Disposed)
                {
                    throw new ArgumentException("Unable to use disposed shader instance.", nameof(shader));
                }

                var clydeShader = (ClydeShaderInstance?) shader;

                _clyde.DrawUseShader(clydeShader?.Handle ?? _clyde._defaultShader.Handle);
            }

            public void Viewport(Box2i viewport)
            {
                _clyde.DrawViewport(viewport);
            }

            public void UseRenderTarget(IRenderTarget? renderTarget)
            {
                var target = (RenderTexture?) renderTarget;

                _clyde.DrawRenderTarget(target?.Handle ?? default);
            }

            public void Clear(Color color)
            {
                _clyde.DrawClear(color);
            }

            public void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, ReadOnlySpan<Vector2> vertices,
                Color color)
            {
                // TODO: Maybe don't stackalloc if the data is too large.
                Span<DrawVertexUV2D> drawVertices = stackalloc DrawVertexUV2D[vertices.Length];
                PadVertices(vertices, drawVertices);

                DrawPrimitives(primitiveTopology, Texture.White, drawVertices, color);
            }

            public void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, ReadOnlySpan<ushort> indices,
                ReadOnlySpan<Vector2> vertices, Color color)
            {
                // TODO: Maybe don't stackalloc if the data is too large.
                Span<DrawVertexUV2D> drawVertices = stackalloc DrawVertexUV2D[vertices.Length];
                PadVertices(vertices, drawVertices);

                DrawPrimitives(primitiveTopology, Texture.White, indices, drawVertices, color);
            }

            public void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
                ReadOnlySpan<DrawVertexUV2D> vertices, Color color)
            {
                if (!(texture is ClydeTexture clydeTexture))
                {
                    throw new ArgumentException("Texture must be a basic texture.");
                }

                var castSpan = MemoryMarshal.Cast<DrawVertexUV2D, Vertex2D>(vertices);

                _clyde.DrawPrimitives(primitiveTopology, clydeTexture.TextureId, castSpan, color);
            }

            public void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
                ReadOnlySpan<ushort> indices,
                ReadOnlySpan<DrawVertexUV2D> vertices, Color color)
            {
                if (!(texture is ClydeTexture clydeTexture))
                {
                    throw new ArgumentException("Texture must be a basic texture.");
                }

                var castSpan = MemoryMarshal.Cast<DrawVertexUV2D, Vertex2D>(vertices);

                _clyde.DrawPrimitives(primitiveTopology, clydeTexture.TextureId, indices, castSpan, color);
            }

            private void PadVertices(ReadOnlySpan<Vector2> input, Span<DrawVertexUV2D> output)
            {
                for (var i = 0; i < output.Length; i++)
                {
                    output[i] = new DrawVertexUV2D(input[i], (0.5f, 0.5f));
                }
            }

            private sealed class DrawingHandleScreenImpl : DrawingHandleScreen
            {
                private readonly RenderHandle _renderHandle;

                public DrawingHandleScreenImpl(RenderHandle renderHandle)
                {
                    _renderHandle = renderHandle;
                }

                public override void SetTransform(in Matrix3 matrix)
                {
                    _renderHandle.SetModelTransform(matrix);
                }

                public override void UseShader(ShaderInstance? shader)
                {
                    _renderHandle.UseShader(shader);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology,
                    ReadOnlySpan<Vector2> vertices,
                    Color color)
                {
                    var realColor = color * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, vertices, realColor);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology,
                    ReadOnlySpan<ushort> indices,
                    ReadOnlySpan<Vector2> vertices, Color color)
                {
                    var realColor = color * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, indices, vertices, realColor);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
                    ReadOnlySpan<DrawVertexUV2D> vertices, Color? color = null)
                {
                    var realColor = (color ?? Color.White) * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, texture, vertices, realColor);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
                    ReadOnlySpan<ushort> indices, ReadOnlySpan<DrawVertexUV2D> vertices, Color? color = null)
                {
                    var realColor = (color ?? Color.White) * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, texture, indices, vertices, realColor);
                }

                public override void DrawLine(Vector2 from, Vector2 to, Color color)
                {
                    _renderHandle.DrawLine(@from, to, color * Modulate);
                }

                public override void DrawRect(UIBox2 rect, Color color, bool filled = true)
                {
                    if (filled)
                    {
                        DrawTextureRect(Texture.White, rect, color);
                    }
                    else
                    {
                        DrawLine(rect.TopLeft, rect.TopRight, color);
                        DrawLine(rect.TopRight, rect.BottomRight, color);
                        DrawLine(rect.BottomRight, rect.BottomLeft, color);
                        DrawLine(rect.BottomLeft, rect.TopLeft, color);
                    }
                }

                public override void DrawTextureRectRegion(Texture texture, UIBox2 rect, UIBox2? subRegion = null,
                    Color? modulate = null)
                {
                    var color = (modulate ?? Color.White) * Modulate;
                    _renderHandle.DrawTextureScreen(texture, rect.TopLeft, rect.TopRight,
                        rect.BottomLeft, rect.BottomRight, color, subRegion);
                }
            }

            private sealed class DrawingHandleWorldImpl : DrawingHandleWorld
            {
                private readonly RenderHandle _renderHandle;

                public DrawingHandleWorldImpl(RenderHandle renderHandle)
                {
                    _renderHandle = renderHandle;
                }

                public override void SetTransform(in Matrix3 matrix)
                {
                    _renderHandle.SetModelTransform(matrix);
                }

                public override void UseShader(ShaderInstance? shader)
                {
                    _renderHandle.UseShader(shader);
                }

                public override void DrawCircle(Vector2 position, float radius, Color color, bool filled = true)
                {
                    //TODO: Scale number of sides based on radius
                    const int Divisions = 8;
                    const float ArcLength = MathF.PI * 2 / Divisions;

                    var filledTriangle = new Vector2[3];

                    // Draws a "circle", but its just a polygon with a bunch of sides
                    // this is the GL_LINES version, not GL_LINE_STRIP
                    for (int i = 0; i < Divisions; i++)
                    {
                        var startPos = new Vector2(MathF.Cos(ArcLength * i) * radius, MathF.Sin(ArcLength * i) * radius);
                        var endPos = new Vector2(MathF.Cos(ArcLength * (i+1)) * radius, MathF.Sin(ArcLength * (i + 1)) * radius);

                        if(!filled)
                            _renderHandle.DrawLine(startPos, endPos, color);
                        else
                        {
                            filledTriangle[0] = startPos;
                            filledTriangle[1] = endPos;
                            filledTriangle[2] = Vector2.Zero;

                            _renderHandle.DrawPrimitives(DrawPrimitiveTopology.TriangleList, filledTriangle, color);
                        }
                    }
                }

                public override void DrawLine(Vector2 from, Vector2 to, Color color)
                {
                    _renderHandle.DrawLine(@from, to, color * Modulate);
                }

                public override void DrawRect(Box2 rect, Color color, bool filled = true)
                {
                    if (filled)
                    {
                        DrawTextureRect(Texture.White, rect, color);
                    }
                    else
                    {
                        DrawLine(rect.TopLeft, rect.TopRight, color);
                        DrawLine(rect.TopRight, rect.BottomRight, color);
                        DrawLine(rect.BottomRight, rect.BottomLeft, color);
                        DrawLine(rect.BottomLeft, rect.TopLeft, color);
                    }
                }

                public override void DrawRect(in Box2Rotated rect, Color color, bool filled = true)
                {
                    if (filled)
                    {
                        DrawTextureRect(Texture.White, rect, color);
                    }
                    else
                    {
                        DrawLine(rect.TopLeft, rect.TopRight, color);
                        DrawLine(rect.TopRight, rect.BottomRight, color);
                        DrawLine(rect.BottomRight, rect.BottomLeft, color);
                        DrawLine(rect.BottomLeft, rect.TopLeft, color);
                    }
                }

                public override void DrawTextureRectRegion(Texture texture, Box2 rect, UIBox2? subRegion = null,
                    Color? modulate = null)
                {
                    var color = (modulate ?? Color.White) * Modulate;

                    _renderHandle.DrawTextureWorld(texture, rect.BottomLeft, rect.BottomRight,
                        rect.TopLeft, rect.TopRight, color, subRegion);
                }

                public override void DrawTextureRectRegion(Texture texture, in Box2Rotated rect,
                    UIBox2? subRegion = null, Color? modulate = null)
                {
                    var color = (modulate ?? Color.White) * Modulate;

                    _renderHandle.DrawTextureWorld(texture, rect.BottomLeft, rect.BottomRight,
                        rect.TopLeft, rect.TopRight, color, subRegion);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology,
                    ReadOnlySpan<Vector2> vertices,
                    Color color)
                {
                    var realColor = color * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, vertices, realColor);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology,
                    ReadOnlySpan<ushort> indices,
                    ReadOnlySpan<Vector2> vertices, Color color)
                {
                    var realColor = color * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, indices, vertices, realColor);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
                    ReadOnlySpan<DrawVertexUV2D> vertices, Color? color = null)
                {
                    var realColor = (color ?? Color.White) * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, texture, vertices, realColor);
                }

                public override void DrawPrimitives(DrawPrimitiveTopology primitiveTopology, Texture texture,
                    ReadOnlySpan<ushort> indices, ReadOnlySpan<DrawVertexUV2D> vertices, Color? color = null)
                {
                    var realColor = (color ?? Color.White) * Modulate;

                    _renderHandle.DrawPrimitives(primitiveTopology, texture, indices, vertices, realColor);
                }
            }
        }
    }
}
