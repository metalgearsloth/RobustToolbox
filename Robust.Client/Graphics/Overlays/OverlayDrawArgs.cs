using JetBrains.Annotations;
using System.Collections.Generic;
using System.Numerics;
using Robust.Client.GameObjects;
using Robust.Client.UserInterface.CustomControls;
using Robust.Shared.Enums;
using Robust.Shared.GameObjects;
using Robust.Shared.Graphics;
using Robust.Shared.Map;
using Robust.Shared.Maths;

namespace Robust.Client.Graphics
{
    /// <summary>
    ///     Parameters passed to <see cref="Overlay.Draw"/>.
    /// </summary>
    [PublicAPI]
    public readonly ref struct OverlayDrawArgs
    {
        /// <summary>
        ///     The overlay space that currently is being rendered for.
        /// </summary>
        public readonly OverlaySpace Space;

        /// <summary>
        ///     The viewport control that is rendering this viewport.
        ///     Not always available.
        /// </summary>
        public readonly IViewportControl? ViewportControl;

        /// <summary>
        ///     The viewport that is rendering this viewport.
        /// </summary>
        public readonly IClydeViewport Viewport;

        /// <summary>
        ///     The drawing handle that you can draw with.
        /// </summary>
        public readonly DrawingHandleBase DrawingHandle;

        /// <summary>
        ///     The screen-space coordinates available to render within.
        ///     Relevant for screen-space overlay rendering.
        /// </summary>
        public readonly UIBox2i ViewportBounds;

        public readonly EntityUid MapUid;

        /// <summary>
        /// <see cref="MapId"/> currently being rendered.
        /// </summary>
        public readonly MapId MapId;

        /// <summary>
        /// Map entity controlled by the viewport's real eye. This remains stable while individual maps in a
        /// z-stack are rendered through <see cref="MapUid"/>.
        /// </summary>
        public readonly EntityUid ViewedMapUid;

        /// <summary>
        /// Map controlled by the viewport's real eye. Background and current-view-only overlays should use this
        /// instead of the per-layer <see cref="MapId"/>.
        /// </summary>
        public readonly MapId ViewedMapId;

        /// <summary>
        ///     Z-level offset of <see cref="MapId"/> relative to the viewport eye's map.
        ///     Zero is the eye map, negative values are below, positive values are above.
        /// </summary>
        public readonly int ZLevelOffset;

        /// <summary>
        /// Whether this is the farthest visible z-level layer, where background-only overlays such as parallax
        /// should be drawn. World overlays that belong to a map should generally use <see cref="ZLevelOffset"/>
        /// instead and draw on every applicable layer.
        /// </summary>
        public readonly bool IsZLevelBackground;

        /// <summary>
        /// Whether this pass is rendering the map currently viewed by the eye.
        /// </summary>
        public bool IsViewedMap => MapId == ViewedMapId;

        /// <summary>
        /// The viewport's real presented eye. Unlike <see cref="LayerEye"/>, this is not displaced while lower
        /// or upper maps in the z-stack are rendered.
        /// </summary>
        public readonly IEye? ViewEye;

        /// <summary>
        /// The projected eye used for this layer. During a z-stack pass this can differ from the viewport's
        /// controlling eye, so world overlays should use this value for layer-local rotation and scale.
        /// </summary>
        public readonly IEye? LayerEye;

        /// <summary>
        /// World translation that makes geometry drawn through <see cref="LayerEye"/> appear as though it were
        /// drawn through <see cref="ViewEye"/>. Background overlays rendered once behind a z stack can apply this
        /// without knowing the network projection offset or the current layer depth.
        /// </summary>
        public Vector2 ViewToLayerWorldOffset
        {
            get
            {
                if (ViewEye == null || LayerEye == null)
                    return Vector2.Zero;

                var viewCenter = ViewEye.Position.Position + ViewEye.Offset;
                var layerCenter = LayerEye.Position.Position + LayerEye.Offset;
                return layerCenter - viewCenter;
            }
        }

        /// <summary>
        /// Maps whose world layers are present in the viewport's completed z-stack.
        /// </summary>
        public readonly IReadOnlySet<EntityUid> VisibleMaps;

        /// <summary>
        ///     AABB enclosing the area visible in the viewport.
        /// </summary>
        public readonly Box2 WorldAABB;

        /// <summary>
        ///     <see cref="Box2Rotated"/> of the area visible in the viewport.
        /// </summary>
        public readonly Box2Rotated WorldBounds;

        public readonly IRenderHandle RenderHandle;

        private readonly TransformSystem? _transformSystem;

        public DrawingHandleScreen ScreenHandle => (DrawingHandleScreen) DrawingHandle;
        public DrawingHandleWorld WorldHandle => (DrawingHandleWorld) DrawingHandle;

        /// <summary>
        /// Gets the exact layer sample used by sprite rendering for an entity in this overlay pass. Entity-attached
        /// overlays should use its position, rotation and opacity instead of filtering by the simulation map.
        /// </summary>
        public bool TryGetEntityRenderLayer(EntityUid entity, out RenderLayerSample sample)
        {
            if (_transformSystem != null && MapUid != EntityUid.Invalid)
                return _transformSystem.TryGetRenderLayerSample(entity, MapUid, out sample);

            sample = default;
            return false;
        }

        /// <summary>
        /// Gets the exact world matrix and opacity used to render an entity in this layer. Grid and entity-owned
        /// overlays should use this when drawing local geometry so moving and rotating parent interpolation stays
        /// aligned with the renderer.
        /// </summary>
        public bool TryGetEntityRenderMatrix(EntityUid entity, out Matrix3x2 matrix, out float opacity)
        {
            if (TryGetEntityRenderLayer(entity, out var sample))
            {
                matrix = Matrix3Helpers.CreateTransform(sample.Position, sample.Rotation);
                opacity = sample.Opacity;
                return true;
            }

            matrix = Matrix3x2.Identity;
            opacity = 0f;
            return false;
        }

        /// <summary>
        /// Gets an entity attachment's single post-stack pose in the controlling eye's map. The returned opacity
        /// is the sum of the entity's actual renderer samples that are present in <see cref="VisibleMaps"/>.
        /// Screen-space entity attachments should use this rather than selecting a simulation map themselves.
        /// </summary>
        public bool TryGetEntityPresentedView(EntityUid entity, out PresentedViewSample sample)
        {
            if (_transformSystem != null && ViewedMapUid != EntityUid.Invalid)
            {
                return _transformSystem.TryGetPresentedViewSample(
                    entity,
                    ViewedMapUid,
                    VisibleMaps,
                    out sample);
            }

            sample = default;
            return false;
        }

        /// <summary>
        /// Gets a post-stack entity attachment matrix and its combined visible opacity.
        /// </summary>
        public bool TryGetEntityPresentedViewMatrix(EntityUid entity, out Matrix3x2 matrix, out float opacity)
        {
            if (TryGetEntityPresentedView(entity, out var sample))
            {
                matrix = Matrix3Helpers.CreateTransform(sample.Position, sample.Rotation);
                opacity = sample.Opacity;
                return true;
            }

            matrix = Matrix3x2.Identity;
            opacity = 0f;
            return false;
        }

        /// <summary>
        /// Projects map-surface coordinates into this layer's render space. Map-owned overlays drawing positions
        /// from another compatible z map should use this rather than reproducing projection-offset arithmetic.
        /// </summary>
        public bool TryProjectMapCoordinates(MapCoordinates coordinates, out Vector2 position)
        {
            if (_transformSystem != null && MapUid != EntityUid.Invalid)
                return _transformSystem.TryProjectMapCoordinatesForLayer(coordinates, MapUid, out position);

            position = coordinates.Position;
            return false;
        }

        /// <summary>
        /// Gets this layer's visible bounds expressed in a compatible source map's canonical coordinates.
        /// Use this for broadphase, grid, or component-tree queries whose results will subsequently be filtered
        /// through <see cref="TryGetEntityRenderLayer"/>.
        /// </summary>
        public bool TryGetMapRenderBounds(
            EntityUid sourceMap,
            out MapId sourceMapId,
            out Box2Rotated sourceBounds)
        {
            if (_transformSystem != null && MapUid != EntityUid.Invalid)
            {
                return _transformSystem.TryGetMapRenderBoundsForLayer(
                    sourceMap,
                    MapUid,
                    WorldBounds,
                    out sourceMapId,
                    out sourceBounds);
            }

            sourceMapId = MapId.Nullspace;
            sourceBounds = WorldBounds;
            return false;
        }

        internal OverlayDrawArgs(
            OverlaySpace space,
            IViewportControl? viewportControl,
            IClydeViewport viewport,
            IRenderHandle renderHandle,
            in UIBox2i viewportBounds,
            in EntityUid mapUid,
            in MapId mapId,
            in EntityUid viewedMapUid,
            in MapId viewedMapId,
            int zLevelOffset,
            bool isZLevelBackground,
            IEye? viewEye,
            IEye? layerEye,
            IReadOnlySet<EntityUid> visibleMaps,
            TransformSystem? transformSystem,
            in Box2 worldAabb,
            in Box2Rotated worldBounds)
        {
            DrawingHandle = space is OverlaySpace.ScreenSpace or OverlaySpace.ScreenSpaceBelowWorld
                ? renderHandle.DrawingHandleScreen
                : renderHandle.DrawingHandleWorld;

            Space = space;
            ViewportControl = viewportControl;
            Viewport = viewport;
            RenderHandle = renderHandle;
            ViewportBounds = viewportBounds;
            MapUid = mapUid;
            MapId = mapId;
            ViewedMapUid = viewedMapUid;
            ViewedMapId = viewedMapId;
            ZLevelOffset = zLevelOffset;
            IsZLevelBackground = isZLevelBackground;
            ViewEye = viewEye;
            LayerEye = layerEye;
            VisibleMaps = visibleMaps;
            _transformSystem = transformSystem;
            WorldAABB = worldAabb;
            WorldBounds = worldBounds;
        }
    }
}
