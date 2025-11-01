using System;

namespace Robust.Shared.Physics
{
    public static class PhysicsConstants
    {
        public const int LengthUnitsPerMetre = 1;

        // Used to detect bad values. Positions greater than about 16km will have precision
        // problems, so 100km as a limit should be fine in all cases.
        public const float Huge = (100000.0f * LengthUnitsPerMetre);

        /// <summary>
        /// The radius of the polygon/edge shape skin. This should not be modified. Making
        /// this smaller means polygons will have an insufficient buffer for continuous collision.
        /// Making it larger may create artifacts for vertex collision.
        /// </summary>
        /// <remarks>
        ///     Default is set to be 2 x linearslop.
        /// </remarks>
        public const float PolygonRadius = 2 * LinearSlop;

        /// <summary>
        /// Minimum buffer distance for positions.
        /// </summary>
        public const float LinearSlop = 0.005f;

        /// <summary>
        /// Minimum buffer distance for angles.
        /// </summary>
        public const float AngularSlop = 2.0f / 180.0f * MathF.PI;

        public const byte MaxPolygonVertices = 8;

        public const float DefaultContactFriction = 0.4f;

        public const float DefaultRestitution = 0f;

        public const float DefaultDensity = 1f;

        // Maximum number of colors in the constraint graph. Constraints that cannot
        // find a color are added to the overflow set which are solved single-threaded.
        // The compound barrel benchmark has minor overflow with 24 colors
        public const int GraphColorCount = 24;

        // Box2D uses limited speculative collision. This reduces jitter.
        // Normally this is 2cm.
        // @warning modifying this can have a significant impact on performance and stability
        public const float SpeculativeDistance = 4.0f * LinearSlop;

        public const int NullIndex = -1;
    }
}
