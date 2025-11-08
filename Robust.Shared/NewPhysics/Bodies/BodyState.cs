using System.Numerics;
using System.Runtime.InteropServices;
using Robust.Shared.Physics;

namespace Robust.Shared.NewPhysics.Bodies;

// Body State
// The body state is designed for fast conversion to and from SIMD via scatter-gather.
// Only awake dynamic and kinematic bodies have a body state.
// This is used in the performance critical constraint solver
//
// The solver operates on the body state. The body state array does not hold static bodies. Static bodies are shared
// across worker threads. It would be okay to read their states, but writing to them would cause cache thrashing across
// workers, even if the values don't change.
// This causes some trouble when computing anchors. I rotate joint anchors using the body rotation every sub-step. For static
// bodies the anchor doesn't rotate. Body A or B could be static and this can lead to lots of branching. This branching
// should be minimized.
//
// Solution 1:
// Use delta rotations. This means anchors need to be prepared in world space. The delta rotation for static bodies will be
// identity using a dummy state. Base separation and angles need to be computed. Manifolds will be behind a frame, but that
// is probably best if bodies move fast.
//
// Solution 2:
// Use full rotation. The anchors for static bodies will be in world space while the anchors for dynamic bodies will be in local
// space. Potentially confusing and bug prone.
//
// Note:
// I rotate joint anchors each sub-step but not contact anchors. Joint stability improves a lot by rotating joint anchors
// according to substep progress. Contacts have reduced stability when anchors are rotated during substeps, especially for
// round shapes.
[StructLayout(LayoutKind.Explicit, Size = sizeof(float) * 8)]
internal record struct BodyState
{
    public static readonly BodyState Identity = new()
    {
        LinearVelocity = Vector2.Zero,
        AngularVelocity = 0f,
        flags = 0,
        deltaPosition = Vector2.Zero,
        deltaRotation = new(1f, 0f),
    };

    [FieldOffset(sizeof(float) * 0)]
    public Vector2 LinearVelocity;

    [FieldOffset(sizeof(float) * 2)]
    public float AngularVelocity;

    // Important flags: locking, dynamic
    // I know it's a uint but 32 bits is 32 bits.
    [FieldOffset(sizeof(float) * 3)]
    public BodyFlags flags;

    // Using delta position reduces round-off error far from the origin
    [FieldOffset(sizeof(float) * 4)]
    public Vector2 deltaPosition;

    // Using delta rotation because I cannot access the full rotation on static bodies in
    // the solver and must use zero delta rotation for static bodies (c,s) = (1,0)
    [FieldOffset(sizeof(float) * 6)]
    public Quaternion2D deltaRotation;
}
