using System;
using Robust.Shared.NewPhysics.Joints;
using Robust.Shared.Physics;
using Robust.Shared.Utility;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    #region Joints

    private static int AssignJointColor(ConstraintGraph graph, int bodyIdA, int bodyIdB, BodyType typeA, BodyType typeB )
    {
        DebugTools.Assert(typeA == BodyType.Dynamic || typeB == BodyType.Dynamic);

        if (typeA != BodyType.Static && typeB != BodyType.Static)
        {
            // Dynamic constraint colors cannot encroach on colors reserved for static constraints
            for ( int i = 0; i < PhysicsConstants.DynamicColorCount; ++i )
            {
                var color = graph.colors[i];
                if (color.BodySet.Get(bodyIdA) || color.BodySet.Get(bodyIdB))
                {
                    continue;
                }

                if (typeA == BodyType.Dynamic)
                {
                    Extensions.SetGrow(ref color.BodySet, bodyIdA, true);
                }

                if (typeB == BodyType.Dynamic)
                {
                    Extensions.SetGrow(ref color.BodySet, bodyIdB, true);
                }

                return i;
            }
        }
        else if ( typeA == BodyType.Dynamic )
        {
            // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
            for ( int i = PhysicsConstants.OverflowIndex - 1; i >= 1; --i )
            {
                var color = graph.colors[i];
                if (color.BodySet.Get(bodyIdA))
                {
                    continue;
                }

                Extensions.SetGrow(ref color.BodySet, bodyIdA, true);
                return i;
            }
        }
        else if ( typeB == BodyType.Dynamic )
        {
            // Static constraint colors build from the end to get higher priority than dyn-dyn constraints
            for ( int i = PhysicsConstants.OverflowIndex - 1; i >= 1; --i )
            {
                var color = graph.colors[i];
                if (color.BodySet.Get(bodyIdB))
                {
                    continue;
                }

                Extensions.SetGrow(ref color.BodySet, bodyIdB, true);
                return i;
            }
        }

        return PhysicsConstants.OverflowIndex;
    }

    private ref JointSim CreateJointInGraph(BaseJoint joint)
    {
        var graph = _constraintGraph;

        int bodyIdA = joint.Edges._00.bodyId;
        int bodyIdB = joint.Edges._01.bodyId;
        var bodyA = _bodies[bodyIdA].Comp;
        var bodyB = _bodies[bodyIdB].Comp;

        int colorIndex = AssignJointColor(graph, bodyIdA, bodyIdB, bodyA.BodyType, bodyB.BodyType);

        graph.colors[colorIndex].JointSims.Add(new JointSim());
        ref var jointSim = ref graph.colors[colorIndex].JointSims[^1];

        joint.ColorIndex = colorIndex;
        joint.LocalIndex = graph.colors[colorIndex].JointSims.Count - 1;
        return ref jointSim;
    }

    private void AddJointToGraph(in JointSim jointSim, BaseJoint joint)
    {
        ref var dstJoint = ref CreateJointInGraph(joint);
        // TODO: Copy data
        throw new NotImplementedException();
    }

    private void RemoveJointFromGraph(int bodyIdA, int bodyIdB, int colorIndex, int localIndex)
    {
        var graph = _constraintGraph;

        DebugTools.Assert(0 <= colorIndex && colorIndex < PhysicsConstants.GraphColorCount);
        var color = graph.colors[colorIndex];

        if (colorIndex != PhysicsConstants.OverflowIndex)
        {
            // May clear static bodies, no effect
            color.BodySet.Set(bodyIdA, false);
            color.BodySet.Set(bodyIdB, false);
        }

        var movedJointSim = color.JointSims.RemoveSwap(localIndex);
        var movedIndex = color.JointSims.Count;

        if (movedIndex != PhysicsConstants.NullIndex)
        {
            // Fix moved joint
            int movedId = movedJointSim.jointId;
            var movedJoint = _joints[movedId];
            DebugTools.Assert(movedJoint.SetIndex == (int) SetType.AwakeSet);
            DebugTools.Assert(movedJoint.ColorIndex == colorIndex);
            DebugTools.Assert(movedJoint.LocalIndex == movedIndex);
            movedJoint.LocalIndex = localIndex;
        }
    }

    #endregion
}
