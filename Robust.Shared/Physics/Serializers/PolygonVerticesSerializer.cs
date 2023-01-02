using System;
using Robust.Shared.Configuration;
using Robust.Shared.IoC;
using Robust.Shared.Maths;
using Robust.Shared.Serialization;
using Robust.Shared.Serialization.Manager;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Sequence;
using Robust.Shared.Serialization.Markdown.Validation;
using Robust.Shared.Serialization.TypeSerializers.Interfaces;

namespace Robust.Shared.Physics.Serializers;

public sealed class PolygonVerticesSerializer : ITypeSerializer<Vector2[], SequenceDataNode>
{
    public ValidationNode Validate(ISerializationManager serializationManager, SequenceDataNode node,
        IDependencyCollection dependencies, ISerializationContext? context = null)
    {
        Span<Vector2> values = stackalloc Vector2[node.Count];
        var count = node.Count;

        for (var i = 0; i < node.Count; i++)
        {
            values[i] = serializationManager.Read<Vector2>(node[i]);
        }

        if (values.Length <= 3)
            values = Vertices.ForceCounterClockwise(values);
        else
            values = GiftWrap.SetConvexHull(values);

        if (count == values.Length)
        {
            return new ValidatedValueNode(node);
        }

        return new ErrorNode(node, $"Non-convex hull found for polygon");
    }

    public Vector2[] Read(ISerializationManager serializationManager, SequenceDataNode node, IDependencyCollection dependencies,
        SerializationHookContext hookCtx, ISerializationContext? context = null, ISerializationManager.InstantiationDelegate<Vector2[]>? instanceProvider = null)
    {
        var values = new Vector2[node.Count];

        for (var i = 0; i < node.Count; i++)
        {
            var value = node[i];
            values[i] = serializationManager.Read<Vector2>(value);
        }

        var convexHulls = dependencies.Resolve<IConfigurationManager>().GetCVar(CVars.ConvexHullPolygons);

        if (convexHulls)
        {
            //FPE note: This check is required as the GiftWrap algorithm early exits on triangles
            //So instead of giftwrapping a triangle, we just force it to be clock wise.
            if (values.Length <= 3)
                values = Vertices.ForceCounterClockwise(values.AsSpan());
            else
                values = GiftWrap.SetConvexHull(values.AsSpan());
        }

        return values;
    }

    public DataNode Write(ISerializationManager serializationManager, Vector2[] value, IDependencyCollection dependencies,
        bool alwaysWrite = false, ISerializationContext? context = null)
    {
        // Validate input
        if (value.Length <= 3)
            value = Vertices.ForceCounterClockwise(value);
        else
            value = GiftWrap.SetConvexHull(value);

        var node = new SequenceDataNode();

        foreach (var vert in value)
        {
            node.Add(serializationManager.WriteValue(vert));
        }

        return node;
    }
}
