using System;
using System.Collections.Generic;
using Robust.Shared.IoC;
using Robust.Shared.Serialization.Manager;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Sequence;
using Robust.Shared.Serialization.Markdown.Validation;
using Robust.Shared.Serialization.Markdown.Value;
using Robust.Shared.Serialization.TypeSerializers.Interfaces;

namespace Robust.Shared.Serialization.TypeSerializers.Implementations.Generic;

public sealed class CustomListSerializer<T, TCustomSerializer>
    : ITypeSerializer<List<T>, SequenceDataNode>
    where TCustomSerializer : ITypeSerializer<T, ValueDataNode>
{
    List<T> ITypeReader<List<T>, SequenceDataNode>.Read(ISerializationManager serializationManager,
        SequenceDataNode node,
        IDependencyCollection dependencies,
        SerializationHookContext hookCtx,
        ISerializationContext? context, ISerializationManager.InstantiationDelegate<List<T>>? instanceProvider)
    {
        var set = instanceProvider != null ? instanceProvider() : new List<T>();

        foreach (var dataNode in node.Sequence)
        {
            var value = serializationManager.Read<T, ValueDataNode, TCustomSerializer>((ValueDataNode)dataNode, hookCtx, context);
            if (value == null)
                throw new InvalidOperationException($"{nameof(TCustomSerializer)} returned a null value when reading using a custom List serializer.");

            set.Add(value);
        }

        return set;
    }

    ValidationNode ITypeValidator<List<T>, SequenceDataNode>.Validate(ISerializationManager serializationManager,
        SequenceDataNode node, IDependencyCollection dependencies, ISerializationContext? context)
    {
        var list = new List<ValidationNode>();
        foreach (var elem in node.Sequence)
        {
            list.Add(serializationManager.ValidateNode<T, ValueDataNode, TCustomSerializer>((ValueDataNode)elem, context));
        }

        return new ValidatedSequenceNode(list);
    }

    public DataNode Write(ISerializationManager serializationManager, List<T> value,
        IDependencyCollection dependencies, bool alwaysWrite = false,
        ISerializationContext? context = null)
    {
        var sequence = new SequenceDataNode();

        foreach (var elem in value)
        {
            sequence.Add(serializationManager.WriteValue<T, TCustomSerializer>(elem, alwaysWrite, context));
        }

        return sequence;
    }
}
