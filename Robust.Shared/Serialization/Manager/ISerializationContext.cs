using System.Collections.Generic;
using Robust.Shared.GameObjects;
using Robust.Shared.IoC;
using Robust.Shared.Serialization.Markdown;
using Robust.Shared.Serialization.Markdown.Validation;
using Robust.Shared.Serialization.Markdown.Value;
using Robust.Shared.Serialization.TypeSerializers.Interfaces;

namespace Robust.Shared.Serialization.Manager;

// TODO Serialization: make this actually not kanser to use holy moly (& allow generics)
public interface ISerializationContext
{
    SerializationManager.SerializerProvider SerializerProvider { get; }

    // This is just here for content tests that may want their own test diffs.
    /// <summary>
    /// Are we currently iterating prototypes or entities for writing.
    /// </summary>
    bool WritingReadingPrototypes { get; }
}

public sealed class EntityCopyContext : ISerializationContext, ITypeSerializer<EntityUid, ValueDataNode>
{
    public SerializationManager.SerializerProvider SerializerProvider { get; }
    public bool WritingReadingPrototypes { get; }

    private Dictionary<EntityUid, EntityUid> _remapped = new();

    public EntityCopyContext(Dictionary<EntityUid, EntityUid> remapped)
    {
        SerializerProvider = new();
        SerializerProvider.RegisterSerializer(this);
        _remapped = remapped;
    }

    public ValidationNode Validate(ISerializationManager serializationManager, ValueDataNode node,
        IDependencyCollection dependencies, ISerializationContext? context = null)
    {
        throw new System.NotImplementedException();
    }

    public EntityUid Read(ISerializationManager serializationManager, ValueDataNode node, IDependencyCollection dependencies,
        SerializationHookContext hookCtx, ISerializationContext? context = null, ISerializationManager.InstantiationDelegate<EntityUid>? instanceProvider = null)
    {
        var parsedUid = EntityUid.Parse(node.Value);
        EntityUid value;

        if (!_remapped.TryGetValue(parsedUid, out value))
        {
            value = parsedUid;
        }

        return value;
    }

    public DataNode Write(ISerializationManager serializationManager, EntityUid value, IDependencyCollection dependencies,
        bool alwaysWrite = false, ISerializationContext? context = null)
    {
        if (_remapped.TryGetValue(value, out var newValue))
        {
            value = newValue;
        }

        return new ValueDataNode(value.ToString());
    }
}

public sealed class EntityDiffContext : ISerializationContext
{
    public SerializationManager.SerializerProvider SerializerProvider { get; }
    public bool WritingReadingPrototypes { get; set; } = true;

    public EntityDiffContext()
    {
        SerializerProvider = new();
        SerializerProvider.RegisterSerializer(this);
    }
}
