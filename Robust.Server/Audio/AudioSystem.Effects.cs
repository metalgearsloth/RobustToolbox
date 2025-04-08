using System.Collections.Generic;
using Robust.Shared.Audio;
using Robust.Shared.Audio.Components;
using Robust.Shared.Audio.Effects;
using Robust.Shared.Collections;
using Robust.Shared.GameObjects;
using Robust.Shared.Prototypes;

namespace Robust.Server.Audio;

public sealed partial class AudioSystem
{
    protected override void InitializeEffect()
    {
        base.InitializeEffect();
        SubscribeLocalEvent<AudioEffectComponent, ComponentAdd>(OnEffectAdd);
        SubscribeLocalEvent<AudioAuxiliaryComponent, ComponentAdd>(OnAuxiliaryAdd);
    }

    private void ShutdownEffect()
    {
    }

    /// <summary>
    /// Reloads all <see cref="AudioPresetPrototype"/> entities.
    /// </summary>
    public void ReloadPresets(PrototypesReloadedEventArgs.PrototypeChangeSet? modified)
    {
        var query = AllEntityQuery<AudioPresetComponent>();
        var toDelete = new ValueList<EntityUid>();

        while (query.MoveNext(out var uid, out var preset))
        {
            if (!string.IsNullOrEmpty(preset.Preset) && modified?.Modified.ContainsKey(preset.Preset) != false)
            {
                toDelete.Add(uid);
            }
        }

        foreach (var ent in toDelete)
        {
            Del(ent);
        }

        foreach (var proto in ProtoMan.EnumeratePrototypes<AudioPresetPrototype>())
        {
            if (_auxiliaries.ContainsKey(proto.ID))
                continue;

            var effect = CreateEffect();
            var aux = CreateAuxiliary();
            SetEffectPreset(effect.Entity, effect.Component, proto);
            SetEffect(aux.Entity, aux.Component, effect.Entity);
            var preset = AddComp<AudioPresetComponent>(aux.Entity);
            preset.Preset = proto.ID;
            _auxiliaries[preset.Preset] = aux.Entity;
        }
    }

    private void OnEffectAdd(EntityUid uid, AudioEffectComponent component, ComponentAdd args)
    {
        component.Effect = new DummyAudioEffect();
    }

    private void OnAuxiliaryAdd(EntityUid uid, AudioAuxiliaryComponent component, ComponentAdd args)
    {
        component.Auxiliary = new DummyAuxiliaryAudio();
    }

    public override (EntityUid Entity, AudioAuxiliaryComponent Component) CreateAuxiliary()
    {
        var (ent, comp) = base.CreateAuxiliary();
        _pvs.AddGlobalOverride(ent);
        return (ent, comp);
    }

    public override (EntityUid Entity, AudioEffectComponent Component) CreateEffect()
    {
        var (ent, comp) = base.CreateEffect();
        _pvs.AddGlobalOverride(ent);
        return (ent, comp);
    }
}
