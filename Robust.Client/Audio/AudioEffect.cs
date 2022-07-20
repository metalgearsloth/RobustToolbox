using System;
using OpenToolkit.Audio.OpenAL;
using OpenToolkit.Audio.OpenAL.Extensions.Creative.EFX;

namespace Robust.Client.Audio;

/// <summary>
/// Reverb presets for audio. These are reusable across as many sources as you like and can be referred to by ID.
/// </summary>
public struct AudioEffect : IDisposable
{
    private ReverbProperties _properties = new();

    internal readonly int EffectHandle;

    public AudioEffect()
    {
        EffectHandle = EFX.GenEffect();
        // TODO:
        //EFX.AuxiliaryEffectSlot(0, );
        EFX.Source(0, EFXSourceInteger3.AuxiliarySendFilter, new[] { EffectHandle, 0 });
    }

    public void SetPreset(ReverbPreset preset)
    {
        // I had an old branch with this already and I could not find it in OpenTK.
        // Yes this is all EAX reverb data copied.
        switch (preset)
        {
            case ReverbPreset.Alley:
                _properties = ReverbPresets.Alley;
                break;
            case ReverbPreset.Arena:
                _properties = ReverbPresets.Arena;
                break;
            case ReverbPreset.Auditorium:
                _properties = ReverbPresets.Auditorium;
                break;
            case ReverbPreset.Bathroom:
                _properties = ReverbPresets.Bathroom;
                break;
            case ReverbPreset.Carpettedhallway:
                _properties = ReverbPresets.CarpetedHallway;
                break;
            case ReverbPreset.CastleAlcove:
                _properties = ReverbPresets.CastleAlcove;
                break;
            case ReverbPreset.CastleCourtyard:
                _properties = ReverbPresets.CastleCourtyard;
                break;
            case ReverbPreset.CastleCupboard:
                _properties = ReverbPresets.CastleCupboard;
                break;
            case ReverbPreset.CastleHall:
                _properties = ReverbPresets.CastleHall;
                break;
            case ReverbPreset.CastleLargeroom:
                _properties = ReverbPresets.CastleLargeRoom;
                break;
            case ReverbPreset.CastleLongpassage:
                _properties = ReverbPresets.CastleLongPassage;
                break;
            case ReverbPreset.CastleMediumroom:
                _properties = ReverbPresets.CastleMediumRoom;
                break;
            case ReverbPreset.CastleShortPassage:
                _properties = ReverbPresets.CastleShortPassage;
                break;
            case ReverbPreset.CastleSmallRoom:
                _properties = ReverbPresets.CastleSmallRoom;
                break;
            case ReverbPreset.Cave:
                _properties = ReverbPresets.Cave;
                break;
            case ReverbPreset.Chapel:
                _properties = ReverbPresets.Chapel;
                break;
            case ReverbPreset.City:
                _properties = ReverbPresets.City;
                break;
            case ReverbPreset.CityAbandoned:
                _properties = ReverbPresets.CityAbandoned;
                break;
            case ReverbPreset.CityLibrary:
                _properties = ReverbPresets.CityLibrary;
                break;
            case ReverbPreset.CityMuseum:
                _properties = ReverbPresets.CityMuseum;
                break;
            case ReverbPreset.CityStreets:
                _properties = ReverbPresets.CityStreets;
                break;
            case ReverbPreset.CitySubway:
                _properties = ReverbPresets.CitySubway;
                break;
            case ReverbPreset.CityUnderpass:
                _properties = ReverbPresets.CityUnderpass;
                break;
            case ReverbPreset.Concerthall:
                _properties = ReverbPresets.ConcertHall;
                break;
            case ReverbPreset.Dizzy:
                _properties = ReverbPresets.Dizzy;
                break;
            case ReverbPreset.DomeSaintPauls:
                _properties = ReverbPresets.DomeSaintPauls;
                break;
            case ReverbPreset.DomeTomb:
                _properties = ReverbPresets.DomeTomb;
                break;
            case ReverbPreset.DrivingCommentator:
                _properties = ReverbPresets.DrivingCommentator;
                break;
            case ReverbPreset.DrivingEmptygrandstand:
                _properties = ReverbPresets.DrivingEmptyGrandStand;
                break;
            case ReverbPreset.DrivingFullgrandstand:
                _properties = ReverbPresets.DrivingFullGrandStand;
                break;
            case ReverbPreset.DrivingIncarLuxury:
                _properties = ReverbPresets.DrivingInCarLuxury;
                break;
            case ReverbPreset.DrivingIncarRacer:
                _properties = ReverbPresets.DrivingInCarRacer;
                break;
            case ReverbPreset.DrivingIncarSports:
                _properties = ReverbPresets.DrivingInCarSports;
                break;
            case ReverbPreset.DrivingPitgarage:
                _properties = ReverbPresets.DrivingPitGarage;
                break;
            case ReverbPreset.DrivingTunnel:
                _properties = ReverbPresets.DrivingTunnel;
                break;
            case ReverbPreset.Drugged:
                _properties = ReverbPresets.Drugged;
                break;
            case ReverbPreset.Dustyroom:
                _properties = ReverbPresets.DustyRoom;
                break;
            case ReverbPreset.FactoryAlcove:
                _properties = ReverbPresets.FactoryAlcove;
                break;
            case ReverbPreset.FactoryCourtyard:
                _properties = ReverbPresets.FactoryCourtyard;
                break;
            case ReverbPreset.FactoryCupboard:
                _properties = ReverbPresets.FactoryCupboard;
                break;
            case ReverbPreset.FactoryHall:
                _properties = ReverbPresets.FactoryHall;
                break;
            case ReverbPreset.FactoryLargeroom:
                _properties = ReverbPresets.FactoryLargeRoom;
                break;
            case ReverbPreset.FactoryLongpassage:
                _properties = ReverbPresets.FactoryLongPassage;
                break;
            case ReverbPreset.FactoryMediumroom:
                _properties = ReverbPresets.FactoryMediumRoom;
                break;
            case ReverbPreset.FactoryShortpassage:
                _properties = ReverbPresets.FactoryShortPassage;
                break;
            case ReverbPreset.FactorySmallroom:
                _properties = ReverbPresets.FactorySmallRoom;
                break;
            case ReverbPreset.Forest:
                _properties = ReverbPresets.Forest;
                break;
            case ReverbPreset.Generic:
                _properties = ReverbPresets.Generic;
                break;
            case ReverbPreset.Hallway:
                _properties = ReverbPresets.Hallway;
                break;
            case ReverbPreset.Hangar:
                _properties = ReverbPresets.Hangar;
                break;
            case ReverbPreset.IcepalaceAlcove:
                _properties = ReverbPresets.IcePalaceAlcove;
                break;
            case ReverbPreset.IcepalaceCourtyard:
                _properties = ReverbPresets.IcePalaceCourtyard;
                break;
            case ReverbPreset.IcepalaceCupboard:
                _properties = ReverbPresets.IcePalaceCupboard;
                break;
            case ReverbPreset.IcepalaceHall:
                _properties = ReverbPresets.IcePalaceHall;
                break;
            case ReverbPreset.IcepalaceLargeroom:
                _properties = ReverbPresets.IcePalaceLargeRoom;
                break;
            case ReverbPreset.IcepalaceLongpassage:
                _properties = ReverbPresets.IcePalaceLongPassage;
                break;
            case ReverbPreset.IcepalaceMediumroom:
                _properties = ReverbPresets.IcePalaceMediumRoom;
                break;
            case ReverbPreset.IcepalaceShortpassage:
                _properties = ReverbPresets.IcePalaceShortPassage;
                break;
            case ReverbPreset.IcepalaceSmallroom:
                _properties = ReverbPresets.IcePalaceSmallRoom;
                break;
            case ReverbPreset.Livingroom:
                _properties = ReverbPresets.LivingRoom;
                break;
            case ReverbPreset.MoodHeaven:
                _properties = ReverbPresets.MoodHeaven;
                break;
            case ReverbPreset.MoodHell:
                _properties = ReverbPresets.MoodHell;
                break;
            case ReverbPreset.MoodMemory:
                _properties = ReverbPresets.MoodMemory;
                break;
            case ReverbPreset.Mountains:
                _properties = ReverbPresets.Mountains;
                break;
            case ReverbPreset.OutdoorsBackyard:
                _properties = ReverbPresets.OutdoorsBackyard;
                break;
            case ReverbPreset.OutdoorsCreek:
                _properties = ReverbPresets.OutdoorsCreek;
                break;
            case ReverbPreset.OutdoorsDeepcanyon:
                _properties = ReverbPresets.OutdoorsDeepCanyon;
                break;
            case ReverbPreset.OutdoorsRollingplains:
                _properties = ReverbPresets.OutdoorsRollingPlains;
                break;
            case ReverbPreset.OutdoorsValley:
                _properties = ReverbPresets.OutdoorsValley;
                break;
            case ReverbPreset.Paddedcell:
                _properties = ReverbPresets.PaddedCell;
                break;
            case ReverbPreset.Parkinglot:
                _properties = ReverbPresets.ParkingLot;
                break;
            case ReverbPreset.PipeLarge:
                _properties = ReverbPresets.PipeLarge;
                break;
            case ReverbPreset.PipeLongthin:
                _properties = ReverbPresets.PipeLongThin;
                break;
            case ReverbPreset.PipeResonant:
                _properties = ReverbPresets.PipeResonant;
                break;
            case ReverbPreset.PipeSmall:
                _properties = ReverbPresets.PipeSmall;
                break;
            case ReverbPreset.Plain:
                _properties = ReverbPresets.Plain;
                break;
            case ReverbPreset.PrefabCaravan:
                _properties = ReverbPresets.PrefabCaravan;
                break;
            case ReverbPreset.PrefabOuthouse:
                _properties = ReverbPresets.PrefabOuthouse;
                break;
            case ReverbPreset.PrefabPractiseroom:
                _properties = ReverbPresets.PrefabPractiseRoom;
                break;
            case ReverbPreset.PrefabSchoolroom:
                _properties = ReverbPresets.PrefabSchoolRoom;
                break;
            case ReverbPreset.PrefabWorkshop:
                _properties = ReverbPresets.PrefabWorkshop;
                break;
            case ReverbPreset.Psychotic:
                _properties = ReverbPresets.Psychotic;
                break;
            case ReverbPreset.Quarry:
                _properties = ReverbPresets.Quarry;
                break;
            case ReverbPreset.Room:
                _properties = ReverbPresets.Room;
                break;
            case ReverbPreset.Sewerpipe:
                _properties = ReverbPresets.Sewerpipe;
                break;
            case ReverbPreset.Smallwaterroom:
                _properties = ReverbPresets.SmallWaterRoom;
                break;
            case ReverbPreset.SpacestationAlcove:
                _properties = ReverbPresets.SpaceStationAlcove;
                break;
            case ReverbPreset.SpacestationCupboard:
                _properties = ReverbPresets.SpaceStationCupboard;
                break;
            case ReverbPreset.SpacestationHall:
                _properties = ReverbPresets.SpaceStationHall;
                break;
            case ReverbPreset.SpacestationLargeroom:
                _properties = ReverbPresets.SpaceStationLargeRoom;
                break;
            case ReverbPreset.SpacestationLongpassage:
                _properties = ReverbPresets.SpaceStationLongPassage;
                break;
            case ReverbPreset.SpacestationMediumroom:
                _properties = ReverbPresets.SpaceStationMediumRoom;
                break;
            case ReverbPreset.SpacestationShortpassage:
                _properties = ReverbPresets.SpaceStationShortPassage;
                break;
            case ReverbPreset.SpacestationSmallroom:
                _properties = ReverbPresets.SpaceStationSmallRoom;
                break;
            case ReverbPreset.SportEmptystadium:
                _properties = ReverbPresets.SportEmptyStadium;
                break;
            case ReverbPreset.SportFullstadium:
                _properties = ReverbPresets.SportFullStadium;
                break;
            case ReverbPreset.SportGynmasium:
                _properties = ReverbPresets.SportGymnasium;
                break;
            case ReverbPreset.SportLargeswimmingpool:
                _properties = ReverbPresets.SportLargeSwimmingPool;
                break;
            case ReverbPreset.SportSmallswimmingpool:
                _properties = ReverbPresets.SportSmallSwimmingPool;
                break;
            case ReverbPreset.SportSquashcourt:
                _properties = ReverbPresets.SportSquashCourt;
                break;
            case ReverbPreset.SportStadimtannoy:
                _properties = ReverbPresets.SportStadiumTannoy;
                break;
            case ReverbPreset.Stonecorridor:
                _properties = ReverbPresets.StoneCorridor;
                break;
            case ReverbPreset.Stoneroom:
                _properties = ReverbPresets.StoneRoom;
                break;
            case ReverbPreset.Underwater:
                _properties = ReverbPresets.Underwater;
                break;
            case ReverbPreset.WoodenAlcove:
                _properties = ReverbPresets.WoodenGalleonAlcove;
                break;
            case ReverbPreset.WoodenCourtyard:
                _properties = ReverbPresets.WoodenGalleonCourtyard;
                break;
            case ReverbPreset.WoodenCupboard:
                _properties = ReverbPresets.WoodenGalleonCupboard;
                break;
            case ReverbPreset.WoodenHall:
                _properties = ReverbPresets.WoodenGalleonHall;
                break;
            case ReverbPreset.WoodenLargeroom:
                _properties = ReverbPresets.WoodenGalleonLargeRoom;
                break;
            case ReverbPreset.WoodenLongpassage:
                _properties = ReverbPresets.WoodenGalleonLongPassage;
                break;
            case ReverbPreset.WoodenMediumroom:
                _properties = ReverbPresets.WoodenGalleonMediumRoom;
                break;
            case ReverbPreset.WoodenShortpassage:
                _properties = ReverbPresets.WoodenGalleonShortPassage;
                break;
            case ReverbPreset.WoodenSmallroom:
                _properties = ReverbPresets.WoodenGalleonSmallRoom;
                break;
            default:
                throw new NotImplementedException();
        }

        EFX.Effect(EffectHandle, EffectInteger.EffectType, (int) EffectType.EaxReverb);
        EFX.Effect(EffectHandle, EffectFloat.ReverbDensity, _properties.Density);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbDiffusion, _properties.Diffusion);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbGain, _properties.Gain);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbGainHF, _properties.GainHF);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbGainLF, _properties.GainLF);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbDecayTime, _properties.DecayTime);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbDecayHFRatio, _properties.DecayHFRatio);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbDecayLFRatio, _properties.DecayLFRatio);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbReflectionsGain, _properties.ReflectionsGain);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbReflectionsDelay, _properties.ReflectionsDelay);
        EFX.Effect(EffectHandle, EffectVector3.EaxReverbReflectionsPan, new [] {_properties.ReflectionsPan.X, _properties.ReflectionsPan.Y, _properties.ReflectionsPan.Z});
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbLateReverbGain, _properties.LateReverbGain);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbLateReverbDelay, _properties.LateReverbDelay);
        EFX.Effect(EffectHandle, EffectVector3.EaxReverbLateReverbPan, new [] {_properties.LateReverbPan.X, _properties.LateReverbPan.Y, _properties.LateReverbPan.Z});
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbEchoTime, _properties.EchoTime);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbEchoDepth, _properties.EchoDepth);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbModulationTime, _properties.ModulationTime);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbModulationDepth, _properties.ModulationDepth);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbAirAbsorptionGainHF, _properties.AirAbsorptionGainHF);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbHFReference, _properties.HFReference);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbLFReference, _properties.LFReference);
        EFX.Effect(EffectHandle, EffectFloat.EaxReverbRoomRolloffFactor, _properties.RoomRolloffFactor);
        EFX.Effect(EffectHandle, EffectInteger.EaxReverbDecayHFLimit, _properties.DecayHFLimit);

        _validate();
    }

    public void Dispose()
    {
        EFX.DeleteEffect(EffectHandle);
    }

    private void _validate()
    {
        if (!EFX.IsEffect(EffectHandle))
            throw new InvalidOperationException($"Audio handle {EffectHandle} is not a valid effect!");

        // TODO: AL.GetError();
    }
}

/// <summary>
/// EAX reverb presets.
/// </summary>
public enum ReverbPreset : byte
{
    None = 0,
    Alley,
    Arena,
    Auditorium,
    Bathroom,
    Carpettedhallway,
    CastleAlcove,
    CastleCourtyard,
    CastleCupboard,
    CastleHall,
    CastleLargeroom,
    CastleLongpassage,
    CastleMediumroom,
    CastleShortPassage,
    CastleSmallRoom,
    Cave,
    Chapel,
    City,
    CityAbandoned,
    CityLibrary,
    CityMuseum,
    CityStreets,
    CitySubway,
    CityUnderpass,
    Concerthall,
    Dizzy,
    DomeSaintPauls,
    DomeTomb,
    DrivingCommentator,
    DrivingEmptygrandstand,
    DrivingFullgrandstand,
    DrivingIncarLuxury,
    DrivingIncarRacer,
    DrivingIncarSports,
    DrivingPitgarage,
    DrivingTunnel,
    Drugged,
    Dustyroom,
    FactoryAlcove,
    FactoryCourtyard,
    FactoryCupboard,
    FactoryHall,
    FactoryLargeroom,
    FactoryLongpassage,
    FactoryMediumroom,
    FactoryShortpassage,
    FactorySmallroom,
    Forest,
    Generic,
    Hallway,
    Hangar,
    IcepalaceAlcove,
    IcepalaceCourtyard,
    IcepalaceCupboard,
    IcepalaceHall,
    IcepalaceLargeroom,
    IcepalaceLongpassage,
    IcepalaceMediumroom,
    IcepalaceShortpassage,
    IcepalaceSmallroom,
    Livingroom,
    MoodHeaven,
    MoodHell,
    MoodMemory,
    Mountains,
    OutdoorsBackyard,
    OutdoorsCreek,
    OutdoorsDeepcanyon,
    OutdoorsRollingplains,
    OutdoorsValley,
    Paddedcell,
    Parkinglot,
    PipeLarge,
    PipeLongthin,
    PipeResonant,
    PipeSmall,
    Plain,
    PrefabCaravan,
    PrefabOuthouse,
    PrefabPractiseroom,
    PrefabSchoolroom,
    PrefabWorkshop,
    Psychotic,
    Quarry,
    Room,
    Sewerpipe,
    Smallwaterroom,
    SpacestationAlcove,
    SpacestationCupboard,
    SpacestationHall,
    SpacestationLargeroom,
    SpacestationLongpassage,
    SpacestationMediumroom,
    SpacestationShortpassage,
    SpacestationSmallroom,
    SportEmptystadium,
    SportFullstadium,
    SportGynmasium,
    SportLargeswimmingpool,
    SportSmallswimmingpool,
    SportSquashcourt,
    SportStadimtannoy,
    Stonecorridor,
    Stoneroom,
    Underwater,
    WoodenAlcove,
    WoodenCourtyard,
    WoodenCupboard,
    WoodenHall,
    WoodenLargeroom,
    WoodenLongpassage,
    WoodenMediumroom,
    WoodenShortpassage,
    WoodenSmallroom,
}
