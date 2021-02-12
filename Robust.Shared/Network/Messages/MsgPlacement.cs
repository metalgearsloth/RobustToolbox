﻿using System;
using Lidgren.Network;
using Robust.Shared.Enums;
using Robust.Shared.GameObjects;
using Robust.Shared.Map;
using Robust.Shared.Maths;

#nullable disable

namespace Robust.Shared.Network.Messages
{
    public class MsgPlacement : NetMessage
    {
        #region REQUIRED
        public static readonly MsgGroups GROUP = MsgGroups.Command;
        public static readonly string NAME = nameof(MsgPlacement);
        public MsgPlacement(INetChannel channel) : base(NAME, GROUP) { }
        #endregion

        public PlacementManagerMessage PlaceType { get; set; }
        public string Align { get; set; }
        public bool IsTile { get; set; }
        public ushort TileType { get; set; }
        public string EntityTemplateName { get; set; }
        public EntityCoordinates EntityCoordinates { get; set; }
        public Direction DirRcv { get; set; }
        public EntityUid EntityUid { get; set; }

        public int Range { get; set; }
        public string ObjType { get; set; }
        public string AlignOption { get; set; }

        public override void ReadFromBuffer(NetIncomingMessage buffer)
        {
            PlaceType = (PlacementManagerMessage) buffer.ReadByte();
            switch (PlaceType)
            {
                case PlacementManagerMessage.RequestPlacement:
                    Align = buffer.ReadString();
                    IsTile = buffer.ReadBoolean();

                    if (IsTile) TileType = buffer.ReadUInt16();
                    else EntityTemplateName = buffer.ReadString();

                    EntityCoordinates = buffer.ReadEntityCoordinates();
                    DirRcv = (Direction)buffer.ReadByte();
                    break;
                case PlacementManagerMessage.StartPlacement:
                    Range = buffer.ReadInt32();
                    IsTile = buffer.ReadBoolean();
                    ObjType = buffer.ReadString();
                    AlignOption = buffer.ReadString();
                    break;
                case PlacementManagerMessage.CancelPlacement:
                case PlacementManagerMessage.PlacementFailed:
                    throw new NotImplementedException();
                case PlacementManagerMessage.RequestEntRemove:
                    EntityUid = new EntityUid(buffer.ReadInt32());
                    break;
            }
        }

        public override void WriteToBuffer(NetOutgoingMessage buffer)
        {
            buffer.Write((byte)PlaceType);
            switch (PlaceType)
            {
                case PlacementManagerMessage.RequestPlacement:
                    buffer.Write(Align);
                    buffer.Write(IsTile);

                    if(IsTile) buffer.Write(TileType);
                    else buffer.Write(EntityTemplateName);

                    buffer.Write(EntityCoordinates);
                    buffer.Write((byte)DirRcv);
                    break;
                case PlacementManagerMessage.StartPlacement:
                    buffer.Write(Range);
                    buffer.Write(IsTile);
                    buffer.Write(ObjType);
                    buffer.Write(AlignOption);
                    break;
                case PlacementManagerMessage.CancelPlacement:
                case PlacementManagerMessage.PlacementFailed:
                    throw new NotImplementedException();
                case PlacementManagerMessage.RequestEntRemove:
                    buffer.Write((int)EntityUid);
                    break;
            }
        }
    }
}
