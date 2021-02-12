using Lidgren.Network;

#nullable disable

namespace Robust.Shared.Network.Messages
{
    /// <summary>
    ///     Sent server to client to notify that a session was accepted and its new ID.
    /// </summary>
    public class MsgViewVariablesOpenSession : NetMessage
    {
        #region REQUIRED

        public const MsgGroups GROUP = MsgGroups.Command;
        public const string NAME = nameof(MsgViewVariablesOpenSession);
        public MsgViewVariablesOpenSession(INetChannel channel) : base(NAME, GROUP) { }

        #endregion

        /// <summary>
        ///     The request ID to identify WHICH request has been granted.
        ///     Equal to <see cref="MsgViewVariablesReqSession.RequestId"/> on the message that requested this session.
        /// </summary>
        public uint RequestId { get; set; }

        /// <summary>
        ///     The session ID with which to refer to the session from now on.
        /// </summary>
        public uint SessionId { get; set; }

        public override void ReadFromBuffer(NetIncomingMessage buffer)
        {
            RequestId = buffer.ReadUInt32();
            SessionId = buffer.ReadUInt32();
        }

        public override void WriteToBuffer(NetOutgoingMessage buffer)
        {
            buffer.Write(RequestId);
            buffer.Write(SessionId);
        }
    }
}
