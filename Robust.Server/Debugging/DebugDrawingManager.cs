using System.Diagnostics;
using Robust.Shared.IoC;
using Robust.Shared.Network;
using Robust.Shared.Network.Messages;
using Robust.Shared.Physics;

namespace Robust.Server.Debugging
{
    internal class DebugDrawingManager : IDebugDrawingManager, IEntityEventSubscriber
    {
        [Dependency] private readonly IServerNetManager _net = default!;

        public void Initialize()
        {
            _net.RegisterNetMessage<MsgRay>(MsgRay.NAME);
            IoCManager
                .Resolve<IEntityManager>()
                .EventBus
                .SubscribeEvent<DebugDrawRayMessage>(EventSource.Local, this, msg =>
                {
                    PhysicsOnDebugDrawRay(msg.Data);
                });
        }

        [Conditional("DEBUG")]
        private void PhysicsOnDebugDrawRay(DebugRayData data)
        {
            var msg = _net.CreateNetMessage<MsgRay>();
            msg.RayOrigin = data.Ray.Position;
            if (data.Results != null)
            {
                msg.DidHit = true;
                msg.RayHit = data.Results.Value.HitPos;
            }
            else
            {
                msg.RayHit = data.Ray.Position + data.Ray.Direction * data.MaxLength;
            }

            _net.ServerSendToAll(msg);
        }
    }
}
