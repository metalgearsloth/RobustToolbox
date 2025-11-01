using System;

namespace Robust.Shared.NewPhysics;

public sealed partial class NewPhysicsSystem
{
    private static Softness MakeSoft( float hertz, float zeta, float h )
    {
        if ( hertz == 0.0f )
        {
            return new Softness()
            {
                biasRate = 0f,
                massScale = 0f,
                impulseScale = 0f,
            };
        }

        float omega = (float) (2.0 * Math.PI * hertz);
        float a1 = 2.0f * zeta + h * omega;
        float a2 = h * omega * a1;
        float a3 = 1.0f / ( 1.0f + a2 );

        // bias = w / (2 * z + hw)
        // massScale = hw * (2 * z + hw) / (1 + hw * (2 * z + hw))
        // impulseScale = 1 / (1 + hw * (2 * z + hw))

        // If z == 0
        // bias = 1/h
        // massScale = hw^2 / (1 + hw^2)
        // impulseScale = 1 / (1 + hw^2)

        // w -> inf
        // bias = 1/h
        // massScale = 1
        // impulseScale = 0

        // if w = pi / 4  * inv_h
        // massScale = (pi/4)^2 / (1 + (pi/4)^2) = pi^2 / (16 + pi^2) ~= 0.38
        // impulseScale = 1 / (1 + (pi/4)^2) = 16 / (16 + pi^2) ~= 0.62

        // In all cases:
        // massScale + impulseScale == 1

        return new Softness()
        {
            biasRate = omega / a1,
            massScale = a2 * a3,
            impulseScale = a3,
        };
    }
}
