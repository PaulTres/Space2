using System.Runtime.CompilerServices;

//why this file EXISTS. Nav3DAgentBase declares the two members the frame driver calls on every agent-like body --
//DoFixedUpdate and Movable -- and Nav3DAgent / Nav3DEvader / Nav3DSphereShell override them one assembly up. A
//C# override cannot narrow the accessibility it inherits, so the pair had to be public on three components the
//manual documents, advertising the driver's own call surface to every customer. This grant lets the base declare
//them internal and the overrides follow.

[assembly: InternalsVisibleTo("Nav3D.API")]

//The avoidance assembly declares itself through the internal seam (IAvoidanceProvider + the attribute).
[assembly: InternalsVisibleTo("Nav3D.LocalAvoidance")]
