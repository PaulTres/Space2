using System.Runtime.CompilerServices;

//why this file EXISTS. The public surface of Nav3D.API is now exactly what the user manual and the shipped AI
//skills describe; everything else a customer used to be able to reach is internal. The five inspector assemblies
//sit outside this one (the placement rule puts a custom inspector in an Editor/ folder with its own .asmdef), so
//without these grants every member an inspector touches would have to stay public purely to be inspectable --
//which is the surface the audit exists to remove. The grants are per named consumer, never a widening.
//
//Nav3D.Demo is deliberately not here: the demo suite compiles against the public surface only, exactly like a
//customer's project, which is what makes it the detector that catches a member hidden too eagerly.

[assembly: InternalsVisibleTo("Nav3D.API.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.Agent.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.AgentLikeEntities.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.Initializer.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.Obstacles.Editor")]
