using System.Runtime.CompilerServices;

//why this file EXISTS. Until the assembly split, every Nav3D type compiled into Assembly-CSharp together with the
//customer's own scripts, so "internal" meant "visible to everyone" and nothing was really encapsulated. Splitting
//the layers would silently have turned each of those types invisible to the sibling that has always used it. These
//grants preserve the status quo exactly, one named consumer at a time, instead of widening the public surface --
//and they grant visibility, never a dependency: the assembly reference still has to exist and Unity still refuses
//a cycle, so the layering is enforced exactly as before.
//
//Nav3D.Demo is deliberately not on any of these lists: the demo suite must compile against the public
//surface only, exactly like a customer's project, and the pure-Pro pack — which deletes Demo wholesale —
//would read a grant naming it as a reference to something that is no longer there.

[assembly: InternalsVisibleTo("Nav3D.Pathfinding.Core")]
[assembly: InternalsVisibleTo("Nav3D.SpaceProcessing")]
[assembly: InternalsVisibleTo("Nav3D.Pathfinding")]
[assembly: InternalsVisibleTo("Nav3D.Agents")]
[assembly: InternalsVisibleTo("Nav3D.LocalAvoidance")]
[assembly: InternalsVisibleTo("Nav3D.API")]
[assembly: InternalsVisibleTo("Nav3D.API.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.Agent.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.Initializer.Editor")]
[assembly: InternalsVisibleTo("Nav3D.API.Obstacles.Editor")]
[assembly: InternalsVisibleTo("Nav3D.SpaceProcessing.ResolutionRegion.Editor")]
[assembly: InternalsVisibleTo("Nav3D.Tools.Editor")]