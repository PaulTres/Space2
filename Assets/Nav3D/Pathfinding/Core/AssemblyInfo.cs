using System.Runtime.CompilerServices;

//why this file EXISTS. Until the assembly split, every Nav3D type compiled into Assembly-CSharp together with the
//customer's own scripts, so "internal" meant "visible to everyone" and nothing was really encapsulated. Splitting
//the layers would silently have turned each of those types invisible to the sibling that has always used it. These
//grants preserve the status quo exactly, one named consumer at a time, instead of widening the public surface --
//and they grant visibility, never a dependency: the assembly reference still has to exist and Unity still refuses
//a cycle, so the layering is enforced exactly as before.

[assembly: InternalsVisibleTo("Nav3D.SpaceProcessing")]
[assembly: InternalsVisibleTo("Nav3D.Pathfinding")]
