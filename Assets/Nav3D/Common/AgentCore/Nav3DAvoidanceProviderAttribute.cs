using System;

namespace Nav3D.Agents
{
    /// <summary>
    /// Declares, on an assembly, the <see cref="IAvoidanceProvider"/> that assembly brings.
    /// </summary>
    [AttributeUsage(AttributeTargets.Assembly, AllowMultiple = false)]
    class Nav3DAvoidanceProviderAttribute : Attribute
    {
        #region Properties

        public Type ProviderType { get; }

        #endregion

        #region Constructors

        public Nav3DAvoidanceProviderAttribute(Type _ProviderType)
        {
            ProviderType = _ProviderType;
        }

        #endregion
    }
}
