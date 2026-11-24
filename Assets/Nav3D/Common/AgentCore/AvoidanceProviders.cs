using System;
using System.Reflection;
using UnityEngine;

namespace Nav3D.Agents
{
    /// <summary>
    /// The local-avoidance implementation this domain has, if any.
    /// </summary>
    //No provider found is the "no local avoidance" case, not an error.
    static class AvoidanceProviders
    {
        #region Constants

        const string SEVERAL_PROVIDERS_WARNING = "Nav3D: several assemblies declare a local-avoidance provider ({0}). " +
                                                 "Using {1}; remove the others to make the choice explicit.";

        const string PROVIDER_CONSTRUCTION_ERROR = "Nav3D: the local-avoidance provider {0} declared by {1} could not be used, " +
                                                   "so agents will run without local avoidance. It must implement " +
                                                   nameof(IAvoidanceProvider) + " and have a parameterless constructor. {2}";

        #endregion

        #region Attributes

        static IAvoidanceProvider s_Current;
        static bool               s_Resolved;

        static readonly object s_ResolveLock = new object();

        #endregion

        #region Properties

        //resolved once per domain, on first use
        public static IAvoidanceProvider Current
        {
            get
            {
                if (s_Resolved)
                    return s_Current;

                lock (s_ResolveLock)
                {
                    if (!s_Resolved)
                    {
                        s_Current  = Resolve();
                        s_Resolved = true;
                    }
                }

                return s_Current;
            }
        }

        #endregion

        #region Service methods

        static IAvoidanceProvider Resolve()
        {
            Type     providerType   = null;
            Assembly declaringOne   = null;
            string   allDeclaringOnes = null;

            foreach (Assembly assembly in AppDomain.CurrentDomain.GetAssemblies())
            {
                Nav3DAvoidanceProviderAttribute attribute;

                try
                {
                    attribute = assembly.GetCustomAttribute<Nav3DAvoidanceProviderAttribute>();
                }
                catch (Exception)
                {
                    //a dynamic or partially loaded assembly whose attributes cannot be read is simply not a candidate
                    continue;
                }

                if (attribute?.ProviderType == null)
                    continue;

                if (providerType == null)
                {
                    providerType     = attribute.ProviderType;
                    declaringOne     = assembly;
                    allDeclaringOnes = assembly.GetName().Name;
                }
                else
                    allDeclaringOnes += ", " + assembly.GetName().Name;
            }

            if (providerType == null)
                return null;

            if (allDeclaringOnes != declaringOne.GetName().Name)
                Debug.LogWarning(string.Format(SEVERAL_PROVIDERS_WARNING, allDeclaringOnes, providerType.FullName));

            try
            {
                //nonPublic: a provider is internal to the assembly that brings it, and so may its constructor be
                return (IAvoidanceProvider) Activator.CreateInstance(providerType, true);
            }
            catch (Exception exception)
            {
                Debug.LogError(string.Format(PROVIDER_CONSTRUCTION_ERROR, providerType.FullName, declaringOne.GetName().Name, exception));

                return null;
            }
        }

        #endregion
    }
}
