using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WindowsFormsApp1.Util
{
    public abstract class Singleton<T> : IDisposable 
    {
        /// <summary>
        /// The single instance of the singleton object.
        /// </summary>
        public static T Instance { get; private set; }

        public void Dispose()
        {
            Singleton<T>.Instance = default(T);
        }

        /// <summary>
        /// Called when the script instance is being loaded.
        /// </summary>
        protected virtual void Awake()
        {
            // Check if there is already an instance of the singleton object
            if (Instance != null)
            {
                // Log a warning message and destroy the new instance
                Console.WriteLine($"Tried to create another instance of {typeof(T)}", this);
                Dispose();
                return;
            }

            // Set the instance to this object
            Instance = (T)(object)this;
        }
    }
}
