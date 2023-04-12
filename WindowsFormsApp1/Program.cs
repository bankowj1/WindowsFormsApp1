using System;
using System.Numerics;
using System.Windows.Forms;
using WindowsFormsApp1.Code;
using WindowsFormsApp1.Data;

namespace WindowsFormsApp1
{
    internal static class Program
    {
        private static Sceen sceen;
        private static Solids solids;

        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        [STAThread]
        private static void Main()
        {
            sceen = new Sceen();
            solids = new Solids();
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            InitSceen();
            TriangleForm triangleForm = new TriangleForm();
            Camera camera = new Camera(new MyPoint(new Vector3(0.5f, 0.2f,0), new Vector3(0.5f, 0.1f, 0.25f)), Vector3.One);
            camera.TriForm = triangleForm;
            foreach (SceenObject sceenObject in sceen.sceenObjects)
            {
                camera.ObjectProjection(sceenObject);
            }
            Application.Run(triangleForm);
        }

        private static void InitSceen()
        {
            sceen.AddSceenObject(solids.CreateCube(new MyPoint(new Vector3(0f, 0f, 0f), Vector3.Zero), 0.5f));
        }
    }
}