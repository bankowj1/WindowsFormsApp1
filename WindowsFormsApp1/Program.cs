using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;
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
        static void Main()
        {
            sceen = new Sceen();
            solids = new Solids();
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            InitSceen();
            TriangleForm triangleForm = new TriangleForm();
            Camera camera = new Camera(new MyPoint(Vector3.Zero, Vector3.UnitZ),Vector3.One);
            camera.TriForm = triangleForm;
            foreach(SceenObject sceenObject in sceen.sceenObjects)
            {
                camera.ObjectProjection(sceenObject);
            }
            Application.Run(triangleForm);
        }

        private static void InitSceen()
        {
            sceen.AddSceenObject(solids.CreateCube(new MyPoint(new Vector3(600,600,600), Vector3.UnitZ), 160));
        }
    }
}
