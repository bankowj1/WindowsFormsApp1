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
        private static UserInput UserInput;
        private static Camera camera;
        private static TriangleForm triangleForm;
        public static Timer timer;

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
            triangleForm = new TriangleForm();
            UserInput = new UserInput(triangleForm);
            camera = new Camera(new MyPoint(new Vector3(0f, 0f,10), new Vector3(0f, 0f, -3.14f)), Vector3.One,UserInput);
            camera.TriForm = triangleForm;
            foreach (SceenObject sceenObject in sceen.sceenObjects)
            {
                camera.ObjectProjection(sceenObject);
            }


            triangleForm.Update();
            
            Console.WriteLine("pretimer");
            timer = new Timer();
            timer.Interval = 600;
            timer.Tick += new EventHandler(Update);
            timer.Start();
            Application.Run(triangleForm);
        }
        public static void Start(object sender, EventArgs e)
        {
            timer.Start();
        }
        public static void Stop(object sender, EventArgs e)
        {
            timer.Stop();
        }
        private async static void Update(object sender, EventArgs e)
        {
            timer.Stop();
            foreach (SceenObject sceenObject in sceen.sceenObjects)
            {
                camera.ObjectProjection(sceenObject);
            }
            await triangleForm.UpdateAsync();

            triangleForm.triangles.Clear();
            timer.Start();
        }

        private static void InitSceen()
        {
            for(int i = 1; i < 3; i++)
            {
                for(int j = 1; j < 3; j++)
                {
                    for(int k = 1; k < 3; k++)
                    {
                        sceen.AddSceenObject(solids.CreateCube(new MyPoint(new Vector3(-0.5f+i* 0.5f, -0.5f + j * 0.5f, 1.1f + k * 0.5f), Vector3.Zero), 0.25f));
                    }
                }
            }
            sceen.AddSceenObject(solids.CreateCube(new MyPoint(new Vector3(0f, 0f, 1.1f), Vector3.Zero), 0.5f));
        }
    }
}