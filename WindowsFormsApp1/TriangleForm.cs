using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace WindowsFormsApp1
{
    public partial class TriangleForm : Form
    {
        private PictureBox pictureBox1;
        public List<Vector2[]> triangles;

        public TriangleForm()
        {
            InitializeComponent();
            pictureBox1 = new PictureBox();
            pictureBox1.Dock = DockStyle.Fill;
            pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
            
            Controls.Add(pictureBox1);
            triangles = new List<Vector2[]>();
        }

        public async Task UpdateAsync()
        {
            Console.WriteLine(triangles.Count);
            pictureBox1.Invalidate();

            var tcs = new TaskCompletionSource<bool>();

            PaintEventHandler paintHandler = null;
            paintHandler = (s, e) =>
            {
                DrawTriangles(s, e);
                pictureBox1.Paint -= paintHandler;
                tcs.TrySetResult(true);
            };

            pictureBox1.Paint += paintHandler;

            await tcs.Task;

            Console.WriteLine("Update Form");
        }

        private void pictureBox1_Paint(object sender, PaintEventArgs e)
        {
            Point[] points = new Point[]
            {
            new Point(10, pictureBox1.Height - 10),
            new Point(pictureBox1.Width - 10, pictureBox1.Height - 10),
            new Point(pictureBox1.Width / 2, 10)
            };
            e.Graphics.DrawPolygon(new Pen(Color.Black, 2), points);
        }

        private async void DrawTriangles(object sender, PaintEventArgs e)
        {
           
                Console.WriteLine("DrawTriangles");
                Console.WriteLine(triangles.Count);
                foreach (Vector2[] vectors in triangles)
                {
                    Console.WriteLine(vectors[1]);
                    Point[] points = new Point[]
                    {
                        new Point((int)Math.Floor(vectors[0].X), (int)Math.Floor(vectors[0].Y)),
                        new Point((int)Math.Floor(vectors[1].X), (int)Math.Floor(vectors[1].Y)),
                        new Point((int)Math.Floor(vectors[2].X), (int)Math.Floor(vectors[2].Y))
                    };
                    e.Graphics.DrawPolygon(new Pen(Color.Black, 2), points);
                }
            
            
        }

        public void AddTriangle(Vector2[] v)
        {
            triangles.Add(v);
        }
    }
}