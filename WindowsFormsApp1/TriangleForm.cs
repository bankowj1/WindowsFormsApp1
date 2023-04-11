using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Windows.Forms;

namespace WindowsFormsApp1
{
    public partial class TriangleForm : Form
    {
        private PictureBox pictureBox1;
        public List<Vector2[]> triangles;
        private Timer timer;

        public TriangleForm()
        {
            InitializeComponent();
            pictureBox1 = new PictureBox();
            pictureBox1.Dock = DockStyle.Fill;
            pictureBox1.SizeMode = PictureBoxSizeMode.StretchImage;
            pictureBox1.Paint += DrawTriangles;
            Controls.Add(pictureBox1);
            triangles = new List<Vector2[]>();
            timer = new Timer();
            timer.Interval = 6000;
            timer.Tick += new EventHandler(timer_Tick);
            timer.Start();
        }

        private void TriangleForm_Load(object sender, EventArgs e)
        {
        }

        private void timer_Tick(object sender, EventArgs e)
        {
            pictureBox1.Invalidate(); // Trigger the Paint event of the PictureBox
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

        public void DrawTriangles(object sender, PaintEventArgs e)
        {
            foreach (Vector2[] vectors in triangles)
            {
                Console.WriteLine(vectors[0].X);
                Console.WriteLine(vectors[0].Y);
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