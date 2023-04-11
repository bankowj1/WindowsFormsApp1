using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using WindowsFormsApp1.Data;

namespace WindowsFormsApp1.Code
{
    public class Solids
    {
        public SceenObject CreateCube(MyPoint point,float size)
        {
            MyPoint[] points = new MyPoint[]
            {
                //0
                new MyPoint(new Vector3(0f, 0f,0f ),
                new Vector3(0f, 0f, 0f )),
                //1
                new MyPoint(new Vector3(size, 0f,0f ),
                new Vector3(0f, 0f, 0f )),
                //2
                new MyPoint(new Vector3(0f, size, 0f ),
                new Vector3(0f, 0f, 0f )),
                //3
                new MyPoint(new Vector3(size, size, 0f ),
                new Vector3(0f, 0f, 0f )),
                //4
                new MyPoint(new Vector3(0f, 0f,size ),
                new Vector3(0f, 0f, 0f )),
                //5
                new MyPoint(new Vector3(size, 0f,size ),
                new Vector3(0f, 0f, 0f )),
                //6d
                new MyPoint(new Vector3(0f,size,size ),
                new Vector3(0f, 0f, 0f )),
                //7
                new MyPoint(new Vector3(size, size, size ),
                new Vector3(0f, 0f, 0f )),
            };
            List<Triangle> triangles = new List<Triangle>
            {   
                // Front face
                new Triangle(new MyPoint[] {points[0], points[3], points[1] }, new MyPoint[] {points[0], points[3], points[1] }), // lower left
                new Triangle(new MyPoint[] {points[0], points[2], points[3] }, new MyPoint[] {points[0], points[2], points[3] }), // upper right
                // Back face
                new Triangle(new MyPoint[] {points[4], points[5], points[7] }, new MyPoint[] {points[4], points[5], points[7] }), // upper left
                new Triangle(new MyPoint[] {points[4], points[7], points[6] }, new MyPoint[] {points[4], points[7], points[6] }), // lower right
                // Left face
                new Triangle(new MyPoint[] {points[0], points[2], points[6] }, new MyPoint[] {points[0], points[2], points[6] }), // lower left
                new Triangle(new MyPoint[] {points[0], points[6], points[4] }, new MyPoint[] {points[0], points[6], points[4] }), // upper right
                // Right face
                new Triangle(new MyPoint[] {points[1], points[7], points[5] }, new MyPoint[] {points[1], points[7], points[5] }), // upper left
                new Triangle(new MyPoint[] {points[1], points[3], points[7] }, new MyPoint[] {points[1], points[3], points[7] }), // lower right
                // Top face
                new Triangle(new MyPoint[] {points[2], points[3], points[7] }, new MyPoint[] {points[2], points[3], points[7] }), // upper left
                new Triangle(new MyPoint[] {points[2], points[7], points[6] }, new MyPoint[] {points[2], points[7], points[6] }), // lower right
                // Bottom face
                new Triangle(new MyPoint[] {points[0], points[1], points[5] }, new MyPoint[] {points[0], points[1], points[5] }), // upper left
                new Triangle(new MyPoint[] {points[0], points[5], points[4] }, new MyPoint[] {points[0], points[5], points[4] }), // lower right
            };
            SceenObject cube = new SceenObject(triangles,point,Vector3.One);
            return cube;
        }
    }
}
