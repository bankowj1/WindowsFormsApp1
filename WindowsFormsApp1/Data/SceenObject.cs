using System.Collections.Generic;
using System.Numerics;

namespace WindowsFormsApp1.Data
{
    public class SceenObject
    {
        protected MyPoint _anchor;
        private Vector3 _scale;
        protected List<Triangle> _triangles;

        public SceenObject(List<Triangle> triangles, MyPoint anchor, Vector3 scale)
        {
            _triangles = triangles;
            _anchor = anchor;
            _scale = scale;
        }

        public SceenObject(MyPoint anchor, Vector3 scale)
        {
            _anchor = anchor;
            _scale = scale;
        }

        public MyPoint Anchor { get => _anchor; set => _anchor = value; }
        public List<Triangle> Triangles { get => _triangles; set => _triangles = value; }
        public Vector3 Scale { get => _scale; set => _scale = value; }

        public void AddTriangle(Triangle triangle)
        {
            _triangles.Add(triangle);
        }
    }
}