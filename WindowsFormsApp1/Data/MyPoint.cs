using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace WindowsFormsApp1.Data
{
    public class MyPoint
    {
        private Vector3 _position;
        private Vector3 _rotation;
        public MyPoint(Vector3 position, Vector3 rotation)
        {
            Position = position;
            Rotation = rotation;
        }

        public Vector3 Position { get => _position; set => _position = value; }
        public Vector3 Rotation { get => _rotation; set => _rotation = value; }
    }
}
