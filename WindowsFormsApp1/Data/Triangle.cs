﻿namespace WindowsFormsApp1.Data
{
    public class Triangle
    {
        private MyPoint[] _points;
        private MyPoint[] _UV;

        public Triangle()
        {
            Points = new MyPoint[3];
            UV = new MyPoint[3];
        }

        public Triangle(MyPoint[] points, MyPoint[] uV)
        {
            Points = points;
            UV = uV;
        }

        public MyPoint[] Points { get => _points; set => _points = value; }
        public MyPoint[] UV { get => _UV; set => _UV = value; }
    }
}