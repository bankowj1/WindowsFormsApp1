using MathNet.Numerics.RootFinding;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Forms;
using WindowsFormsApp1.Data;

namespace WindowsFormsApp1.Code
{
    public class Camera : SceenObject
    {
        private int _resX = 1920;
        private int _resY = 1080;
        private float _theta = 90;
        private float _rotationSpeed = 2.0f;
        private float _randerRange = 1000f;
        private float _closeRange = 0.1f;
        private TriangleForm _triForm;
        private UserInput _userInput;
        private Matrix4x4 _viewMatrix;
        private Quaternion _rotationQ;
        private Matrix4x4 _projectionMatrix;


        int ii=0;

        public TriangleForm TriForm { get => _triForm; set => _triForm = value; }
        public float RanderRange { get => _randerRange; set => _randerRange = value; }
        public float CloseRange { get => _closeRange; set => _closeRange = value; }
        public int ResX { get => _resX; set => _resX = value; }
        public int ResY { get => _resY; set => _resY = value; }
        public float Theta { get => _theta; set => _theta = value; }

        public Camera(List<Triangle> triangles, MyPoint anchor, Vector3 scale, UserInput userInput) : base(triangles, anchor, scale)
        {
            _userInput = userInput;
            Init();
        }

        public Camera(MyPoint anchor, Vector3 scale,UserInput userInput) : base(anchor, scale)
        {
            _userInput = userInput;
            Init();
        }

        public void Init()
        {
            //events
            _userInput.CameraRotation += RotateCamera;
            _userInput.CameraZoom += ZoomCamera;
            _userInput.CameraPosition += PositionCamera;
            //projection matrix
            ProjectMax();
            //view matrix
            CameraMatx();
        }
        private void RotateCamera(object sender, RotEventArgs e)
        {
            Console.WriteLine("Rotation");
            Console.WriteLine(_anchor.Rotation);
            _anchor.Rotation = _anchor.Rotation + (e.V * e.F * _rotationSpeed);
            Console.WriteLine(_anchor.Rotation);
            CameraMatx();
        }



        private void ZoomCamera(object sender, FloatEventArgs e)
        {
            _theta += e.V;
            ProjectMax();
        }
        private void PositionCamera(object sender, Vector3EventArgs e)
        {
            _anchor.Position = Vector3.Add(_anchor.Position, Vector3.Transform(e.V, RotationMatrixDeg(_anchor.Rotation.X, _anchor.Rotation.Y, -_anchor.Rotation.Z)) * 0.1f);
            CameraMatx();
        }
        public void SetRes(int x, int y)
        {
            ResX = x; ResY = y;
        }

        public void ProjectMax()
        {
            float aspectRatio = (float)ResY / ResX;
            float tanHalfFOV = Radians((Theta / 2)); // tangent of half FOV angle
            float q = _randerRange / (_randerRange - _closeRange);
            _projectionMatrix = new Matrix4x4(
                aspectRatio*(1 / tanHalfFOV), 0, 0, 0,
                0, (1/tanHalfFOV), 0, 0,
                0, 0, q, -_closeRange * q,
                0, 0, 1, 0);
            /*
            _projectionMatrix = new Matrix4x4(
                (1/tanHalfFOV) / aspectRatio, 0, 0, 0,
                0, 1/tanHalfFOV, 0, 0,
                0, 0, _randerRange/(_closeRange-_randerRange),  _closeRange* _randerRange / (_closeRange - _randerRange),
                0, 0, -1, 0);*//*
            _projectionMatrix = new Matrix4x4(
                1/ ( tanHalfFOV* aspectRatio), 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0, _randerRange / (_closeRange - _randerRange), _closeRange * _randerRange / (_closeRange - _randerRange),
                0, 0, -1, 0);*/
            /*_projectionMatrix = new Matrix4x4(
                1 / (tanHalfFOV*aspectRatio), 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0, far / (near - far), -near * far / (near - far),
                0, 0, 1, 0);*//*
            _projectionMatrix = new Matrix4x4(
                (1 / tanHalfFOV) / aspectRatio, 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0,-(_randerRange + _closeRange)/(_randerRange-_closeRange), -(2*_randerRange*_closeRange)/ (_randerRange - _closeRange),
                0, 0, 1, 0);
            _projectionMatrix = new Matrix4x4(
                1 / (tanHalfFOV * aspectRatio), 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0, _randerRange / (_closeRange - _randerRange), _closeRange * _randerRange / (_closeRange - _randerRange),
                0, 0, -1, 0);*/

        }

        public void CameraMatx()
        {
            Vector3 cameraPosition = _anchor.Position;
            Vector3 cameraRotation = _anchor.Rotation;

            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, 0,//columna 1x 
                0, 1, 0, 0,//columna 2x
                0, 0, 1, 0,//columna 3x
                cameraPosition.X, cameraPosition.Y, cameraPosition.Z, 1);//columna 4x
            _rotationQ = QuaternionFromEulerDeg(cameraRotation.X, cameraRotation.Y, -cameraRotation.Z);
            
            Matrix4x4 rotationMatrix = RotationMatrixDeg(cameraRotation.X, cameraRotation.Y, -cameraRotation.Z);
            Console.WriteLine(rotationMatrix);
            Console.WriteLine(Matrix4x4.CreateFromYawPitchRoll(cameraRotation.X, cameraRotation.Y, -cameraRotation.Z));
            _viewMatrix =  translationMatrix * rotationMatrix ;
        }

        public void ObjectProjection(SceenObject sceenObject)
        {
            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                sceenObject.Anchor.Position.X, sceenObject.Anchor.Position.Y, sceenObject.Anchor.Position.Z, 1);

            Matrix4x4 scaleMatrix = new Matrix4x4(
               sceenObject.Scale.X, 0, 0, 0,
               0, sceenObject.Scale.Y, 0, 0,
               0, 0, sceenObject.Scale.Z, 0,
               0, 0, 0, 1);

            Quaternion rotationQ = QuaternionFromEulerDeg(sceenObject.Anchor.Rotation.X, sceenObject.Anchor.Rotation.Y, -sceenObject.Anchor.Rotation.Z);

            Matrix4x4 rotationMatrix = RotationMatrixDeg(sceenObject.Anchor.Rotation.X,  sceenObject.Anchor.Rotation.Y, sceenObject.Anchor.Rotation.Z);

            foreach (Triangle triangle in sceenObject.Triangles)
            {
                TriangleProjectrion(triangle, translationMatrix, scaleMatrix, rotationQ,rotationMatrix);
            }
        }

        public void TriangleProjectrion(Triangle triangle, Matrix4x4 translationMatrix, Matrix4x4 scaleMatrix, Quaternion rotationQ,Matrix4x4 rotatiobMatrix)
        {
            Vector2[] v = new Vector2[3];
            int i = 0;
            Vector3[] vector32 = new Vector3[3];
            for (int tris = 0 ; tris < 3; tris++)
            {

                //Vector4 translationWorld = Vector4.Transform(point.Position, _projectionMatrix * Matrix4x4.Transpose( _viewMatrix *(translationMatrix * rotatiobMatrix*scaleMatrix )));
                Console.WriteLine("_anchor.Position");
                Console.WriteLine(triangle.Points[tris].Position);
                vector32[tris] = MultMatxVec3V3(triangle.Points[tris].Position, scaleMatrix*rotatiobMatrix * translationMatrix);
                Console.WriteLine(rotatiobMatrix);
                Console.WriteLine(translationMatrix);
                Console.WriteLine("vector32");
                Console.WriteLine(vector32);
                
            }
            CalNormal(triangle);
            for (int tris = 0; tris < 3; tris++)
            {
                Vector4 vector4 = MultMatxVec3V4(vector32[tris], _projectionMatrix);
                if (Math.Abs(vector4.W) > 0.00001f)
                {
                    vector4.X = vector4.X / vector4.W;
                    vector4.Y = vector4.Y / vector4.W;
                    vector4.Z = vector4.Z / vector4.W;
                }
                v[i] = new Vector2(((vector4.X + 1) / 2) * _resX, ((vector4.Y + 1) / 2) * _resY);
                Console.WriteLine(vector4);
                i++;
            }
            DrawTriangle(v);
        }

        public void DrawTriangle(Vector2[] v)
        {
            TriForm.AddTriangle(v);
            Console.WriteLine(ii++);
        }
        public static Matrix4x4 RotationMatrixDeg(float x, float y, float z)
        {
            return RotationMatrixRad(Radians(x), Radians(y), Radians(z));
        }

        public static Matrix4x4 RotationMatrixRad(float x, float y, float z)
        {
            Matrix4x4 zM = new Matrix4x4(
               (float)Math.Cos(z), (float)Math.Sin(z), 0, 0,
               -(float)Math.Sin(z), (float)Math.Cos(z), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1);
            Matrix4x4 yM = new Matrix4x4(
                (float)Math.Cos(y), 0, -(float)Math.Sin(y), 0,
                0, 1, 0, 0,
                (float)Math.Sin(y), 0, (float)Math.Cos(y), 0,
                0, 0, 0, 1);
            Matrix4x4 xM = new Matrix4x4(
                 1, 0, 0, 0,
                 0, (float)Math.Cos(x), (float)Math.Sin(x), 0,
                 0, -(float)Math.Sin(x), (float)Math.Cos(x), 0,
                 0, 0, 0, 1);
            Matrix4x4 rotationMatrix = zM * yM * xM;
            return rotationMatrix;
        }
        public static Quaternion QuaternionFromEulerDeg(float x, float y, float z)
        {
            return QuaternionFromEulerRad(Radians(x), Radians(y), Radians(z));
        }
            public static Quaternion QuaternionFromEulerRad(float x, float y, float z)
        {
            double cr = Math.Cos(x * 0.5);
            double sr = Math.Sin(x * 0.5);
            double cp = Math.Cos(y * 0.5);
            double sp = Math.Sin(y * 0.5);
            double cy = Math.Cos(z * 0.5);
            double sy = Math.Sin(z * 0.5);
            return new Quaternion(
                x: (float)(sr * cp * cy - cr * sp * sy),
                y: (float)(cr * sp * cy + sr * cp * sy),
                z: (float)(cr * cp * sy - sr * sp * cy),
                w: (float)(cr * cp * cy + sr * sp * sy));
        }

        public static float Radians(float f)
        {
            return (float)((f%360)* Math.PI / 180.0);
        }

        public static Vector3 Transform(Vector3 value, Quaternion rotation)
        {
            var q = new Quaternion(value.X, value.Y, value.Z, 0.0f);

            var res = Quaternion.Conjugate(rotation) * q * rotation;

            return new Vector3(res.X, res.Y, res.Z);
        }
        public static Vector3 MultMatxVec3V3(Vector3 value, Matrix4x4 matrix4X4)
        {
            Vector4 v = MultMatxVec3V4(value, matrix4X4);
            Console.WriteLine(matrix4X4);
            Console.WriteLine(matrix4X4.M43);
            return new Vector3(v.X,v.Y,v.Z);
        }

        public static Vector4 MultMatxVec3V4(Vector3 value, Matrix4x4 matrix4X4)
        {
            return new Vector4(
                value.X * matrix4X4.M11 + value.Y * matrix4X4.M21 + value.Z * matrix4X4.M31 + matrix4X4.M41,
                value.X * matrix4X4.M12 + value.Y * matrix4X4.M22 + value.Z * matrix4X4.M32 + matrix4X4.M42,
                value.X * matrix4X4.M13 + value.Y * matrix4X4.M23 + value.Z * matrix4X4.M33 + matrix4X4.M43,
                value.X * matrix4X4.M14 + value.Y * matrix4X4.M24 + value.Z * matrix4X4.M34 + matrix4X4.M44
                );
        }
        public static void CalNormal(Triangle triangle)
        {
            Vector3 line1, line2;
            line1 = new Vector3(
                triangle.Points[1].Position.X - triangle.Points[0].Position.X,
                triangle.Points[1].Position.Y - triangle.Points[0].Position.Y,
                triangle.Points[1].Position.Z - triangle.Points[0].Position.Z
                );
            line2 = new Vector3(
                triangle.Points[1].Position.X - triangle.Points[0].Position.X,
                triangle.Points[1].Position.Y - triangle.Points[0].Position.Y,
                triangle.Points[1].Position.Z - triangle.Points[0].Position.Z
                );
            triangle.Normal = new Vector3(
                line1.Y * line2.Z - line1.Z * line2.Y,
                line1.Z * line2.X - line1.X * line2.Z,
                line1.X * line2.Y - line1.Y * line2.X
                ) ;
        }
    }
}