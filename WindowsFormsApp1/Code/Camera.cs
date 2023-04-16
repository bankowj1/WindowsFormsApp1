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
        private float _randerRange = 10000f;
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
            _anchor.Rotation = _anchor.Rotation + (e.V * e.F * 2f);
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
            float aspectRatio = (float)ResX / ResY;
            float tanHalfFOV = (float)Math.Tan((Math.PI / 180) * (Theta / 2)); // 1/ tangent of half FOV angle
            float near = Vector3.Dot((Vector3.Zero - _anchor.Position), _anchor.Rotation);
            float far = near + _randerRange;
            //near = Math.Max(near, 0.01f);  // Add a small offset to avoid clipping
            //far = Math.Max(far, near + 0.01f);  // Add a small offset to avoid division by zero
            //float near = -_closeRange + _anchor.Position.Z;//distance of camera SIMPLE
            // float far = near + _randerRange;
            //float A = -(far + near) / (far - near);
            // float B = -(2 * far * near) / (far - near);
            float A = -(_randerRange + _closeRange) / (_randerRange - _closeRange);
            float B = -(2 * _randerRange * _closeRange) / (_randerRange - _closeRange);
            _projectionMatrix = new Matrix4x4(
                1/ (aspectRatio * tanHalfFOV), 0, 0, 0,
                0, (1/tanHalfFOV), 0, 0,
                0, 0, A, B,
                0, 0, -1, 0);
            
           /* _projectionMatrix = new Matrix4x4(
                (1/tanHalfFOV) / aspectRatio, 0, 0, 0,
                0, 1/tanHalfFOV, 0, 0,
                0, 0, _randerRange/(_closeRange-_randerRange),  _closeRange* _randerRange / (_closeRange - _randerRange),
                0, 0, -1, 0);
            _projectionMatrix = new Matrix4x4(
                1/ ( tanHalfFOV* aspectRatio), 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0, _randerRange / (_closeRange - _randerRange), _closeRange * _randerRange / (_closeRange - _randerRange),
                0, 0, -1, 0);
            _projectionMatrix = new Matrix4x4(
                1 / (tanHalfFOV*aspectRatio), 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0, far / (near - far), -near * far / (near - far),
                0, 0, 1, 0);*/
            /*_projectionMatrix = new Matrix4x4(
                (1 / tanHalfFOV) / aspectRatio, 0, 0, 0,
                0, 1 / tanHalfFOV, 0, 0,
                0, 0,-(_randerRange + _closeRange)/(_randerRange-_closeRange), -(2*_randerRange*_closeRange)/ (_randerRange - _closeRange),
                0, 0, 1, 0);*/

        }

        public void CameraMatx()
        {
            Vector3 cameraPosition = _anchor.Position;
            Vector3 cameraRotation = _anchor.Rotation;

            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, cameraPosition.X,
                0, 1, 0, cameraPosition.Y,
                0, 0, 1, cameraPosition.Z,
                0, 0, 0, 1);
            _rotationQ = QuaternionFromEulerDeg(cameraRotation.X, cameraRotation.Y, -cameraRotation.Z);

            Matrix4x4 rotationMatrix = RotationMatrixDeg(cameraRotation.X, cameraRotation.Y, -cameraRotation.Z);

            _viewMatrix =  translationMatrix * rotationMatrix;
        }

        public void ObjectProjection(SceenObject sceenObject)
        {
            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, sceenObject.Anchor.Position.X,
                0, 1, 0, sceenObject.Anchor.Position.Y,
                0, 0, 1, sceenObject.Anchor.Position.Z,
                0, 0, 0, 1);

            Matrix4x4 scaleMatrix = new Matrix4x4(
               sceenObject.Scale.X, 0, 0, 0,
               0, sceenObject.Scale.Y, 0, 0,
               0, 0, sceenObject.Scale.Z, 0,
               0, 0, 0, 1);

            Quaternion rotationQ = QuaternionFromEulerDeg(sceenObject.Anchor.Rotation.X, sceenObject.Anchor.Rotation.Y, -sceenObject.Anchor.Rotation.Z);

            Matrix4x4 rotationMatrix = RotationMatrixDeg(sceenObject.Anchor.Rotation.X,  sceenObject.Anchor.Rotation.Y, -sceenObject.Anchor.Rotation.Z);

            foreach (Triangle triangle in sceenObject.Triangles)
            {
                TriangleProjectrion(triangle, translationMatrix, scaleMatrix, rotationQ,rotationMatrix);
            }
        }

        public void TriangleProjectrion(Triangle triangle, Matrix4x4 translationMatrix, Matrix4x4 scaleMatrix, Quaternion rotationQ,Matrix4x4 rotatiobMatrix)
        {
            Vector2[] v = new Vector2[3];
            int i = 0;
            foreach (MyPoint point in triangle.Points)
            {

                Vector4 translationWorld = Vector4.Transform(point.Position,  Matrix4x4.Transpose(_projectionMatrix * _viewMatrix*(translationMatrix * rotatiobMatrix*scaleMatrix )));

                if (Math.Abs(translationWorld.W) > 0.00001f)
                {
                    translationWorld.X = translationWorld.X/ translationWorld.W;
                    translationWorld.Y = translationWorld.Y / translationWorld.W;
                    translationWorld.Z = translationWorld.Z / translationWorld.W;
                }
                v[i] = new Vector2(((translationWorld.X + 1) / 2) * _resX, ((translationWorld.Y + 1) / 2)* _resY);
                Console.WriteLine(translationWorld);
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
               (float)Math.Cos(z), -(float)Math.Sin(z), 0, 0,
               (float)Math.Sin(z), (float)Math.Cos(z), 0, 0,
               0, 0, 1, 0,
               0, 0, 0, 1);
            Matrix4x4 yM = new Matrix4x4(
                (float)Math.Cos(y), 0, (float)Math.Sin(y), 0,
                0, 1, 0, 0,
                -(float)Math.Sin(y), 0, (float)Math.Cos(y), 0,
                0, 0, 0, 1);
            Matrix4x4 xM = new Matrix4x4(
                 1, 0, 0, 0,
                 0, (float)Math.Cos(x), -(float)Math.Sin(x), 0,
                 0, (float)Math.Sin(x), (float)Math.Cos(x), 0,
                 0, 0, 0, 1);
            Matrix4x4 rotationMatrix = xM * yM * zM;
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
    }
}