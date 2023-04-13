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
            float max =3.14f*2f;
            float halfRange = max/2;
            Console.WriteLine("Rotation");
            Console.WriteLine(_anchor.Rotation);
            Quaternion rotation = Quaternion.CreateFromAxisAngle(e.V, 0.1f);
            Vector3 v = Vector3.Add(_anchor.Rotation, Vector3.Transform(Vector3.One, rotation) * 0.1f*e.F);
            v.X = v.X == -3.14f ? halfRange : ((v.X % max) + halfRange) % max - halfRange;
            v.Y = v.Y == -3.14f ? halfRange : ((v.Y % max) + halfRange) % max - halfRange;
            v.Z = v.Z == -3.14f ? halfRange : ((v.Z % max) + halfRange) % max - halfRange;
            _anchor.Rotation = v;
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
            _anchor.Position = Vector3.Add(_anchor.Position,e.V * 0.1f);
            CameraMatx();
        }
        public void SetRes(int x, int y)
        {
            ResX = x; ResY = y;
        }

        public void ProjectMax()
        {
            float aspectRatio = (float)ResX / ResY;
            float tanHalfFOV = 1 / (float)Math.Tan((Math.PI / 180) * Theta / 2); // 1/ tangent of half FOV angle
            float near = Vector3.Dot((Vector3.Zero - _anchor.Position), _anchor.Rotation) - 1;
            float far = Vector3.Dot((Vector3.Zero - _anchor.Position), _anchor.Rotation) + 0.5f * 1.79f;
            near = Math.Max(near, 0.01f);  // Add a small offset to avoid clipping
            far = Math.Max(far, near + 0.01f);  // Add a small offset to avoid division by zero
            //float near = -_closeRange + _anchor.Position.Z;//distance of camera SIMPLE
            // float far = near + _randerRange;
            //float A = -(far + near) / (far - near);
            // float B = -(2 * far * near) / (far - near);
            float A = -(_randerRange + _closeRange) / (_randerRange - _closeRange);
            float B = -(2 * _randerRange * _closeRange) / (_randerRange - _closeRange);
            _projectionMatrix = new Matrix4x4(
                aspectRatio * tanHalfFOV, 0, 0, 0,
                0, tanHalfFOV, 0, 0,
                0, 0, A, B,
                0, 0, -1, 0);
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

            if (Vector3.Distance(cameraRotation, Vector3.Zero) > 0.00001f)
            {
                Vector3 reference = Vector3.UnitY;
                if (Vector3.Dot(cameraRotation, reference) >= 0.999f)
                {
                    reference = Vector3.UnitX;
                }
                float angle = (float)Math.Acos(Vector3.Dot(cameraRotation, reference));
                Vector3 axis = Vector3.Normalize(Vector3.Cross(cameraRotation, reference));
                _rotationQ = Quaternion.CreateFromAxisAngle(axis, angle);
            }
            else
            {
                _rotationQ = new Quaternion(0, 0, 0, 1);
            }
            //both options
            /*
            Matrix4x4 Rx = new Matrix4x4(
                1, 0, 0, 0,
                0, (float)Math.Cos(cameraRotation.X), -(float)Math.Sin(cameraRotation.X), 0,
                0, (float)Math.Sin(cameraRotation.X), (float)Math.Cos(cameraRotation.X), 0,
                0, 0, 0, 1);
            Matrix4x4 Ry = new Matrix4x4(
                (float)Math.Cos(cameraRotation.Y), 0, (float)Math.Sin(cameraRotation.Y), 0,
                0, 1, 0, 0,
                -(float)Math.Sin(cameraRotation.Y), 0, (float)Math.Cos(cameraRotation.Y), 0,
                0, 0, 0, 1);
            Matrix4x4 Rz = new Matrix4x4(
                (float)Math.Cos(cameraRotation.Z), -(float)Math.Sin(cameraRotation.Z), 0, 0,
                (float)Math.Sin(cameraRotation.Z), (float)Math.Cos(cameraRotation.Z), 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1);
            Matrix4x4 rotationMatrix = Rz * Ry * Rx;*/

            _viewMatrix = translationMatrix;// * rotationMatrix;;
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

            Quaternion rotationQ;

            if (Vector3.Distance(sceenObject.Anchor.Rotation, Vector3.Zero) > 0.00001f)
            {
                Vector3 reference = Vector3.UnitY;
                if (Vector3.Dot(sceenObject.Anchor.Rotation, reference) >= 0.999f)
                {
                    reference = Vector3.UnitX;
                }
                float angle = (float)Math.Acos(Vector3.Dot(sceenObject.Anchor.Rotation, reference));
                
                Vector3 axis = Vector3.Normalize(Vector3.Cross(sceenObject.Anchor.Rotation, reference));
                rotationQ = Quaternion.CreateFromAxisAngle(axis, angle);
                
            }
            else
            {
                rotationQ = new Quaternion(0, 0, 0, 1);
            }

            foreach (Triangle triangle in sceenObject.Triangles)
            {
                TriangleProjectrion(triangle, translationMatrix, scaleMatrix, rotationQ);
            }
        }

        public void TriangleProjectrion(Triangle triangle, Matrix4x4 translationMatrix, Matrix4x4 scaleMatrix, Quaternion rotationQ)
        {
            Vector2[] v = new Vector2[3];
            int i = 0;
            foreach (MyPoint point in triangle.Points)
            {
                //Console.WriteLine("pre all");
                //Console.WriteLine(point.Position);
                //
                Vector3 scaleWorld = Vector3.Transform(point.Position, scaleMatrix);
                //Console.WriteLine("scaleWorld");
                //Console.WriteLine(scaleMatrix);
                //Console.WriteLine(scaleWorld);
                Vector3 rotationWorld = Vector3.Transform(scaleWorld, rotationQ);
                //Console.WriteLine("rotationWorld");
                //Console.WriteLine(rotationQ);
                //Console.WriteLine(rotationWorld);
                Vector3 translationWorld = Vector3.Transform(rotationWorld, Matrix4x4.Transpose(translationMatrix));
                //Console.WriteLine("translationWorld");
                //Console.WriteLine(translationMatrix);
                //Console.WriteLine(translationWorld);
                //
                //
                Vector3 rotationCamera = Vector3.Transform(translationWorld, _rotationQ);
                //Console.WriteLine("rotationCamera");
                //Console.WriteLine(_rotationQ);
                //Console.WriteLine(rotationCamera);
                Vector3 translationCamera = Vector3.Transform(rotationCamera, Matrix4x4.Transpose(_viewMatrix));
                //Console.WriteLine("translationCamera");
                //Console.WriteLine(_viewMatrix);
                Console.WriteLine(translationCamera);
                //Normalized Device coordinates

                _projectionMatrix = Matrix4x4.Transpose(_projectionMatrix);
                Vector4 vector4 = new Vector4(translationCamera, 1);
                vector4 = Vector4.Transform(vector4, _projectionMatrix);
                if (vector4.W > 0.00001f)
                {
                    vector4.X /= (vector4.Z * vector4.W);
                    vector4.Y /= (vector4.Z * vector4.W);
                    
                }
                v[i] = new Vector2(((vector4.X + 1) / 2) * _resX, ((vector4.Y + 1) / 2)* _resY);
                i++;
            }
            DrawTriangle(v);
        }

        public void DrawTriangle(Vector2[] v)
        {
            TriForm.AddTriangle(v);
            Console.WriteLine(ii++);
        }

        public static Vector3 Transform(Vector3 value, Quaternion rotation)
        {
            var q = new Quaternion(value.X, value.Y, value.Z, 0.0f);

            var res = Quaternion.Conjugate(rotation) * q * rotation;

            return new Vector3(res.X, res.Y, res.Z);
        }
    }
}