using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;
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
        private Matrix4x4 _viewMatrix;
        private Quaternion _rotationQ;
        private Matrix4x4 _projectionMatrix ;

        public TriangleForm TriForm { get => _triForm; set => _triForm = value; }
        public float RanderRange { get => _randerRange; set => _randerRange = value; }
        public float CloseRange { get => _closeRange; set => _closeRange = value; }
        public int ResX { get => _resX; set => _resX = value; }
        public int ResY { get => _resY; set => _resY = value; }
        public float Theta { get => _theta; set => _theta = value; }

        public Camera(List<Triangle> triangles, MyPoint anchor, Vector3 scale) : base(triangles, anchor, scale)
        {
            float projScal = RanderRange / (RanderRange - CloseRange);
            float aspectRatio = ResY / ResX;
            float F = 1 / (float)Math.Tan(Theta * 0.5f / 180 * 3.1415);
            
            Console.WriteLine(projScal);
            Console.WriteLine(aspectRatio);
            Console.WriteLine(F);
            _projectionMatrix = new Matrix4x4(
                aspectRatio * F, 0, 0, 0,
                0, F, 0, 0,
                0, 0, projScal, 1,
                0, 0, -CloseRange * projScal, 0);
            Projection();
        }

        public Camera(MyPoint anchor, Vector3 scale) : base(anchor, scale)
        {
            float projScal = RanderRange / (RanderRange - CloseRange);
            float aspectRatio = (float) ResY / ResX;
            float F = 1 / (float)Math.Tan(Theta * 0.5f / 180 * 3.1415);
            Console.WriteLine(projScal);
            Console.WriteLine(aspectRatio);
            Console.WriteLine(F);
            _projectionMatrix = new Matrix4x4(
                aspectRatio * F, 0, 0, 0,
                0, F, 0, 0,
                0, 0, projScal, 1,
                0, 0, -CloseRange * projScal, 0);
            Projection();
        }


        public void SetRes(int x, int y)
        {
            ResX = x; ResY = y; 
        }

        public void Projection()
        {
            Vector3 cameraPosition = _anchor.Position;
            Vector3 cameraRotation = _anchor.Rotation; 

            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, cameraPosition.X,
                0, 1, 0, cameraPosition.Y,
                0, 0, 1, cameraPosition.Z,
                0, 0, 0, 1);


            Vector3 reference = Vector3.UnitY;
            if (Vector3.Dot(cameraRotation, reference) >= 0.999f)
            {
                reference = Vector3.UnitX;
            }
            float angle = (float) Math.Acos(Vector3.Dot(cameraRotation, reference));
            Vector3 axis = Vector3.Normalize(Vector3.Cross(cameraRotation, reference));
            _rotationQ = Quaternion.CreateFromAxisAngle(axis, angle);
            Console.WriteLine("_rotationQ");
            Console.WriteLine(angle);
            Console.WriteLine(axis);
            Console.WriteLine(_rotationQ.X +" " + _rotationQ.Y + " " + _rotationQ.Z);
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


            _viewMatrix = translationMatrix;// * rotationMatrix;// * scaleMatrix;
            Console.WriteLine(_viewMatrix);
        }
            

        public void ObjectProjection(SceenObject sceenObject)
        {
            Matrix4x4 translationMatrix = new Matrix4x4(
                1, 0, 0, sceenObject.Anchor.Position.X,
                0, 1, 0, sceenObject.Anchor.Position.Y,
                0, 0, 1, sceenObject.Anchor.Position.Z,
                0, 0, 0, 1);

            Console.WriteLine();
            Matrix4x4 scaleMatrix = new Matrix4x4(
               sceenObject.Scale.X, 0, 0, 0,
               0, sceenObject.Scale.Y, 0, 0,
               0, 0, sceenObject.Scale.Z, 0,
               0, 0, 0, 1);

            Vector3 reference = Vector3.UnitY;
            if (Vector3.Dot(sceenObject.Anchor.Rotation, reference) >= 0.999f)
            {
                reference = Vector3.UnitX;
            }
            float angle = (float)Math.Acos(Vector3.Dot(sceenObject.Anchor.Rotation, reference));
            Console.WriteLine(Vector3.Cross(sceenObject.Anchor.Rotation, reference));
            Vector3 axis = Vector3.Normalize(Vector3.Cross(sceenObject.Anchor.Rotation, reference));
            Quaternion rotationQ = Quaternion.CreateFromAxisAngle(axis, angle);
            Console.WriteLine("rotationQ");
            Console.WriteLine(angle);
            Console.WriteLine(axis);
            Console.WriteLine(rotationQ);
            foreach (Triangle triangle in sceenObject.Triangles)
            {
                TriangleProjectrion(triangle, translationMatrix, scaleMatrix, rotationQ);
            }
        }

        public void TriangleProjectrion(Triangle triangle, Matrix4x4 translationMatrix, Matrix4x4 scaleMatrix, Quaternion rotationQ)
        {
            Vector2[] v = new Vector2[3];
            int i = 0;
            foreach(MyPoint point in triangle.Points)
            {
                Console.WriteLine("pre all");
                Console.WriteLine(point.Position);

                Vector3 translationWorld = Vector3.Transform(point.Position,Matrix4x4.Transpose( translationMatrix));
                Console.WriteLine("translationWorld");
                Console.WriteLine(translationMatrix);
                Console.WriteLine(translationWorld);
                Vector3 rotationWorld = Vector3.Transform(translationWorld, rotationQ);
                Console.WriteLine("rotationWorld");
                Console.WriteLine(rotationQ);
                Console.WriteLine(rotationWorld);
                Vector3 scaleWorld = Vector3.Transform(rotationWorld, scaleMatrix);
                Console.WriteLine("scaleWorld");
                Console.WriteLine(scaleMatrix);
                Console.WriteLine(scaleWorld);

                Vector3 translationCamera = Vector3.Transform(scaleWorld, Matrix4x4.Transpose(_viewMatrix));
                Console.WriteLine("translationCamera");
                Console.WriteLine(_viewMatrix);
                Console.WriteLine(translationCamera);
                Vector3 rotationCamera = Vector3.Transform(translationCamera, _rotationQ);
                Console.WriteLine("rotationCamera");
                Console.WriteLine(_rotationQ);
                Console.WriteLine(rotationCamera);
                _projectionMatrix = Matrix4x4.Transpose(_projectionMatrix);
                float w = rotationCamera.X * _projectionMatrix.M14 + rotationCamera.Y * _projectionMatrix.M24 + rotationCamera.Z * _projectionMatrix.M34 + _projectionMatrix.M44;
                Console.WriteLine("w");
                Console.WriteLine(w);
                Console.WriteLine(_projectionMatrix);
                Vector3 vector3 = new Vector3(
                    rotationCamera.X * _projectionMatrix.M11 + rotationCamera.Y * _projectionMatrix.M21 + rotationCamera.Z * _projectionMatrix.M31 + _projectionMatrix.M41,
                    rotationCamera.X * _projectionMatrix.M12 + rotationCamera.Y * _projectionMatrix.M22 + rotationCamera.Z * _projectionMatrix.M32 + _projectionMatrix.M42,
                    rotationCamera.X * _projectionMatrix.M13 + rotationCamera.Y * _projectionMatrix.M23 + rotationCamera.Z * _projectionMatrix.M33 + _projectionMatrix.M43
                    );
                if(w > 0.00001f)
                {
                    vector3.X /= w;
                    vector3.Y /= w;
                    vector3.Z /= w;
                }
                v[i] = new Vector2((rotationCamera.X+ _resX) /2, (rotationCamera.Y+ _resY)/2);
                Console.WriteLine("v");
                Console.WriteLine(v[i]);
                i++;
            }
            DrawTriangle(v);
        }

        public void DrawTriangle(Vector2[] v)
        {
            TriForm.AddTriangle(v);
        }

        public static Vector3 Transform(Vector3 value, Quaternion rotation)
        {
            var q = new Quaternion(value.X, value.Y, value.Z, 0.0f);
            Console.WriteLine("transform");
            Console.WriteLine(value);
            var res = Quaternion.Conjugate(rotation) * q * rotation;
            Console.WriteLine("posttransform");
            Console.WriteLine(res);
            return new Vector3(res.X, res.Y, res.Z);
        }
    }
}
