using MathNet.Numerics.RootFinding;
using System;
using System.Collections.Generic;
using System.Numerics;
using System.Windows.Forms;
using WindowsFormsApp1.Data;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;

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
        private Matrix4x4 _projectionMatrix;
        private Matrix4x4 _viewMatrix;
        //rotation
        Quaternion quaternionDirection = new Quaternion(0,0,0,1);
        private Vector3 _forward = Vector3.UnitZ;
        private Vector3 _up = Vector3.UnitY;
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
        public void GUI()
        {
            _triForm.pos = _anchor.Position;
            _triForm.rot = _anchor.Rotation;
            _triForm.theta = _theta;
        }

        private void RotateCamera(object sender, RotEventArgs e)
        {
            Console.WriteLine("curr");
            Console.WriteLine(_anchor.Rotation);
            Console.WriteLine("rotcamdoit");
            Vector3 move = e.V * _rotationSpeed * e.F;
            Console.WriteLine(move);
            Quaternion q1 = QuaternionFromEulerDeg(move.X, move.Y, move.Z);
            q1 = Quaternion.Normalize(q1);
            Console.WriteLine(q1);
            Console.WriteLine(quaternionDirection);
            Console.WriteLine(DegFromQuat(q1));
            Console.WriteLine(DegFromQuat(quaternionDirection));
            Quaternion comb =  quaternionDirection* q1;
            quaternionDirection = Quaternion.Normalize(comb);
            Console.WriteLine(quaternionDirection);
            Vector3 res = DegFromQuat(quaternionDirection);
            Console.WriteLine(res);
           
            
            _anchor.Rotation = new Vector3(res.X%360, res.Y%360, res.Z%360);
            Console.WriteLine(_anchor.Rotation);
            _triForm.rot = _anchor.Rotation;
            CameraMatx();
        }



        private void ZoomCamera(object sender, FloatEventArgs e)
        {
            _theta += e.V;
            if (_theta > 180)
                _theta = 180;
            _triForm.theta = _theta;
            ProjectMax();
        }
        private void PositionCamera(object sender, RotEventArgs e)
        {
            //_anchor.Position = Vector3.Add(_anchor.Position, Vector3.Transform(e.V, ) * 0.1f);
            Vector3 move = e.V*e.F;
            move = Transform(move, quaternionDirection);
            Console.WriteLine(_anchor.Rotation);
            move = move * 0.1f;
            _anchor.Position = Vector3.Add(_anchor.Position,new Vector3(move.X,move.Y,-move.Z));
            Console.WriteLine(_anchor.Position);
            _triForm.pos = _anchor.Position;
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
            if(quaternionDirection == new Quaternion(0,0,0,1))
                quaternionDirection = QuaternionFromEulerDeg(cameraRotation.X, cameraRotation.Y, cameraRotation.Z);
            quaternionDirection = Quaternion.Normalize(quaternionDirection);
            //Matrix4x4.Invert(PointAt(cameraPosition, cameraPosition+_forward, _up),out _viewMatrix);
            //Console.WriteLine("_viewMatrix");
            //Console.WriteLine(PointAt(cameraPosition, cameraPosition + _forward, _up));
            //Matrix4x4.Invert( ViewFromQuat(quaternionDirection, cameraPosition),out _viewMatrix);
            //Console.WriteLine("_viewMatrix");
            //Console.WriteLine(ViewFromQuat(quaternionDirection, cameraPosition));
            Matrix4x4.Invert(RotFromQuad(quaternionDirection) * translationMatrix, out _viewMatrix);
            Console.WriteLine("rotmat");
            Console.WriteLine(_viewMatrix);

            /*        
            Console.WriteLine(Matrix4x4.CreateFromYawPitchRoll(cameraRotation.X, cameraRotation.Y, -cameraRotation.Z));
            _viewMatrix =  translationMatrix * rotationMatrix ;*/
        }
        public Matrix4x4 ViewFromQuat(Quaternion q,Vector3 centre)
        {
            float sqw = q.W * q.W;
            float sqx = q.X * q.X;
            float sqy = q.Y * q.Y;
            float sqz = q.Z * q.Z;
            Matrix4x4 m = new Matrix4x4();
            m.M11 =  sqx - sqy - sqz + sqw;
            m.M22 = -sqx + sqy - sqz + sqw;
            m.M33 = -sqx - sqy + sqz + sqw;

            float tmp1 = q.X * q.Y;
            float tmp2 = q.Z * q.W;
            m.M21 = 2.0f * (tmp1 + tmp2);
            m.M12 = 2.0f * (tmp1 - tmp2);

            tmp1 = q.X * q.Z;
            tmp2 = q.Y * q.W;
            m.M31 = 2.0f * (tmp1 - tmp2);
            m.M13 = 2.0f * (tmp1 + tmp2);

            tmp1 = q.Y * q.Z;
            tmp2 = q.X * q.W;
            m.M23 = 2.0f * (tmp1 + tmp2);
            m.M32 = 2.0f * (tmp1 - tmp2);

            m.M41 = centre.X - centre.X * m.M11 - centre.Y * m.M21 - centre.Z * m.M31;
            m.M42 = centre.Y - centre.X * m.M12 - centre.Y * m.M22 - centre.Z * m.M32;
            m.M43 = centre.Z - centre.X * m.M13 - centre.Y * m.M32 - centre.Z * m.M33;
            m.M14 = m.M24 = m.M34 = 0.0f;
	        m.M44 = 1.0f;
            return m;
        }
        public Matrix4x4 RotFromQuad(Quaternion q)
        {
            float sqw = q.W * q.W;
            float sqx = q.X * q.X;
            float sqy = q.Y * q.Y;
            float sqz = q.Z * q.Z;
            float xy = q.X * q.Y;
            float wz = q.W * q.Z;
            float xz = q.X * q.Z;
            float wy = q.W * q.Y;
            float yz = q.Y * q.Z;
            float wx = q.W * q.X;
            Matrix4x4 m = new Matrix4x4();
            m.M11 = 1.0f - 2.0f * (sqy + sqz);
            m.M12 = 2.0f * (xy + wz);
            m.M13 = 2.0f * (xz - wy);
            m.M14 = 0.0f;
            m.M21 = 2.0f * (xy - wz);
            m.M22 = 1.0f - 2.0f * (sqz + sqx);
            m.M23 = 2.0f * (yz + wx);
            m.M24 = 0.0f;
            m.M31 = 2.0f * (xz + wy);
            m.M32 = 2.0f * (yz - wx);
            m.M33 = 1.0f - 2.0f * (sqy + sqx);
            m.M34 = 0.0f;
            m.M41 = 0.0f;
            m.M42 = 0.0f;
            m.M43 = 0.0f;
            m.M44 = 1.0f;
            return m;
        }


        public Matrix4x4 PointAt(Vector3 eye,Vector3 at, Vector3 up)
        {
            //calc new forward
            Vector3 newForward = at - eye;
            newForward = Vector3.Normalize(newForward);

            //calc new up
            Vector3 a = newForward * Vector3.Dot(up, newForward);
            Vector3 newUp = up - a;
            newUp = Vector3.Normalize(newUp);

            Vector3 newRight = Vector3.Cross(newUp, newForward);

            /*
            Matrix4x4 m = new Matrix4x4(
                newRight.X, newRight.Y, newRight.Z, 0.0f,
                newUp.X, newUp.Y, newUp.Z, 0.0f,
                newForward.X, newForward.Y, newForward.Z, 0.0f,
                eye.X, eye.Y, eye.Z, 1.0f);*/
            Matrix4x4 m = new Matrix4x4(
                newRight.X, newRight.Y, newRight.Z, 0.0f,
                newUp.X, newUp.Y, newUp.Z, 0.0f,
                newForward.X, newForward.Y, newForward.Z, 0.0f,
                -Vector3.Dot(newRight, eye), -Vector3.Dot(newUp, eye), -Vector3.Dot(newForward, eye), 1.0f);
            return m;
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

           
            Matrix4x4 rotationMatrix = RotationMatrixDeg(sceenObject.Anchor.Rotation.X,  sceenObject.Anchor.Rotation.Y, sceenObject.Anchor.Rotation.Z);

            foreach (Triangle triangle in sceenObject.Triangles)
            {
                TriangleProjectrion(triangle, translationMatrix, scaleMatrix,rotationMatrix);
            }
        }

        public void TriangleProjectrion(Triangle triangle, Matrix4x4 translationMatrix, Matrix4x4 scaleMatrix, Matrix4x4 rotatiobMatrix)
        {
            Vector2[] v = new Vector2[3];
            int i = 0;
            Vector3[] vWorld = new Vector3[3];
            for (int tris = 0 ; tris < 3; tris++)
            {

                //Vector4 translationWorld = Vector4.Transform(point.Position, _projectionMatrix * Matrix4x4.Transpose( _viewMatrix *(translationMatrix * rotatiobMatrix*scaleMatrix )));
                //Console.WriteLine("_anchor.Position");
                //Console.WriteLine(triangle.Points[tris].Position);
                vWorld[tris] = MultMatxVec3V3(triangle.Points[tris].Position, scaleMatrix*rotatiobMatrix * translationMatrix);
                //Console.WriteLine(rotatiobMatrix);
                //Console.WriteLine(translationMatrix);
                //Console.WriteLine("vector32");
                //Console.WriteLine(vWorld);
                
            }
            CalNormal(triangle);
            for (int tris = 0; tris < 3; tris++)
            {
                Vector3 vView = MultMatxVec3V3(vWorld[tris], _viewMatrix);


                Vector4 vector4 = MultMatxVec3V4(vView, _projectionMatrix);
                if (Math.Abs(vector4.W) > 0.00001f)
                {
                    vector4.X = vector4.X / vector4.W;
                    vector4.Y = vector4.Y / vector4.W;
                    vector4.Z = vector4.Z / vector4.W;
                }
                v[i] = new Vector2(((vector4.X + 1) / 2) * _resX, ((vector4.Y + 1) / 2) * _resY);
                //Console.WriteLine(vector4);
                i++;
            }
            DrawTriangle(v);
        }

        public void DrawTriangle(Vector2[] v)
        {
            TriForm.AddTriangle(v);
            //Console.WriteLine(ii++);
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
        public static Vector3 DegFromQuat(Quaternion q)
        {
            Vector3 v = RadFromQuat(q);
            return new Vector3(v.X *(float)(180/Math.PI), v.Y * (float)(180 / Math.PI), v.Z * (float)(180 / Math.PI));
        }

        public static Vector3 RadFromQuat(Quaternion q)
        {
            Vector3 v = new Vector3();
            // roll / x
            double sinr_cosp = 2 * (q.W * q.X + q.Y * q.Z);
            double cosr_cosp = 1 - 2 * (q.X * q.X + q.Y * q.Y);
            v.X = (float)Math.Atan2(sinr_cosp, cosr_cosp);

            // pitch / y
            double sinp = 2 * (q.W * q.Y - q.Z * q.X);
            if (Math.Abs(sinp) >= 1)
            {
                if(sinp < 0)
                {
                    v.Y = -(float)Math.PI / 2;
                }
                else
                {
                    v.Y = (float)Math.PI / 2;
                }
            }
            else
            {
                v.Y = (float)Math.Asin(sinp);
            }

            // yaw / z
            double siny_cosp = 2 * (q.W * q.Z + q.X * q.Y);
            double cosy_cosp = 1 - 2 * (q.Y * q.Y + q.Z * q.Z);
            v.Z = (float)Math.Atan2(siny_cosp, cosy_cosp);
            return v;
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
        public static Vector3 MultQuatVec3V3(Vector3 v, Quaternion q)
        {
            Vector3 u = new Vector3(q.X, q.Y, q.Z);
            float s = q.W;
            return 2.0f * Vector3.Dot(u, v) * u + (s * s - Vector3.Dot(u, u)) * v + 2.0f * s * Vector3.Cross(u, v);
        }
        public static Vector3 MultMatxVec3V3(Vector3 value, Matrix4x4 matrix4X4)
        {
            Vector4 v = MultMatxVec3V4(value, matrix4X4);
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