using System.Windows.Forms;
using System;
using WindowsFormsApp1.Util;
using System.Numerics;

namespace WindowsFormsApp1.Code
{
    public class UserInput
    {
        public event EventHandler<RotEventArgs> CameraRotation;
        public event EventHandler<RotEventArgs> CameraPosition;
        public event EventHandler<FloatEventArgs> CameraZoom;
        private TriangleForm _triForm;

        public UserInput(TriangleForm triForm)
        {
            
            _triForm = triForm;
            Init();
        }
        
        public void Init()
        {
            _triForm.KeyDown += Form_KeyDown;
            _triForm.MouseWheel += Form_Scroll;
        }
        private void Form_Scroll(object sender, MouseEventArgs e)
        {
            float delta = e.Delta > 0 ? 5f : -5f; // adjust the delta as desired
            CameraZoom?.Invoke(this,new FloatEventArgs(delta));
        }
        private void Form_KeyDown(object sender, KeyEventArgs e)
        {
            Vector3 rotation;
            switch (e.KeyCode)
            {
                case Keys.Q:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera 
                        rotation = new Vector3(0f, 0f, 1);
                        CameraRotation?.Invoke(this, new RotEventArgs(rotation,-1f));
                        break;
                    }
                    else
                    {
                        rotation = new Vector3(90f, 0f, 0f);
                        CameraPosition?.Invoke(this, new RotEventArgs(rotation, 1f));
                        break;
                    }
                    break;
                case Keys.E:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera 
                        rotation = new Vector3(0f, 0f, 1f);
                        CameraRotation?.Invoke(this, new RotEventArgs(rotation,1f));
                        break;
                    
                    }
                    else
                    {
                        rotation = new Vector3(90f, 0f, 0f);
                        CameraPosition?.Invoke(this, new RotEventArgs(rotation, -1f));
                        break;
                    }
                    break;
                case Keys.W:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera UP
                        rotation = new Vector3(1f, 0f, 0f);
                        CameraRotation?.Invoke(this, new RotEventArgs(rotation,1f));
                        break;
                    }
                    else
                    {
                        rotation = new Vector3(0f, 0f, 0f);
                        CameraPosition?.Invoke(this, new RotEventArgs(rotation,1f));
                        break;
                    }
                    break;
                case Keys.S:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera DOWN
                        rotation = new Vector3(1f, 0f, 0f);
                        CameraRotation?.Invoke(this, new RotEventArgs(rotation, -1f));
                        break;
                    }
                    else
                    {
                        rotation = new Vector3(0f, 0f, 0f);
                        CameraPosition?.Invoke(this, new RotEventArgs(rotation, -1f));
                        break;
                    }
                    break;
                case Keys.A:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera UP
                        rotation = new Vector3(0f, 1f, 0f);
                        CameraRotation?.Invoke(this, new RotEventArgs(rotation, -1f));
                        break;
                    }
                    else
                    {
                        rotation = new Vector3(0f, 90f, 0f);
                        CameraPosition?.Invoke(this, new RotEventArgs(rotation, 1f));
                        break;
                    }
                    break;
                case Keys.D:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera UP
                        rotation = new Vector3(0f, 1f, 0f);
                        CameraRotation?.Invoke(this, new RotEventArgs(rotation, 1f));
                        break;
                    }
                    else
                    {
                        rotation = new Vector3(0f, 90f, 0f);
                        CameraPosition?.Invoke(this, new RotEventArgs(rotation, -1f));
                        break;
                    }
                    break;
            }
        }
    }
    public class Vector3EventArgs : EventArgs
    {
        public Vector3 V { get; private set; }

        public Vector3EventArgs(Vector3 v)
        {
            V = v;
        }
    }
    public class RotEventArgs : EventArgs
    {
        public Vector3 V { get; private set; }
        public float F { get; private set; }

        public RotEventArgs(Vector3 v, float f)
        {
            V = v;
            F = f;
        }
    }
    public class FloatEventArgs : EventArgs
    {
        public float V { get; private set; }

        public FloatEventArgs(float v)
        {
            V = v;
        }
    }
}