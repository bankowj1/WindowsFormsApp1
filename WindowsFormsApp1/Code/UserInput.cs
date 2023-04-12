using System.Windows.Forms;
using System;
using WindowsFormsApp1.Util;
using System.Numerics;

namespace WindowsFormsApp1.Code
{
    public class UserInput
    {
        public event EventHandler<Vector3EventArgs> CameraRotation;
        private TriangleForm _triForm;

        public UserInput(TriangleForm triForm)
        {
            
            _triForm = triForm;
            Init();
        }
        
        public void Init()
        {
            _triForm.KeyDown += Form_KeyDown;
        }
        private void Form_KeyDown(object sender, KeyEventArgs e)
        {
            Vector3 rotation;
            switch (e.KeyCode)
            {
                case Keys.Q:
                    // Rotate camera left
                    rotation = new Vector3(0f, -1f, 0f);
                    CameraRotation?.Invoke(this, new Vector3EventArgs(rotation));
                    break;
                case Keys.E:
                    // Rotate camera right
                    rotation = new Vector3(0f, 1f, 0f);
                    CameraRotation?.Invoke(this, new Vector3EventArgs(rotation));
                    break;
                case Keys.W:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera UP
                        rotation = new Vector3(1f, 0f, 0f);
                        CameraRotation?.Invoke(this, new Vector3EventArgs(rotation));
                        break;
                    }
                    else
                    {
                        // W is pressed
                    }
                    break;
                case Keys.A:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera UP
                        rotation = new Vector3(0f, 0f, -1f);
                        CameraRotation?.Invoke(this, new Vector3EventArgs(rotation));
                        break;
                    }
                    else
                    {
                        // A is pressed
                    }
                    break;
                case Keys.S:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera DOWN
                        rotation = new Vector3(-1f, 0f, 0f);
                        CameraRotation?.Invoke(this, new Vector3EventArgs(rotation));
                        break;
                    }
                    else
                    {
                        // S is pressed
                    }
                    break;
                case Keys.D:
                    if (e.Modifiers == Keys.Shift)
                    {
                        // Rotate camera UP
                        rotation = new Vector3(0f, 0f, 1f);
                        CameraRotation?.Invoke(this, new Vector3EventArgs(rotation));
                        break;
                    }
                    else
                    {
                        // D is pressed
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
}