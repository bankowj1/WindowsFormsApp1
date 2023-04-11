using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace WindowsFormsApp1.Data
{
    public class Sceen
    {
        public List<SceenObject> sceenObjects;
        public Sceen()
        {
            sceenObjects = new List<SceenObject>();
        }

        public void AddSceenObject(SceenObject sceenObject)
        {
            sceenObjects.Add(sceenObject);
        }
    }
}
