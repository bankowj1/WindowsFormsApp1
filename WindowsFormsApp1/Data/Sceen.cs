using System.Collections.Generic;

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