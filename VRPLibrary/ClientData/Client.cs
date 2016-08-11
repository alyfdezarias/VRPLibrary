using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.ClientData
{
    public class Client : ICloneable
    {
        public int ID { get; internal set; }

        public Client(int id)
        {
            if (id < 1) throw new ArgumentOutOfRangeException();
            ID = id;
        }

        public virtual XElement ToXmlFormat()
        {
            return new XElement("client", new XAttribute("id", ID));
        }

        public override string ToString()
        {
            return string.Format("ID={0}", ID);
        }


        public virtual object Clone()
        {
            return new Client(ID);
        }
    }
}
