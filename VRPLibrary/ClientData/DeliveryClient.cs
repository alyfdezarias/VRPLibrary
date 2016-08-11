using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.ClientData
{
    public class DeliveryClient : Client
    {
        public double Delivery { get; set; }

        public DeliveryClient(int id, double delivery)
            : base(id)
        {
            if (delivery < 0) throw new ArgumentOutOfRangeException("Delivery can not be negative");
            Delivery = delivery;
        }

        public override XElement ToXmlFormat()
        {
            XElement node = base.ToXmlFormat();
            node.Add(new XAttribute("delivery",Delivery));
            return node;
        }

        public override string ToString()
        {
            return base.ToString() + string.Format(" Delivery={0}", Delivery);
        }

        public override object Clone()
        {
            return new DeliveryClient(ID, Delivery);
        }
    }
}
