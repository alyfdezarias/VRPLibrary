using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.ClientData
{
    public class PickupDeliveryClient : DeliveryClient
    {
        public double Pickup { get; set; }

        public PickupDeliveryClient(int id, double delivery, double pickup)
            : base(id, delivery)
        {
            if (pickup < 0) throw new ArgumentOutOfRangeException("Pickup con not be negative");
            Pickup = pickup;
        }

        public override XElement ToXmlFormat()
        {
            XElement node = base.ToXmlFormat();
            node.Add(new XAttribute("pickup", Pickup));
            return node;
        }

        public double DemandDifference
        {
            get { return Pickup - Delivery; }
        }

        public double Demand
        {
            get { return Pickup + Delivery; }
        }

        public override string ToString()
        {
            return base.ToString() + string.Format(" Pickup={0}", Pickup);
        }

        public override object Clone()
        {
            return new PickupDeliveryClient(ID, Delivery, Pickup);
        }
    }
}
