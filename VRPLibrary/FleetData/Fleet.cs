using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.FleetData
{
    public class Fleet : List<VehicleType>
    {
        public Fleet() : base() { }

        public double TotalCapacity
        {
            get { return this.Sum(v => v.Capacity * v.Count); }
        }

        public int FleetSize
        {
            get { return this.Sum(v => v.Count); }
        }

        public static Fleet CreateHomogeneousFleet(int count, double capacity)
        {
            Fleet f = new Fleet();
            f.Add(new VehicleType(capacity, count));
            return f;
        }

        public virtual XElement ToXMLFormat()
        {
            XElement node = new XElement("fleet",
                                         new XAttribute("size", FleetSize),
                                         new XAttribute("totalCapacity", TotalCapacity),
                                         from v in this
                                         select v.ToXmlFormat());
            return node;
        }

        public static Fleet LoadFromXML(XElement document)
        {
            Fleet newFleet = new Fleet();
            var vehicles = from v in document.Descendants("vehicleType")
                           select new VehicleType(
                           double.Parse(v.Attribute("capacity").Value),
                           int.Parse(v.Attribute("count").Value));
            newFleet.AddRange(vehicles);
            return newFleet;
        }
    }
}
