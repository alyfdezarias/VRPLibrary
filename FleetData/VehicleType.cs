using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.FleetData
{
    public class VehicleType
    {
        public int Count { get; set; }
        public double Capacity { get; set; }

        public VehicleType(double capacity, int count)
        {
            if (capacity < 0 || count < 1 ) throw new ArgumentOutOfRangeException();
            Capacity = capacity;
            Count = count;
        }

        public VehicleType(double capacity) : this(capacity, 1) { }

        public virtual XElement ToXmlFormat()
        {
            return new XElement("vehicleType", 
                                new XAttribute("capacity", Capacity),
                                new XAttribute("count", Count));
        }

        public override string ToString()
        {
            return string.Format("Capacity={0} Count={1}", Capacity, Count);
        }
    }
}
