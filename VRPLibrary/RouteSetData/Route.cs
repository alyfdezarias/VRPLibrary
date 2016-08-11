using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using VRPLibrary.FleetData;

namespace VRPLibrary.RouteSetData
{
    public class Route: List<int>, ICloneable
    {
        public VehicleType Vehicle { get; set; }

        public Route(VehicleType vehicle)
            : base()
        {
            Vehicle = vehicle;
        }

        public int SelectRandomClientIndex(Random rdObj)
        {
            return rdObj.Next(this.Count);
        }

        public bool IsEmpty
        {
            get { return Count == 0; }
        }

        #region ICloneable Members

        public object Clone()
        {
            Route clone = new Route(Vehicle);
            clone.AddRange(this);
            return clone;
        }

        #endregion

        public virtual XElement ToXMLFormat()
        {
            XElement node = new XElement("route");
            node.Add(Vehicle.ToXmlFormat());
            node.Add(new XElement("items", new XAttribute("count", Count),
                                 from c in this
                                 select new XElement("client", new XAttribute("id", c))));
            return node;
        }

        public static Route LoadFromXML(XElement route)
        {
            double capacity = double.Parse(route.Element("vehicleType").Attribute("capacity").Value);
            int count = int.Parse(route.Element("vehicleType").Attribute("count").Value);
            VehicleType vehicle = new VehicleType(capacity, count);

            var clients = from c in route.Descendants("client")
                          select (int.Parse(c.Attribute("id").Value));

            Route newRoute = new Route(vehicle);
            newRoute.AddRange(clients);
            return newRoute;
        }

        public override string ToString()
        {
            string r = "";
            foreach (var item in this)
            {
                r += string.Format("{0}->", item);
            }
            return string.Format("Route {0}", r);
        }

        public bool IsEqual(Route r)
        {
            if (r.Count != Count)
                return false;
            for (int i = 0; i < Count; i++)
            {
                if (r[i] != this[i])
                    return false;
            }
            return true;
        }

        public void InsertAtRandomPosition(int clientID, Random rdObj)
        {
            int index = rdObj.Next(Count + 1);
            if (index < Count)
                Insert(index, clientID);
            else Add(clientID);
        }
    }
}
