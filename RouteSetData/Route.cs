using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using System.IO;
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

        public List<int> RemoveRandomClients(int count, Random rdObj)
        {
            List<int> removed = new List<int>();
            while (removed.Count < count)
            {
                int index = rdObj.Next(this.Count);
                removed.Add(this[index]);
                this.RemoveAt(index);
            }
            return removed;
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

        public static Route LoadFromXMLFile(string path)
        {
            if (Path.GetExtension(path) != ".xml")
                throw new ArgumentException();
            XElement document = XElement.Load(path);
            return LoadFromXML(document);
        }

        public override string ToString()
        {
            //string r = "";
            //foreach (var item in this)
            //{
            //    r += string.Format("{0}->", item);
            //}
            //return string.Format("Route {0}", r);
            return string.Join("", this.Select(c => c.ToString("D3"))); //fijo en tres digitos
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

        public void InsertAtRandomPosition(List<int> clients, Random rdObj)
        {
            int index = rdObj.Next(Count + 1);
            if (index < Count)
                InsertRange(index, clients);
            else AddRange(clients);
        }
    }

    public class ExtRouteInfo
    {
        public double Cost { get; set; }
        public Route Current { get; set; }

        public ExtRouteInfo(Route current, double cost)
        {
            Current = (Route)current.Clone();
            Cost = cost;
        }

        public bool Dominates(ExtRouteInfo other)
        {
            if (Current.Count != other.Current.Count) return false;
            for (int i = 0; i < Current.Count; i++)
                if (!other.Current.Contains(Current[i])) return false;
            return Cost <= other.Cost;
        }
    }
}
