using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using VRPLibrary.FleetData;
using System.IO;
using VRPLibrary.ClientData;

namespace VRPLibrary.RouteSetData
{
    public class RouteSet: List<Route>, ICloneable
    {
        public RouteSet() : base() { }
        public RouteSet(IEnumerable<Route> routes)
            : base(routes)
        { }

        public bool IsEmpty
        {
            get { return CountClientServed == 0; }
        }

        public int CountNotEmpty
        {
            get { return this.Count(r => (r.Count > 0)); }
        }

        public int SelectNotEmptyRouteIndex(Random rdObj)
        {
            List<int> availables = new List<int>();
            for (int i = 0; i < Count; i++)
            {
                if (!this[i].IsEmpty)
                    availables.Add(i);
            }
            if (availables.Count > 0)
                return availables[rdObj.Next(availables.Count)];
            return -1;
        }

        public Route SelectNotEmptyRoute(Random rdObj)
        {
            int routeIndex = SelectNotEmptyRouteIndex(rdObj);
            if (routeIndex < 0) return null;
            return this[routeIndex];
        }

        public Route SelectDiffetenRoute(int selected, Random rdObj)
        {
            if (Count == 1) return null;
            int index = rdObj.Next(Count);
            if (index == selected)
                index = (selected + 1) % Count;
            return this[index];
        }

        #region ICloneable Members

        public virtual object Clone()
        {
            RouteSet clone = new RouteSet();
            foreach (var item in this)
                clone.Add((Route)(item.Clone()));
            return clone;
        }

        #endregion

        public virtual XElement ToXMLFormat()
        {
            return new XElement("routeSet",
                                new XAttribute("count", Count),
                                from r in this
                                select r.ToXMLFormat());
        }

        public static RouteSet LoadFromXML(XElement document)
        {
            var routes = from r in document.Descendants("route")
                         select r;
            RouteSet solution = new RouteSet();
            foreach (var item in routes)
                solution.Add(Route.LoadFromXML(item));
            return solution;
        }

        public static RouteSet LoadFromXML(string path)
        {
            if (Path.GetExtension(path) != ".xml")
                throw new ArgumentException();
            XElement document = XElement.Load(path);
            return LoadFromXML(document);
        }

        public override string ToString()
        {

            //string s = "";
            //foreach (var r in this)
            //{
            //    s += string.Format("{0}\n", r.ToString());
            //}
            //return string.Format("r{0} c{1}\n{2}", Count, this.Sum(x => x.Count), s);
            List<string> routes = new List<string>(this.Select(r => string.Format("{0}*", r.ToString())));
            routes.Sort();
            return string.Format("r{0} c{1} {2}", CountNotEmpty, this.Sum(x => x.Count), string.Join("", routes.Select(r => r)));
        }

        public int CountClientServed
        {
            get { return this.Sum(r => r.Count); }
        }

        public static RouteSet BuildEmptyRouteSet(Fleet fleet)
        {
            RouteSet solution = new RouteSet();
            foreach (var item in fleet)
                for (int i = 0; i < item.Count; i++)
                    solution.Add(new Route(item));
            return solution;
        }

        public static RouteSet BuildSeedRouteSet(Fleet fleet, List<PickupDeliveryClient> seedClients, Random rdObj)
        {
            RouteSet solution = new RouteSet();
            foreach (var item in fleet)
                for (int i = 0; i < item.Count; i++)
                {
                    Route r = new Route(item);
                    /*cuando se trate de flota heterogenea si hay q determinar quienes son los clientes factibles para la ruta*/
                    //var feasibles = from s in seedClients where s.Pickup <= item.Capacity && s.Delivery <= item.Capacity select s.ID;/*todos los clientes son validos en todos los vehiculos*/
                    int seedIndex = rdObj.Next(seedClients.Count);
                    int seedID = seedClients[seedIndex].ID;
                    r.Add(seedID);
                    seedClients.RemoveAt(seedIndex);
                    solution.Add(r);
                }
            return solution;


        }

        public bool IsCoincidentRouteSet(RouteSet solution)
        {
            if (Count != solution.Count) return false;
            for (int i = 0; i < Count; i++)
            {
                if (!this[i].IsEqual(solution[i]))
                    return false;
            }
            return true;
        }

        public bool ContainsRoute(Route other)
        {
            for (int i = 0; i < Count; i++)
            {
                if (this[i].IsEqual(other))
                    return true;
            }
            return false;
        }

        public void RemoveClient(int clientID)
        {
            for (int i = 0; i < Count; i++)
            {
                if (!this[i].IsEmpty)
                {
                    int index = this[i].IndexOf(clientID);
                    if (index > -1)
                    {
                        this[i].RemoveAt(index);
                        break;
                    }
                }
            }
        }

        public void CleanUnusedRoutes()
        {
            int index = 0;
            while (index < Count)
            {
                if (this[index].IsEmpty)
                    RemoveAt(index);
                else
                    index++;
            }
        }

        public void InsertAtRandomPosition(int clientID, Random rdObj)
        {
            int routeIndex = rdObj.Next(Count);
            this[routeIndex].InsertAtRandomPosition(clientID, rdObj);
        }

    }
}
