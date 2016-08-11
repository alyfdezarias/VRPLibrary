using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using VRPLibrary.ClientData;
using VRPLibrary.FleetData;
using VRPLibrary.RouteSetData;
using System.IO;

namespace VRPLibrary.ProblemData
{
    public abstract class VehicleRoutingProblem<TClient> where TClient : Client
        
    {
        public string ProblemName { get; set; }
        public Fleet Vehicles { get; internal set; }
        public double[,] TravelDistance { get; internal set; }
        public ClientSet<TClient> Clients { get; internal set; }

        protected VehicleRoutingProblem(ClientSet<TClient> clientSet, Fleet vehicles, TravelData travelDistance)
        {
            Clients = clientSet;
            Vehicles = vehicles;
            TravelDistance = travelDistance.Data;
            ProblemName = "VRP";
        }

        protected VehicleRoutingProblem(string path)
        {
            if (Path.GetExtension(path) != ".xml")
                throw new ArgumentException();
            XElement document = XElement.Load(path);
            var data = TravelData.LoadFromXML(document);
            TravelDistance = data.Data;
            ProblemName = document.Attribute("problemName").Value;
            //TODO: falta leer los datos de la flota
        }

        #region Feasibility
        public abstract bool IsFeasible(Route current);

        public virtual bool IsFeasible(RouteSet solution)
        {
            return 0 == solution.Count(r => !IsFeasible(r));
        }

        public virtual int CountFeasible(RouteSet solution)
        {
            return solution.Count(r => !r.IsEmpty && IsFeasible(r));
        }

        public virtual int CountUnfeasibles(RouteSet solution)
        {
            return solution.Count(r => !IsFeasible(r));
        }

        public abstract bool CanInsertFeasible(Route solution, int clientID);

        #endregion

        

        #region XML

        public virtual XElement ToXMLFormat()
        {
            TravelData td = new TravelData(TravelDistance);
            XElement node = new XElement("vrp",
                                         new XAttribute("problemName", ProblemName),
                                         Clients.ToXmlFormat(),
                                         Vehicles.ToXMLFormat(),
                                         td.ToXMLFormat());
            return node;
        }

        public virtual void SaveToXML(string path)
        {
            if (Path.GetExtension(path) != ".xml")
                throw new ArgumentException();
            ToXMLFormat().Save(path);
        }

        #endregion
    }
}
