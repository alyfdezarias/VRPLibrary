using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using System.Text.RegularExpressions;
using System.IO;
using VRPLibrary.ClientData;
using VRPLibrary.FleetData;
using VRPLibrary.RouteSetData;

namespace VRPLibrary.ProblemData
{
    public class VRPSimultaneousPickupDelivery : VehicleRoutingProblem<PickupDeliveryClient>
    {
        public VRPSimultaneousPickupDelivery(ClientSet<PickupDeliveryClient> clientSet, Fleet vehicles, TravelData travelDistance)
            : base(clientSet, vehicles, travelDistance)
        { }
        

        public VRPSimultaneousPickupDelivery(string path)
            : base(path)
        {
            if (Path.GetExtension(path) != ".xml")
                throw new ArgumentException();
            XElement document = XElement.Load(path);
            Clients = ClientSet<PickupDeliveryClient>.LoadPickupDeliveryClientSetFromXML(document);
            Vehicles = Fleet.LoadFromXML(document);
        }

        public override bool CanInsertFeasible(Route solution, int clientID)
        {
            return CanInsertWeakFeasible(solution, clientID);
        }

        #region Load

        public virtual double TotalDelivery(Route current)
        {
            return current.Sum(c => Clients[c].Delivery);
        }

        public virtual double TotalPickup(Route current)
        {
            return current.Sum(c => Clients[c].Pickup);
        }

        public virtual bool Preprossesing
        {
            get { return Clients.Sum(c => c.Delivery) <= Vehicles.TotalCapacity && Clients.Sum(c => c.Pickup) <= Vehicles.TotalCapacity; }
        }

        public int CompareByDemandDifference(int clientIDi, int clientIDj)
        {
            return Clients[clientIDi].DemandDifference.CompareTo(Clients[clientIDj].DemandDifference);
        }

        #endregion

        #region Weak Feasible

        public override bool IsFeasible(Route current)
        {
            return TotalDelivery(current) <= current.Vehicle.Capacity && TotalPickup(current) <= current.Vehicle.Capacity;
        }

        public bool CanInsertWeakFeasible(Route current, int clientID)
        {
            return Math.Max(TotalPickup(current) + Clients[clientID].Pickup, TotalDelivery(current) + Clients[clientID].Delivery) <= current.Vehicle.Capacity;
        }

        public int CountWeakUnfeasibles(RouteSet solution)
        {
            return solution.Count(r => !IsFeasible(r));
        }

        public bool CanSwapWeakFeasible(Route orRoute, Route deRoute, int orID, int deID)
        {
            double orDelivery = TotalDelivery(orRoute);
            double orPickup = TotalPickup(orRoute);
            double deDelivery = TotalDelivery(deRoute);
            double dePickup = TotalPickup(deRoute);

            return Math.Max(orDelivery - Clients[orID].Delivery + Clients[deID].Delivery, orPickup - Clients[orID].Pickup + Clients[deID].Pickup) <= orRoute.Vehicle.Capacity &&
                   Math.Max(deDelivery - Clients[deID].Delivery + Clients[orID].Delivery, dePickup - Clients[deID].Pickup + Clients[orID].Pickup) <= deRoute.Vehicle.Capacity;
        }

        #endregion

        #region Strong Feasible

        public RouteSet OrderByLoadDifference(RouteSet solution)
        {
            RouteSet newSolution = (RouteSet)solution.Clone();
            for (int i = 0; i < newSolution.Count; i++)
            {
                newSolution[i] = OrderByLoadDifference(newSolution[i]);
            }
            return newSolution;
        }

        public Route OrderByLoadDifference(Route current)
        {
            current.Sort(CompareByDemandDifference);
            return current;
        }

        public bool IsStrongFeasible(Route current)
        {
            if (!IsFeasible(current))
                return false;
            return StrongOverLoad(current) == 0;
        }

        public bool IsStrongFeasible(RouteSet solution)
        {
            return 0 == solution.Count(r => !IsStrongFeasible(r));
        }

        public double WeakOverload(RouteSet solution)
        {
            return solution.Sum(r => WeakOverload(r));
        }

        public double WeakOverload(Route current)
        {
            return Math.Max(TotalPickup(current) - current.Vehicle.Capacity, 0) + Math.Max(TotalDelivery(current) - current.Vehicle.Capacity, 0);
        }

        public double StrongOverLoad(RouteSet solution)
        {
            return solution.Sum(r => StrongOverLoad(r));
        }

        public double StrongOverLoad(Route current)
        {
            double load = TotalDelivery(current);
            double penalization = 0;
            penalization += Math.Max(0, load - current.Vehicle.Capacity);
            foreach (var item in current)
            {
                load += Clients[item].DemandDifference;
                penalization += Math.Max(0, load - current.Vehicle.Capacity);
            }
            return penalization;
        }

        public double StrongRemoveOverload(Route current, int index, int count)
        {
            var loadInRange = LoadInRange(current, index, count);
            double newLoad = TotalDelivery(current) - loadInRange.Item2;
            double newPenalty = Math.Max(0, newLoad - current.Vehicle.Capacity);

            for (int i = 0; i < current.Count; i++)
            {
                if (index <= i && i < index + count ) continue;
                newLoad += Clients[current[i]].DemandDifference;
                newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
            }

            return newPenalty;
        }

        public Tuple<double, double> LoadInRange(Route current, int index, int count)
        {
            double delivery = 0;
            double pickup = 0;
            for (int i = 0; i < count; i++)
            {
                delivery += Clients[current[index + i]].Delivery;
                pickup += Clients[current[index + i]].Pickup;
            }
            return new Tuple<double, double>(pickup, delivery);
        }

        public double StrongAddOverload(Route current, int index, List<int> range)
        {
            double newLoad = TotalDelivery(current) + range.Sum(c => Clients[c].Delivery);
            double newPenalty = Math.Max(0, newLoad - current.Vehicle.Capacity);

            for (int i = 0; i < current.Count + 1; i++)
            {
                if (index == i)
                {
                    foreach (var c in range)
                    {
                        newLoad += Clients[c].DemandDifference;
                        newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
                    }
                }
                if (i < current.Count)
                {
                    newLoad += Clients[current[i]].DemandDifference;
                    newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
                }
            }
            return newPenalty;
        }

        public double StrongReplaceOverload(Route current, int index, int count, List<int> range)
        {
            var loadInRange = LoadInRange(current, index, count);
            double newLoad = TotalDelivery(current) - loadInRange.Item2 + range.Sum(c => Clients[c].Delivery);
             double newPenalty = Math.Max(0, newLoad - current.Vehicle.Capacity);

            for (int i = 0; i < current.Count; i++)
            {
                if (index == i)
                {
                    foreach (var c in range)
                    {
                        newLoad += Clients[c].DemandDifference;
                        newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
                    }
                }
                else if (index < i && i < index + count) continue;
                else
                {
                    newLoad += Clients[current[i]].DemandDifference;
                    newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
                }
            }
            return newPenalty;
        }

        public double StrongIntraReplaceOverload(Route current, int orIndex, int deIndex)
        {
            double newLoad = TotalDelivery(current);
            double newPenalty = Math.Max(0, newLoad - current.Vehicle.Capacity);

            for (int i = 0; i < current.Count; i++)
            {
                if (i == orIndex) continue;
                if (i == deIndex)
                {
                    newLoad += Clients[current[orIndex]].DemandDifference;
                    newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
                }
                newLoad += Clients[current[i]].DemandDifference;
                newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
            }
            return newPenalty;
        }

        public double StrongIntraSwapOverload(Route current, int orIndex, int deIndex)
        {
            double newLoad = TotalDelivery(current);
            double newPenalty = Math.Max(0, newLoad - current.Vehicle.Capacity);

            for (int i = 0; i < current.Count; i++)
            {
                if (i == orIndex) newLoad += Clients[current[deIndex]].DemandDifference;
                else if (i == deIndex) newLoad += Clients[current[orIndex]].DemandDifference;
                else newLoad += Clients[current[i]].DemandDifference;
                newPenalty += Math.Max(0, newLoad - current.Vehicle.Capacity);
            }
            return newPenalty;
        }

        public override int CountUnfeasibles(RouteSet solution)
        {
            return solution.Count(r => !IsStrongFeasible(r));
        }

        public double GetLoadAfter(Route current, int cID)
        {
            double load = TotalDelivery(current);
            int previous = 0;
            for (int i = 0; i < current.Count; i++)
            {
                if (cID == previous)
                    return load;
                load += Clients[current[i]].DemandDifference;
                previous = current[i];
            }
            return load;
        }

        public double GetLoadAfterExcluding(Route current, int afterID, int excludingID)
        {
            double load = TotalDelivery(current);
            if (current.Contains(excludingID))
                load -= Clients[excludingID].Delivery;
            int previous = 0;
            for (int i = 0; i < current.Count; i++)
            {
                if (afterID == previous)
                    return load;
                if (current[i] != excludingID)
                    load += Clients[current[i]].DemandDifference;
                previous = current[i];
            }
            return load;
        }


        #endregion

        #region XML

        public static VRPSimultaneousPickupDelivery LoadFromXML(string path)
        {
            return new VRPSimultaneousPickupDelivery(path);
        }

        #endregion

        #region Parse Dethlof Problem

        public enum DethloffType { SCA, CON };

        private static Tuple<DethloffType, int, int> ParseDethloffName(string path)
        {
            Regex nameexp = new Regex(@"(?<type>\w+)(?<x>\d)(?<y>\d)");
            Match m = nameexp.Match(Path.GetFileNameWithoutExtension(path));
            DethloffType type = (m.Groups["type"].Value == "cc05") ? DethloffType.CON : DethloffType.SCA;
            return new Tuple<DethloffType, int, int>(type, int.Parse(m.Groups["x"].Value), int.Parse(m.Groups["y"].Value));
        }

        public static VRPSimultaneousPickupDelivery LoadDenthoffProblem(string path)
        {
            Tuple<DethloffType, int, int> dataName = ParseDethloffName(path);
            DethloffType type = dataName.Item1;
            int x = dataName.Item2;
            int y = dataName.Item3;

            StreamReader reader = new StreamReader(path);
            Fleet f = ParseFleetInfo(reader, x + 1);
            int clientAmount = int.Parse(reader.ReadLine().Trim());
            TravelData m = ParseTravelMatrix(reader, clientAmount);
            ClientSet<PickupDeliveryClient> c = ParseClientInfo(reader, clientAmount);
            VRPSimultaneousPickupDelivery instance = new VRPSimultaneousPickupDelivery(c, f, m);
            instance.ProblemName = type.ToString() + x.ToString() + "-" + y.ToString();
            return instance;
        }

        private static TravelData ParseTravelMatrix(StreamReader reader, int clientAmount)
        {
            double[,] matrix = new double[clientAmount+1, clientAmount+1];
            for (int i = 0; i < clientAmount+1; i++)
            {
                for (int j = 0; j < clientAmount+1; j++)
                {
                    matrix[i, j] = double.Parse(reader.ReadLine());
                }
            }
            return new TravelData(matrix);
        }

        private static ClientSet<PickupDeliveryClient> ParseClientInfo(StreamReader reader, int clientAmount)
        {
            ClientSet<PickupDeliveryClient> clients = new ClientSet<PickupDeliveryClient>();
            for (int i = 0; i < clientAmount; i++)
            {
                double delivery = double.Parse(reader.ReadLine());
                double pickup = double.Parse(reader.ReadLine());
                clients.Add(new PickupDeliveryClient(i + 1, delivery, pickup));
            }
            return clients;
        }

        private static Fleet ParseFleetInfo(StreamReader reader, int fleetsize)
        {
            double capacity = double.Parse(reader.ReadLine().Trim());
            return Fleet.CreateHomogeneousFleet(fleetsize, capacity);
        }

        #endregion

        #region Parse Salh and Nagy Problems

        public static int ParseNagySalhiName(string path)
        {
            Regex nameexp = new Regex(@"vrpnc(?<id>\d+)");
            Match m = nameexp.Match(Path.GetFileNameWithoutExtension(path));
            return int.Parse(m.Groups["id"].Value);
        }

        public static VRPSimultaneousPickupDelivery LoadSalhiNagyProblem(string path, int fleet, bool serieX)
        {
            //formato del fichero 
            //clientes capacidad tiempo-maximo tiempo-ruptura
            //depot-x depot-y
            //para cada clietne
            //x y demand

            int id = ParseNagySalhiName(path);

            StreamReader reader = new StreamReader(path);

            string[] specification = reader.ReadLine().Split(' ');
            int clientAmount = int.Parse(specification[1]);
            int vehicleCapacity = int.Parse(specification[2]);

            string[] depot = reader.ReadLine().Split(' ');
            double xDepot = double.Parse(depot[1]);
            double yDepot = double.Parse(depot[2]);

            List<Tuple<double, double, double>> clientsInfo = new List<Tuple<double, double, double>>(clientAmount + 1);
            clientsInfo.Add(new Tuple<double, double, double>(xDepot, yDepot, 0));
            while (!reader.EndOfStream)
            {
                string[] client = reader.ReadLine().Split(' ');
                if (client.Length == 4)
                    clientsInfo.Add(new Tuple<double, double, double>(double.Parse(client[1]), double.Parse(client[2]), double.Parse(client[3])));
            }

            TravelData matrix = BuildTravelCostMatrix(clientsInfo);
            ClientSet<PickupDeliveryClient> clientSet = BuildClientSet(clientsInfo, serieX);
            Fleet vehicles = Fleet.CreateHomogeneousFleet(fleet, vehicleCapacity);
            reader.Close();

            VRPSimultaneousPickupDelivery problem = new VRPSimultaneousPickupDelivery(clientSet, vehicles, matrix);
            string serie = (serieX) ? "X" : "Y";
            problem.ProblemName = string.Format("CMT{0}-{1}-{2}{3}", id, clientAmount, fleet, serie);
            return problem;
        }

        private static TravelData BuildTravelCostMatrix(List<Tuple<double, double, double>> clientInfo)
        {
            double[,] matrix = new double[clientInfo.Count, clientInfo.Count];

            for (int i = 0; i < clientInfo.Count; i++)
            {
                for (int j = i + 1; j < clientInfo.Count; j++)
                {
                    matrix[i, j] = matrix[j, i] = Math.Sqrt(Math.Pow(clientInfo[i].Item1 - clientInfo[j].Item1, 2) + Math.Pow(clientInfo[i].Item2 - clientInfo[j].Item2, 2));
                }
            }
            return new TravelData(matrix);
        }

        private static ClientSet<PickupDeliveryClient> BuildClientSet(List<Tuple<double, double, double>> clientsInfo, bool serieX)
        {
            ClientSet<PickupDeliveryClient> clients = new ClientSet<PickupDeliveryClient>();
            for (int i = 1; i < clientsInfo.Count; i++)
            {
                double rate = Math.Min(clientsInfo[i].Item1 / clientsInfo[i].Item2, clientsInfo[i].Item2 / clientsInfo[i].Item1);
                double demand1 = Math.Round(rate * clientsInfo[i].Item3,8);
                double demand2 = Math.Round((1 - rate) * clientsInfo[i].Item3,8);
                if (serieX)
                    clients.Add(new PickupDeliveryClient(i, demand1, demand2));
                else
                    clients.Add(new PickupDeliveryClient(i, demand2, demand1));
            }
            return clients;
        }

        #endregion

        #region Combine Problems

        public static VRPSimultaneousPickupDelivery Combine(VRPSimultaneousPickupDelivery problem1, VRPSimultaneousPickupDelivery problem2)
        {
            ClientSet<PickupDeliveryClient> clientSet = new ClientSet<PickupDeliveryClient>();
            foreach (var item in problem1.Clients)
                clientSet.Add(item);
            for (int i = 0; i < problem2.Clients.Count; i++)
            {
                PickupDeliveryClient newClient = (PickupDeliveryClient)problem2.Clients[i + 1].Clone();
                newClient.ID = clientSet.Count + 1;
                clientSet.Add(newClient);
            }
            Fleet fleet = new Fleet();
            foreach (var item in problem1.Vehicles)
                fleet.Add(item);
            foreach (var item in problem2.Vehicles)
                fleet.Add(item);

            double[,] travel = new double[clientSet.Count + 1, clientSet.Count + 1];
            double maxCost = int.MaxValue / 10;
            for (int i = 0; i < travel.GetLength(0); i++)
                for (int j = i + 1; j < travel.GetLength(1); j++)
                    travel[i, j] = travel[j, i] = maxCost;
            for (int i = 0; i < problem1.Clients.Count + 1; i++)
                for (int j = 0; j < problem1.Clients.Count + 1; j++)
                    travel[i, j] = problem1.TravelDistance[i, j];
            for (int i = 0; i < problem2.Clients.Count + 1; i++)
                for (int j = 0; j < problem2.Clients.Count + 1; j++)
                {
                    if (i == 0 && j != 0)
                        travel[0, problem1.Clients.Count + j] = problem2.TravelDistance[0, j];
                    else if (j == 0 && i != 0)
                        travel[problem1.Clients.Count + i, 0] = problem2.TravelDistance[i, 0];
                    else if (i != 0 && j != 0)
                        travel[problem1.Clients.Count + i, problem1.Clients.Count + j] = problem2.TravelDistance[i, j];
                }
            return new VRPSimultaneousPickupDelivery(clientSet, fleet, new TravelData(travel));
        }
        #endregion

        #region Parse Taillar HVRP Problems

        public static VRPSimultaneousPickupDelivery LoadTaillarProblems(string path)
        {
            StreamReader reader = new StreamReader(path);
            var clientInfo = LoadTaillardClients(reader);
            Fleet fleet = LoadTaillardFleet(reader);
            reader.Close();

            return new VRPSimultaneousPickupDelivery(clientInfo.Item1, fleet, clientInfo.Item2);
        }

        private static Tuple<ClientSet<PickupDeliveryClient>, TravelData> LoadTaillardClients(StreamReader reader)
        {
            int clients = int.Parse(reader.ReadLine().Trim());
            List<double> deliveryDemand = new List<double>();
            List<Tuple<double, double>> locations = new List<Tuple<double, double>>();
            Regex clientsexp = new Regex(@"(?<id>\d+)\s+(?<x>\d+)\s+(?<y>\d+)\s+(?<dem>\d+)");

            for (int i = 0; i < clients + 1; i++)
            {
                string line = reader.ReadLine().Trim();
                Match m = clientsexp.Match(line);
                deliveryDemand.Add(double.Parse(m.Groups["dem"].Value));
                locations.Add(new Tuple<double, double>(double.Parse(m.Groups["x"].Value), double.Parse(m.Groups["y"].Value)));
            }

            ClientSet<PickupDeliveryClient> clientSet = new ClientSet<PickupDeliveryClient>();
            for (int i = 1; i < clients + 1; i++)
            {
                PickupDeliveryClient c = new PickupDeliveryClient(i, deliveryDemand[i], 0);
                clientSet.Add(c);
            }

            TravelData matrix = LoadTaillarTravelMatrix(locations);

            return new Tuple<ClientSet<PickupDeliveryClient>, TravelData>(clientSet, matrix);
        }

        private static Fleet LoadTaillardFleet(StreamReader reader)
        {
            int vehicles = int.Parse(reader.ReadLine().Trim());

            Regex vehiclesexp = new Regex(@"(?<capacity>\d+(\.\d+)?)\s+(?<fixed>\d+(\.\d+)?)\s+(?<variable>\d+(\.\d+)?)\s+(?<count>\d+)");
            Fleet fleet = new Fleet();

            for (int i = 0; i < vehicles; i++)
            {
                string line = reader.ReadLine().Trim();
                Match m = vehiclesexp.Match(line);
                var capacity = double.Parse(m.Groups["capacity"].Value);
                var fixedCost = double.Parse(m.Groups["fixed"].Value);
                var varCost = double.Parse(m.Groups["variable"].Value);
                var count = int.Parse(m.Groups["count"].Value);

                fleet.Add(new VehicleType(capacity, count));
            }
            return fleet;
        }

        private static TravelData LoadTaillarTravelMatrix(List<Tuple<double, double>> locations)
        {
            double[,] matrix = new double[locations.Count, locations.Count];

            for (int i = 0; i < locations.Count; i++)
            {
                for (int j = i + 1; j < locations.Count; j++)
                {
                    matrix[i, j] = matrix[j, i] = Math.Sqrt(Math.Pow(locations[i].Item1 - locations[j].Item1, 2) + Math.Pow(locations[i].Item2 - locations[j].Item2, 2));
                }
            }
            return new TravelData(matrix);
        }
        #endregion
    }
}
