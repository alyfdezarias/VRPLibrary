using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;

using VRPLibrary.ClientData;
using VRPLibrary.FleetData;
using VRPLibrary.ProblemData;
using VRPLibrary.RouteSetData;
using VRPLibrary.SolutionStrategy.GAMS;

namespace VRPLibrary.SolutionStrategy.VRPSPD
{
    public class ColumnGenrationProcedure: PenalizationLocalSearch
    {
        public ColumnGenrationProcedure(VRPSimultaneousPickupDelivery problemData) : base(problemData)
        { }

        public double LowerBound(string folderPath, RouteSet solution, int maxESPPRCTime)
        {
            return LowerBound(folderPath, solution, ProblemData.Vehicles[0].Capacity, maxESPPRCTime);
        }

        public double LowerBound(string folderPath, RouteSet solution, double capacity, int maxESPPRCTime)
        {
            double[] dualVars = GetDualVariables(solution);
            double[,] reducedEdges = GetReducedCostEdges(dualVars);
            List<Route> negativeCostRoutes = ElementaryShortestPathResourceConstraints(reducedEdges, capacity, maxESPPRCTime);
            List<Route> newFounded = NewRouteFounded(solution, negativeCostRoutes);
            List<ExtRouteInfo> currentPool = BuildCoveringPool(solution, newFounded);
            List<Tuple<Route, double>> coveringSelection = SolveRestrictedMasterProblem(folderPath, "coveringPool", currentPool);
            return coveringSelection.Sum(r => GetCost(r.Item1) * r.Item2);
        }

        protected List<Tuple<Route, double>> SolveRestrictedMasterProblem(string folderPath, string coveringPath, List<ExtRouteInfo> currentPool)
        {
            ColumnSelectionGAMS gamsProcedure = new ColumnSelectionGAMS(ProblemData);
            DirectoryInfo coveringDir = Directory.CreateDirectory(folderPath).CreateSubdirectory(coveringPath);
            List<Tuple<Route, double>> coveringSelection = gamsProcedure.Solve(coveringDir.FullName, currentPool);
            for (int i = 0; i < coveringSelection.Count; i++)
            {
                coveringSelection[i].Item1.ToXMLFormat().Save(Path.Combine(coveringDir.FullName, string.Format("r={0} s={1}.xml", i, coveringSelection[i].Item2)));
            }

            return coveringSelection;
        }

        public List<ExtRouteInfo> BuildCoveringPool(RouteSet solution, List<Route> negCost)
        {
            List<ExtRouteInfo> pool = new List<ExtRouteInfo>();
            foreach (var item in solution)
                pool.Add(new ExtRouteInfo(item, GetCost(item)));
            foreach (var item in negCost)
                pool.Add(new ExtRouteInfo(item, GetCost(item)));
            return RemovePoolRedundancy(pool);
        }


        public List<Route> NewRouteFounded(RouteSet solution, List<Route> candidates)
        {
            List<Route> newRoutes = new List<Route>();
            foreach (var item in candidates)
            {
                if (!solution.ContainsRoute(item))
                    newRoutes.Add(item);
            }
            return newRoutes;
        }

        protected List<ExtRouteInfo> RemovePoolRedundancy(List<ExtRouteInfo> currentPool)
        {
            bool[] dominates = new bool[currentPool.Count];
            for (int i = 0; i < currentPool.Count; i++)
            {
                if (dominates[i]) continue;
                for (int j = 0; j < currentPool.Count; j++)
                {
                    if (i == j || dominates[j]) continue;
                    if (currentPool[i].Dominates(currentPool[j]))
                        dominates[j] = true ;
                }
            }
            List<ExtRouteInfo> undominates = new List<ExtRouteInfo>();
            for (int i = 0; i < currentPool.Count; i++)
            {
                if (!dominates[i])
                    undominates.Add(currentPool[i]);
            }
            return undominates;
        }

        public List<Tuple<int, double>> GetDualVariables(string folderPath, RouteSet solution)
        {
            /*Solo permite obtener soluciones extremales del dual que son muy malas para la generacion de columnas*/
            ColumnSelectionGAMS gamsProcedure = new ColumnSelectionGAMS(ProblemData);
            return gamsProcedure.DualSolve(folderPath, GetExtRouteInfo(solution));
        }

        public double[] GetDualVariables(RouteSet solution)
        {
            /*
            *lamda_0 = 0 variable dual no positiva asociada a la cantidad de vehiculos
            *lamda_i = R_j / |R_j| forall i \in R_j variable dual asociada a los clientes
            */
            double[] dualVars = new double[ProblemData.Clients.Count+1];
            foreach (var route in solution)
            {
                double cost = GetTravelCost(route);
                foreach (var c in route)
                    dualVars[c] = cost / route.Count;
            }
            return dualVars;
        }

        private List<ExtRouteInfo> GetExtRouteInfo(RouteSet solution)
        {
            List<ExtRouteInfo> info = new List<ExtRouteInfo>();
            foreach (var r in solution)
            {
                info.Add(new ExtRouteInfo(r, GetTravelCost(r)));
            }
            return info;
        }

        public double[,] GetReducedCostEdges(List<Tuple<int, double>> dualVars)
        {
            double[,] newCost = (double[,])ProblemData.TravelDistance.Clone();
            foreach (var v in dualVars)
            {
                for (int j = 0; j < newCost.GetLength(0); j++)
                {
                    if (v.Item1 == j) continue;
                    newCost[v.Item1, j] -= v.Item2;
                }
            }
            return newCost;
        }

        public double[,] GetReducedCostEdges(double[] dualVars)
        {
            double[,] newCost = (double[,])ProblemData.TravelDistance.Clone();
            for (int i = 0; i < newCost.GetLength(0); i++)
            {
                for (int j = 0; j < newCost.GetLength(1); j++)
                {
                    if (i == j) continue;
                    newCost[i, j] -= dualVars[i];
                }
            }
            return newCost;
        }

        public List<Route> ElementaryShortestPathResourceConstraints(double[,] reducedCost, double capacity, int maxESPRCTime)
        {
            LabelStorage storage = new LabelStorage(ProblemData.Clients.Count, capacity);
            Queue<int> nodesToBeAnalized = new Queue<int>();
            nodesToBeAnalized.Enqueue(0);

            List<LabelInfo> newLabels;

            Stopwatch timer = new Stopwatch();
            timer.Start();

            while (nodesToBeAnalized.Count > 0 && maxESPRCTime > timer.Elapsed.TotalSeconds)
            {
                int currentNode = nodesToBeAnalized.Dequeue();
                for (int i = 1; i <= ProblemData.Clients.Count; i++)
                {
                    if (currentNode == i) continue;
                    newLabels = new List<LabelInfo>();
                    foreach (var lb in storage[currentNode])
                    {
                        if (!lb.UnreachableNodes[i]) /*el nodo i es alcanzable desde el actual*/
                        {
                            LabelInfo ext = ExtendStrongFeasible(reducedCost, lb, currentNode, i, capacity);
                            if(ext!= null) 
                                newLabels.Add(ext);
                        }
                    }
                    if (storage.UpdateLabels(newLabels,i))/*diferenciar los dos depositos: origen vs destino*/
                    {
                        if (!nodesToBeAnalized.Contains(i))
                            nodesToBeAnalized.Enqueue(i);
                    }
                }
                if(nodesToBeAnalized.Count == 0)
                    Console.WriteLine();
            }
            timer.Stop();
            return storage.GetNegativeReduceCostRoute(reducedCost);
                        
        }

        private LabelInfo ExtendStrongFeasible(double[,] reducedCost, LabelInfo currentLabel, int currentNode, int clientID, double capacity) /*extiende el label actual con el nodo clientID*/
        {
            CapacityResourceInfo resourceExt = GetCapacityExtension(currentLabel.CapacityInfo, clientID, capacity);
            if (resourceExt == null)
                return null; /*No es posible realizar una extension strong feasible*/
            bool[] unreachablesExt = UpdateUnreachablesNodes(currentLabel.UnreachableNodes, clientID, resourceExt, capacity);
            Route partial = (Route)currentLabel.PartialRoute.Clone();
            if (clientID != 0) partial.Add(clientID);
            return new LabelInfo(resourceExt, unreachablesExt, unreachablesExt.Count(x => x == true), currentLabel.Cost + reducedCost[currentNode, clientID], partial);
        }

        private bool[] UpdateUnreachablesNodes(bool[] currentUnreachables, int clientID, CapacityResourceInfo newResourceInfo, double capacity)
        {
            bool[] newUnreachables = new bool[currentUnreachables.Length];
            for (int i = 0; i < newUnreachables.Length; i++)
            {
                if (i == clientID)
                    newUnreachables[i] = true;
                else if (!currentUnreachables[i])
                    newUnreachables[i] = IsUnreachable(newResourceInfo, i, capacity);
                else newUnreachables[i] = true;

            }
            return newUnreachables;

        }

        private bool IsUnreachable(CapacityResourceInfo currentResourceInfo, int id, double capacity)
        {
            if (id == 0) return false;
            /*Factibilidad Debil*/
            if (!IsWeakExtension(currentResourceInfo, id, capacity)) return true;
            /*Factibilidad Fuerte*/
            CapacityResourceInfo strongExt = GetCapacityExtension(currentResourceInfo, id, capacity);
            return strongExt == null; 
        }

        private bool IsWeakExtension(CapacityResourceInfo currentResourceInfo, int id, double capacity)
        {
            return currentResourceInfo.CurrentDelivery + ProblemData.Clients[id].Delivery <= capacity &&
                currentResourceInfo.CurrentPickup + ProblemData.Clients[id].Pickup <= capacity;
          
        }

        private CapacityResourceInfo GetCapacityExtension(CapacityResourceInfo currentResourceInfo, int clientID, double capacity)
        {
            if (IsWeakExtension(currentResourceInfo, clientID, capacity))/*en teoria esto siempre deberia ser true pq el nodo clientID no esta marcado como inalcanzable*/
            {
                double newDelivery = currentResourceInfo.CurrentDelivery + ProblemData.Clients[clientID].Delivery;

                double newPickup = currentResourceInfo.CurrentPickup + ProblemData.Clients[clientID].Pickup;
                double newLoad = currentResourceInfo.MaxDelivery - newDelivery + newPickup;

                if (newDelivery <= currentResourceInfo.MaxDelivery)/*la carga inicial permite satisfacer la demanda de clientID*/
                {
                    if (newLoad <= capacity)/*el camino extendido hasta clientID es strong feasible*/
                        return new CapacityResourceInfo(currentResourceInfo.MaxDelivery, newDelivery, newPickup);

                    double deltaLoad = newLoad - capacity; /*cto hay q disminour la carga inicial para garantizar factibilidad en clientID*/
                    if (currentResourceInfo.MaxDelivery - deltaLoad >= newDelivery) /*la reduccion en la carga inicial es factible*/
                        return new CapacityResourceInfo(currentResourceInfo.MaxDelivery - deltaLoad, newDelivery, newPickup);
                    return null;
                }
                return null;
            }
            return null;
        }
    }


    public class CapacityResourceInfo
    {
        public double MaxDelivery { get; set; }
        public double CurrentDelivery { get; set; }
        public double CurrentPickup { get; set; }

        public CapacityResourceInfo(double maxDelivery, double delivery, double pickup)
        {
            MaxDelivery = maxDelivery;
            CurrentDelivery = delivery;
            CurrentPickup = pickup;
        }

        public double GetCurrentLoad
        {
            get { return MaxDelivery - CurrentDelivery + CurrentPickup; }
        }

        public override string ToString()
        {
            return $"M={MaxDelivery} D={CurrentDelivery} P={CurrentPickup}";
        }

        public override bool Equals(object obj)
        {
            var other = obj as CapacityResourceInfo;
            return MaxDelivery == other.MaxDelivery &&
                CurrentDelivery == other.CurrentDelivery &&
                CurrentPickup == other.CurrentPickup;
        }
        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    public class LabelInfo
    {
        public CapacityResourceInfo CapacityInfo { get; set; }
        public int UnreachableNodesAmount { get; set; }
        public bool[] UnreachableNodes { get; set; }
        public double Cost { get; set; }
        public Route PartialRoute { get; set; }

        public LabelInfo(CapacityResourceInfo capacity, bool[] unreachables, int unreachablesAmount, double cost, Route partial)
        {
            CapacityInfo = capacity;
            UnreachableNodes = unreachables;
            UnreachableNodesAmount = unreachablesAmount;
            Cost = cost;
            PartialRoute = (Route)partial.Clone();
        }

        public bool Dominates(LabelInfo other)
        {
            /*dominance roule for strong feasibility*/
            return Cost <= other.Cost
                && UnreachableNodesAmount <= UnreachableNodesAmount
                && CapacityInfo.MaxDelivery >= other.CapacityInfo.MaxDelivery /*este es mas eficiente de verificar que la siguiente condicion*/
                && DominantesUnreachablesNode(other.UnreachableNodes);
        }

        private bool DominantesUnreachablesNode(bool[] unreachable)
        {
            for (int i = 0; i < UnreachableNodes.Length; i++)
                if (UnreachableNodes[i] && !unreachable[i])return false;
            return true;
        }

        public override string ToString()
        {
            return $"C={Cost} U={UnreachableNodesAmount} {CapacityInfo.ToString()}";
        }

        public override bool Equals(object obj)
        {
            var otherLb = obj as LabelInfo;
            return Cost == otherLb.Cost &&
                UnreachableNodesAmount == otherLb.UnreachableNodesAmount &&
                CapacityInfo.Equals(otherLb.CapacityInfo) &&
                IsCoincideUnreachableVector(otherLb.UnreachableNodes);
                
        }

        private bool IsCoincideUnreachableVector(bool[] other)
        {
            for (int i = 0; i < UnreachableNodes.Length; i++)
            {
                if (UnreachableNodes[i] != other[i]) return false;
            }
            return true;
        }

        public override int GetHashCode()
        {
            return base.GetHashCode();
        }
    }

    public class LabelStorage
    {
        public Dictionary<int, List<LabelInfo>> Storage { get; set; }

        public LabelStorage(int clientsAmount, double capacity)
        {
            Storage = new Dictionary<int, List<LabelInfo>>(clientsAmount + 1);
            for (int i = 0; i < clientsAmount+1; i++)
            {
                Storage.Add(i, new List<LabelInfo>());
            }
            Storage[0].Add(new LabelInfo(new CapacityResourceInfo(capacity, 0, 0), new bool[clientsAmount + 1], 0, 0, new Route(new VehicleType(capacity))));
        }

        public List<LabelInfo> this[int id]
        {
            get { return Storage[id]; }
        }

        public bool UpdateLabels(List<LabelInfo> newLabels, int id)
        {
            bool[] dominates = GetDominates(newLabels, id);
            bool modification = false;
            Stack<int> toRemove = new Stack<int>();
            int lbAmount = Storage[id].Count;
            for (int i = 0; i < dominates.Length; i++)
            {
                if (i < lbAmount && dominates[i])/*old label q esta dominado => hay q eliminarlo*/
                {
                    toRemove.Push(i);
                    modification = true;
                }
                if (i >= lbAmount && !dominates[i]) /*new label q no esta dominado => hay q add*/
                {
                    Storage[id].Add(newLabels[i - lbAmount]);
                    modification = true;
                }
            }
            var rm = new List<LabelInfo>();
            while (toRemove.Count > 0)
            {
                int index = toRemove.Pop();
                rm.Add(Storage[id][index]);
                Storage[id].RemoveAt(index);
            }
            if (modification)
                return modification;
            return modification;
        }

        private bool[] GetDominates(List<LabelInfo> newLabels, int id)
        {
            List<LabelInfo> tmpList = new List<LabelInfo>(Storage[id]);
            tmpList.AddRange(newLabels);
            bool[] dominates = new bool[tmpList.Count];

            for (int i = 0; i < tmpList.Count; i++)
            {
                if (dominates[i]) continue;
                for (int j = 0; j < tmpList.Count; j++)
                {
                    if (i == j) continue;
                    if (!dominates[j] && tmpList[i].Dominates(tmpList[j]))
                        dominates[j] = true;
                }
            }

            return dominates;
        }

        public List<Route> GetNegativeReduceCostRoute(double[,] reducedCost)
        {
            List<Route> negative = new List<Route>();
            foreach (var item in Storage)
                foreach (var lb in item.Value)
                    if (lb.PartialRoute.Count > 0 && lb.Cost + reducedCost[lb.PartialRoute[lb.PartialRoute.Count - 1], 0] < 0)
                        negative.Add(lb.PartialRoute);
            return negative;
        }
    }
}
