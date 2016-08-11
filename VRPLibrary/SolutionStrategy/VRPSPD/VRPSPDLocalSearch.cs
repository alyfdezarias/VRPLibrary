using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using VRPLibrary.ClientData;
using VRPLibrary.FleetData;
using VRPLibrary.ProblemData;
using VRPLibrary.RouteSetData;
using MetaheuristicsLibrary;

namespace VRPLibrary.SolutionStrategy.VRPSPD
{
    public abstract class VRPSPDLocalSearch : IterationNotifier<RouteSet>
    {
        public double OverloadFactor { get; set; }
        public VRPSimultaneousPickupDelivery ProblemData { get; set; }
        public event IterationEventHandler<RouteSet> IterationInfo;

        protected VRPSPDLocalSearch(VRPSimultaneousPickupDelivery problemData)
        {
            ProblemData = problemData;
            OverloadFactor = 0;
        }

        protected void OnIteration(IterationEventArgs<RouteSet> args)
        {
            if (IterationInfo != null)
                IterationInfo(this, args);
        }

        public virtual double GetTravelCost(RouteSet solution)
        {
            return solution.Sum(r => GetTravelCost(r));
        }

        public virtual double GetTravelCost(Route current)
        {
            int previous = 0;
            double cost = 0;
            for (int i = 0; i < current.Count; i++)
            {
                cost += ProblemData.TravelDistance[previous, current[i]];
                previous = current[i];
            }
            cost += ProblemData.TravelDistance[previous, 0];
            return cost;
        }

        public virtual double GetCost(RouteSet solution)
        {
            if (OverloadFactor == 0)
                return GetTravelCost(solution);
            return GetTravelCost(solution) + OverloadFactor * ProblemData.StrongOverLoad(solution);
        }

        public virtual double GetCost(Route r)
        {
            if (OverloadFactor == 0) return GetTravelCost(r);
            return GetTravelCost(r) + OverloadFactor * ProblemData.StrongOverLoad(r);

        }

        #region Greedy Randomized Solution

        public RouteSet BuildGreedyRandomizedSolution(Random rdObj, double alpha)
        {
            List<PickupDeliveryClient> availableClients = new List<PickupDeliveryClient>(from c in ProblemData.Clients
                                                                                         select c);
            RouteSetRCLProgress greedyRd = new RouteSetRCLProgress(ProblemData, RouteSet.BuildEmptyRouteSet(ProblemData.Vehicles), 0, 0);
            while (availableClients.Count > 0)
            {
                List<ClientRCL> rcl = BuildRCL(greedyRd, availableClients, alpha);
                int rdSelection = rdObj.Next(rcl.Count);
                ClientRCL selectedClient = rcl[rdSelection];
                if (selectedClient.InsertAtNewRoute && availableClients.Count < ProblemData.Clients.Count) /*la verificacion se hace para el caso base*/
                {
                    greedyRd.CloseRoute();
                    if (!greedyRd.HasAvailablesRoutes)
                        greedyRd.IncreasseRoutes();
                }
                greedyRd.GetLoadingRoute.Add(selectedClient.ClientInfo.ID);
                RemoveClientFromAvailableList(availableClients, selectedClient.ClientInfo.ID);
                greedyRd.LoadingDelivery += selectedClient.ClientInfo.Delivery;
                greedyRd.LoadinglPickup += selectedClient.ClientInfo.Pickup;
            }
            greedyRd.PartialSolution.CleanUnusedRoutes();
            return greedyRd.PartialSolution;
        }

        private void RemoveClientFromAvailableList(List<PickupDeliveryClient> availableCliens, int clientID)
        {
            availableCliens.Remove(ProblemData.Clients[clientID]);
        }

        protected List<ClientRCL> BuildRCL(RouteSetRCLProgress partialSolution, List<PickupDeliveryClient> availableClients, double alpha)
        {
            Route loadingRoute = partialSolution.GetLoadingRoute;
            List<ClientRCL> allNotEmptyInsertions = new List<ClientRCL>();
            List<ClientRCL> allEmptyInsertions = new List<ClientRCL>();
            foreach (var c in availableClients)
            {
                int referenceClient = partialSolution.GetReferenceClientID(loadingRoute);
                if (referenceClient != 0 && CanInsertWeakFeasible(c, loadingRoute.Vehicle.Capacity, partialSolution.LoadinglPickup, partialSolution.LoadingDelivery))
                    allNotEmptyInsertions.Add(new ClientRCL(c, DeltaCostNotEmptyRoute(c, referenceClient), false));
                if (CanInsertWeakFeasible(c, partialSolution.GetNextEmptyRouteCapacity, 0, 0))
                    allEmptyInsertions.Add(new ClientRCL(c, DeltaCostEmptyRoute(c), true));
            }

            List<ClientRCL> availableRCL = (allNotEmptyInsertions.Count != 0) ? allNotEmptyInsertions : allEmptyInsertions;

            double minCost = availableRCL.Min(x => x.DeltaCost);
            double maxCost = availableRCL.Max(x => x.DeltaCost);

            List<ClientRCL> rcl = new List<ClientRCL>();
            return new List<ClientRCL>(from x in availableRCL
                                       where x.DeltaCost >= minCost && x.DeltaCost <= minCost + alpha * (maxCost - minCost)
                                       select x);

        }

        protected bool CanInsertWeakFeasible(PickupDeliveryClient c, double vehicleCapacity, double loadingPickup, double loadingDelivery)
        {
            return c.Delivery + loadingDelivery <= vehicleCapacity && c.Pickup + loadingPickup <= vehicleCapacity;
        }

        protected double DeltaCostNotEmptyRoute(PickupDeliveryClient c, int referenceClient)
        {
            double removed = ProblemData.TravelDistance[referenceClient, 0];
            double added = ProblemData.TravelDistance[referenceClient, c.ID] + ProblemData.TravelDistance[c.ID, 0];
            return added - removed;
        }

        protected double DeltaCostEmptyRoute(PickupDeliveryClient c)
        {
            return ProblemData.TravelDistance[0, c.ID] + ProblemData.TravelDistance[c.ID, 0];
        }

        #endregion

        #region Neighborhoods

        protected virtual RouteSet Neighborhood<TMove>(RouteSet solution, Exploration expCondition, Random rdObj,
                                                Func<RouteSet, double> cost,
                                                Func<RouteSet, Func<TMove, bool>, Func<TMove, double>, IEnumerable<TMove>> movements,
                                                Func<TMove, bool> isAllowedMovement,
                                                Func<TMove, double> deltaCost) where TMove : Move
        {
            RouteSet newSolution = (RouteSet)solution.Clone();
            if (newSolution.IsEmpty) return newSolution;
            Move bestMove = null;

            double currentCost = cost(newSolution);
            double bestCost = currentCost;

            foreach (var item in movements(newSolution, isAllowedMovement, deltaCost))
            {
                if (expCondition == Exploration.TotalRandom && rdObj.NextDouble() > 0.5)
                {
                    bestMove = item;
                    break;
                }
                if (currentCost + item.deltaCost < bestCost)
                {
                    bestCost = currentCost + item.deltaCost;
                    bestMove = item;
                    if (expCondition == Exploration.Random && rdObj.NextDouble() > 0.5)
                        break;
                    if (expCondition == Exploration.FirstImprovement)
                        break;
                }
            }

            if (bestMove != null)
                bestMove.Do();
            return newSolution;
        }

        #region Intra Route
        protected IEnumerable<IntraMove> ClientIntraMove(Route current, int orIndex,
                                                        Func<IntraMove, bool> isAllowedMovement,
                                                        Func<IntraMove, double> deltaCost)
        {
            for (int i = 0; i < current.Count + 1; i++)
            {
                if (i == orIndex) continue;
                IntraMove m = new IntraMove(current, orIndex, i, 0);
                if (isAllowedMovement(m))
                {
                    m.deltaCost = deltaCost(m);
                    yield return m;
                }
            }
        }

        protected IEnumerable<IntraMove> ClientIntraMove(RouteSet newSolution, Func<IntraMove, bool> isAllowedMovement, Func<IntraMove, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count; j++)
                    foreach (var item in ClientIntraMove(newSolution[i], j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet IntraMove(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<IntraMove>(solution, expCondition, rdObj,
                                GetCost, ClientIntraMove, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<IntraSwap> ClientIntraSwap(Route current, int orIndex, Func<IntraSwap, bool> isAllowedMovement, Func<IntraSwap, double> deltaCost)
        {
            for (int i = 0; i < current.Count; i++)
            {
                if (i == orIndex) continue;
                IntraSwap m = new IntraSwap(current, orIndex, i, 0);
                if (isAllowedMovement(m))
                {
                    m.deltaCost = deltaCost(m);
                    yield return m;
                }
            }
        }

        protected IEnumerable<IntraSwap> ClientIntraSwap(RouteSet newSolution, Func<IntraSwap, bool> isAllowedMovement, Func<IntraSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count; j++)
                    foreach (var item in ClientIntraSwap(newSolution[i], j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet IntraSwap(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<IntraSwap>(solution, expCondition, rdObj,
                                GetCost, ClientIntraSwap, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<TwoOpt> Client2Opt(Route current, int orIndex, Func<TwoOpt, bool> isAllowedMovement,
                                                    Func<TwoOpt, double> deltaCost)
        {
            for (int i = orIndex + 1; i < current.Count; i++)
            {
                TwoOpt m = new TwoOpt(current, orIndex, i, 0);
                if (isAllowedMovement(m))
                {
                    m.deltaCost = deltaCost(m);
                    yield return m;
                }
            }
        }

        protected IEnumerable<TwoOpt> Client2Opt(RouteSet newSolution, Func<TwoOpt, bool> isAllowedMovement, Func<TwoOpt, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count - 1; j++)
                    foreach (var item in Client2Opt(newSolution[i], j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet TwoOpt(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<TwoOpt>(solution, expCondition, rdObj,
                                GetCost, Client2Opt, IsAllowedMovement, DeltaCost);
        }

        #endregion

        #region Inter Route
        protected IEnumerable<InterMove> ClientInterMove(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                          Func<InterMove, bool> isAllowedMovement,
                                                          Func<InterMove, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                for (int j = 0; j < newSolution[i].Count; j++)
                {
                    InterMove m = new InterMove(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                    {
                        m.deltaCost = deltaCost(m);
                        yield return m;
                    }
                }
            }
        }

        protected virtual IEnumerable<InterMove> ClientInterMove(RouteSet newSolution, Func<InterMove, bool> isAllowedMovement, Func<InterMove, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count; j++)
                    foreach (var item in ClientInterMove(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet InterMove(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<InterMove>(solution, expCondition, rdObj,
                                GetCost, ClientInterMove, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<TwoInterMove> ClientTwoInterMove(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                          Func<TwoInterMove, bool> isAllowedMovement,
                                                          Func<TwoInterMove, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                for (int j = 0; j < newSolution[i].Count; j++)
                {
                    TwoInterMove m = new TwoInterMove(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                    {
                        m.deltaCost = deltaCost(m);
                        yield return m;
                    }
                }
            }
        }

        protected virtual IEnumerable<TwoInterMove> ClientTwoInterMove(RouteSet newSolution, Func<TwoInterMove, bool> isAllowedMovement, Func<TwoInterMove, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (newSolution[i].Count < 2) continue;
                for (int j = 0; j < newSolution[i].Count - 1; j++)
                    foreach (var item in ClientTwoInterMove(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
            }
        }

        public virtual RouteSet TwoInterMove(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<TwoInterMove>(solution, expCondition, rdObj,
                                GetCost, ClientTwoInterMove, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<InterSwap> ClientInterSwap(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                        Func<InterSwap, bool> isAllowedMovement,
                                                        Func<InterSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                if (newSolution[i].Count < 1) continue;
                for (int j = 0; j < newSolution[i].Count; j++)
                {
                    InterSwap m = new InterSwap(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                    {
                        m.deltaCost = deltaCost(m);
                        yield return m;
                    }
                }
            }
        }

        protected virtual IEnumerable<InterSwap> ClientInterSwap(RouteSet newSolution, Func<InterSwap, bool> isAllowedMovement, Func<InterSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count; j++)
                    foreach (var item in ClientInterSwap(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet InterSwap(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<InterSwap>(solution, expCondition, rdObj,
                                GetCost, ClientInterSwap, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<TwoOneInterSwap> ClientTwoOneInterSwap(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                        Func<TwoOneInterSwap, bool> isAllowedMovement,
                                                        Func<TwoOneInterSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                if (newSolution[i].Count < 1) continue;
                for (int j = 0; j < newSolution[i].Count; j++)
                {
                    TwoOneInterSwap m = new TwoOneInterSwap(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                    {
                        m.deltaCost = deltaCost(m);
                        yield return m;
                    }
                }
            }
        }

        protected virtual IEnumerable<TwoOneInterSwap> ClientTwoOneInterSwap(RouteSet newSolution, Func<TwoOneInterSwap, bool> isAllowedMovement, Func<TwoOneInterSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count - 1; j++)
                    foreach (var item in ClientTwoOneInterSwap(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet TwoOneInterSwap(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<TwoOneInterSwap>(solution, expCondition, rdObj,
                                GetCost, ClientTwoOneInterSwap, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<TwoTwoInterSwap> ClientTwoTwoInterSwap(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                        Func<TwoTwoInterSwap, bool> isAllowedMovement,
                                                        Func<TwoTwoInterSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                if (newSolution[i].Count < 2) continue;
                for (int j = 0; j < newSolution[i].Count-1; j++)
                {
                    TwoTwoInterSwap m = new TwoTwoInterSwap(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                    {
                        m.deltaCost = deltaCost(m);
                        yield return m;
                    }
                }
            }
        }

        protected virtual IEnumerable<TwoTwoInterSwap> ClientTwoTwoInterSwap(RouteSet newSolution, Func<TwoTwoInterSwap, bool> isAllowedMovement, Func<TwoTwoInterSwap, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count - 1; j++)
                    foreach (var item in ClientTwoTwoInterSwap(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet TwoTwoInterSwap(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<TwoTwoInterSwap>(solution, expCondition, rdObj,
                                GetCost, ClientTwoTwoInterSwap, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<CrossoverRoute> ClientCrossover(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                         Func<CrossoverRoute, bool> isAllowedMovement,
                                                         Func<CrossoverRoute, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                if (newSolution[i].IsEmpty)
                {
                    CrossoverRoute m = new CrossoverRoute(current, newSolution[i], orIndex, -1, 0);
                    m.deltaCost = deltaCost(m);
                    yield return m;
                }
                for (int j = 0; j < newSolution[i].Count; j++)
                {
                    CrossoverRoute m = new CrossoverRoute(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                        m.deltaCost = deltaCost(m);
                    yield return m;
                }
            }
        }

        protected virtual IEnumerable<CrossoverRoute> ClientCrossover(RouteSet newSolution, Func<CrossoverRoute, bool> isAllowedMovement, Func<CrossoverRoute, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count; j++)
                    foreach (var item in ClientCrossover(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet CrossoverRoute(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<CrossoverRoute>(solution, expCondition, rdObj,
                                GetCost, ClientCrossover, IsAllowedMovement, DeltaCost);
        }

        protected IEnumerable<ReverseCrossoverRoute> ClientReverseCrossover(RouteSet newSolution, Route current, int routeIndex, int orIndex,
                                                                       Func<ReverseCrossoverRoute, bool> isAllowedMovement,
                                                                       Func<ReverseCrossoverRoute, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
            {
                if (routeIndex == i) continue;
                if (newSolution[i].IsEmpty)
                {
                    ReverseCrossoverRoute m = new ReverseCrossoverRoute(current, newSolution[i], orIndex, -1, 0);
                    m.deltaCost = deltaCost(m);
                    yield return m;
                }
                for (int j = 0; j < newSolution[i].Count; j++)
                {
                    ReverseCrossoverRoute m = new ReverseCrossoverRoute(current, newSolution[i], orIndex, j, 0);
                    if (isAllowedMovement(m))
                    {
                        m.deltaCost = deltaCost(m);
                        yield return m;
                    }
                }
            }
        }

        protected virtual IEnumerable<ReverseCrossoverRoute> ClientReverseCrossover(RouteSet newSolution, Func<ReverseCrossoverRoute, bool> isAllowedMovement, Func<ReverseCrossoverRoute, double> deltaCost)
        {
            for (int i = 0; i < newSolution.Count; i++)
                for (int j = 0; j < newSolution[i].Count; j++)
                    foreach (var item in ClientReverseCrossover(newSolution, newSolution[i], i, j, isAllowedMovement, deltaCost))
                        yield return item;
        }

        public virtual RouteSet ReverseCrossoverRoute(RouteSet solution, Exploration expCondition, Random rdObj)
        {
            return Neighborhood<ReverseCrossoverRoute>(solution, expCondition, rdObj,
                                GetCost, ClientReverseCrossover, IsAllowedMovement, DeltaCost);
        }

        #endregion

        #region Custom Neighborhoods

        public List<Func<RouteSet, Exploration, Random, RouteSet>> GetAllNeighborhoods()
        {
            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods = new List<Func<RouteSet, Exploration, Random, RouteSet>>();
            neighborhoods.Add(IntraMove);
            neighborhoods.Add(IntraSwap);
            neighborhoods.Add(TwoOpt);
            neighborhoods.Add(InterMove);
            neighborhoods.Add(TwoInterMove);
            neighborhoods.Add(InterSwap);
            neighborhoods.Add(TwoOneInterSwap);
            neighborhoods.Add(TwoTwoInterSwap);
            neighborhoods.Add(CrossoverRoute);
            //neighborhoods.Add(ReverseCrossoverRoute);
            return neighborhoods;
        }

        public List<Func<RouteSet, Exploration, Random, RouteSet>> GetIntraRouteNeighborhoods()
        {
            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods = new List<Func<RouteSet, Exploration, Random, RouteSet>>();
            neighborhoods.Add(IntraMove);
            neighborhoods.Add(IntraSwap);
            neighborhoods.Add(TwoOpt);
            return neighborhoods;
        }

        public List<Func<RouteSet, Exploration, Random, RouteSet>> GetInterRouteNeighborhoods()
        {
            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods = new List<Func<RouteSet, Exploration, Random, RouteSet>>();
            neighborhoods.Add(InterMove);
            neighborhoods.Add(TwoInterMove);
            neighborhoods.Add(InterSwap);
            neighborhoods.Add(TwoOneInterSwap);
            neighborhoods.Add(TwoTwoInterSwap);
            neighborhoods.Add(CrossoverRoute);
            //neighborhoods.Add(ReverseCrossoverRoute);
            return neighborhoods;
        }

        #endregion


        #endregion

        #region Allowed Movements
        public virtual bool IsAllowedMovement(IntraMove m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(IntraSwap m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(TwoOpt m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(InterMove m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(TwoInterMove m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(InterSwap m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(TwoOneInterSwap m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(TwoTwoInterSwap m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(CrossoverRoute m)
        {
            return true;
        }
        public virtual bool IsAllowedMovement(ReverseCrossoverRoute m)
        {
            return true;
        }
        #endregion

        #region Delta Travel Cost
        public virtual double DeltaTravelCost(IntraMove m)
        {
            if (m.orIndex == m.deIndex) return 0;
            return RemoveDeltaTravelCost(m.current, m.orIndex,1) + AddInTheSameDeltaTravelCost(m.current, m.orIndex, m.deIndex);
        }

        protected virtual double RemoveDeltaTravelCost(Route orRoute, int orIndex, int count)
        {
            int prev = (orIndex == 0) ? 0 : orRoute[orIndex - 1];
            int next = (orIndex + count < orRoute.Count) ? orRoute[orIndex + count] : 0;
            double remove = ProblemData.TravelDistance[prev, orRoute[orIndex]] + ProblemData.TravelDistance[orRoute[orIndex + count - 1], next];
            double add = ProblemData.TravelDistance[prev, next];
            return add - remove;
        }

        protected virtual double AddInTheSameDeltaTravelCost(Route deRoute, int clientIndex, int deIndex)
        {
            int client = deRoute[clientIndex];
            int shiftLeft = (clientIndex + 1 == deIndex) ? -2 : -1;
            int prev = (deIndex + shiftLeft) >= 0 ? deRoute[deIndex + shiftLeft] : 0;
            int next = (deIndex < deRoute.Count) ? deRoute[deIndex] : 0;
            double remove = ProblemData.TravelDistance[prev, next];
            double add = ProblemData.TravelDistance[prev, client] + ProblemData.TravelDistance[client, next];
            return add - remove;
        }

        public virtual double DeltaTravelCost(IntraSwap m)
        {
            int min = Math.Min(m.orIndex, m.deIndex);
            int max = Math.Max(m.orIndex, m.deIndex);
            m.orIndex = min;
            m.deIndex = max;

            double remove = 0, add = 0;
            if (m.orIndex + 1 == m.deIndex)
            {
                int prev = (m.orIndex == 0) ? 0 : m.current[m.orIndex - 1];
                int next = (m.deIndex == m.current.Count - 1) ? 0 : m.current[m.deIndex + 1];

                remove = ProblemData.TravelDistance[prev, m.current[m.orIndex]] + ProblemData.TravelDistance[m.current[m.orIndex], m.current[m.deIndex]] + ProblemData.TravelDistance[m.current[m.deIndex], next];
                add = ProblemData.TravelDistance[prev, m.current[m.deIndex]] + ProblemData.TravelDistance[m.current[m.deIndex], m.current[m.orIndex]] + ProblemData.TravelDistance[m.current[m.orIndex], next];
            }
            else
            {
                remove = NotSequentialIntraSwapPartialRemove(m.current, m.orIndex) + NotSequentialIntraSwapPartialRemove(m.current, m.deIndex);
                add = NotSequentialIntraSwapPartialAdd(m.current, m.orIndex, m.current[m.deIndex]) + NotSequentialIntraSwapPartialAdd(m.current, m.deIndex, m.current[m.orIndex]);
            }
            return add - remove;
        }

        protected virtual double NotSequentialIntraSwapPartialRemove(Route current, int index)
        {
            int prev = (index == 0) ? 0 : current[index - 1];
            int next = (index == current.Count - 1) ? 0 : current[index + 1];
            return ProblemData.TravelDistance[prev, current[index]] + ProblemData.TravelDistance[current[index], next];
        }

        protected virtual double NotSequentialIntraSwapPartialAdd(Route current, int index, int clientID)
        {
            int prev = (index == 0) ? 0 : current[index - 1];
            int next = (index == current.Count - 1) ? 0 : current[index + 1];
            return ProblemData.TravelDistance[prev, clientID] + ProblemData.TravelDistance[clientID, next];
        }

        public virtual double DeltaTravelCost(TwoOpt m)
        {
            int prev = (m.orIndex > 0) ? m.current[m.orIndex - 1] : 0;
            int next = (m.deIndex == m.current.Count - 1) ? 0 : m.current[m.deIndex + 1];
            double remove = ProblemData.TravelDistance[prev, m.current[m.orIndex]] + ProblemData.TravelDistance[m.current[m.deIndex], next];
            double add = ProblemData.TravelDistance[prev, m.current[m.deIndex]] + ProblemData.TravelDistance[m.current[m.orIndex], next];
            return ReverseRangeTravelCost(m.current, m.orIndex, m.deIndex) + add - remove;
        }

        protected virtual double ReverseRangeTravelCost(Route current, int orIndex, int deIndex)
        {
            double remove = 0;
            double add = 0;
            for (int i = orIndex; i < deIndex; i++)
            {
                remove += ProblemData.TravelDistance[current[i], current[i + 1]];
                add += ProblemData.TravelDistance[current[i + 1], current[i]];
            }
            return add - remove;
        }

        public virtual double DeltaTravelCost(InterMove m)
        {
            return RemoveDeltaTravelCost(m.current, m.orIndex,1) + AddInNewDeltatTravelCost(m.deRoute, m.current[m.orIndex], m.current[m.orIndex], m.deIndex);
        }

        protected virtual double AddInNewDeltatTravelCost(Route deRoute, int startClient, int endClient, int deIndex)
        {
            int prev = (deIndex == 0) ? 0 : deRoute[deIndex - 1];
            int next = (deIndex < deRoute.Count) ? deRoute[deIndex] : 0;
            double remove = ProblemData.TravelDistance[prev, next];
            double add = ProblemData.TravelDistance[prev, startClient] + ProblemData.TravelDistance[endClient, next];
            return add - remove;
        }

        public virtual double DeltaTravelCost(InterSwap m)
        {
            int orID = m.current[m.orIndex];
            if (m.deRoute.IsEmpty)
                return RemoveDeltaTravelCost(m.current, m.orIndex,1) + ProblemData.TravelDistance[0, m.orIndex] + ProblemData.TravelDistance[m.orIndex, 0];
            return InterSwapDeltaCost(m.current, m.orIndex,1, m.deRoute[m.deIndex], m.deRoute[m.deIndex]) + InterSwapDeltaCost(m.deRoute, m.deIndex,1, m.current[m.orIndex], m.current[m.orIndex]);
        }

        protected virtual double InterSwapDeltaCost(Route current, int index, int count, int startClient, int endClient)
        {
            int prev = (index == 0) ? 0 : current[index - 1];
            int next = (index+count < current.Count - 1) ? current[index +count] : 0;
            double remove = ProblemData.TravelDistance[prev, current[index]] + ProblemData.TravelDistance[current[index+count-1], next];
            double add = ProblemData.TravelDistance[prev, startClient] + ProblemData.TravelDistance[endClient, next];
            return add - remove;
        }

        public virtual double DeltaTravelCost(CrossoverRoute m)
        {
            int orNext = (m.orIndex == m.current.Count - 1) ? 0 : m.current[m.orIndex + 1];
            double orRange = RangeTravelCost(m.current, m.orIndex + 1, -1);

            double remove = 0;
            double add = 0;
            if (m.deIndex >= 0)
            {
                int deNext = (m.deIndex == m.deRoute.Count - 1) ? 0 : m.deRoute[m.deIndex + 1];
                double deRange = RangeTravelCost(m.deRoute, m.deIndex + 1, -1);

                remove = ProblemData.TravelDistance[m.current[m.orIndex], orNext] + orRange + ProblemData.TravelDistance[m.deRoute[m.deIndex], deNext] + deRange;
                add = ProblemData.TravelDistance[m.current[m.orIndex], deNext] + deRange + ProblemData.TravelDistance[m.deRoute[m.deIndex], orNext] + orRange;
            }
            else
            {
                remove = ProblemData.TravelDistance[m.current[m.orIndex], orNext] + orRange;
                add = ProblemData.TravelDistance[0, orNext] + orRange;
            }

            return add - remove;
        }

        public virtual double DeltaTravelCost(ReverseCrossoverRoute m)
        {
            int orNext = (m.orIndex == m.current.Count - 1) ? 0 : m.current[m.orIndex + 1];
            double orRange = RangeTravelCost(m.current, m.orIndex + 1, -1);

            double remove = 0;
            double add = 0;

            if (m.deIndex >= 0)
            {
                int deNext = (m.deIndex == m.deRoute.Count - 1) ? 0 : m.deRoute[m.deIndex + 1];
                double deRange = RangeTravelCost(m.deRoute, m.deIndex + 1, -1);

                remove = ProblemData.TravelDistance[m.current[m.orIndex], orNext] + orRange + ProblemData.TravelDistance[m.deRoute[m.deIndex], deNext] + deRange;
                add = ProblemData.TravelDistance[m.current[m.orIndex], m.deRoute[m.deRoute.Count - 1]] + ReverseRangeTravelCost(m.deRoute, m.deIndex + 1) +
                      ProblemData.TravelDistance[m.deRoute[m.deIndex], m.current[m.current.Count - 1]] + ReverseRangeTravelCost(m.current, m.orIndex + 1);
            }
            else
            {
                remove = ProblemData.TravelDistance[m.current[m.orIndex], orNext] + orRange;
                add = ProblemData.TravelDistance[0, m.current[m.current.Count - 1]] + ReverseRangeTravelCost(m.current, m.orIndex + 1);
            }

            return add - remove;
        }

        protected virtual double ReverseRangeTravelCost(Route current, int index)
        {
            double cost = 0;
            int next = 0;
            for (int i = index; i < current.Count; i++)
            {
                cost += ProblemData.TravelDistance[current[i], next];
                next = current[i];
            }
            return cost;
        }

        protected virtual double RangeTravelCost(Route current, int index, int count)
        {
            int loopBound = (count < 0) ? current.Count : count;
            double remove = 0;
            for (int i = index; i < loopBound; i++)
            {
                int next = (i == current.Count - 1) ? 0 : current[i + 1];
                remove += ProblemData.TravelDistance[current[i], next];
            }
            return remove;
        }

        public virtual double DeltaTravelCost(TwoInterMove m)
        {
            return RemoveDeltaTravelCost(m.current, m.orIndex, 2) + AddInNewDeltatTravelCost(m.deRoute, m.current[m.orIndex], m.current[m.orIndex + 1 /*+2-1*/], m.deIndex); 
        }

        public virtual double DeltaTravelCost(TwoOneInterSwap m)
        {
            return InterSwapDeltaCost(m.current, m.orIndex, 2, m.deRoute[m.deIndex], m.deRoute[m.deIndex]) +
                InterSwapDeltaCost(m.deRoute, m.deIndex, 1, m.current[m.orIndex], m.current[m.orIndex + 1 /*+2-1*/]);
        }

        public virtual double DeltaTravelCost(TwoTwoInterSwap m)
        {
            return InterSwapDeltaCost(m.current, m.orIndex, 2, m.deRoute[m.deIndex], m.deRoute[m.deIndex+1 /*+2-1*/]) +
                InterSwapDeltaCost(m.deRoute, m.deIndex, 2, m.current[m.orIndex], m.current[m.orIndex + 1 /*+2-1*/]);
        }
        

        #endregion

        #region Delta Overload
        public virtual double DeltaOverload(IntraMove m)
        {
            double currentOverlaod = ProblemData.StrongOverLoad(m.current);
            double newOverload = ProblemData.StrongIntraReplaceOverload(m.current, m.orIndex, m.deIndex);
            return newOverload - currentOverlaod;
        }

        public virtual double DeltaOverload(IntraSwap m)
        {
            double currentOverlaod = ProblemData.StrongOverLoad(m.current);
            double newOverload = ProblemData.StrongIntraSwapOverload(m.current, m.orIndex, m.deIndex);
            return newOverload - currentOverlaod;
        }

        public virtual double DeltaOverload(TwoOpt m)
        {
            double currentOverload = ProblemData.StrongOverLoad(m.current);
            double newOverload = 0;
            double load = ProblemData.TotalDelivery(m.current);
            for (int i = 0; i < m.orIndex; i++)
            {
                load += ProblemData.Clients[m.current[i]].DemandDifference;
                newOverload += Math.Max(0, load - m.current.Vehicle.Capacity);
            }
            for (int i = m.deIndex; i >= m.orIndex; i--)
            {
                load += ProblemData.Clients[m.current[i]].DemandDifference;
                newOverload += Math.Max(0, load - m.current.Vehicle.Capacity);
            }
            for (int i = m.deIndex + 1; i < m.current.Count; i++)
            {
                load += ProblemData.Clients[m.current[i]].DemandDifference;
                newOverload += Math.Max(0, load - m.current.Vehicle.Capacity);
            }
            return newOverload - currentOverload;

        }

        public virtual double DeltaOverload(InterMove m)
        {
            double remove = StrongRemoveDeltaOverload(m.current, m.orIndex, 1);
            double add = StrongAddDeltaOverload(m.deRoute, m.deIndex, new List<int> { m.current[m.orIndex]});
            return add + remove;
        }

        protected virtual double StrongRemoveDeltaOverload(Route current, int index, int count)
        {
            double currentOverlaod = ProblemData.StrongOverLoad(current);
            double newOverload = ProblemData.StrongRemoveOverload(current, index,count);
            return newOverload - currentOverlaod;
        }

        protected virtual double StrongAddDeltaOverload(Route current, int index, List<int> range)
        {
            double currentOverlaod = ProblemData.StrongOverLoad(current);
            double newOverload = ProblemData.StrongAddOverload(current, index, range);
            return newOverload - currentOverlaod;
        }

        public virtual double DeltaOverload(InterSwap m)
        {
            double replacement = 0;
            if (m.deRoute.IsEmpty)
            {
                replacement += StrongRemoveDeltaOverload(m.current, m.orIndex, 1);
                replacement += StrongAddDeltaOverload(m.deRoute, 0, new List<int> { m.current[m.orIndex] });
            }
            else
                replacement += StrongReplacementOverload(m.current, m.orIndex, 1, new List<int> { m.deRoute[m.deIndex]}) + StrongReplacementOverload(m.deRoute, m.deIndex, 1, new List<int> { m.current[m.orIndex] });
            return replacement;
        }

        protected virtual double StrongReplacementOverload(Route current, int index, int count, List<int> range)
        {
            double currentOverlaod = ProblemData.StrongOverLoad(current);
            double newOverload = ProblemData.StrongReplaceOverload(current, index, count, range);
            return newOverload - currentOverlaod;
        }

        public virtual double DeltaOverload(CrossoverRoute m)
        {
            double currentOverload = ProblemData.StrongOverLoad(m.current) + ProblemData.StrongOverLoad(m.deRoute);
            double newOverload = ReplaceRangeOverload(m.current, m.deRoute, m.orIndex, m.deIndex) + ReplaceRangeOverload(m.deRoute, m.current, m.deIndex, m.orIndex);
            return newOverload - currentOverload;
        }

        protected virtual double ReplaceRangeOverload(Route orRoute, Route deRoute, int orIndex, int deIndex)
        {
            double overload = 0;
            double load = ReplaceRangeStartLoad(orRoute, deRoute, orIndex, deIndex);
            if (orIndex >= 0)
            {
                for (int i = 0; i <= orIndex; i++)
                {
                    load += ProblemData.Clients[orRoute[i]].DemandDifference;
                    overload += Math.Max(0, load - orRoute.Vehicle.Capacity);
                }
            }
            if (deIndex >= 0)
            {
                for (int i = deIndex + 1; i < deRoute.Count; i++)
                {
                    load += ProblemData.Clients[deRoute[i]].DemandDifference;
                    overload += Math.Max(0, load - orRoute.Vehicle.Capacity);
                }
            }
            return overload;
        }

        protected virtual double ReplaceRangeStartLoad(Route orRoute, Route deRoute, int orIndex, int deIndex)
        {
            double load = 0;
            if (orIndex >= 0)
                for (int i = 0; i <= orIndex; i++)
                    load += ProblemData.Clients[orRoute[i]].Delivery;
            if (deIndex >= 0)
                for (int i = deIndex + 1; i < deRoute.Count; i++)
                    load += ProblemData.Clients[deRoute[i]].Delivery;
            return load;
        }

        public virtual double DeltaOverload(ReverseCrossoverRoute m)
        {
            double currentOverload = ProblemData.StrongOverLoad(m.current) + ProblemData.StrongOverLoad(m.deRoute);
            double newOverload = ReplaceReverseRangeOverload(m.current, m.deRoute, m.orIndex, m.deIndex) + ReplaceReverseRangeOverload(m.deRoute, m.current, m.deIndex, m.orIndex);
            return newOverload - currentOverload;
        }

        protected virtual double ReplaceReverseRangeOverload(Route orRoute, Route deRoute, int orIndex, int deIndex)
        {
            double overload = 0;
            double load = ReplaceRangeStartLoad(orRoute, deRoute, orIndex, deIndex);
            if (orIndex >= 0)
            {
                for (int i = 0; i <= orIndex; i++)
                {
                    load += ProblemData.Clients[orRoute[i]].DemandDifference;
                    overload += Math.Max(0, load - orRoute.Vehicle.Capacity);
                }
            }
            if (deIndex >= 0)
            {
                for (int i = deRoute.Count - 1; i > deIndex; i--)
                {
                    load += ProblemData.Clients[deRoute[i]].DemandDifference;
                    overload += Math.Max(0, load - orRoute.Vehicle.Capacity);
                }
            }
            return overload;
        }

        public virtual double DeltaOverload(TwoInterMove m)
        {
            double remove = StrongRemoveDeltaOverload(m.current, m.orIndex, 2);
            double add = StrongAddDeltaOverload(m.deRoute, m.deIndex, m.current.GetRange(m.orIndex, 2));
            return add + remove;
        }

        public virtual double DeltaOverload(TwoOneInterSwap m)
        {
            return StrongReplacementOverload(m.current, m.orIndex, 2, new List<int> { m.deRoute[m.deIndex] }) + 
                StrongReplacementOverload(m.deRoute, m.deIndex, 1, m.current.GetRange(m.orIndex, 2));
        }

        public virtual double DeltaOverload(TwoTwoInterSwap m)
        {
            return StrongReplacementOverload(m.current, m.orIndex, 2, m.deRoute.GetRange(m.deIndex, 2)) +
                StrongReplacementOverload(m.deRoute, m.deIndex, 2, m.current.GetRange(m.orIndex, 2));
        }

        #endregion

        #region Delta Cost

        public virtual double DeltaCost(IntraMove m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(IntraSwap m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(TwoOpt m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(InterMove m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(InterSwap m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(CrossoverRoute m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(ReverseCrossoverRoute m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(TwoInterMove m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(TwoOneInterSwap m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        public virtual double DeltaCost(TwoTwoInterSwap m)
        {
            if (OverloadFactor == 0) return DeltaTravelCost(m);
            return DeltaTravelCost(m) + OverloadFactor * DeltaOverload(m);
        }

        #endregion

        #region Shaking

        public List<Func<RouteSet, Random, RouteSet>> GetAllShakingProcedures()
        {
            List<Func<RouteSet, Random, RouteSet>> shaking = new List<Func<RouteSet, Random, RouteSet>>();
            //shaking.Add(Reverse);
            shaking.Add(Exchange);
            shaking.Add(RandomShaking);
            return shaking;
        }

        public RouteSet Reverse(RouteSet solution, Random rdObj)
        {
            RouteSet newSolution = (RouteSet)solution.Clone();
            newSolution.CleanUnusedRoutes();
            Route current = newSolution[rdObj.Next(newSolution.Count)];
            int index = rdObj.Next(current.Count);
            int remaning = current.Count - index;
            if (remaning < 2) return newSolution;
            int reverseAmount = rdObj.Next(remaning);
            if (reverseAmount < 2) return newSolution;
            current = Reverse(current, index, reverseAmount);
            return newSolution;
        }

        protected Route Reverse(Route current, int index, int count)
        {
            current.Reverse(index, count);
            return current;
        }

        public RouteSet Exchange(RouteSet solution, Random rdObj)
        {
            RouteSet newSolution = (RouteSet)solution.Clone();
            newSolution.CleanUnusedRoutes();
            if (newSolution.Count < 2) return newSolution;
            int xRouteIndex = rdObj.Next(newSolution.Count);

            Route xRoute = newSolution[xRouteIndex];
            Route yRoute = newSolution.SelectDiffetenRoute(xRouteIndex, rdObj);

            int xIndex = rdObj.Next(xRoute.Count);
            int xRemaning = xRoute.Count - xIndex;
            int xCount = rdObj.Next(xRemaning);

            int yIndex = rdObj.Next(yRoute.Count);
            int yRemaing = yRoute.Count - yIndex;
            int yCount = rdObj.Next(yRemaing);

            var exRoutes = Exchange(xRoute, xIndex, xCount, yRoute, yIndex, yCount);
            return newSolution;
        }

        protected Tuple<Route, Route> Exchange(Route xRoute, int xIndex, int xCount, Route yRoute, int yIndex, int yCount)
        {
            List<int> xRange = xRoute.GetRange(xIndex, xCount);
            List<int> yRange = yRoute.GetRange(yIndex, yCount);
            xRoute.RemoveRange(xIndex, xCount);
            xRoute.InsertRange(xIndex, yRange);
            yRoute.RemoveRange(yIndex, yCount);
            yRoute.InsertRange(yIndex, xRange);
            return new Tuple<Route, Route>(xRoute, yRoute);
        }

        public RouteSet RandomShaking(RouteSet solution, Random rdObj)
        {
            return RandomShaking(solution, rdObj, 0.2);
        }

        public RouteSet RandomShaking(RouteSet solution, Random rdObj, double shakingFactor)
        {
            RouteSet newSolution = (RouteSet)solution.Clone();
            newSolution.CleanUnusedRoutes();
            int clientsToRemove = ClientsToRemove(ProblemData.Clients.Count, shakingFactor);
            List<int> clientsToReinsert = ClientsToReinsert(newSolution, rdObj, clientsToRemove);
            for (int i = 0; i < clientsToReinsert.Count; i++)
            {
                newSolution.InsertAtRandomPosition(clientsToReinsert[i], rdObj);
            }
            return newSolution;
        }

        private int ClientsToRemove(int clientsAmount, double shakingFactor)
        {
            return (int)Math.Max(Math.Floor(clientsAmount * shakingFactor), 1);
        }

        private List<int> ClientsToReinsert(RouteSet solution, Random rdObj, int clientsToRemove)
        {
            solution.CleanUnusedRoutes();
            List<int> clientsToReinsert = new List<int>();
            while (clientsToReinsert.Count < clientsToRemove)
            {
                int r = rdObj.Next(solution.Count);
                int c = rdObj.Next(solution[r].Count);
                clientsToReinsert.Add(solution[r][c]);
                solution[r].RemoveAt(c);
                if (solution[r].IsEmpty)
                    solution.RemoveAt(r);
            }
            return clientsToReinsert;
        }
        #endregion
    }

    public class RouteSetRCLProgress
    {
        public VRPSimultaneousPickupDelivery ProblemData { get; private set; }
        public RouteSet PartialSolution { get; set; }
        public double LoadinglPickup { get; set; }
        public double LoadingDelivery { get; set; }
        private int currentRoute;

        public RouteSetRCLProgress(VRPSimultaneousPickupDelivery problemData, RouteSet partialSolution, double pickup, double delivery)
        {
            ProblemData = problemData;
            PartialSolution = partialSolution;
            LoadingDelivery = delivery;
            LoadinglPickup = pickup;
            currentRoute = 0;
        }

        public Route GetLoadingRoute
        {
            get { return PartialSolution[currentRoute]; }
        }

        public double GetNextEmptyRouteCapacity
        {
            get
            {
                if (currentRoute + 1 < PartialSolution.Count)
                    return PartialSolution[currentRoute + 1].Vehicle.Capacity;
                else
                    return PartialSolution[0].Vehicle.Capacity; /*el crecimiento de las rutas es ciclico*/
            }
        }

        public int GetReferenceClientID(Route loadingRoute)
        {
            if (loadingRoute.IsEmpty)
                return 0;
            return loadingRoute[loadingRoute.Count - 1];
        }

        public void CloseRoute()
        {
            currentRoute++;
            LoadingDelivery = 0;
            LoadinglPickup = 0;
        }

        public bool HasAvailablesRoutes
        {
            get { return currentRoute < PartialSolution.Count; }
        }

        public void IncreasseRoutes()
        {
            RouteSet newRoutes = RouteSet.BuildEmptyRouteSet(ProblemData.Vehicles);
            PartialSolution.AddRange(newRoutes);
        }

    }

    public class ClientRCL
    {
        public PickupDeliveryClient ClientInfo { get; set; }
        public double DeltaCost { get; set; }
        internal bool InsertAtNewRoute { get; set; }

        public ClientRCL(PickupDeliveryClient client, double cost, bool insertNew)
        {
            ClientInfo = client;
            DeltaCost = cost;
            InsertAtNewRoute = insertNew;
        }
    }

    public abstract class Move
    {
        public double deltaCost { get; set; }
        public abstract void Do();
    }

    public class IntraMove : Move
    {
        public Route current { get; set; }
        public int orIndex { get; set; }
        public int deIndex { get; set; }


        public IntraMove(Route current, int orIndex, int deIndex, double deltaCost)
        {
            this.current = current;
            this.orIndex = orIndex;
            this.deIndex = deIndex;
            this.deltaCost = deltaCost;
        }

        public override void Do()
        {
            if (deIndex < current.Count)
                current.Insert(deIndex, current[orIndex]);
            else current.Add(current[orIndex]);
            if (orIndex < deIndex)
                current.RemoveAt(orIndex);
            else
                current.RemoveAt(orIndex + 1);
        }
    }

    public class IntraSwap : IntraMove
    {
        public IntraSwap(Route current, int orIndex, int deIndex, double deltaCost)
            : base(current, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            int orID = current[orIndex];
            current[orIndex] = current[deIndex];
            current[deIndex] = orID;
        }
    }

    public class TwoOpt : IntraMove
    {
        public TwoOpt(Route current, int orIndex, int deIndex, double deltaCost)
            : base(current, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            int count = deIndex - orIndex + 1;
            current.Reverse(orIndex, count);
        }
    }

    public class InterMove : IntraMove
    {
        public Route deRoute { get; set; }

        public InterMove(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, orIndex, deIndex, deltaCost)
        {
            this.deRoute = deRoute;
        }

        public override void Do()
        {
            if (deIndex < deRoute.Count)
                deRoute.Insert(deIndex, current[orIndex]);
            else deRoute.Add(current[orIndex]);
            current.RemoveAt(orIndex);
        }
    }

    public class TwoInterMove : InterMove
    {
        /*los clientes son consecutivos*/
        public TwoInterMove(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, deRoute, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            if (deIndex < deRoute.Count)
                deRoute.InsertRange(deIndex, current.GetRange(orIndex,2));
            else deRoute.AddRange(current.GetRange(orIndex, 2));
            current.RemoveRange(orIndex, 2);
        }
    }

    public class InterSwap : InterMove
    {
        public InterSwap(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, deRoute, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            var orID = current[orIndex];
            if (deRoute.IsEmpty)
            {
                deRoute.Add(orID);
                current.RemoveAt(orIndex);
            }
            else
            {
                current[orIndex] = deRoute[deIndex];
                deRoute[deIndex] = orID;
            }
        }
    }

    public class TwoOneInterSwap : InterMove
    {
        /*los clientes son consecutivos*/
        public TwoOneInterSwap(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, deRoute, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            if (deRoute.IsEmpty)
            {
                deRoute.AddRange(current.GetRange(orIndex, 2));
                current.RemoveRange(orIndex, 2);
            }
            else
            {
                var oRange = current.GetRange(orIndex, 2);
                current[orIndex] = deRoute[deIndex];
                current.RemoveAt(orIndex + 1);
                deRoute.InsertRange(deIndex, oRange);
                deRoute.RemoveAt(deIndex + 2);
            }
        }
    }

    public class TwoTwoInterSwap : InterMove
    {
        /*los clientes son consecutivos*/
        public TwoTwoInterSwap(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, deRoute, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            var xID = current[orIndex];
            var yID = current[orIndex + 1];
            if (deRoute.IsEmpty)
            {
                deRoute.AddRange(new List<int> { xID, yID });
                current.RemoveRange(orIndex, 2);
            }
            else
            {
                current[orIndex] = deRoute[deIndex];
                current[orIndex + 1] = deRoute[deIndex + 1];
                deRoute[deIndex] = xID;
                deRoute[deIndex + 1] = yID;
            }
        }
    }

    public class CrossoverRoute : InterMove
    {
        public CrossoverRoute(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, deRoute, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            int orRemaning = current.Count - orIndex - 1;
            List<int> orRange = current.GetRange(orIndex + 1, orRemaning);
            current.RemoveRange(orIndex + 1, orRemaning);
            if (deIndex >= 0)
            {
                /*ruta de destino no vacia*/
                int deRemaning = deRoute.Count - deIndex - 1;
                List<int> deRange = deRoute.GetRange(deIndex + 1, deRemaning);
                deRoute.RemoveRange(deIndex + 1, deRemaning);
                current.AddRange(deRange);
            }
            deRoute.AddRange(orRange);
        }
    }

    public class ReverseCrossoverRoute : CrossoverRoute
    {
        public ReverseCrossoverRoute(Route orRoute, Route deRoute, int orIndex, int deIndex, double deltaCost)
            : base(orRoute, deRoute, orIndex, deIndex, deltaCost)
        { }

        public override void Do()
        {
            int orRemaning = current.Count - orIndex - 1;
            List<int> orRange = current.GetRange(orIndex + 1, orRemaning);
            orRange.Reverse();
            current.RemoveRange(orIndex + 1, orRemaning);
            if (deIndex >= 0)
            {
                /*ruta de destino no vacia*/
                int deRemaning = deRoute.Count - deIndex - 1;
                List<int> deRange = deRoute.GetRange(deIndex + 1, deRemaning);
                deRange.Reverse();
                deRoute.RemoveRange(deIndex + 1, deRemaning);
                current.AddRange(deRange);
            }
            deRoute.AddRange(orRange);
        }
    }
}
