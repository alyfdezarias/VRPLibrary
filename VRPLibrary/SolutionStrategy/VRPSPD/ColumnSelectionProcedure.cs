using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;

using VRPLibrary.ProblemData;
using VRPLibrary.RouteSetData;
using VRPLibrary.SolutionStrategy.GAMS;

using MetaheuristicsLibrary;
using MetaheuristicsLibrary.SimulatedAnnealing;
using MetaheuristicsLibrary.VariableNeighborhoodSearch;

namespace VRPLibrary.SolutionStrategy.VRPSPD
{
    public class ColumnSelectionProcedure : PenalizationLocalSearch
    {
        public ColumnSelectionProcedure(VRPSimultaneousPickupDelivery problemData) : base(problemData) { }

        #region Solutions Pool

        protected List<RouteSet> BuildSolutionPool<TProcedure, TParameters>(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                                                            Exploration expCondition, Random rdObj,
                                                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                                                            List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                                                            TProcedure procedure, TParameters parameters)
                                                                            where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<RouteSet> initialPool = BuildInitialPool(alphaStep, alphaPool, rdObj);
            OverloadFactor = 0;
            return BuildSolutionPool(overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, procedure, parameters, initialPool);
        }

        private List<RouteSet> BuildInitialPool(double alphaStep, int alphaPool, Random rdObj)
        {
            List<RouteSet> initialPool = new List<RouteSet>();
            for (double alpha = 0.1;  alpha < 1; alpha+= alphaStep)
            {
                initialPool.AddRange(GenerateGreedyRdPool(alpha, rdObj, alphaPool));
            }
            return initialPool;
        }

        private List<RouteSet> BuildSolutionPool<TProcedure, TParameters>(double overloadFactor, int shakings, Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shakingProcedures, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<RouteSet> solutionPool = new List<RouteSet>();
            foreach (var solution in initialPool)
            {
                overloadFactor = 0;
                RouteSet current = solution;
                for (int i = 0; i < shakings; i++)
                {
                    current = shakingProcedures[rdObj.Next(shakingProcedures.Count)](current, rdObj);
                    current = procedure.Solve(parameters, current, neighborhoods, expCondition, GetCost, rdObj);
                    var clone = (RouteSet)current.Clone();
                    clone.CleanUnusedRoutes();
                    solutionPool.Add(clone);
                    if (!ProblemData.IsStrongFeasible(current))
                        OverloadFactor += overloadFactor;
                }
                
            }
            return solutionPool;
        }

        public List<RouteSet> BuildSolutionPool(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                                Exploration expCondition, Random rdObj,
                                                List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                                List<Func<RouteSet, Random, RouteSet>> shakingProcedures)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return BuildSolutionPool(alphaStep, alphaPool, overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, vndProcedure, new NullParameters());
        }

        public List<RouteSet> RdVNDBuildSolutionPool(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                                Exploration expCondition, Random rdObj,
                                                List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                                List<Func<RouteSet, Random, RouteSet>> shakingProcedures)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return BuildSolutionPool(alphaStep, alphaPool, overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, vndProcedure, new NullParameters());
        }

        public List<RouteSet> BuildSolutionPool(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                                Exploration expCondition, Random rdObj,
                                                List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                                List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                                SimulatedAnnealingParameters saParams)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return BuildSolutionPool(alphaStep, alphaPool, overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, saProcedure, saParams);
        }

        public List<RouteSet> BuildSolutionPool(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                                Exploration expCondition, Random rdObj,
                                                List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                                List<Func<RouteSet, Random, RouteSet>> shakingProcedure,
                                                double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            List<RouteSet> initialPool = BuildInitialPool(alphaStep, alphaPool, rdObj);
            OverloadFactor = 0;
            SimulatedAnnealingParameters saParams = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            return BuildSolutionPool(overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedure, saProcedure, saParams, initialPool);
        }

        #endregion

        public List<ExtRouteInfo> BuildRoutePool(List<RouteSet> solutionPool, double overloadFactor, string gamsFolderPath)
        {
            OverloadFactor = 0;
            List<ExtRouteInfo> routePool = new List<ExtRouteInfo>();
            SingleRouteSPDGAMS gamsProcedure = new SingleRouteSPDGAMS(ProblemData);
            DirectoryInfo gamsDir = Directory.CreateDirectory(gamsFolderPath);
            int i = 0;

            foreach (var solution in solutionPool)
            {
                foreach (var route in solution)
                {
                    if (route.IsEmpty) continue;
                    if (ProblemData.IsFeasible(route) && !ExtRouteInfo.IsInPool(routePool, route))
                    {
                        DirectoryInfo gamsIteration = gamsDir.CreateSubdirectory(i.ToString());
                        Route current = gamsProcedure.Solve(gamsIteration.FullName, route);
                        routePool.Add(new ExtRouteInfo(current, GetTravelCost(route)));
                    }
                }
            }
            return routePool;
        }

        protected RouteSet BuildSolution(List<Route> selectedRoutes)
        {
            List<int> overlapping = OverlappingClients(selectedRoutes);
            while(overlapping.Count > 0)
            {
                Dictionary<int, List<OverlappingClientInfo>> removeInfo = RemoveOverlappingInfo(selectedRoutes, overlapping);
                var bestOption = GetBestDeltaCostInfo(removeInfo);
                selectedRoutes[bestOption.Item2.RouteIndex].RemoveAt(bestOption.Item2.ClientIndex);
                if (removeInfo[bestOption.Item1].Count == 2)/*el cliente solo aparece en dos rutas, luego al eliminarlo de una no hay mas overlapping q lo incluya a el*/
                    overlapping.Remove(bestOption.Item1);
            }
            return new RouteSet(selectedRoutes);
        }

        protected Tuple<int, OverlappingClientInfo> GetBestDeltaCostInfo(Dictionary<int, List<OverlappingClientInfo>> removeInfo)
        {
            double bestDeltaCost = int.MaxValue;
            int clientID = -1;
            OverlappingClientInfo info = null;
            foreach (var item in removeInfo)
            {
                foreach (var o in item.Value)
                {
                    if (o.DeltaCost < bestDeltaCost)
                    {
                        bestDeltaCost = o.DeltaCost;
                        clientID = item.Key;
                        info = o;
                    }
                }
            }
            return new Tuple<int, OverlappingClientInfo>(clientID, info);
        }

        protected List<int> OverlappingClients(List<Route> selectedRoutes)
        {
            List<int> overlapping = new List<int>();
            int[] inSolution = new int[ProblemData.Clients.Count];
            foreach (var item in selectedRoutes)
            {
                foreach (var c in item)
                {
                    if (inSolution[c-1] == 1) /*los clientes empiezan a numerar a partir de 1*/
                        overlapping.Add(c);
                    inSolution[c-1]++;
                }
            }
            return overlapping;
        }

        protected Dictionary<int, List<OverlappingClientInfo>> RemoveOverlappingInfo(List<Route> selected, List<int> overlapping)
        {
            Dictionary<int, List<OverlappingClientInfo>> overlappingInfo = new Dictionary<int, List<OverlappingClientInfo>>();
            for (int i = 0; i < selected.Count; i++)
            {
                foreach (var c in overlapping)
                {
                    OverlappingClientInfo info = GetRemoveOverlappingInfo(selected[i], c, i);
                    if (info != null)
                    {
                        if (overlappingInfo.ContainsKey(c))
                            overlappingInfo[c].Add(info);
                        else overlappingInfo.Add(c, new List<OverlappingClientInfo>() { info });
                    }
                }
            }
            return overlappingInfo;
        }

        protected OverlappingClientInfo GetRemoveOverlappingInfo(Route r, int clientID, int routeIndex)
        {
            int clientIndex = r.IndexOf(clientID);
            if (clientIndex < 0) return null;
            return new OverlappingClientInfo(routeIndex, clientIndex, RemoveDeltaTravelCost(r, clientIndex,1));
        }

        #region Solve
        protected RouteSet Solve<TProcedure, TParameters>(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                                          Exploration expCondition, Random rdObj,
                                                          List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                                          List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                                          string gamsFolderPath,
                                                          TProcedure procedure, TParameters parameters)
                                                          where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = 0;
            List<RouteSet> initialPool = BuildInitialPool(alphaStep, alphaPool, rdObj);
            Console.WriteLine("inital pool {0}", initialPool.Count);
            return Solve(overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, procedure, parameters, initialPool);
        }

        private RouteSet Solve<TProcedure, TParameters>(double overloadFactor, int shakings, Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shakingProcedures, string gamsFolderPath, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<RouteSet> solutionPool = BuildSolutionPool(overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, procedure, parameters, initialPool);
            Console.WriteLine("solution pool {0}", solutionPool.Count);
            List<ExtRouteInfo> routesPool = BuildRoutePool(solutionPool, overloadFactor, gamsFolderPath);
            Console.WriteLine("routes pool {0}", routesPool.Count);
            ColumnSelectionGAMS gamsProcedure = new ColumnSelectionGAMS(ProblemData);
            List<Route> selected = gamsProcedure.Solve(gamsFolderPath, routesPool);
            for (int i = 0; i < selected.Count; i++)
            {
                selected[i].ToXMLFormat().Save(Path.Combine(gamsFolderPath, string.Format("overlapping{0}.xml", i)));
            }
            return BuildSolution(selected);
        }

        public RouteSet Solve(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                 Exploration expCondition, Random rdObj,
                                 List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                 List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                 string gamsFolderPath)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> procedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return Solve(alphaStep, alphaPool, overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, procedure, new NullParameters());
        }

        public RouteSet RdVNDSolve(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                 Exploration expCondition, Random rdObj,
                                 List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                 List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                 string gamsFolderPath)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> procedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return Solve(alphaStep, alphaPool, overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, procedure, new NullParameters());
        }

        public RouteSet Solve(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                 Exploration expCondition, Random rdObj,
                                 List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                 List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                 string gamsFolderPath, SimulatedAnnealingParameters saParams)
        {
            SimulatedAnnealingProcedure<RouteSet> procedure = new SimulatedAnnealingProcedure<RouteSet>();
            return Solve(alphaStep, alphaPool, overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, procedure, saParams);
        }

        public RouteSet Solve(double alphaStep, int alphaPool, double overloadFactor, int shakings,
                                 Exploration expCondition, Random rdObj,
                                 List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                 List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                 string gamsFolderPath,
                                 double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = 0;
            List<RouteSet> initialPool = BuildInitialPool(alphaStep,alphaPool, rdObj);
            SimulatedAnnealingProcedure<RouteSet> procedure = new SimulatedAnnealingProcedure<RouteSet>();
            SimulatedAnnealingParameters saParams = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            return Solve(overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, procedure, saParams, initialPool); 
        }
        #endregion

    }

    public class ExtRouteInfo
    {
        public double Cost { get; private set; }
        public Route Current { get; private set; }
        public ExtRouteInfo(Route current, double cost)
        {
            Current = current;
            Cost = cost;
        }
        public bool IsSimilar(Route r)
        {
            if (r.Count != Current.Count) return false;
            foreach (var item in Current)
            {
                if (r.IndexOf(item) < 0)
                    return false;
            }
            return true;
        }

        public static bool IsInPool(List<ExtRouteInfo> routePool, Route r)
        {
            foreach (var item in routePool)
                if (item.IsSimilar(r)) return true;
            return false;
        }
    }

    public class OverlappingClientInfo
    {
        public int RouteIndex { get; set;}
        public int ClientIndex { get; set;}
        public double DeltaCost { get; set;}

        public OverlappingClientInfo(int routeIndex, int clientIndex, double deltaCost)
        {
            RouteIndex = routeIndex;
            ClientIndex = clientIndex;
            DeltaCost = deltaCost;
        }
    }
}
