using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;

using VRPLibrary.ProblemData;
using VRPLibrary.RouteSetData;
using MetaheuristicsLibrary;
using MetaheuristicsLibrary.VariableNeighborhoodSearch;
using VRPLibrary.SolutionStrategy.GAMS;

namespace VRPLibrary.SolutionStrategy.VRPSPD
{
    public class IteratedRouteGenerationProcedure : ColumnGenrationProcedure
    {
        public IteratedRouteGenerationProcedure(VRPSimultaneousPickupDelivery problemData)
            : base(problemData)
        { }

        #region Single Solve
        protected RouteSet HeuristicCGSolve<TProcedure, TParameters>(RouteSet initial, Exploration expCondition, double overloadFactor, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             TProcedure procedure, TParameters parameters,
                             string folderPath, int maxIterations,
                             bool includedDualInfo, bool useCompletePool,
                             bool parallelexe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {

            Stopwatch timer = new Stopwatch();
            int iteration = 0;

            OverloadFactor = overloadFactor;

            RouteSet bestSolution = procedure.Solve(parameters, initial, neighborhoods, expCondition, GetCost, rdObj);
            double bestCost = GetCost(bestSolution);
            RouteSet current = (RouteSet)initial.Clone();
            double currentCost = GetCost(initial);
            double currentMPCost = int.MaxValue;
            double refCost = int.MaxValue;
            List<Tuple<Route, double>> coveringSelection = new List<Tuple<Route, double>>();
            List<ExtRouteInfo> completePool = new List<ExtRouteInfo>();  //GetInitialPool(current);
            double[] previousDual = GetDualVariables(bestSolution);
            double[] currentDual = GetDualVariables(bestSolution);
            

            timer.Start();
            do
            {
                refCost = currentMPCost;
                previousDual = currentDual;
                List<ExtRouteInfo> shkPool = null;
                
                if (includedDualInfo)
                {
                    DirectoryInfo gamsDir = Directory.CreateDirectory(Path.Combine(folderPath, string.Format("gams{0}", iteration)));
                    double[] dualExtreme = GetExtremeDualVariables(gamsDir.FullName, completePool, ProblemData.Clients.Count + 1);
                    currentDual = GetDualLinearCombination(dualExtreme, previousDual, rdObj);
                    shkPool = BuildDualRestrictedPool(current, expCondition, rdObj, neighborhoods, shaking, iteration, timer, procedure, parameters, currentDual, parallelexe);
                }
                else shkPool = BuildRestrictedPool(current, expCondition, rdObj, neighborhoods, shaking, iteration, timer, procedure, parameters, parallelexe);

                if (useCompletePool)
                {
                    completePool.AddRange(shkPool);
                    completePool = RemovePoolRedundancy(completePool);
                }
                else
                    completePool = new List<ExtRouteInfo>(shkPool);

                coveringSelection = SolveRestrictedMasterProblem(folderPath, string.Format("coveringPool{0}", iteration), completePool);
                currentMPCost = coveringSelection.Sum(r => GetCost(r.Item1) * r.Item2);

                current = procedure.Solve(parameters, BuildSolution(coveringSelection, rdObj), neighborhoods, expCondition, GetCost, rdObj);
                currentCost = GetCost(current);

                if (currentCost < bestCost)
                {
                    bestSolution = (RouteSet)current.Clone();
                    bestCost = currentCost;
                    OnIteration(new IterationEventArgs<RouteSet>(bestSolution, iteration, timer.Elapsed.TotalSeconds));
                }

                iteration++;
                
            } while (currentMPCost < refCost && iteration < maxIterations);
            timer.Stop();
            return bestSolution;
        }

        private double[] GetDualLinearCombination(double[] extreme, double[] previous, Random rdObj)
        {
            if (extreme.Sum() == 0) return previous;//caso inicial

            double[] dualCombination = new double[extreme.Length];
            double alpha = rdObj.NextDouble();
            for (int i = 0; i < dualCombination.Length; i++)
            {
                dualCombination[i] = alpha * extreme[i] + (1 - alpha) * previous[i];
            }
            return dualCombination;
        }

        public RouteSet HeuristicCGSolve(RouteSet initial, Exploration expCondition, double overloadFactor, Random rdObj,
                                     List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                     List<Func<RouteSet, Random, RouteSet>> shaking,
                                     string folderPath, int maxIterations, 
                                     bool completePool, bool dualInfo)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> procedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return HeuristicCGSolve(initial, expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, new NullParameters(), folderPath, maxIterations, dualInfo, completePool, false);
        }

        public RouteSet ParallelExeHeuristicCGSolve(RouteSet initial, Exploration expCondition, double overloadFactor, Random rdObj,
                                     List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                     List<Func<RouteSet, Random, RouteSet>> shaking,
                                     string folderPath, int maxIterations,
                                     bool completePool, bool dualInfo)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> procedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return HeuristicCGSolve(initial, expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, new NullParameters(), folderPath, maxIterations, dualInfo, completePool, true);
        }

        public RouteSet RdVNDHeuristicCGSolve(RouteSet initial, Exploration expCondition, double overloadFactor, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             string folderPath, int maxIterations,
                             bool completePool, bool dualInfo)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> procedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return HeuristicCGSolve(initial, expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, new NullParameters(), folderPath, maxIterations, dualInfo, completePool, false);
        }

        public RouteSet ParallelExeRdVNDHeuristicCGSolve(RouteSet initial, Exploration expCondition, double overloadFactor, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             string folderPath, int maxIterations,
                             bool completePool, bool dualInfo)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> procedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return HeuristicCGSolve(initial, expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, new NullParameters(), folderPath, maxIterations, dualInfo, completePool, true);
        }

        #endregion

        #region Multi Solve

        protected RouteSet HeuristicCGSolve<TProcedure, TParameters>(List<RouteSet> pool, Exploration expCondition, double overloadFactor, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             TProcedure procedure, TParameters parameters,
                             string folderPath, int maxIterations,
                             bool includedDualInfo, bool useCompletePool, 
                             bool parallelexe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {

            RouteSet bestSolution = (RouteSet)pool[0].Clone();
            double bestCost = GetCost(bestSolution);

            for (int i = 0; i < pool.Count; i++)
            {
                DirectoryInfo iterationDir = Directory.CreateDirectory(folderPath).CreateSubdirectory(string.Format("mcg{0}", i));
                RouteSet current = HeuristicCGSolve(pool[i], expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, parameters, iterationDir.FullName, maxIterations, includedDualInfo, useCompletePool, parallelexe);
                double currentCost = GetCost(current);
                if (currentCost < bestCost)
                {
                    bestCost = currentCost;
                    bestSolution = (RouteSet)current.Clone();
                    OnIteration(new IterationEventArgs<RouteSet>(bestSolution));
                }
            }
            return bestSolution;
        }

        public RouteSet HeuristicCGSolve(List<RouteSet> pool, Exploration expCondition, double overloadFactor, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             string folderPath, int maxIterations,
                             bool completePool, bool dualInfo)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> procedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return HeuristicCGSolve(pool, expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, new NullParameters(), folderPath, maxIterations, dualInfo, completePool, false);
        }

        public RouteSet RdVNDHeuristicCGSolve(List<RouteSet> pool, Exploration expCondition, double overloadFactor, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             string folderPath, int maxIterations,
                             bool completePool, bool dualInfo)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> procedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return HeuristicCGSolve(pool, expCondition, overloadFactor, rdObj, neighborhoods, shaking, procedure, new NullParameters(), folderPath, maxIterations, dualInfo, completePool, false);
        }

        #endregion

        private List<ExtRouteInfo> BuildRestrictedPool<TProcedure, TParameters>(RouteSet initial, Exploration expCondition, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             int iteration, Stopwatch timer,
                             TProcedure procedure, TParameters parameters, 
                             bool parallelexe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<Tuple<RouteSet, double>> shkPool = (parallelexe)?ParallelExeParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, initial, initial, iteration, timer): ParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, initial, initial, iteration, timer);
            return BuildRestrictedPool(shkPool);
        }

        private List<ExtRouteInfo> BuildRestrictedPool(List<Tuple<RouteSet, double>> shkPool)
        {
            List<ExtRouteInfo> routesPool = new List<ExtRouteInfo>();
            foreach (var item in shkPool)
                foreach (var r in item.Item1)
                    routesPool.Add(new ExtRouteInfo(r, GetCost(r)));
            return RemovePoolRedundancy(routesPool);
        }

        private Tuple<RouteSet, List<int>> BuildOverlappedSolution(List<Tuple<Route, double>> coveringSelection)
        {
            coveringSelection.Sort((x, y) => -1 * x.Item2.CompareTo(y.Item2));

            bool[] included = new bool[ProblemData.Clients.Count + 1];/*numeracion de los clientes comienza en 1 para n tener q estar restando 1 en las verificaciones*/
            int toInclude = ProblemData.Clients.Count;
            List<int> overlapped = new List<int>();

            RouteSet overSolution = new RouteSet();
            int routeIndex = 0;
            do
            {
                bool newIncluded = false;
                List<int> newOverlapped = new List<int>();
                foreach (var c in coveringSelection[routeIndex].Item1)
                {
                    if (!included[c])
                    {
                        included[c] = true;
                        newIncluded = true;
                        toInclude--;
                    }
                    else
                        newOverlapped.Add(c);
                }
                if (newIncluded)
                {
                    overSolution.Add(coveringSelection[routeIndex].Item1);
                    overlapped.AddRange(newOverlapped);
                }
                routeIndex++;
            }
            while (toInclude > 0);
            return new Tuple<RouteSet, List<int>>(overSolution, overlapped);
        }

        public RouteSet BuildSolution(List<Tuple<Route, double>> coveringSelection, Random rdObj)
        {
            var overSolution = BuildOverlappedSolution(coveringSelection);
            List<int> overlappedClients = overSolution.Item2;
            while (overlappedClients.Count > 0)
            {
                int clientIndex = rdObj.Next(overlappedClients.Count);
                int client = overlappedClients[clientIndex];
                overlappedClients.RemoveAt(clientIndex);

                List<Route> cRoutes = new List<Route>(from r in overSolution.Item1
                                                      where r.Contains(client)
                                                      select r);
                int routeIndex = rdObj.Next(cRoutes.Count);
                for (int i = 0; i < cRoutes.Count; i++)
                {
                    if (i != routeIndex)
                        cRoutes[i].Remove(client);
                }
            }
            return overSolution.Item1;
        }

        //public double[] GetDualVariables(List<Tuple<Route, double>> coveringSelection)
        //{
        //    //no gatantiza que sea solucion optima del dual
        //    double[] vars = new double[ProblemData.Clients.Count + 1];
        //    foreach (var item in coveringSelection)
        //    {
        //        double routePartialCost = GetCost(item.Item1) * item.Item2;
        //        foreach (var c in item.Item1)
        //        {
        //            vars[c] += routePartialCost / item.Item1.Count;
        //        }
        //    }
        //    return vars;
        //}

        public double[] GetExtremeDualVariables(string folderPath, List<ExtRouteInfo> pool, int dualVectorSize)
        {
            double[] dual = new double[dualVectorSize];
            if (pool.Count > 0)
            {
                ColumnSelectionGAMS gamsProcedure = new ColumnSelectionGAMS(ProblemData);
                var dualvalues = gamsProcedure.DualSolve(folderPath, pool);
                foreach (var item in dualvalues)
                    dual[item.Item1] = item.Item2;
            }
            return dual;
        }

        private List<Tuple<RouteSet, double>> DualParallelShaking<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, RouteSet bestSolution, RouteSet current, int iteration, Stopwatch timer, double[,] dualCost) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<Tuple<RouteSet, double>> parallelSolutions = new List<Tuple<RouteSet, double>>();
            VRPSimultaneousPickupDelivery dualProblem = ProblemData.GetDualReduceCostProblem(dualCost);
            PenalizationLocalSearch dualProcedure = new PenalizationLocalSearch(dualProblem);
            dualProcedure.OverloadFactor = OverloadFactor;

            for (int k = 0; k < shaking.Count; k++)
            {
                var shk = shaking[k](current, rdObj);
                var imp = procedure.Solve(parameters, shk, neighborhoods, expCondition, dualProcedure.GetCost, rdObj);/*para la construccion de las nuevas rutas se usan los costos reducidos*/
                parallelSolutions.Add(new Tuple<RouteSet, double>(imp, GetCost(imp)));
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, imp, (iteration + 1) * k, timer.Elapsed.TotalSeconds));/*iteration+1 pq las iteraciones comienzan en cero*/
            }

            return parallelSolutions;
        }

        private List<Tuple<RouteSet, double>> ParallelExeDualParallelShaking<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, RouteSet bestSolution, RouteSet current, int iteration, Stopwatch timer, double[,] dualCost) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            Tuple<RouteSet, double>[] parallelSolutions = new Tuple<RouteSet, double>[shaking.Count];
            VRPSimultaneousPickupDelivery dualProblem = ProblemData.GetDualReduceCostProblem(dualCost);
            PenalizationLocalSearch dualProcedure = new PenalizationLocalSearch(dualProblem);
            dualProcedure.OverloadFactor = OverloadFactor;

            Parallel.For(0, shaking.Count, i => {
                Random pRdObj = new Random((i + 1) * (i + 2) * Environment.TickCount);
                var shk = shaking[i](current, pRdObj);
                var imp = procedure.Solve(parameters, shk, neighborhoods, expCondition, dualProcedure.GetCost, pRdObj);/*para la construccion de las nuevas rutas se usan los costos reducidos*/
                parallelSolutions[i] = new Tuple<RouteSet, double>(imp, GetCost(imp));
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, imp, (iteration + 1) * i, timer.Elapsed.TotalSeconds));/*iteration+1 pq las iteraciones comienzan en cero*/
            });

            return parallelSolutions.ToList();
        }

        private List<ExtRouteInfo> BuildDualRestrictedPool<TProcedure, TParameters>(RouteSet initial, Exploration expCondition, Random rdObj,
                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                             List<Func<RouteSet, Random, RouteSet>> shaking,
                             int iteration, Stopwatch timer,
                             TProcedure procedure, TParameters parameters,
                             double[] dualvars, 
                             bool parallelexe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            double[,] dualCost = GetReducedCostEdges(dualvars);
            List<Tuple<RouteSet, double>> shkPool = (parallelexe)? ParallelExeDualParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, initial, initial, iteration, timer, dualCost)
                :DualParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, initial, initial, iteration, timer, dualCost);
            return BuildRestrictedPool(shkPool);
        }

        private List<Tuple<Route, double>> GetInitialCoveringSelection(RouteSet initial)
        {
            List<Tuple<Route, double>> masterProblem = new List<Tuple<Route, double>>();
            foreach (var r in initial)
            {
                masterProblem.Add(new Tuple<Route, double>(r, 1));
            }
            return masterProblem;
        }

        public List<ExtRouteInfo> GetInitialPool(RouteSet initial)
        {
            List<ExtRouteInfo> pool = new List<ExtRouteInfo>();
            foreach (var item in initial)
            {
                pool.Add(new ExtRouteInfo(item, GetCost(item)));
            }
            return pool;
        }

    }

}
