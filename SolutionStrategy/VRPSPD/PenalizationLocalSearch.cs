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
    public class PenalizationLocalSearch : VRPSPDLocalSearch
    {
        public PenalizationLocalSearch(VRPSimultaneousPickupDelivery problemData) : base(problemData) { }

        #region Basic Penalization
        public RouteSet BasicPenalization(RouteSet initial, Exploration expCondition, Random rdObj, double overloadFactor,
                                         List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            OverloadFactor = overloadFactor;
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return vndProcedure.Solve(initial, neighborhoods, expCondition, GetCost, rdObj);
        }

        public RouteSet RdVNDBasicPenalization(RouteSet initial, Exploration expCondition, Random rdObj, double overloadFactor,
                                         List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            OverloadFactor = overloadFactor;
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return vndProcedure.Solve(initial, neighborhoods, expCondition, GetCost, rdObj);
        }

        public RouteSet BasicPenalization(RouteSet initial, Exploration expCondition, Random rdObj, double overloadFactor,
                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                            SimulatedAnnealingParameters saParameters)
        {
            OverloadFactor = overloadFactor;
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return saProcedure.Solve(saParameters, initial, neighborhoods, expCondition, GetCost, rdObj);

        }

        public RouteSet BasicPenalization(RouteSet initial, Exploration expCondition, Random rdObj, double overloadFactor,
                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                            double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            SimulatedAnnealingParameters saParameters = new SimulatedAnnealingParameters(GetCost(initial), saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
            return BasicPenalization(initial, expCondition, rdObj, overloadFactor, neighborhoods, saParameters);
        }

        #endregion

        #region Multi Start Penalization

        protected RouteSet MultiStartPenalization<TProcedure, TParameters>(Exploration expCondition, Random rdObj, double overloadFactor, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            Stopwatch timer = new Stopwatch();
            RouteSet bestSolution = initialPool[0];
            double bestCost = GetCost(bestSolution);
            int i = 0;
            timer.Start();
            foreach (var item in initialPool)
            {
                RouteSet current = procedure.Solve(parameters, item, neighborhoods, expCondition, GetCost, rdObj);
                double currentCost = GetCost(current);
                if (currentCost < bestCost)
                {
                    bestCost = currentCost;
                    bestSolution = (RouteSet)current.Clone();
                }
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                i++;
            }
            timer.Stop();
            return bestSolution;
        }

        protected RouteSet ParallelMultiStartPenalization<TProcedure, TParameters>(Exploration expCondition, Random rdObj, double overloadFactor, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            Stopwatch timer = new Stopwatch();
            Tuple<RouteSet, double>[] solutions = new Tuple<RouteSet, double>[initialPool.Count];

            Parallel.For(0, initialPool.Count, i => {
                Random pRdObj = new Random((i+1) * (i+2) * Environment.TickCount);
                RouteSet current = procedure.Solve(parameters, initialPool[i], neighborhoods, expCondition, GetCost, pRdObj);
                double cost = GetCost(current);
                solutions[i] = new Tuple<RouteSet, double>(current, cost);
            });

            return solutions.Aggregate((min, x) => min.Item2 < x.Item2 ? min : x).Item1;
        }

        public RouteSet MultiStartPenalization(List<RouteSet> initialPool, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartPenalization(expCondition, rdObj, overloadFactor, neighborhoods, vndProcedure, new NullParameters(), initialPool);
        }

        public RouteSet ParallelMultiStartPenalization(List<RouteSet> initialPool, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return ParallelMultiStartPenalization(expCondition, rdObj, overloadFactor, neighborhoods, vndProcedure, new NullParameters(), initialPool);
        }


        public RouteSet RdVNDMultiStartPenalization(List<RouteSet> initialPool, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartPenalization(expCondition, rdObj, overloadFactor, neighborhoods, vndProcedure, new NullParameters(), initialPool);
        }

        public RouteSet ParallelRdVNDMultiStartPenalization(List<RouteSet> initialPool, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return ParallelMultiStartPenalization(expCondition, rdObj, overloadFactor, neighborhoods, vndProcedure, new NullParameters(), initialPool);
        }

        public RouteSet MultiStartPenalization(List<RouteSet> initialPool, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartPenalization(expCondition, rdObj, overloadFactor, neighborhoods, saProcedure, saParameters, initialPool);
        }

        public RouteSet MultiStartPenalization(List<RouteSet> initialPool, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartPenalization(expCondition, rdObj, overloadFactor, neighborhoods, saProcedure, saParameters, initialPool);
        }

        protected SimulatedAnnealingParameters ComputeSAParameters(double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount, List<RouteSet> initialPool)
        {
            List<double> tmp0 = new List<double>();
            List<double> tmpn = new List<double>();
            foreach (var item in initialPool)
            {
                double t0 = SimulatedAnnealingParameters.ComputeInitialTemperature(GetCost(item), saProbability, saWorst);
                tmp0.Add(t0);
                tmpn.Add(SimulatedAnnealingParameters.ComputeFinalTemperature(t0, saCoolingRate, saCoolerAmount));
            }
            return new SimulatedAnnealingParameters(tmp0.Average(), tmpn.Average(), saRepetitions, saFactor, saCoolingRate);
        }

        public List<RouteSet> GenerateGreedyRdPool(double alpha, Random rdObj, int poolSize)
        {
            List<Tuple<RouteSet, double>> pool = new List<Tuple<RouteSet, double>>();
            while (pool.Count < poolSize)
            {
                RouteSet current = BuildGreedyRandomizedSolution(rdObj, alpha);
                if (!IsInPool(current, pool))
                    pool.Add(new Tuple<RouteSet, double>(current, GetTravelCost(current)));
            }
            return new List<RouteSet>(pool.Select(x => x.Item1));

        }

        protected bool IsInPool(RouteSet current, List<Tuple<RouteSet, double>> pool)
        {
            double cost = GetTravelCost(current);
            foreach (var item in pool)
                if (cost == item.Item2) return true;
            return false;
        }

        #endregion

        #region Shaking

        protected RouteSet ShakingPenalization<TProcedure, TParameters>(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               TProcedure procedure, TParameters parameters) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            Stopwatch timer = new Stopwatch();
            RouteSet bestSolution = procedure.Solve(parameters, initial, neighborhoods, expCondition, GetCost, rdObj);
            double bestCost = GetCost(bestSolution);
            RouteSet current = (RouteSet)bestSolution.Clone();
            timer.Start();
            for (int i = 0; i < reStarts; i++)
            {
                SimpleShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, ref bestSolution, ref bestCost, ref current);
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
            }
            timer.Stop();
            return bestSolution;

        }

        private void SimpleShaking<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, ref RouteSet bestSolution, ref double bestCost, ref RouteSet current) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            current = shaking[rdObj.Next(shaking.Count)](current, rdObj);
            current = procedure.Solve(parameters, current, neighborhoods, expCondition, GetCost, rdObj);
            double currentCost = GetCost(current);
            if (currentCost < bestCost)
            {
                bestCost = currentCost;
                bestSolution = (RouteSet)current.Clone();
            }
        }

        public RouteSet ShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return ShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return ShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, vndProcedure, new NullParameters());
        }

        public RouteSet ShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking,
                                              SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return ShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, saProcedure, saParameters);
        }

        public RouteSet ShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking,
                                              double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            OverloadFactor = overloadFactor;
            SimulatedAnnealingParameters saParameters = new SimulatedAnnealingParameters(GetCost(initial), saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
            return ShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, saProcedure, saParameters);
        }
        #endregion

        #region Multi Start & Shaking

        protected RouteSet MultiStartShakingPenalization<TProcedure, TParameters>(int restarts, Exploration expCondition, Random rdObj, double overloadFactor, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            RouteSet bestSolution = initialPool[0];
            double bestCost = GetCost(bestSolution);
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var solution in initialPool)
            {
                RouteSet current = procedure.Solve(parameters, solution, neighborhoods, expCondition, GetCost, rdObj);
                for (int j = 0; j < restarts; j++)
                {
                    SimpleShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, ref bestSolution, ref bestCost, ref current);
                    OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                    i++;
                }
            }
            timer.Stop();
            return bestSolution;
        }

        public RouteSet MutiStartShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, vndProcedure, new NullParameters(), initialPool);
        }

        public RouteSet RdVNDMutiStartShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, vndProcedure, new NullParameters(), initialPool);
        }

        public RouteSet MutiStartShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, saProcedure, saParameters, initialPool);
        }

        public RouteSet MutiStartShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, saProcedure, saParameters, initialPool);
        }

        #endregion

        #region Parallel Shaking

        protected RouteSet ParallelShakingPenalization<TProcedure, TParameters>(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               TProcedure procedure, TParameters parameters,
                                               bool parallelExe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            Stopwatch timer = new Stopwatch();
            RouteSet bestSolution = procedure.Solve(parameters, initial, neighborhoods, expCondition, GetCost, rdObj);
            double bestCost = GetCost(bestSolution);
            RouteSet current = (RouteSet)bestSolution.Clone();
            timer.Start();
            for (int i = 0; i < reStarts; i++)
            {
                ParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, ref bestSolution, ref bestCost, ref current, i, timer, parallelExe);
                //OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
            }
            timer.Stop();
            return bestSolution;
        }

        private void ParallelShaking<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, ref RouteSet bestSolution, ref double bestCost, ref RouteSet current, int iteration, Stopwatch timer, bool parallelExe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<Tuple<RouteSet, double>> parallelSolutions = (parallelExe)? ParallelExeParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, bestSolution, current, iteration, timer) : ParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, bestSolution, current, iteration, timer);
            var ccurrent = parallelSolutions.Aggregate((min, x) => (min.Item2 < x.Item2) ? min : x);
            current = ccurrent.Item1;
            if (ccurrent.Item2 < bestCost)
            {
                bestCost = ccurrent.Item2;
                bestSolution = (RouteSet)current.Clone();
            }
            OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, (iteration + 1) * (shaking.Count), timer.Elapsed.TotalSeconds));/*iteration+1 pq las iteraciones comienzan en cero*/
        }

        protected List<Tuple<RouteSet, double>> ParallelShaking<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, RouteSet bestSolution, RouteSet current, int iteration, Stopwatch timer) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            List<Tuple<RouteSet, double>> parallelSolutions = new List<Tuple<RouteSet, double>>();
            for (int k = 0; k < shaking.Count; k++)
            {
                var shk = shaking[k](current, rdObj);
                var imp = procedure.Solve(parameters, shk, neighborhoods, expCondition, GetCost, rdObj);
                parallelSolutions.Add(new Tuple<RouteSet, double>(imp, GetCost(imp)));
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, imp, (iteration + 1) * k, timer.Elapsed.TotalSeconds));/*iteration+1 pq las iteraciones comienzan en cero*/
            }

            return parallelSolutions;
        }

        protected List<Tuple<RouteSet, double>> ParallelExeParallelShaking<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, RouteSet bestSolution, RouteSet current, int iteration, Stopwatch timer) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            Tuple<RouteSet, double>[] parallelSolutions = new Tuple<RouteSet, double>[shaking.Count];
            Parallel.For(0, shaking.Count, i => {
                var shk = shaking[i](current, rdObj);
                Random prdObj = new Random((i + 1) * (i + 2) * Environment.TickCount);
                var imp = procedure.Solve(parameters, shk, neighborhoods, expCondition, GetCost, prdObj);
                parallelSolutions[i] = new Tuple<RouteSet, double>(imp, GetCost(imp));
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, imp, (iteration + 1) * i, timer.Elapsed.TotalSeconds));/*iteration+1 pq las iteraciones comienzan en cero*/
            });
            return parallelSolutions.ToList();
        }

        public RouteSet ParallelShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return ParallelShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, vndProcedure, new NullParameters(), false);
        }

        public RouteSet RdVNDParallelShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return ParallelShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, vndProcedure, new NullParameters(), false);
        }

        public RouteSet ParallelExeParallelShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return ParallelShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, vndProcedure, new NullParameters(), true);
        }

        public RouteSet ParallelExeRdVNDParallelShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return ParallelShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, vndProcedure, new NullParameters(), true);
        }

        public RouteSet ParallelShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking,
                                              SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return ParallelShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, saProcedure, saParameters, false);
        }

        public RouteSet ParallelShakingPenalization(RouteSet initial, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                              List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                              List<Func<RouteSet, Random, RouteSet>> shaking,
                                              double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            OverloadFactor = overloadFactor;
            SimulatedAnnealingParameters saParameters = new SimulatedAnnealingParameters(GetCost(initial), saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
            return ParallelShakingPenalization(initial, reStarts, expCondition, rdObj, overloadFactor,
                                       neighborhoods, shaking, saProcedure, saParameters, false);
        }


        #endregion

        #region Multi Start & Parallel Shaking

        protected RouteSet MultiStartParallelShakingPenalization<TProcedure, TParameters>(int restarts, Exploration expCondition, Random rdObj, double overloadFactor, 
            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool, bool parallelExe) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            RouteSet bestSolution = initialPool[0];
            double bestCost = GetCost(bestSolution);
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var solution in initialPool)
            {
                RouteSet current = procedure.Solve(parameters, solution, neighborhoods, expCondition, GetCost, rdObj);
                for (int j = 0; j < restarts; j++)
                {
                    ParallelShaking(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, ref bestSolution, ref bestCost, ref current, i, timer, parallelExe);
                    //OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                    i++;
                }
            }
            timer.Stop();
            return bestSolution;
        }

        public RouteSet MutiStartParallelShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartParallelShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, vndProcedure, new NullParameters(), initialPool, false);
        }

        public RouteSet RdVNDMutiStartParallelShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartParallelShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, vndProcedure, new NullParameters(), initialPool, false);
        }

        public RouteSet MutiStartParallelShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartParallelShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, saProcedure, saParameters, initialPool, false);
        }

        public RouteSet MutiStartParallelShakingPenalization(List<RouteSet> initialPool, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartParallelShakingPenalization(reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, saProcedure, saParameters, initialPool, false);
        }

        #endregion

    }
}
