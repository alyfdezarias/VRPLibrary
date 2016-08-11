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
            return vndProcedure.Solve(new NullParameters(), initial, neighborhoods, expCondition, GetCost, rdObj);
        }

        public RouteSet RdVNDBasicPenalization(RouteSet initial, Exploration expCondition, Random rdObj, double overloadFactor,
                                         List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            OverloadFactor = overloadFactor;
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return vndProcedure.Solve(new NullParameters(), initial, neighborhoods, expCondition, GetCost, rdObj);
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

        protected RouteSet MultiStartPenalization<TProcedure, TParameters>(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               TProcedure procedure, TParameters parameters) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            return MultiStartPenalization(expCondition, rdObj, neighborhoods, procedure, parameters, initialPool);
        }

        private RouteSet MultiStartPenalization<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
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

        public RouteSet MultiStartPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartPenalization(alpha, reStarts, expCondition, rdObj, overloadFactor, neighborhoods, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDMultiStartPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartPenalization(alpha, reStarts, expCondition, rdObj, overloadFactor, neighborhoods, vndProcedure, new NullParameters());
        }

        public RouteSet MultiStartPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartPenalization(alpha, reStarts, expCondition, rdObj, overloadFactor, neighborhoods, saProcedure, saParameters);
        }

        public RouteSet MultiStartPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartPenalization(expCondition, rdObj, neighborhoods, saProcedure, saParameters, initialPool);
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
            RouteSet bestSolution = initial;
            double bestCost = GetCost(bestSolution);
            RouteSet current = initial;
            timer.Start();
            for (int i = 0; i < reStarts; i++)
            {
                current = shaking[rdObj.Next(shaking.Count)](current, rdObj);
                current = procedure.Solve(parameters, current, neighborhoods, expCondition, GetCost, rdObj);
                double currentCost = GetCost(current);
                if (currentCost < bestCost)
                {
                    bestCost = currentCost;
                    bestSolution = (RouteSet)current.Clone();
                }
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
            }
            timer.Stop();
            return bestSolution;

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

        protected RouteSet MultiStartShakingPenalization<TProcedure, TParameters>(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               TProcedure procedure, TParameters parameters) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            return MultiStartShakingPenalization(expCondition, rdObj, neighborhoods, shaking, procedure, parameters, initialPool);
        }

        private RouteSet MultiStartShakingPenalization<TProcedure, TParameters>(Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shaking, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            RouteSet bestSolution = null;
            double bestCost = int.MaxValue;
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var solution in initialPool)
            {
                RouteSet current = solution;
                foreach (var shk in shaking)
                {
                    current = shk(current, rdObj);
                    current = procedure.Solve(parameters, current, neighborhoods, expCondition, GetCost, rdObj);
                    double currentCost = GetCost(current);
                    if (currentCost < bestCost)
                    {
                        bestCost = currentCost;
                        bestSolution = (RouteSet)current.Clone();
                    }
                    OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                    i++;
                }
            }
            timer.Stop();
            return bestSolution;
        }

        public RouteSet MutiStartShakingPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartShakingPenalization(alpha, reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDMutiStartShakingPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartShakingPenalization(alpha, reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, vndProcedure, new NullParameters());
        }

        public RouteSet MutiStartShakingPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartShakingPenalization(alpha, reStarts, expCondition, rdObj, overloadFactor, neighborhoods, shaking, saProcedure, saParameters);
        }

        public RouteSet MutiStartShakingPenalization(double alpha, int reStarts, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shaking,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartShakingPenalization(expCondition, rdObj, neighborhoods, shaking, saProcedure, saParameters, initialPool);
        }

        #endregion

        #region Multi Start & Multi Shaking

        protected RouteSet MultiStartMultiShakingPenalization<TProcedure, TParameters>(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                               TProcedure procedure, TParameters parameters) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            return MultiStartMultiShakingPenalization(shakings, expCondition, rdObj, neighborhoods, shakingProcedures, procedure, parameters, initialPool);
        }

        private RouteSet MultiStartMultiShakingPenalization<TProcedure, TParameters>(int shakings, Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shakingProcedures, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            RouteSet bestSolution = null;
            double bestCost = int.MaxValue;
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var solution in initialPool)
            {
                RouteSet current = solution;
                for (int j = 0; j < shakings; j++)
                {
                    current = shakingProcedures[rdObj.Next(shakingProcedures.Count)](current, rdObj);
                    current = procedure.Solve(parameters, current, neighborhoods, expCondition, GetCost, rdObj);
                    double currentCost = GetCost(current);
                    if (currentCost < bestCost)
                    {
                        bestCost = currentCost;
                        bestSolution = (RouteSet)current.Clone();
                    }
                    OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                    i++;
                }
            }
            timer.Stop();
            return bestSolution;
        }

        public RouteSet MultiStartMultiShakingPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartMultiShakingPenalization(alpha, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, shakingProcedures, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDMultiStartMultiShakingPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartMultiShakingPenalization(alpha, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, shakingProcedures, vndProcedure, new NullParameters());
        }

        public RouteSet MultiStartMultiShakingPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                               SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartMultiShakingPenalization(alpha, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, shakingProcedures, saProcedure, saParameters);
        }

        public RouteSet MultiStartMultiShakingPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartMultiShakingPenalization(shakings, expCondition, rdObj, neighborhoods, shakingProcedures, saProcedure, saParameters, initialPool);
        }

        #endregion

        #region BiLevel Penalization

        protected RouteSet BiLevelPenalization<TProcedure, TParameters>
                                            (RouteSet initial, Exploration interExploration, Exploration intraExploration, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> interNeighborhoods,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> intraNeigborhoods,
                                             TProcedure procedure, TParameters parameters)
                                             where TProcedure : LocalSearchProcedure<RouteSet, TParameters>

        {
            OverloadFactor = overloadFactor;
            RouteSet bestSolution = initial;
            double bestCost = GetCost(bestSolution);
            OnIteration(new IterationEventArgs<RouteSet>(bestSolution));

            List<Func<RouteSet, Exploration, Random, RouteSet>> interAvailables = new List<Func<RouteSet, Exploration, Random, RouteSet>>();
            interAvailables.AddRange(interNeighborhoods);
            RouteSet current = (RouteSet)initial.Clone();
            double currentCost = GetCost(current);
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            while (interAvailables.Count > 0)
            {
                int neighborIndex = rdObj.Next(interAvailables.Count);
                RouteSet interNgh = interAvailables[neighborIndex](current, interExploration, rdObj);
                double interCost = GetCost(interNgh);
                if (interCost < currentCost)
                {
                    current = procedure.Solve(parameters, interNgh, intraNeigborhoods, intraExploration, GetCost, rdObj);
                    currentCost = GetCost(current);
                    interAvailables.Clear();
                    interAvailables.AddRange(interNeighborhoods);
                    if (currentCost < bestCost)
                    {
                        bestCost = currentCost;
                        bestSolution = (RouteSet)current.Clone();
                    }

                }
                else interAvailables.RemoveAt(neighborIndex);
                i++;
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
            }
            timer.Stop();
            return bestSolution;
        }

        public RouteSet BiLevelPenalization(RouteSet initial, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return BiLevelPenalization(initial, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDBiLevelPenalization(RouteSet initial, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return BiLevelPenalization(initial, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, vndProcedure, new NullParameters());
        }

        public RouteSet BiLevelPenalization(RouteSet initial, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2, 
                                             SimulatedAnnealingParameters saParameters)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return BiLevelPenalization(initial, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, saProcedure, saParameters);
        }

        public RouteSet BiLevelPenalization(RouteSet initial, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2,
                                             double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = overloadFactor;
            SimulatedAnnealingParameters saParameters = new SimulatedAnnealingParameters(GetCost(initial), saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
            return BiLevelPenalization(initial, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, saParameters);
        }
                
        #endregion

        #region Multi Start BiLevel Penalization

        protected RouteSet MultiStartBiLevelPenalization<TProcedure, TParameters>
                                            (double alpha, int reStarts, Exploration interExploration, Exploration intraExploration, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> interNeighborhoods,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> intraNeighborhoods,
                                             TProcedure procedure, TParameters parameters)
                                            where TProcedure : LocalSearchProcedure<RouteSet, TParameters>

        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            return MultiStartBiLevelPenalization(interExploration, intraExploration, rdObj, interNeighborhoods, intraNeighborhoods, procedure, parameters, initialPool);

        }

        private RouteSet MultiStartBiLevelPenalization<TProcedure, TParameters>(Exploration interExploration, Exploration intraExploration, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> interNeighborhoods, List<Func<RouteSet, Exploration, Random, RouteSet>> intraNeighborhoods, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            RouteSet bestSolution = initialPool[0];
            double bestCost = GetCost(bestSolution);
            OnIteration(new IterationEventArgs<RouteSet>(bestSolution));
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var item in initialPool)
            {
                List<Func<RouteSet, Exploration, Random, RouteSet>> interAvailables = new List<Func<RouteSet, Exploration, Random, RouteSet>>();
                interAvailables.AddRange(interNeighborhoods);
                RouteSet current = (RouteSet)item.Clone();
                double currentCost = GetCost(current);

                while (interAvailables.Count > 0)
                {
                    int neighborIndex = rdObj.Next(interAvailables.Count);
                    RouteSet interNgh = interAvailables[neighborIndex](current, interExploration, rdObj);
                    double interCost = GetCost(interNgh);
                    if (interCost < currentCost)
                    {
                        current = procedure.Solve(parameters, interNgh, intraNeighborhoods, intraExploration, GetCost, rdObj);
                        currentCost = GetCost(current);
                        interAvailables.Clear();
                        interAvailables.AddRange(interNeighborhoods);
                        if (currentCost < bestCost)
                        {
                            bestCost = currentCost;
                            bestSolution = (RouteSet)current.Clone();
                        }
                    }
                    else interAvailables.RemoveAt(neighborIndex);
                    i++;
                    OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                }
            }
            return bestSolution;
        }

        public RouteSet MutiStartBiLevelPenalization(double alpha, int reStarts, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartBiLevelPenalization(alpha, reStarts, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDMutiStartBiLevelPenalization(double alpha, int reStarts, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartBiLevelPenalization(alpha, reStarts, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, vndProcedure, new NullParameters());
        }

        public RouteSet MutiStartBiLevelPenalization(double alpha, int reStarts, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2, 
                                             SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartBiLevelPenalization(alpha, reStarts, expConditionL1, expConditionL2, rdObj, overloadFactor, neighborhoodsL1, neighborhoodsL2, saProcedure, saParameters);
        }

        public RouteSet MutiStartBiLevelPenalization(double alpha, int reStarts, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2,
                                             double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            return MultiStartBiLevelPenalization(expConditionL1, expConditionL2, rdObj, neighborhoodsL1, neighborhoodsL2, saProcedure, saParameters, initialPool);
        }
        #endregion

        #region Multi Start & Muti Shaking BiLevel Penalization

        protected RouteSet MultiStartMultiShakingBiLevelPenalization<TProcedure, TParameters>
                                            (double alpha, int reStarts, int shakings, Exploration interExploration, Exploration intraExploration, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> interNeighborhoods,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> intraNeighborhoods,
                                             List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                             TProcedure procedure, TParameters parameters)
                                            where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            return MultiStartMultiShakingBiLevelPenalization(shakings, interExploration, intraExploration, rdObj, interNeighborhoods, intraNeighborhoods, shakingProcedures, procedure, parameters, initialPool);

        }

        private RouteSet MultiStartMultiShakingBiLevelPenalization<TProcedure, TParameters>(int shakings, Exploration interExploration, Exploration intraExploration, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> interNeighborhoods, List<Func<RouteSet, Exploration, Random, RouteSet>> intraNeighborhoods, List<Func<RouteSet, Random, RouteSet>> shakingProcedures, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            RouteSet bestSolution = initialPool[0];
            double bestCost = GetCost(bestSolution);
            OnIteration(new IterationEventArgs<RouteSet>(bestSolution));
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var item in initialPool)
            {
                List<Func<RouteSet, Exploration, Random, RouteSet>> interAvailables = new List<Func<RouteSet, Exploration, Random, RouteSet>>();
                interAvailables.AddRange(interNeighborhoods);
                RouteSet current = item;
                double currentCost = GetCost(current);

                for (int s = 0; s < shakings; s++)
                {
                    current = shakingProcedures[rdObj.Next(shakingProcedures.Count)](current, rdObj);
                    while (interAvailables.Count > 0)
                    {
                        int neighborIndex = rdObj.Next(interAvailables.Count);
                        RouteSet interNgh = interAvailables[neighborIndex](current, interExploration, rdObj);
                        double interCost = GetCost(interNgh);
                        if (interCost < currentCost)
                        {
                            current = procedure.Solve(parameters, interNgh, intraNeighborhoods, intraExploration, GetCost, rdObj);
                            currentCost = GetCost(current);
                            interAvailables.Clear();
                            interAvailables.AddRange(interNeighborhoods);
                            if (currentCost < bestCost)
                            {
                                bestCost = currentCost;
                                bestSolution = (RouteSet)current.Clone();
                            }
                        }
                        else interAvailables.RemoveAt(neighborIndex);
                        i++;
                        OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                    }
                }
            }
            return bestSolution;
        }


        public RouteSet MultiStartMultiShakingBiLevelPenalization(double alpha, int reStarts, int shakings, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2,
                                             List<Func<RouteSet, Random, RouteSet>> shakingProcedures)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartMultiShakingBiLevelPenalization(alpha, reStarts, shakings, expConditionL1, expConditionL2, rdObj, overloadFactor,
                                                             neighborhoodsL1, neighborhoodsL2, shakingProcedures, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDMultiStartMultiShakingBiLevelPenalization(double alpha, int reStarts, int shakings, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                             List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2,
                                             List<Func<RouteSet, Random, RouteSet>> shakingProcedures)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartMultiShakingBiLevelPenalization(alpha, reStarts, shakings, expConditionL1, expConditionL2, rdObj, overloadFactor,
                                                             neighborhoodsL1, neighborhoodsL2, shakingProcedures, vndProcedure, new NullParameters());
        }

        public RouteSet MultiStartMultiShakingBiLevelPenalization(double alpha, int reStarts, int shakings, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2,
                                            List<Func<RouteSet, Random, RouteSet>> shakingProcedures, 
                                            SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartMultiShakingBiLevelPenalization(alpha, reStarts, shakings, expConditionL1, expConditionL2, rdObj, overloadFactor,
                                                             neighborhoodsL1, neighborhoodsL2, shakingProcedures, saProcedure, saParameters);
        }

        public RouteSet MultiStartMultiShakingBiLevelPenalization(double alpha, int reStarts, int shakings, Exploration expConditionL1, Exploration expConditionL2, Random rdObj, double overloadFactor,
                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL1,
                                            List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoodsL2,
                                            List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                            double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            OverloadFactor = overloadFactor;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            return MultiStartMultiShakingBiLevelPenalization(shakings, expConditionL1, expConditionL2, rdObj, 
                                                            neighborhoodsL1, neighborhoodsL2, shakingProcedures, saProcedure, saParameters,initialPool);
        }

        #endregion

        #region Multi Start & Double Shaking Penalization

        protected RouteSet MultiStartMultiShakingDoubleSearchPenalization<TProcedure, TParameters>(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                               string gamsFolderPath,
                                               TProcedure procedure, TParameters parameters) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            OverloadFactor = 0;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            return MultiStartMultiShakingDoubleSearchPenalization(overloadFactor,shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, procedure, parameters, initialPool);
        }

        private RouteSet MultiStartMultiShakingDoubleSearchPenalization<TProcedure, TParameters>(double overloadFactor, int shakings, Exploration expCondition, Random rdObj, List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods, List<Func<RouteSet, Random, RouteSet>> shakingProcedures, string gamsFolderPath, TProcedure procedure, TParameters parameters, List<RouteSet> initialPool) where TProcedure : LocalSearchProcedure<RouteSet, TParameters>
        {
            RouteSet bestSolution = initialPool[0];
            double bestCost = GetCost(bestSolution);
            SingleRouteSPDGAMS gamsProcedure = new SingleRouteSPDGAMS(ProblemData);
            DirectoryInfo gamsDir = Directory.CreateDirectory(gamsFolderPath);
            int i = 0;
            Stopwatch timer = new Stopwatch();
            timer.Start();
            foreach (var solution in initialPool)
            {
                RouteSet current = solution;
                double currentCost = 0;
                for (int j = 0; j < shakings; j++)
                {
                    current = shakingProcedures[rdObj.Next(shakingProcedures.Count)](current, rdObj);
                    current = procedure.Solve(parameters, current, neighborhoods, expCondition, GetCost, rdObj);
                    currentCost = GetCost(current);
                    if (currentCost < bestCost)
                    {
                        bestCost = currentCost;
                        bestSolution = (RouteSet)current.Clone();
                    }
                    if (ProblemData.IsFeasible(current))
                    {
                        DirectoryInfo gamsIteration = gamsDir.CreateSubdirectory(string.Format("{0}-{1}",i.ToString(), j.ToString()));
                        current = gamsProcedure.Solve(gamsIteration.FullName, current);
                        currentCost = GetCost(current);
                        if (currentCost < bestCost)
                        {
                            bestCost = currentCost;
                            bestSolution = (RouteSet)current.Clone();
                        }
                    }
                    else
                    {
                        OverloadFactor += overloadFactor;
                        bestCost = GetCost(bestSolution);
                    }
                }
                OnIteration(new IterationEventArgs<RouteSet>(bestSolution, current, i, timer.Elapsed.TotalSeconds));
                i++;
            }
            timer.Stop();
            return bestSolution;
        }


        public RouteSet MultiStartMultiShakingDoubleSearchPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures, string gamsFolderPath)
        {
            VariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new VariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartMultiShakingDoubleSearchPenalization(alpha, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, shakingProcedures, gamsFolderPath, vndProcedure, new NullParameters());
        }

        public RouteSet RdVNDMultiStartDoubleShakingPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures, string gamsFolderPath)
        {
            RandomVariableNeighborhoodDescentProcedure<RouteSet> vndProcedure = new RandomVariableNeighborhoodDescentProcedure<RouteSet>();
            return MultiStartMultiShakingDoubleSearchPenalization(alpha, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, shakingProcedures, gamsFolderPath, vndProcedure, new NullParameters());
        }

        public RouteSet MultiStartMultiShakingDoubleSearchPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures,
                                               string gamsFolderPath,
                                               SimulatedAnnealingParameters saParameters)
        {
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartMultiShakingDoubleSearchPenalization(alpha, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, shakingProcedures, gamsFolderPath, saProcedure, saParameters);
        }

        public RouteSet MultiStartMultiShakingDoubleSearchPenalization(double alpha, int reStarts, int shakings, Exploration expCondition, Random rdObj, double overloadFactor,
                                               List<Func<RouteSet, Exploration, Random, RouteSet>> neighborhoods,
                                               List<Func<RouteSet, Random, RouteSet>> shakingProcedures, string gamsFolderPath,
                                               double saProbability, double saWorst, int saRepetitions, double saFactor, double saCoolingRate, int saCoolerAmount)
        {
            OverloadFactor = 0;
            List<RouteSet> initialPool = GenerateGreedyRdPool(alpha, rdObj, reStarts);
            SimulatedAnnealingParameters saParameters = ComputeSAParameters(saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount, initialPool);
            SimulatedAnnealingProcedure<RouteSet> saProcedure = new SimulatedAnnealingProcedure<RouteSet>();
            return MultiStartMultiShakingDoubleSearchPenalization(overloadFactor, shakings, expCondition, rdObj, neighborhoods, shakingProcedures, gamsFolderPath, saProcedure, saParameters, initialPool);
        }

        #endregion

    }
}
