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
    public class StrongFeasibleLocalSearch: PenalizationLocalSearch
    {
        //public double StrongThreshold { get; set; }
        protected double epsilon = 0.0001;
        public StrongFeasibleLocalSearch(VRPSimultaneousPickupDelivery problemData)
            : base(problemData)
        {
            //StrongThreshold = 0;
        }

        #region Strong Feasibility

        //public double AlphaStrongFeasibility(Route current)
        //{
        //    double overload = ProblemData.StrongOverLoad(current);
        //    if (overload <= current.Vehicle.Capacity) return 0;
        //    return (overload - current.Vehicle.Capacity) / current.Vehicle.Capacity;
        //}

        public override bool IsAllowedMovement(IntraMove m)
        {
            return ProblemData.StrongIntraReplaceOverload(m.current, m.orIndex, m.deIndex) <= epsilon;
        }

        public override bool IsAllowedMovement(IntraSwap m)
        {
            return ProblemData.StrongIntraSwapOverload(m.current, m.orIndex, m.deIndex) <= epsilon;
        }

        public override bool IsAllowedMovement(TwoOpt m)
        {
            return TwoOptStrongOverload(m) <= epsilon;
        }

        public override bool IsAllowedMovement(InterMove m)
        {
            return ProblemData.StrongAddOverload(m.deRoute, m.deIndex, new List<int> { m.current[m.orIndex] })<= epsilon;
        }

        public override bool IsAllowedMovement(InterSwap m)
        {
            if (m.deRoute.IsEmpty)
                return Math.Max(ProblemData.Clients[m.current[m.orIndex]].Delivery, ProblemData.Clients[m.current[m.orIndex]].Pickup) <= m.deRoute.Vehicle.Capacity;
            return ProblemData.StrongReplaceOverload(m.deRoute, m.deIndex, 1, new List<int> { m.current[m.orIndex] }) <= epsilon;
        }

        public override bool IsAllowedMovement(TwoInterMove m)
        {
            return ProblemData.StrongAddOverload(m.deRoute, m.deIndex, m.current.GetRange(m.orIndex, 2)) <= epsilon;
        }

        public override bool IsAllowedMovement(TwoOneInterSwap m)
        {
            return ProblemData.StrongReplaceOverload(m.current, m.orIndex, 2, new List<int> { m.deRoute[m.deIndex] }) <= epsilon &&
                ProblemData.StrongReplaceOverload(m.deRoute, m.deIndex, 1, m.current.GetRange(m.orIndex, 2)) <= epsilon;
        }

        public override bool IsAllowedMovement(TwoTwoInterSwap m)
        {
            return ProblemData.StrongReplaceOverload(m.current, m.orIndex, 2, m.deRoute.GetRange(m.deIndex, 2)) <= epsilon &&
                ProblemData.StrongReplaceOverload(m.deRoute, m.deIndex, 2, m.current.GetRange(m.orIndex, 2)) <= epsilon;
        }

        public override bool IsAllowedMovement(CrossoverRoute m)
        {
            return ReplaceRangeOverload(m.current, m.deRoute, m.orIndex, m.deIndex) <= epsilon &&
                ReplaceRangeOverload(m.deRoute, m.current, m.deIndex, m.orIndex) <= epsilon;
        }

        public override bool IsAllowedMovement(ReverseCrossoverRoute m)
        {
            return ReplaceReverseRangeOverload(m.current, m.deRoute, m.orIndex, m.deIndex) <= epsilon &&
                ReplaceReverseRangeOverload(m.deRoute, m.current, m.deIndex, m.orIndex) <= epsilon;
        }
        #endregion


    }
}
