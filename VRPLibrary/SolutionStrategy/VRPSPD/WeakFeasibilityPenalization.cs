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
    public class WeakFeasibilityPenalization : PenalizationLocalSearch
    {
        public double WeakThreshold { get; set; }

        public WeakFeasibilityPenalization(VRPSimultaneousPickupDelivery problemData) : base(problemData)
        {
            WeakThreshold = 0.2;
        }

        #region Weak Feasibility Threshold

        public double AlphaWeakFeasibility(Route current)
        {
            double maxDemand = Math.Max(ProblemData.TotalPickup(current), ProblemData.TotalDelivery(current));
            if (maxDemand <= current.Vehicle.Capacity) return 0;
            return (maxDemand - current.Vehicle.Capacity) / current.Vehicle.Capacity;
        }

        public override bool IsAllowedMovement(InterMove m)
        {
            double maxDemand = Math.Max(ProblemData.TotalPickup(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Pickup, ProblemData.TotalDelivery(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Delivery);
            if (maxDemand <= m.deRoute.Vehicle.Capacity) return true;
            return (maxDemand - m.deRoute.Vehicle.Capacity) / m.deRoute.Vehicle.Capacity < WeakThreshold;
        }

        public override bool IsAllowedMovement(InterSwap m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, new List<int> { m.current[m.orIndex] }, new List<int> { m.deRoute[m.deIndex] }) < WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, new List<int> { m.deRoute[m.deIndex] }, new List<int> { m.current[m.orIndex] }) < WeakThreshold;
        }

        protected double AlphaWeakFeasibilityAfterReplace(Route current, List<int> oldRange, List<int> newRange)
        {
            double pickup = ProblemData.TotalPickup(current) - oldRange.Sum(c => ProblemData.Clients[c].Pickup) + newRange.Sum(c => ProblemData.Clients[c].Pickup);
            double delivery = ProblemData.TotalDelivery(current) - oldRange.Sum(c => ProblemData.Clients[c].Delivery) + newRange.Sum(c => ProblemData.Clients[c].Delivery);
            double maxDemand = Math.Max(pickup, delivery);
            if (maxDemand <= current.Vehicle.Capacity) return 0;
            return (maxDemand - current.Vehicle.Capacity) / current.Vehicle.Capacity;
        }

        public override bool IsAllowedMovement(CrossoverRoute m)
        {
            return AlphaWeakFeasibilityAfterReplaceRange(m.current, m.deRoute, m.orIndex, m.deIndex) < WeakThreshold &&
                AlphaWeakFeasibilityAfterReplaceRange(m.deRoute, m.current, m.deIndex, m.orIndex) < WeakThreshold;
        }

        protected double AlphaWeakFeasibilityAfterReplaceRange(Route orRoute, Route deRoute, int orIndex, int deIndex)
        {
            double orDelivery = GetReplaceRangeStartLoad(orRoute, deRoute, orIndex, deIndex);
            double orPickup = GetReplaceRangeEndLoad(orRoute, deRoute, orIndex, deIndex);
            double maxDemand = Math.Max(orDelivery, orPickup);
            if (maxDemand <= orRoute.Vehicle.Capacity) return 0;
            return (maxDemand - orRoute.Vehicle.Capacity) / orRoute.Vehicle.Capacity;
        }

        protected double GetReplaceRangeStartLoad(Route orRoute, Route deRoute, int orIndex, int deIndex)
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

        protected double GetReplaceRangeEndLoad(Route orRoute, Route deRoute, int orIndex, int deIndex)
        {
            double load = 0;
            for (int i = 0; i <= orIndex; i++)
                load += ProblemData.Clients[orRoute[i]].Pickup;
            for (int i = deIndex + 1; i < deRoute.Count; i++)
                load += ProblemData.Clients[deRoute[i]].Pickup;
            return load;
        }

        public override bool IsAllowedMovement(ReverseCrossoverRoute m)
        {
            return IsAllowedMovement((CrossoverRoute)m);
        }

        public override bool IsAllowedMovement(TwoInterMove m)
        {
            double maxDemand = Math.Max(ProblemData.TotalPickup(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Pickup + ProblemData.Clients[m.current[m.orIndex+1]].Pickup, 
                ProblemData.TotalDelivery(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Delivery + ProblemData.Clients[m.current[m.orIndex+1]].Delivery);
            if (maxDemand <= m.deRoute.Vehicle.Capacity) return true;
            return (maxDemand - m.deRoute.Vehicle.Capacity) / m.deRoute.Vehicle.Capacity < WeakThreshold;
        }

        public override bool IsAllowedMovement(TwoOneInterSwap m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, m.current.GetRange(m.orIndex, 2), new List<int> { m.deRoute[m.deIndex] }) < WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, new List<int> { m.deRoute[m.deIndex] }, m.current.GetRange(m.orIndex, 2)) < WeakThreshold;
        }

        public override bool IsAllowedMovement(TwoTwoInterSwap m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, m.current.GetRange(m.orIndex, 2), m.deRoute.GetRange(m.deIndex, 2)) < WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex, 2), m.current.GetRange(m.orIndex, 2)) < WeakThreshold;
        }

        #endregion

    }
}
