using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;

using VRPLibrary.ProblemData;
using VRPLibrary.RouteSetData;

namespace VRPLibrary.SolutionStrategy.VRPSPD
{
    public class alphaWeakFeasibilityPenalization : PenalizationLocalSearch
    {
        public double WeakThreshold { get; set; }

        public alphaWeakFeasibilityPenalization(VRPSimultaneousPickupDelivery problemData) : base(problemData)
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

        public override bool IsAllowedMovement(IntraMove m)
        {
            return AlphaWeakFeasibility(m.current) <= WeakThreshold;
        }

        public override bool IsAllowedMovement(IntraSwap m)
        {
            return AlphaWeakFeasibility(m.current) <= WeakThreshold;
        }

        public override bool IsAllowedMovement(TwoOpt m)
        {
            return AlphaWeakFeasibility(m.current) <= WeakThreshold;
        }

        public override bool IsAllowedMovement(InterMove m)
        {
            double maxDemand = Math.Max(ProblemData.TotalPickup(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Pickup, ProblemData.TotalDelivery(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Delivery);
            if (maxDemand <= m.deRoute.Vehicle.Capacity) return true;
            return (maxDemand - m.deRoute.Vehicle.Capacity) / m.deRoute.Vehicle.Capacity <= WeakThreshold;
        }

        public override bool IsAllowedMovement(InterSwap m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, new List<int> { m.current[m.orIndex] }, new List<int> { m.deRoute[m.deIndex] }) <= WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, new List<int> { m.deRoute[m.deIndex] }, new List<int> { m.current[m.orIndex] }) <= WeakThreshold;
        }

        protected double AlphaWeakFeasibilityAfterReplace(Route current, List<int> oldRange, List<int> newRange)
        {
            double maxDemand = MaxDemandAfterReplace(current, oldRange, newRange);
            if (maxDemand <= current.Vehicle.Capacity) return 0;
            return (maxDemand - current.Vehicle.Capacity) / current.Vehicle.Capacity;
        }

        protected double MaxDemandAfterReplace(Route current, List<int> oldRange, List<int> newRange)
        {
            double pickup = ProblemData.TotalPickup(current) - oldRange.Sum(c => ProblemData.Clients[c].Pickup) + newRange.Sum(c => ProblemData.Clients[c].Pickup);
            double delivery = ProblemData.TotalDelivery(current) - oldRange.Sum(c => ProblemData.Clients[c].Delivery) + newRange.Sum(c => ProblemData.Clients[c].Delivery);
            double maxDemand = Math.Max(pickup, delivery);
            return maxDemand;
        }

        public override bool IsAllowedMovement(CrossoverRoute m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, m.current.GetRange(m.orIndex + 1, m.current.Count - m.orIndex - 1), m.deRoute.GetRange(m.deIndex + 1, m.deRoute.Count - m.deIndex - 1)) <= WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex + 1, m.deRoute.Count - m.deIndex - 1), m.current.GetRange(m.orIndex + 1, m.current.Count - m.orIndex - 1)) <= WeakThreshold;
        }

        public override bool IsAllowedMovement(ReverseCrossoverRoute m)
        {
            return IsAllowedMovement((CrossoverRoute)m);
        }

        public override bool IsAllowedMovement(TwoInterMove m)
        {
            double maxDemand = Math.Max(ProblemData.TotalPickup(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Pickup + ProblemData.Clients[m.current[m.orIndex + 1]].Pickup,
                ProblemData.TotalDelivery(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Delivery + ProblemData.Clients[m.current[m.orIndex + 1]].Delivery);
            if (maxDemand <= m.deRoute.Vehicle.Capacity) return true;
            return (maxDemand - m.deRoute.Vehicle.Capacity) / m.deRoute.Vehicle.Capacity <= WeakThreshold;
        }

        public override bool IsAllowedMovement(TwoOneInterSwap m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, m.current.GetRange(m.orIndex, 2), new List<int> { m.deRoute[m.deIndex] }) <= WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, new List<int> { m.deRoute[m.deIndex] }, m.current.GetRange(m.orIndex, 2)) <= WeakThreshold;
        }

        public override bool IsAllowedMovement(TwoTwoInterSwap m)
        {
            return AlphaWeakFeasibilityAfterReplace(m.current, m.current.GetRange(m.orIndex, 2), m.deRoute.GetRange(m.deIndex, 2)) <= WeakThreshold &&
                AlphaWeakFeasibilityAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex, 2), m.current.GetRange(m.orIndex, 2)) <= WeakThreshold;
        }

        #endregion

    }

    public class WeakFeasiblePenalization : alphaWeakFeasibilityPenalization
    {
        public WeakFeasiblePenalization(VRPSimultaneousPickupDelivery problemData) : base(problemData)
        {
            WeakThreshold = 0;
        }

        public override bool IsAllowedMovement(IntraMove m)
        {
            return ProblemData.IsFeasible(m.current);
        }

        public override bool IsAllowedMovement(IntraSwap m)
        {
            return ProblemData.IsFeasible(m.current);
        }

        public override bool IsAllowedMovement(TwoOpt m)
        {
            return ProblemData.IsFeasible(m.current);
        }

        public override bool IsAllowedMovement(InterMove m)
        {
            double delivery = ProblemData.TotalDelivery(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Delivery;
            double pickup = ProblemData.TotalPickup(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Pickup;
            return Math.Max(delivery, pickup) <= m.deRoute.Vehicle.Capacity;
        }

        public override bool IsAllowedMovement(InterSwap m)
        {
            return WeakFeasibleAfterReplace(m.current, m.current.GetRange(m.orIndex, 1), m.deRoute.GetRange(m.deIndex, 1)) &&
                WeakFeasibleAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex, 1), m.current.GetRange(m.orIndex, 1));
        }
        
        protected bool WeakFeasibleAfterReplace(Route current, List<int> oldRange, List<int> newRange)
        {
            return MaxDemandAfterReplace(current, oldRange, newRange) <= current.Vehicle.Capacity;
        }

        public override bool IsAllowedMovement(TwoInterMove m)
        {
            double delivery = ProblemData.TotalDelivery(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Delivery + ProblemData.Clients[m.current[m.orIndex+1]].Delivery;
            double pickup = ProblemData.TotalPickup(m.deRoute) + ProblemData.Clients[m.current[m.orIndex]].Pickup + +ProblemData.Clients[m.current[m.orIndex + 1]].Pickup;
            return Math.Max(delivery, pickup) <= m.deRoute.Vehicle.Capacity;
        }

        public override bool IsAllowedMovement(TwoOneInterSwap m)
        {
            return WeakFeasibleAfterReplace(m.current, m.current.GetRange(m.orIndex, 2), m.deRoute.GetRange(m.deIndex, 1)) &&
                WeakFeasibleAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex, 1), m.current.GetRange(m.orIndex, 2));
        }

        public override bool IsAllowedMovement(TwoTwoInterSwap m)
        {
            return WeakFeasibleAfterReplace(m.current, m.current.GetRange(m.orIndex, 2), m.deRoute.GetRange(m.deIndex, 2)) &&
                WeakFeasibleAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex, 2), m.current.GetRange(m.orIndex, 2));
        }

        public override bool IsAllowedMovement(CrossoverRoute m)
        {
            return WeakFeasibleAfterReplace(m.current, m.current.GetRange(m.orIndex + 1, m.current.Count - m.orIndex - 1), m.deRoute.GetRange(m.deIndex + 1, m.deRoute.Count - m.deIndex - 1)) &&
                WeakFeasibleAfterReplace(m.deRoute, m.deRoute.GetRange(m.deIndex + 1, m.deRoute.Count - m.deIndex - 1), m.current.GetRange(m.orIndex + 1, m.current.Count - m.orIndex - 1));
        }

        public override bool IsAllowedMovement(ReverseCrossoverRoute m)
        {
            return IsAllowedMovement((CrossoverRoute)m);
        }

    }
    
}
