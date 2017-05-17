using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using VRPLibrary.ProblemData;
using VRPLibrary.ClientData;
using VRPLibrary.RouteSetData;
using System.IO;

namespace VRPLibrary.SolutionStrategy.GAMS
{
    public class OptimalSPDGAMS:GamsSolverWrapper<PickupDeliveryClient>
    {

        public OptimalSPDGAMS(VRPSimultaneousPickupDelivery problemData, string gamsPath)
            : base(problemData, gamsPath)
        { }

        public OptimalSPDGAMS(VRPSimultaneousPickupDelivery problemData)
            : base(problemData)
        { }

        public virtual double OptimalSolve(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_Optimal;
            return ModelSolve(folderPath, "spdsolution.dat");
        }

        protected double ModelSolve(string folderPath, string solutionFileName)
        {
            Solve(folderPath, true, true);
            string solutionPath = Path.Combine(folderPath, solutionFileName);
            StreamReader solutionCost = new StreamReader(solutionPath);
            return double.Parse(solutionCost.ReadLine().Trim());
        }

        public double OptimalWeakFeasibleSolve(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_WeakFeasibleOptimal;
            return ModelSolve(folderPath, "spdweaksolution.dat");
        }

        protected override void IncludeClientData(StreamWriter data)
        {
            data.WriteLine("parameter E(i) delivery");
            data.WriteLine("/");
            foreach (var c in ProblemData.Clients)
            {
                data.WriteLine("C{0}\t{1}", c.ID, c.Delivery);
            }
            data.WriteLine("/;");
            data.WriteLine("parameter R(i) pickup");
            data.WriteLine("/");
            foreach (var c in ProblemData.Clients)
            {
                data.WriteLine("C{0}\t{1}", c.ID, c.Pickup);
            }
            data.WriteLine("/;");
        }
    }

    public class OptimalHomogeneousSPDGAMS : OptimalSPDGAMS
    {
        public OptimalHomogeneousSPDGAMS(VRPSimultaneousPickupDelivery problemData, string gamsPath)
            : base(problemData, gamsPath)
        { }

        public OptimalHomogeneousSPDGAMS(VRPSimultaneousPickupDelivery problemData)
            : base(problemData)
        { }

        protected override void DeclareFleetSet(StreamWriter data)
        {
            /*no hacer nada*/
        }

        protected override void IncludeFleetData(StreamWriter data)
        {
            data.WriteLine($"scalar K fleet size /{ProblemData.Vehicles.FleetSize}/;");
            data.WriteLine($"scalar Q vehicle capacity /{ProblemData.Vehicles[0].Capacity}/;");
        }

        public override double OptimalSolve(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_Optimal_Homogenea;
            return ModelSolve(folderPath, "hspdsolution.dat");
        }

        public double StrongOptimal(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_StrongOptimal_Homogenea;
            return ModelSolve(folderPath, "strongVRPSPD.dat");
        }

        public double LinearRelaxationSolve(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_Optimal_Homogenea_Linear;
            return ModelSolve(folderPath, "linear.dat");
        }

        public double StrongLinearRelaxationSolve(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_Optimal_Homogenea_StrongLinear;
            return ModelSolve(folderPath, "stronglinear.dat");
        }
    }
}
