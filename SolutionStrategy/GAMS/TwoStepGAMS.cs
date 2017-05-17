using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;
using System.IO;

using VRPLibrary.ProblemData;
using VRPLibrary.ClientData;
using VRPLibrary.RouteSetData;


namespace VRPLibrary.SolutionStrategy.GAMS
{
    public class TwoStepGAMS:OptimalSPDGAMS
    {
        public TwoStepGAMS(VRPSimultaneousPickupDelivery problem, string gamsPath)
            : base(problem, gamsPath)
        { }

        public TwoStepGAMS(VRPSimultaneousPickupDelivery problem)
            : base(problem)
        { }

        public RouteSet WeakSolve(string folderPath)
        {
            templete = GAMSTemplate.VRPSPD_WeakFeasible;
            Solve(folderPath, false, false);
            string weakSolutionPath = Path.Combine(folderPath, "weaksolution.dat");
            StreamReader solutionFile = new StreamReader(weakSolutionPath);
            int clientes = int.Parse(solutionFile.ReadLine().Trim().Replace(".00", ""));
            if(clientes == ProblemData.Clients.Count)
                return ParseWeakSolution(solutionFile);
            return null;
        }

        public RouteSet Solve(string folderPath)
        {
            DirectoryInfo weakDir = Directory.CreateDirectory(folderPath).CreateSubdirectory("weak");
            RouteSet weakSolution = WeakSolve(weakDir.FullName);
            if (weakSolution != null)
            {
                SingleRouteSPDGAMS sgProcedure = new SingleRouteSPDGAMS((VRPSimultaneousPickupDelivery)ProblemData);
                return sgProcedure.Solve(folderPath, weakSolution);
            }
            return null;
        }

        protected RouteSet ParseWeakSolution(StreamReader weakSolution)
        {
            RouteSet weakRS = RouteSet.BuildEmptyRouteSet(ProblemData.Vehicles);
            Regex routeExp = new Regex(@"(?<c>C\d+)\S+(?<v>V\d+)\S+(?<a>\d\.d+)");
            while (!weakSolution.EndOfStream)
            {
                string line = weakSolution.ReadLine();
                if (line == "") break;
                //Match mline = routeExp.Match(line);
                int c_index = line.IndexOf('C');
                int c_sep = line.IndexOf(' ', c_index);
                int c = int.Parse(line.Substring(c_index + 1, c_sep - c_index - 1));
                int v_index = line.IndexOf('V');
                int v_sep = line.IndexOf(' ', v_index);
                int v = int.Parse(line.Substring(v_index + 1, v_sep - v_index - 1));
                int dot_index = line.IndexOf('.');
                int val = int.Parse(line.Substring(dot_index - 1, 1));
                if (val != 0)
                    weakRS[v - 1].Add(c);
            }
            weakSolution.Close();
            return weakRS;
        }
    }
}
