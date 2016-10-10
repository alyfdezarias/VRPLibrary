using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Text.RegularExpressions;
using System.Diagnostics;

using VRPLibrary.ProblemData;
using VRPLibrary.RouteSetData;
using VRPLibrary.SolutionStrategy.VRPSPD;
using VRPLibrary.SolutionStrategy.GAMS;
using ExperimentsApp;
using MetaheuristicsLibrary;

namespace TesterApp
{
    class Program
    {
        static void Main(string[] args)
        {
            Random rdObj = new Random(0);
            Dictionary<string, string> parameters = TesterTools.LoadParametersData(args);
            DirectoryInfo benckmarkDir = Directory.CreateDirectory(parameters["benchmark"]);
            DirectoryInfo outputDir = (parameters.ContainsKey("output")) ? Directory.CreateDirectory(parameters["output"]) : Directory.CreateDirectory("\\");
            StreamWriter logs = new StreamWriter(Path.Combine(outputDir.FullName, "logs.csv"));
            logs.WriteLine("problem, tc, overload, vnd tc, vnd o, vndr tc, vndr o");
            double alpha = (parameters.ContainsKey("alpha"))? double.Parse(parameters["alpha"]): 0.1;
            double overloadFactor = double.Parse(parameters["overload"]);
            double farInsertionFactor = double.Parse(parameters["farinsertion"]);
            double overloadImproveFactor = parameters.ContainsKey("overImprove")? double.Parse(parameters["overImprove"]): 10;
            string method = (parameters.ContainsKey("method")) ? parameters["method"] : "";


            foreach (var item in benckmarkDir.GetFiles())
            {
                VRPSimultaneousPickupDelivery problem = new VRPSimultaneousPickupDelivery(item.FullName);
                PenalizationLocalSearch procedure = new PenalizationLocalSearch(problem);
                SingleRouteSPDGAMS gamsProcedure = new SingleRouteSPDGAMS(problem);
                string problemName = Path.GetFileNameWithoutExtension(item.Name);
                DirectoryInfo problemDir = outputDir.CreateSubdirectory(problemName);
                StreamWriter localLogs = new StreamWriter(Path.Combine(problemDir.FullName, "local.csv"));
                localLogs.WriteLine(problemName);
                localLogs.WriteLine("tc, overload, vnd tc, vnd o, vndr tc, vndr o");

                Console.WriteLine(Path.GetFileNameWithoutExtension(item.Name));
                List<RouteSet> startpool = procedure.BuildGreedySeedPool(rdObj, alpha, farInsertionFactor, overloadFactor);
                if (method == "startgams" || method == "gams")
                    startpool = GAMSImprove(startpool, problem, procedure, gamsProcedure, problemDir, problemName, "ss");

                List<RouteSet> vndpool = new List<RouteSet>();
                List<RouteSet> vndreversepool = new List<RouteSet>();

                var neighborhoods = procedure.GetAllNeighborhoods();
                var rvneighborhoods = procedure.GetAllNeighborhoods();
                rvneighborhoods.Reverse();

                for (int i = 0; i < startpool.Count; i++)
                {
                    Console.WriteLine("ss {0}", procedure.GetTravelCost(startpool[i]));
                    startpool[i].ToXMLFormat().Save(Path.Combine(problemDir.FullName, string.Format("ss{0}.xml", i.ToString())));

                    RouteSet vndsolution = procedure.BasicPenalization(startpool[i], Exploration.FirstImprovement, rdObj, overloadImproveFactor, neighborhoods);
                    Console.WriteLine("vnd {0}", procedure.GetTravelCost(vndsolution));
                    vndsolution.ToXMLFormat().Save(Path.Combine(problemDir.FullName, string.Format("vnd{0}.xml", i.ToString())));
                    vndpool.Add(vndsolution);

                    RouteSet vndreversesolution = procedure.BasicPenalization(startpool[i], Exploration.FirstImprovement, rdObj, overloadImproveFactor, rvneighborhoods);

                    Console.WriteLine("reverse {0}", procedure.GetTravelCost(vndreversesolution));
                    vndreversesolution.ToXMLFormat().Save(Path.Combine(problemDir.FullName, string.Format("reverse{0}.xml", i.ToString())));
                    vndreversepool.Add(vndreversesolution);
                }

                if (method == "endgams" || method == "gams")
                {
                    vndpool = GAMSImprove(vndpool, problem, procedure, gamsProcedure, problemDir, problemName, "vnd");
                    vndreversepool = GAMSImprove(vndreversepool, problem, procedure, gamsProcedure, problemDir, problemName, "reverse");
                }

                double minCost = int.MaxValue;
                int minIndex = -1;

                for (int i = 0; i < startpool.Count; i++)
                {
                    double vndCost = procedure.GetCost(vndpool[i]);
                    double reverseCost = procedure.GetCost(vndreversepool[i]);
                    double min = Math.Min(vndCost, reverseCost);
                    if (min < minCost)
                    {
                        minCost = min;
                        minIndex = i;
                        Console.WriteLine("min = {0} index ={1}", minCost, minIndex);
                    }
                    localLogs.WriteLine("{0}, {1}, {2}, {3}, {4}, {5}",
                        procedure.GetTravelCost(startpool[i]), problem.StrongOverLoad(startpool[i]),
                        procedure.GetTravelCost(vndpool[i]), problem.StrongOverLoad(vndpool[i]),
                        procedure.GetTravelCost(vndreversepool[i]), problem.StrongOverLoad(vndreversepool[i]));
                }
                localLogs.Close();
                logs.WriteLine("{0}, {1}, {2}, {3}, {4}, {5}, {6}", problemName,
                        procedure.GetTravelCost(startpool[minIndex]), problem.StrongOverLoad(startpool[minIndex]),
                        procedure.GetTravelCost(vndpool[minIndex]), problem.StrongOverLoad(vndpool[minIndex]),
                        procedure.GetTravelCost(vndreversepool[minIndex]), problem.StrongOverLoad(vndreversepool[minIndex]));

            }
            logs.Close();
        }

        private static List<RouteSet> GAMSImprove(List<RouteSet> pool, VRPSimultaneousPickupDelivery problem, PenalizationLocalSearch procedure, SingleRouteSPDGAMS gamsProcedure, DirectoryInfo outputDir, string problemName, string prefix)
        {
            List<RouteSet> gams = new List<RouteSet>();
            for (int i = 0; i < pool.Count; i++)
            {
                DirectoryInfo exeDir = outputDir.CreateSubdirectory(string.Format("{0}{1}", prefix, i.ToString()));
                RouteSet solution = gamsProcedure.Solve(exeDir.FullName, pool[i]);
                gams.Add(solution);
            }
            return gams;
        }
             
    }
}
