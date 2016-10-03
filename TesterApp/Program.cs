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

namespace TesterApp
{
    class Program
    {
        static void Main(string[] args)
        {
            Random rdObj = new Random(0);
            Dictionary<string, string> parameters = TesterTools.LoadParametersData(args);
            DirectoryInfo benckmarkDir = Directory.CreateDirectory(parameters["benchmark"]);
            DirectoryInfo outputDir = parameters.ContainsKey("output") ? Directory.CreateDirectory(parameters["output"]) : Directory.CreateDirectory("\\");
            StreamWriter logs = new StreamWriter(Path.Combine(outputDir.FullName, "logs.csv"));
            double alpha = double.Parse(parameters["alpha"]);
            int poolSize = int.Parse(parameters["pool"]);
            double overloadFactor = 1;
            double farInsertionFactor = 0.5;
            string method = parameters["method"];
            logs.WriteLine("problem, tc, overload, gams, avetc, aveoverload, avegams");
            Console.WriteLine("problem, tc, overload, gams, avetc, aveoverload, avegams");

            foreach (var item in benckmarkDir.GetFiles())
            {
                VRPSimultaneousPickupDelivery problem = new VRPSimultaneousPickupDelivery(item.FullName);
                PenalizationLocalSearch procedure = new PenalizationLocalSearch(problem);
                SingleRouteSPDGAMS gamsProcedure = new SingleRouteSPDGAMS(problem);
                string problemName = Path.GetFileNameWithoutExtension(item.Name);
                DirectoryInfo problemDir = outputDir.CreateSubdirectory(problemName);

                Console.WriteLine(Path.GetFileNameWithoutExtension(item.Name));
                List<RouteSet> pool = null;

                if (method == "greedy")
                    pool = procedure.GenerateGreedyRdPool(alpha, rdObj, poolSize);
                else if (method == "seed")
                    pool = procedure.BuildGreedySeedPool(rdObj, alpha, 0);
                else if (method == "overlaodseed")
                    pool = procedure.BuildGreedySeedPool(rdObj, alpha, overloadFactor);
                else if (method == "farseed")
                    pool = procedure.BuildGreedySeedPool(rdObj, alpha, farInsertionFactor, 0);
                else if (method == "overloadfarseed")
                    pool = procedure.BuildGreedySeedPool(rdObj, alpha, farInsertionFactor, overloadFactor);
                else if (method == "feasibleseed")
                    pool = procedure.BuildFeasibleGreedySeedPool(rdObj, alpha);
                else if (method == "feasiblefarseed")
                    pool = procedure.BuildFeasibleGreedySeedPool(rdObj, alpha, farInsertionFactor);

                if(pool!= null && pool.Count>0)
                    GAMSImprove(pool, problem, procedure, gamsProcedure, problemDir.CreateSubdirectory(method), problemName, logs);
            }
            logs.Close();
        }

        private static void GAMSImprove(List<RouteSet> pool, VRPSimultaneousPickupDelivery problem, PenalizationLocalSearch procedure, SingleRouteSPDGAMS gamsProcedure, DirectoryInfo outputDir, string problemName, StreamWriter logs)
        {
            List<RouteSet> gams = new List<RouteSet>();
            double minGams = int.MaxValue;
            int minIndex = -1;
            for (int i = 0; i < pool.Count; i++)
            {
                DirectoryInfo exeDir = outputDir.CreateSubdirectory(i.ToString());
                RouteSet solution = gamsProcedure.Solve(exeDir.FullName, pool[i]);
                gams.Add(solution);
                double gamsCost = procedure.GetTravelCost(solution);
                if (gamsCost < minGams)
                {
                    minGams = gamsCost;
                    minIndex = i;
                }
                pool[i].ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("s{0}.xml",i)));
                pool[i].ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("g{0}.xml",i)));
            }

            double tc = pool.Average(s => procedure.GetTravelCost(s));
            double overload = pool.Average(s => problem.StrongOverLoad(s));
            double tcgams = gams.Average(s => procedure.GetTravelCost(s));

            logs.WriteLine("{0}, {1}, {2}, {3}, {4}, {5}, {6}", problemName, procedure.GetTravelCost(pool[minIndex]), problem.StrongOverLoad(pool[minIndex]), minGams, tc, overload, tcgams);
            Console.WriteLine("{0}, {1}, {2}, {3}, {4}, {5}, {6}", problemName, procedure.GetTravelCost(pool[minIndex]), problem.StrongOverLoad(pool[minIndex]), minGams, tc, overload, tcgams);
        }
             
    }
}
