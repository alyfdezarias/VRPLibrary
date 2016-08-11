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

using MetaheuristicsLibrary;

namespace ExperimentsApp
{
    class Program
    {
        static void Main(string[] args)
        {
            Random rdObj = new Random();
            Stopwatch timer = new Stopwatch();
            Dictionary<string, string> parameters = TesterTools.LoadParametersData(args);

            DirectoryInfo benckmarkDir = Directory.CreateDirectory(parameters["benchmark"]);
            DirectoryInfo outputDir = parameters.ContainsKey("output") ? Directory.CreateDirectory(parameters["output"]) : Directory.CreateDirectory("\\");
            string logsFileName = Path.Combine(outputDir.FullName, "logs.csv");
            StreamWriter logs = new StreamWriter(logsFileName);
            logs.WriteLine("problem, tc, overload, routes, time");
            logs.Close();

            string method = parameters.ContainsKey("method") ? parameters["method"]: "";
            string mh = parameters.ContainsKey("mh") ? parameters["mh"] : "vnd";

            //parameters
            int reStarts = parameters.ContainsKey("restarts") ? int.Parse(parameters["restarts"]) : 20; 
            int shakings = parameters.ContainsKey("shaking")?int.Parse(parameters["shaking"]):30;
            double overloadFactor = parameters.ContainsKey("overload") ? double.Parse(parameters["overload"]) : 10;
            double weakThreshold = parameters.ContainsKey("weak") ? double.Parse(parameters["weak"]) : 0.2;
            Exploration expCondition = parameters.ContainsKey("expCondition") ? (Exploration)Enum.Parse(typeof(Exploration), parameters["expCondition"]) : Exploration.Random;
            Exploration interExp = parameters.ContainsKey("interexp") ? (Exploration)Enum.Parse(typeof(Exploration), parameters["interexp"]) : Exploration.Random;

            //fixed parameters
            double greedyRdFactor = 0.2;
            double saProbability = 0.3;
            double saWorst = 0.2;
            int saRepetitions = 15;
            double saFactor = 1;/*old value 1.01*/
            double saCoolingRate = 0.95;
            int saCoolerAmount = 50;

            RouteSet current = null;

            foreach (var item in benckmarkDir.GetFiles())
            {
                VRPSimultaneousPickupDelivery problem = new VRPSimultaneousPickupDelivery(item.FullName);
                Console.WriteLine(Path.GetFileNameWithoutExtension(item.Name));

                PenalizationLocalSearch procedure = new PenalizationLocalSearch(problem);
                ColumnSelectionProcedure csProcedure = new ColumnSelectionProcedure(problem);
                SingleRouteSPDGAMS gamsProcedure = new SingleRouteSPDGAMS(problem);
                WeakFeasibilityPenalization wfProcedure = new WeakFeasibilityPenalization(problem);

                timer.Reset();

                if (method == "msls")/*multi start local search*/
                {
                    if (mh == "vnd")
                    {
                        timer.Start();
                        current = procedure.MultiStartPenalization(greedyRdFactor, reStarts, expCondition, rdObj, overloadFactor, procedure.GetAllNeighborhoods());
                        timer.Stop();
                        Console.WriteLine("vnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vnd_{0}", item.Name)));
                    }
                    else if (mh == "vndr")
                    {
                        var neighborhoods = procedure.GetAllNeighborhoods();
                        neighborhoods.Reverse();
                        timer.Start();
                        current = procedure.MultiStartPenalization(greedyRdFactor, reStarts, expCondition, rdObj, overloadFactor, neighborhoods);
                        timer.Stop();
                        Console.WriteLine("vnd reverse {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vndr_{0}", item.Name)));
                    }
                    else if (mh == "rdvnd")
                    {
                        timer.Start();
                        current = procedure.RdVNDMultiStartPenalization(greedyRdFactor, reStarts, expCondition, rdObj, overloadFactor, procedure.GetAllNeighborhoods());
                        timer.Stop();
                        Console.WriteLine("rdvnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("rdvnd_{0}", item.Name)));
                    }
                    else if (mh == "sa")
                    {
                        timer.Start();
                        current = procedure.MultiStartPenalization(greedyRdFactor, reStarts, expCondition, rdObj, overloadFactor, procedure.GetAllNeighborhoods(),
                                                                    saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
                        timer.Stop();
                        Console.WriteLine("sa {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("sa_{0}", item.Name)));
                    }
                }
                else if (method == "msms")/*multi start multi shaking*/
                {
                    if (mh == "vnd")
                    {
                        timer.Start();
                        current = procedure.MultiStartMultiShakingPenalization(greedyRdFactor, reStarts, shakings, expCondition, rdObj, overloadFactor, procedure.GetAllNeighborhoods(), procedure.GetAllShakingProcedures());
                        timer.Stop();
                        Console.WriteLine("vnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vnd_{0}", item.Name)));
                    }
                    if (mh == "vndr")
                    {
                        var neighborhoods = procedure.GetAllNeighborhoods();
                        neighborhoods.Reverse();
                        timer.Start();
                        current = procedure.MultiStartMultiShakingPenalization(greedyRdFactor, reStarts, shakings, expCondition, rdObj, overloadFactor, neighborhoods, procedure.GetAllShakingProcedures());
                        timer.Stop();
                        Console.WriteLine("vnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vndr_{0}", item.Name)));
                    }
                    else if (mh == "rdvnd")
                    {
                        timer.Start();
                        current = procedure.RdVNDMultiStartMultiShakingPenalization(greedyRdFactor, reStarts, shakings, expCondition, rdObj, overloadFactor, procedure.GetAllNeighborhoods(), procedure.GetAllShakingProcedures());
                        timer.Stop();
                        Console.WriteLine("rdvnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("rdvnd_{0}", item.Name)));
                    }
                    else if (mh == "sa")
                    {
                        timer.Start();
                        current = procedure.MultiStartMultiShakingPenalization(greedyRdFactor, reStarts, shakings, expCondition, rdObj, overloadFactor, procedure.GetAllNeighborhoods(), procedure.GetAllShakingProcedures(),
                                                                                saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
                        timer.Stop();
                        Console.WriteLine("sa {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("sa_{0}", item.Name)));
                    }
                }
                else if (method == "msb")/*multi start bi level*/
                {
                    if (mh == "vnd")
                    {
                        timer.Start();
                        current = procedure.MutiStartBiLevelPenalization(greedyRdFactor, reStarts, interExp,expCondition, rdObj, overloadFactor,
                                                                         procedure.GetInterRouteNeighborhoods(), procedure.GetIntraRouteNeighborhoods());
                        timer.Stop();
                        Console.WriteLine("vnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vnd_{0}", item.Name)));
                    }
                    else if (mh == "rdvnd")
                    {
                        timer.Start();
                        current = procedure.RdVNDMutiStartBiLevelPenalization(greedyRdFactor, reStarts, interExp, expCondition, rdObj, overloadFactor,
                                                                            procedure.GetInterRouteNeighborhoods(), procedure.GetIntraRouteNeighborhoods());
                        timer.Stop();
                        Console.WriteLine("rdvnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("rdvnd_{0}", item.Name)));
                    }
                    else if (mh == "sa")
                    {
                        timer.Start();
                        current = procedure.MutiStartBiLevelPenalization(greedyRdFactor, reStarts, interExp, expCondition, rdObj, overloadFactor, procedure.GetInterRouteNeighborhoods(), procedure.GetIntraRouteNeighborhoods(),
                                                                            saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);
                        timer.Stop();
                        Console.WriteLine("sa+vnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("sa_{0}", item.Name)));
                    }
                }
                else if (method == "msmsb")/*multi start multi shaking bilevel*/
                {
                    if (mh == "vnd")
                    {
                        timer.Start();
                        current = procedure.MultiStartMultiShakingBiLevelPenalization(greedyRdFactor, reStarts, shakings, interExp, expCondition, rdObj, overloadFactor,
                                                                                      procedure.GetInterRouteNeighborhoods(), procedure.GetIntraRouteNeighborhoods(), procedure.GetAllShakingProcedures());
                        timer.Stop();
                        Console.WriteLine("vnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vnd_{0}", item.Name)));
                    }
                    else if (mh == "rdvnd")
                    {
                        timer.Start();
                        current = procedure.RdVNDMultiStartMultiShakingBiLevelPenalization(greedyRdFactor, reStarts, shakings, interExp, expCondition, rdObj, overloadFactor,
                                                                                      procedure.GetInterRouteNeighborhoods(), procedure.GetIntraRouteNeighborhoods(), procedure.GetAllShakingProcedures());
                        timer.Stop();
                        Console.WriteLine("rdvnd {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("rdvnd_{0}", item.Name)));
                    }
                    else if (mh == "sa")
                    {
                        timer.Start();
                        current = procedure.MultiStartMultiShakingBiLevelPenalization(greedyRdFactor, reStarts, shakings, interExp, expCondition, rdObj, overloadFactor,
                                                                                      procedure.GetInterRouteNeighborhoods(), procedure.GetIntraRouteNeighborhoods(), procedure.GetAllShakingProcedures(),
                                                                                      saProbability, saWorst, saRepetitions, saFactor, saCoolingRate, saCoolerAmount);

                        timer.Stop();
                        Console.WriteLine("sa {0} strong feasible {1}", procedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("sa_{0}", item.Name)));
                    }
                }
                else if (method == "cs")
                {
                    if (mh == "vnd")
                    {
                        DirectoryInfo gDir = outputDir.CreateSubdirectory(Path.GetFileNameWithoutExtension(item.Name));
                        current = csProcedure.Solve(greedyRdFactor, reStarts, overloadFactor, shakings, expCondition, rdObj, csProcedure.GetAllNeighborhoods(), csProcedure.GetAllShakingProcedures(), gDir.FullName);
                        Console.WriteLine("vnd {0} strong feasible {1}", csProcedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("vnd_{0}", item.Name)));
                    }
                }
                else if (method == "gams")
                {
                    current = null;
                    DirectoryInfo problemDir = benckmarkDir.CreateSubdirectory(Path.GetFileNameWithoutExtension(item.Name));
                    int index = 0;
                    foreach (var files in problemDir.GetFiles())
                    {
                        current = RouteSet.LoadFromXML(files.FullName);
                        if (problem.IsStrongFeasible(current))
                        {
                            DirectoryInfo gDir = outputDir.CreateSubdirectory(Path.GetFileNameWithoutExtension(string.Format("{0}_{1}", index, item.Name)));
                            current = gamsProcedure.Solve(gDir.FullName, current);
                            current.ToXMLFormat().Save(Path.Combine(outputDir.FullName, string.Format("gams_{0}_{1}", index, item.Name)));
                            Console.WriteLine("gams {0} strong feasible {1}", csProcedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                        }
                        else
                            Console.WriteLine("gams {0} strong feasible {1}", csProcedure.GetTravelCost(current), problem.IsStrongFeasible(current));
                    }
                }
                else if (method == "folder")
                {
                    DirectoryInfo problemDir = benckmarkDir.CreateSubdirectory(Path.GetFileNameWithoutExtension(item.Name));
                }
                logs = new StreamWriter(logsFileName, true);
                if (current != null)
                    logs.WriteLine("{0}, {1}, {2}, {3}, {4}", Path.GetFileNameWithoutExtension(item.Name), procedure.GetTravelCost(current), problem.StrongOverLoad(current), current.CountNotEmpty, timer.Elapsed.TotalSeconds);
                else
                    logs.WriteLine("{0}, -, -", Path.GetFileNameWithoutExtension(item.Name));
                logs.Close();
            }
        }
    }
    
}
