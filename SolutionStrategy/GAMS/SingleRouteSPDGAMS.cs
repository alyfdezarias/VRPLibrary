using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Text.RegularExpressions;

using VRPLibrary.ProblemData;
using VRPLibrary.ClientData;
using VRPLibrary.RouteSetData;

namespace VRPLibrary.SolutionStrategy.GAMS
{
    public class SingleRouteSPDGAMS: GamsSolverWrapper<PickupDeliveryClient>
    {
        public SingleRouteSPDGAMS(VRPSimultaneousPickupDelivery problemData, string gamsPath)
            : base(problemData, gamsPath)
        { }


        public SingleRouteSPDGAMS(VRPSimultaneousPickupDelivery problemData)
            : base(problemData)
        { }


        private int[] MappingClients(Route spdRoute)
        {
            int[] clientsMapping = new int[spdRoute.Count + 1];
            for (int i = 0; i < spdRoute.Count; i++)
            {
                clientsMapping[i + 1] = spdRoute[i];
            }
            return clientsMapping;
        }

        public RouteSet Solve(string folderPath, RouteSet solution)
        {
            RouteSet newSolution = new RouteSet();
            for (int i = 0; i < solution.Count; i++)
            {
                if (solution[i].IsEmpty) continue;
                string routePath = Path.Combine(folderPath, i.ToString());
                DirectoryInfo routeDir = Directory.CreateDirectory(routePath);
                int[] mapping = GenerateDataFile(routePath, solution[i]);
                templete = GAMSTemplate.VRPSPD_RouteOptimal;
                GenerateSolverFile(routePath);
                Execute(routePath);
                Route newRoute = ParseGamsSolution(routePath, mapping, solution[i]);
                newSolution.Add(newRoute);
            }
            return newSolution;
        }

        public Route Solve(string folderPath, Route current)
        {
            DirectoryInfo routeDir = Directory.CreateDirectory(folderPath);
            int[] mapping = GenerateDataFile(folderPath, current);
            templete = GAMSTemplate.VRPSPD_RouteOptimal;
            GenerateSolverFile(folderPath);
            Execute(folderPath);
            return ParseGamsSolution(folderPath, mapping, current);
        }

        #region Generate Data
        private int[] GenerateDataFile(string routePath, Route spdRoute)
        {

            int[] mapping = MappingClients(spdRoute);
            StreamWriter data = new StreamWriter(Path.Combine(routePath, "data.inc"));
            DeclareSets(data, spdRoute);
            IncludeClientData(data, mapping);
            data.WriteLine("scalar Q capacity /{0}/;", spdRoute.Vehicle.Capacity);
            IncludeTravelData(data, mapping);
            data.Close();
            return mapping;
        }

        private void DeclareSets(StreamWriter data, Route spdRoute)
        {
            data.WriteLine("sets");
            data.WriteLine("i clients /C0*C{0}/", spdRoute.Count);
            data.WriteLine("alias(i,j)");
        }

        protected override void IncludeClientData(StreamWriter data)
        {
            throw new NotImplementedException();
        }
        private void IncludeClientData(StreamWriter data, int[] mapping)
        {
            data.WriteLine("parameter E(i) delivery");

            data.WriteLine("/");
            for (int i = 1; i < mapping.Length; i++)
            {
                data.WriteLine("C{0}\t{1}", i, ProblemData.Clients[mapping[i]].Delivery);
            }
            data.WriteLine("/;");

            data.WriteLine("parameter R(i) pickup");
            data.WriteLine("/");
            for (int i = 1; i < mapping.Length; i++)
            {
                data.WriteLine("C{0}\t{1}", i, ProblemData.Clients[mapping[i]].Pickup);
            }
            data.WriteLine("/;");
        }

        private void IncludeTravelData(StreamWriter data, int[] mapping)
        {
            data.WriteLine("parameter C(i,j) travelinfo");
            data.WriteLine("/");
            for (int i = 0; i < mapping.Length; i++)
                for (int j = 0; j < mapping.Length; j++)
                {
                    if (i == j) continue;
                    data.WriteLine("C{0}.C{1}\t{2}", i, j, ProblemData.TravelDistance[mapping[i], mapping[j]]);
                }
            data.WriteLine("/;");
        }
        #endregion

        #region Parse Solution
        protected Route ParseGamsSolution(string routePath, int[] mapping, Route currentRoute)
        {
            //TODO!!!!! Verificar q exista el fichero, si no existe es q gams no pudo encontrar solucion para alguna ruta
            Route newRoute = new Route(currentRoute.Vehicle);
            Regex routeExp = new Regex(@"(?<ci>C\d+)\S+(?<cj>C\d+)\S+(?<v>\d\.d+)");
            StreamReader reader = new StreamReader(Path.Combine(routePath, "tspspdsolution.dat"));
            double cost = double.Parse(reader.ReadLine().Trim());
            if (cost == 0)
            {
                Console.WriteLine("weak feasible {0} strong feasible {1}", ProblemData.IsFeasible(currentRoute), ((VRPSimultaneousPickupDelivery)ProblemData).IsStrongFeasible(currentRoute));
                return currentRoute;
            }
            Dictionary<int, int> routeInfo = GetRouteInfo(reader);
            int lastCLient = 0;
            int newClient = routeInfo[lastCLient];
            while (newClient != 0)
            {
                newRoute.Add(mapping[newClient]);
                lastCLient = newClient;
                newClient = routeInfo[lastCLient];
            }
            return newRoute;
        }

        private Dictionary<int, int> GetRouteInfo(StreamReader reader)
        {
            Dictionary<int, int> routeInfo = new Dictionary<int, int>();
            while (!reader.EndOfStream)
            {
                string line = reader.ReadLine();
                if (line == "") break;
                //Match mline = routeExp.Match(line);
                int ci_index = line.IndexOf('C');
                int ci_sep = line.IndexOf(' ', ci_index);
                int ci = int.Parse(line.Substring(ci_index + 1, ci_sep - ci_index - 1));
                int cj_index = line.LastIndexOf('C');
                int cj_sep = line.IndexOf(' ', cj_index);
                int cj = int.Parse(line.Substring(cj_index + 1, cj_sep - cj_index - 1));
                int dot_index = line.IndexOf('.');
                int val = int.Parse(line.Substring(dot_index - 1, 1));
                if (val != 0)
                    routeInfo.Add(ci, cj);
            }
            reader.Close();
            return routeInfo;
        }

        #endregion
    }
}
