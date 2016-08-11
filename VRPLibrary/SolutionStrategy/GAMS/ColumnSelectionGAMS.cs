﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using System.Text.RegularExpressions;

using VRPLibrary.ProblemData;
using VRPLibrary.ClientData;
using VRPLibrary.FleetData;
using VRPLibrary.RouteSetData;
using VRPLibrary.SolutionStrategy.VRPSPD;

namespace VRPLibrary.SolutionStrategy.GAMS
{
    public class ColumnSelectionGAMS:GamsSolverWrapper<PickupDeliveryClient>
    {
        public ColumnSelectionGAMS(VRPSimultaneousPickupDelivery problemData, string gamsPath):
            base(problemData, gamsPath){ }

        public ColumnSelectionGAMS(VRPSimultaneousPickupDelivery problemData) :
            base(problemData) { }


        protected override void IncludeClientData(StreamWriter data)
        {
            throw new NotImplementedException();
        }

        #region Generate Data
        private void GenerateDataFile(string routePath, List<ExtRouteInfo> pool)
        {

            StreamWriter data = new StreamWriter(Path.Combine(routePath, "data.inc"));
            DeclareSets(data, pool);
            IncludeRouteData(data, pool);
            IncludeRouteClientRelation(data, pool);
            data.Close();
        }

        private void DeclareSets(StreamWriter data, List<ExtRouteInfo> pool)
        {
            data.WriteLine("sets");
            data.WriteLine("i clients /C1*C{0}/", ProblemData.Clients.Count);
            data.WriteLine("j routes /R0*R{0}/", pool.Count - 1);

        }

        private void IncludeRouteData(StreamWriter data, List<ExtRouteInfo> pool)
        {
            data.WriteLine("parameter R(j) cost");
            data.WriteLine("/");
            for (int i = 0; i < pool.Count; i++)
            {
                data.WriteLine("R{0}\t{1}", i, pool[i].Cost);
            }
            data.WriteLine("/;");
        }

        private void IncludeRouteClientRelation(StreamWriter data, List<ExtRouteInfo> pool)
        {
            data.WriteLine("parameter A(i,j) client route relation");
            data.WriteLine("/");
            for (int i = 1; i < ProblemData.Clients.Count + 1; i++)
                for (int j = 0; j < pool.Count; j++)
                {
                    if (pool[j].Current.Contains(i))
                        data.WriteLine("C{0}.R{1}\t{2}", i, j, 1);
                    else
                        data.WriteLine("C{0}.R{1}\t{2}", i, j, 0);
                }
            data.WriteLine("/;");
        }
        #endregion 

        public List<Route> Solve(string folderPath, List<ExtRouteInfo> pool)
        {
            templete = GAMSTemplate.VRPSPD_SetPartition;
            GenerateDataFile(folderPath, pool);
            GenerateSolverFile(folderPath);
            Execute(folderPath);
            return ParseSolution(folderPath, pool);
        }

        #region Parse Solution

        public List<Route> ParseSolution(string folderPath, List<ExtRouteInfo> pool)
        {
            //TODO!!!!! Verificar q exista el fichero, si no existe es q gams no pudo encontrar solucion para alguna ruta
            List<Route> selected = new List<Route>();
            Regex routeExp = new Regex(@"(?<rj>R\d+)\S+(?<v>\d\.d+)");
            StreamReader reader = new StreamReader(Path.Combine(folderPath, "setpartition.dat"));
            double cost = double.Parse(reader.ReadLine().Trim());
            while (!reader.EndOfStream)
            {
                string line = reader.ReadLine();
                if (line == "") break;
                //Match mline = routeExp.Match(line);
                int rj_index = line.IndexOf('R');
                int rj_sep = line.IndexOf(' ', rj_index);
                int rj = int.Parse(line.Substring(rj_index + 1, rj_sep - rj_index - 1));
                int dot_index = line.IndexOf('.');
                int val = int.Parse(line.Substring(dot_index - 1, 1));
                if (val != 0)
                    selected.Add(pool[rj].Current);
            }
            return selected;
        }

        #endregion
    }
}
