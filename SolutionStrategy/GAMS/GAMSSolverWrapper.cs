using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.IO;
using VRPLibrary.ClientData;
using VRPLibrary.ProblemData;
using VRPLibrary.FleetData;
using VRPLibrary.RouteSetData;

//C:\Program Files (x86)\GAMS22.2\gams.exe en Sissi
namespace VRPLibrary.SolutionStrategy.GAMS
{
    public abstract class GamsSolverWrapper<TClient> where TClient : Client
    {
        public string GamsSolverPath { get; private set; }
        public VehicleRoutingProblem<TClient> ProblemData { get; private set; }
        protected byte[] templete;

        protected GamsSolverWrapper(VehicleRoutingProblem<TClient> problemData, string gamsSolverPath)
        {
            ProblemData = problemData;
            GamsSolverPath = gamsSolverPath;
        }

        protected GamsSolverWrapper(VehicleRoutingProblem<TClient> problemData)
            : this(problemData, @"C:\Program Files (x86)\GAMS22.2\gams.exe")
        { }

        public void GenerateDataFile(string dataPath, bool includeDepot, bool includeTravelData)
        {
            StreamWriter data = new StreamWriter(Path.Combine(dataPath, "data.inc"));

            DeclareSets(data, includeDepot);
            IncludeClientData(data);
            IncludeFleetData(data);
            if (includeTravelData)
                IncludeTravelMatrixData(data);
            data.Close();
        }

        #region DataSet
        protected virtual void DeclareSets(StreamWriter data, bool includeDepot)
        {
            data.WriteLine("sets");
            DeclareFleetSet(data);
            DeclareClientSet(data, includeDepot);
        }

        protected virtual void DeclareClientSet(StreamWriter data, bool includeDepot)
        {
            int start = (includeDepot) ? 0 : 1;
            data.WriteLine("i clients /C{0}*C{1}/", start, ProblemData.Clients.Count);
            data.WriteLine("alias(i,j)");
        }

        protected virtual void DeclareFleetSet(StreamWriter data)
        {
            data.WriteLine("k vehicles /V1*V{0}/", ProblemData.Vehicles.FleetSize);
        }
        #endregion

        #region ClientData
        protected abstract void IncludeClientData(StreamWriter data);
        #endregion

        #region FleetData
        protected virtual void IncludeFleetData(StreamWriter data)
        {
            data.WriteLine("parameter Q(k) capacity");
            data.WriteLine("/");
            int index = 1;
            foreach (var item in ProblemData.Vehicles)
                for (int i = 0; i < item.Count; i++)
                {
                    data.WriteLine("V{0}\t{1}", index, item.Capacity);
                    index++;
                }
            data.WriteLine("/;");
        }
        #endregion

        #region TravelData
        protected virtual void IncludeTravelMatrixData(StreamWriter data)
        {
            data.WriteLine("parameter C(i,j) travelinfo");
            data.WriteLine("/");
            for (int i = 0; i < ProblemData.TravelDistance.GetLength(0); i++)
                for (int j = 0; j < ProblemData.TravelDistance.GetLength(0); j++)
                {
                    if (i == j) continue;
                    data.WriteLine("C{0}.C{1}\t{2}", i, j, ProblemData.TravelDistance[i, j]);
                }
            data.WriteLine("/;");
        }


        #endregion

        #region Solve

        public void GenerateSolverFile(string folderPath)
        {
            StreamWriter solver = new StreamWriter(Path.Combine(folderPath, "solver.gms"));

            StreamReader solvertemplante = new StreamReader(new MemoryStream(templete));
            solver.Write(solvertemplante.ReadToEnd());
            solvertemplante.Close();
            solver.Close();
        }

        protected Process GetProcessInfo(string folderPath)
        {
            Process process = new Process();
            process.StartInfo.FileName = GamsSolverPath;
            process.StartInfo.Arguments = "solver.gms";
            process.StartInfo.UseShellExecute = false;
            process.StartInfo.WorkingDirectory = Path.GetFullPath(folderPath);
            return process;
        }

        public virtual void Execute(string folderPath)
        {
            Process process = GetProcessInfo(folderPath);
            process.Start();
            process.WaitForExit();
        }

        public virtual void Solve(string folderPath, bool includeDepot, bool includeTravelData)
        {
            GenerateDataFile(folderPath, includeDepot, includeTravelData);
            GenerateSolverFile(folderPath);
            Execute(folderPath);
        }
        #endregion
    }
}
