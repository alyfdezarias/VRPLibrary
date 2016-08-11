using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Text.RegularExpressions;
using System.IO;

namespace ExperimentsApp
{
    public class TesterTools
    {
        public static Dictionary<string, string> LoadParametersData(string[] args)
        {
            Regex parametersExpression = new Regex(@"(?<option>\w+)=(?<value>\d+(\.\d+)?|\.\d+|\w+)");
            Dictionary<string, string> parameters = new Dictionary<string, string>();

            foreach (var item in args)
            {
                Match m = parametersExpression.Match(item);
                parameters.Add(m.Groups["option"].Value, m.Groups["value"].Value);
            }
            return parameters;
        }
    }
}
