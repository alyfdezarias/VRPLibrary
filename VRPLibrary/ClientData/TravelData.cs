using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.ClientData
{
    public class TravelData
    {
        public double[,] Data;

        public TravelData(double[,] distance)
        {
            if (distance.GetLength(0) != distance.GetLength(1)) throw new ArgumentException("Matrix must be square");
            this.Data = distance;
        }

        private int Size
        {
            get { return Data.GetLength(0); }
        }

        public virtual XElement ToXMLFormat()
        {
            XElement node = new XElement("travelData", new XAttribute("size", Size));
            for (int i = 0; i < Size; i++)
            {
                XElement row = new XElement("row", new XAttribute("id", i));
                for (int j = 0; j < Size; j++)
                {
                    XElement col = new XElement("column",
                                                 new XAttribute("id", j),
                                                 new XAttribute("value", Data[i, j]));
                    row.Add(col);
                }
                node.Add(row);
            }
            return node;
        }

        public static TravelData LoadFromXML(XElement document)
        {
            int size = int.Parse(document.Element("travelData").Attribute("size").Value);
            var values = from r in document.Descendants("row")
                         from c in r.Descendants("column")
                         select new
                         {
                             R = int.Parse(r.Attribute("id").Value),
                             C = int.Parse(c.Attribute("id").Value),
                             V = double.Parse(c.Attribute("value").Value)
                         };
            double[,] data = new double[size, size];
            foreach (var item in values)
            {
                data[item.R, item.C] = item.V;
            }
            return new TravelData(data);
        }

        public double GetDistance(int i, int j)
        {
            return Data[i, j];
        }

    }

}
