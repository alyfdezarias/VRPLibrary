using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;

namespace VRPLibrary.ClientData
{
    public class ClientSet<TClient> : List<TClient> where TClient : Client
    {
        public ClientSet() : base() { }
        public ClientSet(IEnumerable<TClient> clients) : base(clients) { }

        public new TClient this[int id]
        {
            get { return base[id - 1]; }
        }

        public virtual XElement ToXmlFormat()
        {
            return new XElement("clientSet",
                                 from c in this
                                 select c.ToXmlFormat());
        }

        public static ClientSet<Client> LoadClientSetFromXML(XElement document)
        {
            return new ClientSet<Client>(
                from c in document.Descendants("client")
                select new Client(int.Parse(c.Attribute("id").Value)));
        }

        public static ClientSet<DeliveryClient> LoadDeliveryClientSetFromXML(XElement document)
        {
            return new ClientSet<DeliveryClient>(
                from c in document.Descendants("client")
                select new DeliveryClient(int.Parse(c.Attribute("id").Value), 
                                          double.Parse(c.Attribute("delivery").Value)));
        }

        public static ClientSet<PickupDeliveryClient> LoadPickupDeliveryClientSetFromXML(XElement document)
        {
            return new ClientSet<PickupDeliveryClient>(
                from c in document.Descendants("client")
                select new PickupDeliveryClient(int.Parse(c.Attribute("id").Value),
                                                Math.Round(double.Parse(c.Attribute("delivery").Value),8),
                                                Math.Round(double.Parse(c.Attribute("pickup").Value),8)));
        }
        
    }
}
