using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Antflow.Traffic.Class
{
    public class LocalRules
    {
        public double BoothWidth;
        public double BoothDepth;
        public double ManeuveringArea;

        public LocalRules()
        {
            BoothDepth = 5;
            BoothWidth = 2;
            ManeuveringArea = 7;
        }
    }
}
