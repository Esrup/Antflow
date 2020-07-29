using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Antflow
{
    class A_star
    {
        public A_star Parent;
        public Point3d Position;

        public double g = 0;
        public double h = 0;
        public double f = 0;

        public A_star(A_star parent, Point3d position)
        {
            Parent = parent;
            Position = position;
        }


        
    }
}
