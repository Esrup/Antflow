using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using Grasshopper.Kernel;
using Rhino.Geometry;


namespace Antflow
{
    class Person
    {
        public Point3d Position;
        public List<Point3d> Path;
        public List<Point3d> Interests;
        public double TravedledDist;
        public Vector3d Facing;
        public Point3d CurrentTarget;


        public Person(Point3d startPT, List<Point3d> interests)
        {
            //Create Person
            Position = startPT;
            Interests = interests;
            TravedledDist = 0.0;
            Path = new List<Point3d>();
            Facing = new Vector3d(0, 1, 0);
        }

        public Boolean UpdateTarget(Point3d newTarget)
        {
            //update Target
            CurrentTarget = newTarget;

            return true;
        }

        public Boolean removeinterest()
        {
            for (int i = 0; i <Interests.Count; i++)
            {
                if (Interests[i].Equals(CurrentTarget))
                {
                    Interests.RemoveAt(i);
                }
            }

            return true;
        }

        public Boolean UpdatePos(Point3d newPos, Vector3d direction)
        {
            //Add current position to path
            Path.Add(Position);

            
            //Add distance
            TravedledDist = TravedledDist + Position.DistanceTo(newPos);

            //Change new position
            Position = newPos;

            //Update Direction
            Facing = direction;

            return true;
        }







    }

    
}
