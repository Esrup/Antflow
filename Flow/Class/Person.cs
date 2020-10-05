/* Copyright 2020 Mikkel Esrup Steenberg 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), 
to deal in the Software without restriction, including without limitation the rights to use, 
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.*/
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
