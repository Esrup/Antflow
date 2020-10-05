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

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Antflow
{
    public class Shortest_Walk_Terrain : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Shortest_Walk_Terrain class.
        /// </summary>
        public Shortest_Walk_Terrain()
          : base("Shortest_Walk", "Astar",
              "Description",
              "AntFlow", "Flow")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Person", "Person", "Person", GH_ParamAccess.item);
            pManager.AddPointParameter("Interests", "Interests", "Interests", GH_ParamAccess.list);
            pManager.AddMeshParameter("Obstacles", "Obstacles", "Obstacles", GH_ParamAccess.list);
            pManager.AddMeshParameter("Terrain", "Terrain", "Terrain", GH_ParamAccess.list);
            pManager.AddNumberParameter("Sloping", "Slope", "Maximum sloping", GH_ParamAccess.item, 0);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Path", "Path", "Path", GH_ParamAccess.tree);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare inputs
            Point3d person = new Point3d();
            List<Point3d> interests = new List<Point3d>();
            List<Mesh> obstacles = new List<Mesh>();
            List<Mesh> terrain = new List<Mesh>();
            double slope = 0;

            //Declare lists
            List<Point3d> path = new List<Point3d>();
            Grasshopper.DataTree<Point3d> tree = new Grasshopper.DataTree<Point3d>();

            //Get Inputs
            DA.GetData(0, ref person);
            DA.GetDataList(1, interests);
            DA.GetDataList(2, obstacles);
            DA.GetDataList(3, terrain);
            DA.GetData(4, ref slope);

            //Run Astar ||choose nearest interest
            //int mincount = int.MaxValue;
            //for (int i = 0; i < interests.Count; i++)
            //{
            //    List<Point3d> apath = RunAstar(person, interests[i], obstacles);
            //    mincount = apath.Count;
            //    path.Clear();
            //    path = apath;
            //    //Declare outputs
            //    DA.SetDataList(0, path);


            //}

            

            for (int i = 0; i < interests.Count; i++)
            {
                path = RunAstarTerrain(person, interests[i], obstacles, terrain, slope);
                tree.AddRange(path, new Grasshopper.Kernel.Data.GH_Path(i));
            }
            DA.SetDataTree(0, tree);
        }

        public static List<Point3d> RunAstarTerrain(Point3d person, Point3d interest, List<Mesh> obstacles, List<Mesh> terrain, double slope)
        {

            //Run solver

            //Declare Lists
            List<A_star> open = new List<A_star>();
            List<A_star> closed = new List<A_star>();
            List<Point3d> Pointpaths = new List<Point3d>();
            List<Point3d> finalPath = new List<Point3d>();
            List<A_star> children = new List<A_star>();

            

            //Declare var
            Point3d position;
            Boolean run = true;

            //Movement list
            List<Point3d> move = new List<Point3d>();
            
            move.Add(new Point3d(0, -1, 0));
            move.Add(new Point3d(0, 1, 0));
            move.Add(new Point3d(-1, 0, 0));
            move.Add(new Point3d(1, 0, 0));
            move.Add(new Point3d(-1, -1, 0));
            move.Add(new Point3d(-1, 1, 0));
            move.Add(new Point3d(1, -1, 0));
            move.Add(new Point3d(1, 1, 0));


            //Interest on mesh
            List<Point3d> intList = new List<Point3d>();
            intList.Add(interest);
            try
            {
                interest = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(terrain, intList, new Vector3d(0, 0, 1), 0)[0];
            }
            catch (Exception)
            {
                try
                {
                    interest = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(terrain, intList, new Vector3d(0, 0, -1), 0)[0];
                }
                catch (Exception)
                {

                    return null;
                }
            }
            
            

            //Declare start and end

            A_star start = new A_star(null, person);
            A_star end = new A_star(null, interest);


            //Add startpoint to open list
            open.Add(start);

            // Join obstacles in one Mesh
            Mesh obsMesh = new Mesh();
            foreach (Mesh mesh in obstacles)
            {
                obsMesh.Append(mesh);
            }

            //Pathfinding
            while (run)
            {

                //Get current node
                if (open.Count == 0)
                {



                    break;
                }
                
                A_star currentNode = open[0];
                int currentIndex = 0;

                for (int i = 0; i < open.Count; i++)
                {
                    if (open[i].f < currentNode.f)
                    {
                        currentNode = open[i];
                        currentIndex = i;
                    }
                }




                //Remove current node from open list and add to closed
                open.RemoveAt(currentIndex);
                closed.Add(currentNode);


                //If at endpoint, traceback the parents
                position = currentNode.Position;
                Pointpaths.Add(position);


                if (currentNode.Position.DistanceTo(interest) < 2)
                {
                    A_star tracer = currentNode;
                    while (tracer.g > 0)
                    {
                        finalPath.Add(tracer.Position);
                        tracer = tracer.Parent;


                    }
                    //Set Outputs
                    finalPath.Add(start.Position);
                    finalPath.Reverse();
                    finalPath.Add(end.Position);
                    return finalPath;

                }


                //Generate children
                children.Clear();
                foreach (Point3d new_position in move)
                {
                    List<Point3d> stupidlist = new List<Point3d>();

                    Point3d child_position = new Point3d(currentNode.Position.X + new_position.X, currentNode.Position.Y + new_position.Y, currentNode.Position.Z + new_position.Z);
                    stupidlist.Add(child_position);
                    //Project point to terrain
                    //Try z positive
                    try
                    {
                        child_position = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(terrain, stupidlist, new Vector3d(0, 0, 1), 0)[0];
                    }
                    catch (Exception)
                    {

                        try
                        {
                            //Try z Negative
                            child_position = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(terrain, stupidlist, new Vector3d(0, 0, -1), 0)[0];
                        }
                        catch (Exception)
                        {

                            continue;
                        }
                    }
                               

                    //Check if new point is in obstacle

                    if (obsMesh.IsPointInside(child_position, 0, false))
                    {
                        continue;
                    }

                    //Check if max sloping
                    if (slope != 0)
                    {
                        double terrain_slope = (child_position.Z - currentNode.Position.Z) / Math.Sqrt(Math.Pow(child_position.Y - currentNode.Position.Y, 2) + Math.Pow(child_position.X - currentNode.Position.X, 2)); 
                        if (terrain_slope * 100 > slope)
                        {
                            continue;
                        }
                    }
                    A_star new_node = new A_star(currentNode, child_position);
                    children.Add(new_node);
                }


                //Find best child
                foreach (A_star child in children)
                {

                    Boolean breakout = false;
                    foreach (A_star close in closed)
                    {
                        if (child.Position.DistanceTo(close.Position) < 1)
                        {
                            breakout = true;
                            break;
                        }

                    }

                    if (breakout == true)
                    {
                        continue;
                    }

                    child.g = currentNode.g + 1;
                    child.h = child.Position.DistanceTo(end.Position);
                    child.f = child.g + child.h;

                    for (int i = 0; i < open.Count; i++)
                    {
                        double dist = child.Position.DistanceTo(open[i].Position);
                        if (child.Position.DistanceTo(open[i].Position) < 1)
                        {
                            if (child.g > open[i].g)
                            {
                                breakout = true;
                                break;
                            }
                            else
                            {
                                open.RemoveAt(i);
                            }

                        }



                    }

                    if (breakout == true)
                    {
                        continue;
                    }

                    open.Add(child);


                    if (open.Count > 10000)
                    {
                        run = false;
                    }
                }










            }
            return finalPath;
        }


        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("48abb6d3-066d-4db5-944f-93a83e28a63f"); }
        }
    }
}