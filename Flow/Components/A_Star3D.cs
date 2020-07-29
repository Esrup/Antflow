using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Antflow
{
    public class A_Star3D : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Shortest_Walk_Terrain class.
        /// </summary>
        public A_Star3D()
          : base("Shortest_Walk3D", "Astar3D",
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
            pManager.AddMeshParameter("Floors", "Floors", "Floors", GH_ParamAccess.list);
            pManager.AddNumberParameter("Sloping", "Slope", "Maximum sloping", GH_ParamAccess.item, 0);
            
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Path", "Path", "Path", GH_ParamAccess.tree);
            pManager.AddPointParameter("Closed", "Closed", "Closed", GH_ParamAccess.list);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {

            //Declare warning & messages
            string warning = "";
            string error = "";

            //Declare inputs
            Point3d person = new Point3d();
            List<Point3d> interests = new List<Point3d>();
            List<Mesh> obstacles = new List<Mesh>();
            List<Mesh> floor = new List<Mesh>();
            double slope = 0;
            double Person_Height = 1.8;

            //Declare lists
            List<Point3d> path = new List<Point3d>();
            List<Point3d> Closed = new List<Point3d>();
            Grasshopper.DataTree<Point3d> tree = new Grasshopper.DataTree<Point3d>();

            //Get Inputs
            DA.GetData(0, ref person);
            DA.GetDataList(1, interests);
            DA.GetDataList(2, obstacles);
            DA.GetDataList(3, floor);
            DA.GetData(4, ref slope);

        

            for (int i = 0; i < interests.Count; i++)
            {
                Results sim = RunAstar3d(person, interests[i], obstacles, floor, slope, Person_Height);
                tree.AddRange(sim.Path, new Grasshopper.Kernel.Data.GH_Path(i));
                Closed = sim.Closed;
                warning = sim.Warning;
                error = sim.Error;
            }
            DA.SetDataTree(0, tree);
            DA.SetDataList(1, Closed);


            if ( warning!= null)
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, warning);
            if (error != null)
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, error);

        }

        public struct Results
        {
           
            public List<Point3d> Path { get; set; }
            public List<Point3d> Closed { get; set; }
            public String Warning { get; set; }
            public String Error { get; set; }

        }

        

        public static Results RunAstar3d(Point3d person, Point3d interest, List<Mesh> obstacles, List<Mesh> Floors, double slope, double Person_Height)
        {

            //Run solver

            //Declare Lists
            List<A_star> open = new List<A_star>();
            List<A_star> closed = new List<A_star>();
            List<Point3d> Pointpaths = new List<Point3d>();
            List<Point3d> finalPath = new List<Point3d>();
            List<A_star> children = new List<A_star>();
            List<Point3d> closedlist = new List<Point3d>();




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


            ////Interest on mesh
            List<Point3d> intList = new List<Point3d>
            {
                interest
            };
            try
            {
                Point3d[] templist = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(Floors, intList, new Vector3d(0, 0, -1), 0);
                double mindist = double.MaxValue;
                Point3d temp_pos = interest;
                for (int i = 0; i < templist.Length; i++)
                {
                    double dist = templist[i].DistanceTo(interest);
                    if (dist < mindist)
                    {
                        temp_pos = templist[i];
                        mindist = dist;
                    }
                }
                interest = temp_pos;
            }
            catch (Exception)
            {

                foreach (A_star node in closed)
                {
                    closedlist.Add(node.Position);
                }
                
                return new Results { Path = finalPath, Closed = closedlist, Error = "Interest not placed above a floor" }; ;

            }

            ////Start on mesh
            List<Point3d> startList = new List<Point3d>
            {
                person
            };
            try
            {
                Point3d[] templist = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(Floors, startList, new Vector3d(0, 0, -1), 0);
                double mindist = double.MaxValue;
                Point3d temp_pos = person;
                for (int i = 0; i < templist.Length; i++)
                {
                    double dist = templist[i].DistanceTo(person);
                    if (dist < mindist)
                    {
                        temp_pos = templist[i];
                        mindist = dist;
                    }
                }
                person = temp_pos;
            }
            catch (Exception)
            {

                foreach (A_star node in closed)
                {
                    closedlist.Add(node.Position);
                }
                return new Results { Path = finalPath, Closed = closedlist, Error = "Person not placed above a floor" }; ;

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
                    //Check for final obstacle
                    Vector3d vectorToFinal = interest - currentNode.Position;
                    Ray3d ray = new Ray3d(currentNode.Position, vectorToFinal);
                    double rayT = Rhino.Geometry.Intersect.Intersection.MeshRay(obsMesh, ray);
                    if (rayT < 0 | ray.PointAt(rayT).DistanceTo(currentNode.Position) > 2)
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
                        
                        
                        foreach (A_star node in closed)
                        {
                            closedlist.Add(node.Position);
                        }
                        return new Results { Path = finalPath, Closed = closedlist };
                    }

                }


                //Generate children
                children.Clear();
                foreach (Point3d new_position in move)
                {
                    List<Point3d> stupidlist = new List<Point3d>();

                    Point3d child_position = new Point3d(currentNode.Position.X + new_position.X, currentNode.Position.Y + new_position.Y, currentNode.Position.Z + new_position.Z);
                    child_position.Z = child_position.Z + Person_Height;
                    stupidlist.Add(child_position);
                    //Project point to terrain
                    //Try z positive
                    try
                    {
                        Point3d[] child_positions = Rhino.Geometry.Intersect.Intersection.ProjectPointsToMeshes(Floors, stupidlist, new Vector3d(0, 0, -1), 0);
                        double mindist = double.MaxValue;
                        Point3d temp_child = child_position;
                        for (int i = 0; i < child_positions.Length; i++)
                        {
                            double dist = child_positions[i].DistanceTo(child_position);
                            if (dist < mindist)
                            {
                                temp_child = child_positions[i];
                                mindist = dist;
                            }
                        }
                        child_position = temp_child;
                    }
                    catch (Exception)
                    {


                        continue;

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
                        if ((Math.Atan(terrain_slope)*180/Math.PI)  > slope)
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

                    double height_dif = child.Position.Z - interest.Z;

                    child.g = currentNode.g + 1;
                    child.h = child.Position.DistanceTo(end.Position) + (Math.Abs(height_dif) * 100);
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


                    if (open.Count > 100000)
                    {
                        run = false;
                    }
                }










            }
            
            foreach (A_star node in closed)
            {
                closedlist.Add(node.Position);
            }
            return new Results { Path = finalPath, Closed = closedlist, Error = $"Exceeded count threshold of {open.Count}" };


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
            get { return new Guid("f6dce5a6-6796-4ee8-a5ae-110960e44906"); }
        }
    }
}