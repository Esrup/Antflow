using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Antflow
{
    public class Shortest_Walk : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Shortest_Walk class.
        /// </summary>
        public Shortest_Walk()
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

            //Declare lists
            List<Point3d> path = new List<Point3d>();
            Grasshopper.DataTree<Point3d> tree = new Grasshopper.DataTree<Point3d>();

            //Get Inputs
            DA.GetData(0, ref person);
            DA.GetDataList(1, interests);
            DA.GetDataList(2, obstacles);

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
                path = RunAstar(person, interests[i], obstacles);
                tree.AddRange(path, new Grasshopper.Kernel.Data.GH_Path(i));
            }
            DA.SetDataTree(0, tree);
        }

        public static List<Point3d> RunAstar(Point3d person, Point3d interest, List<Mesh> obstacles)
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
                    Point3d child_position = new Point3d(currentNode.Position.X + new_position.X, currentNode.Position.Y + new_position.Y, currentNode.Position.Z + new_position.Z);

                    //Check if new point is in obstacle

                    if (obsMesh.IsPointInside(child_position, 0, false))
                    {
                        continue;
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
                        if(child.Position.DistanceTo(close.Position) < 1)
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
            get { return new Guid("eb30a4f6-cba2-4d38-8761-d9481ac2a9a0"); }
        }
    }
}