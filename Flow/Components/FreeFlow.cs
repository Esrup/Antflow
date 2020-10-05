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
using System.Threading.Tasks;
using Grasshopper.Kernel;
using Rhino.Geometry;
using GH_IO;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace Antflow
{
    public class FreeFlow : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public FreeFlow()
          : base("FreeFlow", "Flow",
              "Description",
              "AntFlow", "Flow")
        {
        }

       
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("StartingPoint", "StartingPoint", "StartingPoint", GH_ParamAccess.item);
            pManager.AddPointParameter("Interest", "Interest", "Interest", GH_ParamAccess.list);
            pManager.AddMeshParameter("Obstacles", "Obstacles", "Obstacles", GH_ParamAccess.list);
            pManager.AddNumberParameter("Distance", "Distance", "Distance", GH_ParamAccess.item);
            pManager.AddIntegerParameter("ViewAngle", "ViewAngle", "ViewAngle", GH_ParamAccess.item, 360);
            pManager.AddIntegerParameter("ViewDistance", "ViewDistance", "ViewDistance", GH_ParamAccess.item, 30);
        }

      
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddPointParameter("Person", "Person", "Person", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "Direction", "Direction", GH_ParamAccess.item);
            pManager.AddPointParameter("Path", "Path", "Path", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare Variables
            Point3d startingPoint = new Point3d(0,0,0);
            List<Point3d> interests = new List<Point3d>();
            List<Mesh> obstacles = new List<Mesh>();
            double distance = 0;
            int viewDist = 30;
            int viewAngle = 180;
            Mesh obsMesh = new Mesh();
            double targetradius = 1.1;
            double walkingdist = 1;


            

            //Attach inputs
            DA.GetData(0, ref startingPoint);
            DA.GetDataList(1, interests);
            DA.GetDataList(2, obstacles);
            DA.GetData(3, ref distance);
            DA.GetData(4, ref viewAngle);
            DA.GetData(5, ref viewDist);

            // Join obstacles in one Mesh
            foreach (Mesh mesh in obstacles)
            {
                obsMesh.Append(mesh);
            }

            //Starting

            //Create Person
            Person firstperson = new Person(startingPoint, interests);

            //While loop (Moving)
            while (firstperson.TravedledDist < distance)
            {


                //Check if ESC is pressed and if true, Abort
                if (GH_Document.IsEscapeKeyDown())
                {
                    GH_Document GHDocument = OnPingDocument();
                    GHDocument.RequestAbortSolution();
                }

                    //Find Target

                    PointCloud interestcloud = new PointCloud(firstperson.Interests);

                Boolean targetisinterest = true;

                //Check if any interests are left
                if (interestcloud.Count < 1)
                {
                    //If no interest left, move along Y axis
                    firstperson.UpdateTarget(firstperson.Position + new Point3d(0, walkingdist, 0));
                    targetisinterest = false;
                    break;
                }

                while (true)
                {
                    
                    
                    //Find closets point
                    int intindex = interestcloud.ClosestPoint(firstperson.Position);
                    if (firstperson.Position.DistanceTo(interestcloud[intindex].Location) > viewDist)
                    {
                        //If closets point is not within viewdist, move along Y axis
                        firstperson.UpdateTarget(firstperson.Position + new Point3d(0, walkingdist, 0));
                        targetisinterest = false;
                        break;
                    }
                    
                    //Check for within fieldview
                    if (viewAngle < Rhino.RhinoMath.ToDegrees( Vector3d.VectorAngle(firstperson.Facing, new Vector3d(interestcloud[intindex].X - firstperson.Position.X, interestcloud[intindex].Y - firstperson.Position.Y, interestcloud[intindex].Z - firstperson.Position.Z))))
                    {
                        //If the interest is not in view remove interest from interestcloud
                        interestcloud.RemoveAt(intindex);
                        continue;
                    }


                    //Check for view collision
                    Ray3d visibleRay = new Ray3d(firstperson.Position, new Vector3d(interestcloud[intindex].X - firstperson.Position.X, interestcloud[intindex].Y - firstperson.Position.Y, interestcloud[intindex].Z - firstperson.Position.Z));
                    double RayT = Rhino.Geometry.Intersect.Intersection.MeshRay(obsMesh, visibleRay);
                    if (RayT > 0)
                    {
                        //If there is no direct view remove interest from interestcloud
                        interestcloud.RemoveAt(intindex);
                        continue;
                    }
                    else
                    {
                        firstperson.UpdateTarget(interestcloud[intindex].Location);
                        break;
                    }
                }

                
                //Check if arrived at Target
                if (firstperson.Position.DistanceTo(firstperson.CurrentTarget) < targetradius & targetisinterest == true)
                {
                    firstperson.removeinterest();
                    continue;
                }

               
                //Move in direction
                Vector3d direction = new Vector3d(firstperson.CurrentTarget.X - firstperson.Position.X, firstperson.CurrentTarget.Y - firstperson.Position.Y, firstperson.CurrentTarget.Z - firstperson.Position.Z);
                direction.Unitize();
                direction = Rhino.Geometry.Vector3d.Multiply(direction, walkingdist);

                Point3d newpos = firstperson.Position + direction;
                Boolean update = firstperson.UpdatePos(newpos, direction);

                DA.SetData(0, firstperson.Position);
                DA.SetData(1, firstperson.Facing);

                //System.Timers.Timer timer = new System.Timers.Timer(1000);


            }

            DA.SetDataList(2, firstperson.Path);








        }


        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return null;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("80270e40-25fe-4615-bba1-a1d0d6dba65e"); }
        }
    }



}
