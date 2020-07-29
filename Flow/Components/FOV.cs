using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;


namespace Antflow
{
    public class FOV : GH_Component
    {
      
        public FOV()
          : base("Field of View", "FOV",
              "Description",
              "AntFlow", "Flow")
        {
        }

 
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPointParameter("Person", "Person", "Person", GH_ParamAccess.item);
            pManager.AddVectorParameter("Direction", "Direction", "Direction", GH_ParamAccess.item);
            pManager.AddNumberParameter("ViewDistance", "ViewDistance", "ViewDistance", GH_ParamAccess.item);
            pManager.AddIntegerParameter("ViewAngle", "ViewAngle", "ViewAngle", GH_ParamAccess.item);
            pManager.AddMeshParameter("Obstacles", "Obstacles", "Obstacles", GH_ParamAccess.list);
        }


        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("ViewMesh", "ViewMesh", "ViewMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("FreeFOVAngle", "FreeFOVAngle", "FreeFOVAngle", GH_ParamAccess.item);
        }

         protected override void SolveInstance(IGH_DataAccess DA)
        {
            Point3d person = new Point3d(0,0,0);
            int angle = 0;
            Vector3d direction = new Vector3d(0,0,0);
            double viewDistance = 0;
            List<Mesh> obstacles = new List<Mesh>();


            List<Ray3d> viewRays = new List<Ray3d>();
            List<Point3d> Rayhits = new List<Point3d>();
            int fovfree = 0;

            Mesh meshes = new Mesh();




            DA.GetData(0, ref person);
            DA.GetData(1, ref direction);
            DA.GetData(2, ref viewDistance);
            DA.GetData(3, ref angle);
            DA.GetDataList(4, obstacles);

            int viewRes = angle;
            
            foreach (Mesh mesh in obstacles)
            {
                meshes.Append(mesh);
            }



            direction.Z = 0;


            direction.Rotate(Rhino.RhinoMath.ToRadians(angle / 2), new Vector3d(0, 0, 1));
            Line viewAngleA = new Line(person, direction, viewDistance);

            direction.Rotate(-Rhino.RhinoMath.ToRadians(angle / 2) * 2, new Vector3d(0, 0, 1));
            Line viewAngleB = new Line(person, direction, viewDistance);

            List<Line> Liness = new List<Line>();
            Liness.Add(viewAngleA);
            Liness.Add(viewAngleB);

            for (int i = 0; i < viewRes + 2; i++)
            {
                if (i > 1)
                {
                    direction.Rotate(Rhino.RhinoMath.ToRadians((angle / viewRes)), new Vector3d(0, 0, 1));
                }

                Ray3d ray = new Ray3d(person, direction);
                double rayT = Rhino.Geometry.Intersect.Intersection.MeshRay(meshes, ray);

                Vector3d newvec = Rhino.Geometry.Vector3d.Multiply(direction, viewDistance);

                if (rayT > 0)
                {
                    if (person.DistanceTo(ray.PointAt(rayT)) < viewDistance)
                    {
                        Rayhits.Add(ray.PointAt(rayT));


                    }
                    else
                    {
                        Rayhits.Add(new Point3d(newvec.X + person.X, newvec.Y + person.Y, person.Z));
                        fovfree = fovfree + 1;
                    }
                }
                else
                {
                    Rayhits.Add(new Point3d(newvec.X + person.X, newvec.Y + person.Y, person.Z));
                    fovfree = fovfree + 1;
                }
            }

            Mesh viewMesh = new Mesh();
            viewMesh.Vertices.UseDoublePrecisionVertices = true;

            //Make mesh from hitspoints
            viewMesh.Vertices.Add(new Point3f((float)person.X, (float)person.Y, (float)person.Z));
            viewMesh.VertexColors.Add(System.Drawing.Color.Ivory);

            for (int i = 0; i < Rayhits.Count; i++)
            {
                viewMesh.Vertices.Add(new Point3f((float)Rayhits[i].X, (float)Rayhits[i].Y, (float)Rayhits[i].Z));
                viewMesh.VertexColors.Add(System.Drawing.Color.Ivory);
            }
            for (int i = 0; i < (viewRes + 2); i++)
            {
                viewMesh.Faces.AddFace(0, i, i + 1);
            }


            viewMesh.Faces.RemoveAt(0);



            DA.SetData(0, viewMesh);
            DA.SetData(1, fovfree);


         

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
            get { return new Guid("6227b912-4673-484a-bd28-2614fe23650f"); }
        }
    }
}