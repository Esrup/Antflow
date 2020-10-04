using System;
using System.Collections.Generic;
using Antflow.Traffic.Class;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using System.Threading.Tasks;

namespace Antflow.Traffic
{
    public class Parking : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the Parking class.
        /// </summary>
        public Parking()
          : base("Parking", "Parking",
              "Description",
              "AntFlow", "Traffic")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddCurveParameter("Boundery", "Boundery", "Boundery", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Plane", "Plane", "Plane", GH_ParamAccess.item);
            pManager[1].Optional = true;
            pManager.AddGeometryParameter("Obstacles", "Obstacles", "Obstacles", GH_ParamAccess.list);
            pManager[2].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Parkingspaces", "Parkingspaces", "Parkingspaces", GH_ParamAccess.list);
            pManager.AddGeometryParameter("Otherlines", "Otherlines", "Otherlines", GH_ParamAccess.list);
            pManager.AddGeometryParameter("Debug", "Debug", "Debug", GH_ParamAccess.list);
            
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare inputs
            Curve BounderyCrv = null;
            Plane? xyPlane = null;
            List<GeometryBase> obstacles = new List<GeometryBase>();

            //Get inputs
            DA.GetData(0, ref BounderyCrv);
            DA.GetData(1, ref xyPlane);
            DA.GetDataList(2, obstacles);

            //Select localrules TODO
            var localrules = new LocalRules();

            //Create Parkinglots - Check if there is a plane userinput
            ParkingLot parkingLot;

            if (xyPlane.HasValue == false)
            {
                //Create Parking lot with the best Plane orientation
                int[] numberOfParkinglots = new int[180];
                int[] degrees = new int[180];
                int maxNoLots = int.MinValue;
                int degreeIndex = 0;

                Parallel.For(0, 180, i =>
                {
                    Plane newPlane = Plane.WorldXY;
                    newPlane.Rotate(i*(Math.PI / 180.0), new Vector3d(0, 0, 1));
                    try
                    {
                        var tempParkingLot = new ParkingLot(BounderyCrv, localrules, newPlane, obstacles);

                        numberOfParkinglots.SetValue(tempParkingLot.parkingspaceNO, i);
                        degrees.SetValue(i, i);

                    }
                    catch
                    {

                    }

                });

                for (int i = 0; i < 180; i++)
                {
                    try
                    {
                        if (numberOfParkinglots[i] > maxNoLots)
                        {
                            maxNoLots = numberOfParkinglots[i];
                            degreeIndex = i;
                        }

                    }
                    catch 
                    {

                       
                    }
                }
                Plane bestPlane = Plane.WorldXY;
                bestPlane.Rotate(degreeIndex * (Math.PI / 180.0), new Vector3d(0, 0, 1));
                parkingLot = new ParkingLot(BounderyCrv, localrules, bestPlane, obstacles);
            }
            else
            {
            //Create Parking lot with user choosen Plane
            
            parkingLot = new ParkingLot(BounderyCrv, localrules, xyPlane.Value, obstacles);
            }

            List<Curve> SpaceCurves = new List<Curve>();
            List<Rectangle3d> debuging = new List<Rectangle3d>();
            foreach (Parkingspace lot in parkingLot.Spaces)
            {
                SpaceCurves.Add(lot.Curves);
                //debuging.Add(lot.ManeuveringBox);
            }
            
            //Set Outputs
            DA.SetDataList(0, SpaceCurves);
            DA.SetDataList(1, parkingLot.notParkingSpaces);
            DA.SetDataList(2, parkingLot.debug);




        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
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
            get { return new Guid("ee0a53f4-7e57-4e66-9d54-c3406f25d2bd"); }
        }
    }
}