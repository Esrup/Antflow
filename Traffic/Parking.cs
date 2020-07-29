using System;
using System.Collections.Generic;
using Antflow.Traffic.Class;
using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

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
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddCurveParameter("Parkingspaces", "Parkingspaces", "Parkingspaces", GH_ParamAccess.list);
            
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            //Declare inputs
            Curve BounderyCrv = null;
            Plane xyPlane = Plane.WorldXY;

            //Get inputs
            DA.GetData(0, ref BounderyCrv);

            //Select localrules TODO
            var localrules = new LocalRules();

            //Create Parking lot
            var parkingLot = new ParkingLot(BounderyCrv, localrules, xyPlane);
            
            //Set Outputs
            DA.SetDataList(0, parkingLot.parkingSpaces);
            
           
            

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