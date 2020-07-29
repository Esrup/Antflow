using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using System.Threading.Tasks;


namespace Antflow.Traffic.Class
{
    public class ParkingLot 
    {

        public int NoParkinglots { get; }

        //Declare List
        public List<Curve> parkingSpaces = new List<Curve>();
        private List<Curve> CenterLines = new List<Curve>();
        private List<Curve> parkingEnds = new List<Curve>();

     

        public ParkingLot(Curve bounderyCurve, LocalRules localRules, Plane plane)
        {
            CreateParkingLot(bounderyCurve, localRules, plane);
        }


        private void CreateParkingLot(Curve bounderyCurve, LocalRules localRules, Plane plane)
        {
            //TODO Change Curve offset
            Curve[] offsetBoundery = bounderyCurve.Offset(Plane.WorldXY, localRules.BoothDepth, 0.0, CurveOffsetCornerStyle.Sharp);
            offsetBoundery = Curve.JoinCurves(offsetBoundery, 0.5);

            //Make Outer parkingplaces
            List<Curve> outerParkingLines = new List<Curve>();
            double[] divpar = offsetBoundery[0].DivideByLength(localRules.BoothWidth, true, false, out Point3d[] points);
            for (int i = 0; i < points.Length; i++)
            {
                
                Vector3d Dir = offsetBoundery[0].TangentAt(divpar[i]);
                Dir.PerpendicularTo(Dir);
                Dir.Unitize();
                Dir = Vector3d.Multiply(localRules.BoothDepth, Dir);
                //One way
                Line parkingLine = new Line(points[i], Dir);
                //Check direction and if wrong reverse
                if (offsetBoundery[0].Contains(parkingLine.PointAtLength(0.5), plane, 0.0) != PointContainment.Outside)
                {
                    Dir.Reverse();
                    parkingLine = new Line(points[i], Dir);
                }

                

                outerParkingLines.Add(parkingLine.ToNurbsCurve());
                
            }
            
            
            //Check for overlapping line caused by degrees larger than 180 
            List<int> intertsectionCounts = new List<int>();

            for (int j = 0; j < outerParkingLines.Count; j++)
            {
                int counter = 0;
                for (int k = j + 1; k < outerParkingLines.Count; k++)
                {
                    CurveIntersections intersectionEvent = Rhino.Geometry.Intersect.Intersection.CurveCurve(outerParkingLines[j], outerParkingLines[k], 0.00, 0.00);
                    if (intersectionEvent.Count > 0)
                    {
                        counter++;
                        break;
                    }
                }
                intertsectionCounts.Add(counter);
                
            }

            for (int l = 0; l < outerParkingLines.Count; l++)
            {
                if (intertsectionCounts[l] == 0)
                {
                    parkingSpaces.Add(outerParkingLines[l]);
                }
                
            }


            //Make bounderybox and starterline
            Curve innerBounderyCrv = bounderyCurve.Offset(plane, (localRules.BoothDepth + localRules.ManeuveringArea), 0.5, CurveOffsetCornerStyle.Sharp)[0];
            BoundingBox bounderyBox = innerBounderyCrv.GetBoundingBox(plane);
            Point3d[] boxCorners = bounderyBox.GetCorners();
            Line startLine = new Line(boxCorners[0], boxCorners[1]);
            Line endLine = new Line(boxCorners[3], boxCorners[2]);

            // Direction of minner booths
            Vector3d boothDir = startLine.UnitTangent;
            boothDir.Unitize();
            boothDir = Vector3d.Multiply(localRules.BoothDepth, boothDir);

            //Copy center lines according to distancerules
            List<Line> copyLines = new List<Line>();
            for (double i = 0; i < startLine.Length;)
            {
                Point3d startPT = startLine.PointAtLength(i);
                Point3d endPT = endLine.PointAtLength(i);
                copyLines.Add(new Line(startPT, endPT));
                i = i + (localRules.BoothDepth * 2) + localRules.ManeuveringArea;
            }

            //Find inner curves
            foreach (Line line in copyLines)
            {
                Curve crvline = line.ToNurbsCurve();
                var curveEvent = Intersection.CurveCurve(innerBounderyCrv, crvline, 0, 0);
                if (curveEvent != null)
                {
                    List<double> curveT = new List<double>();
                    for (int i = 0; i < curveEvent.Count; i++)
                    {
                        var crvLine = curveEvent[i];
                        curveT.Add(crvLine.ParameterB);

                    }

                    Curve[] curveSplit = crvline.Split(curveT);

                    foreach (Curve curve in curveSplit)
                    {

                        double midT = (curve.Domain.T0 + curve.Domain.T1) / 2;

                        if (innerBounderyCrv.Contains(curve.PointAt(midT), Plane.WorldXY, 0).Equals(PointContainment.Inside))
                        {
                            CenterLines.Add(curve);
                        }
                    }

                }
            }

            //Parking Booths
            foreach (Curve curve in CenterLines)
            {
                List<Curve> parkingLines = new List<Curve>();
                curve.DivideByLength(localRules.BoothWidth, true, false, out Point3d[] midpoints);
                foreach (Point3d point in midpoints)
                {
                    //One way
                    Line parkingLine = new Line(point, boothDir);
                    parkingLines.Add(parkingLine.ToNurbsCurve());
                    //Opposite way
                    boothDir.Reverse();
                    parkingLine = new Line(point, boothDir);
                    parkingLines.Add(parkingLine.ToNurbsCurve());
                }

                //Check for collision with driving path

                List<Curve> finalParkingLines = new List<Curve>();
                for (int i = 0; i < parkingLines.Count; i++)
                {
                    CurveIntersections crvEvent = Intersection.CurveCurve(innerBounderyCrv, parkingLines[i], 0.0001, 0.0000);
                    if (crvEvent.Count > 0)
                    {
                        parkingEnds.Add(parkingLines[i]);

                    }
                    else
                    {
                        finalParkingLines.Add(parkingLines[i]);
                    }
                }



                parkingSpaces.AddRange(finalParkingLines);
            }


            parkingSpaces.AddRange(CenterLines);
        }
    }


}
