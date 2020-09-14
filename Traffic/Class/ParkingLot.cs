using System;
using System.Collections.Generic;

using System.Text;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using System.Threading.Tasks;
using Rhino;

namespace Antflow.Traffic.Class
{
    public class ParkingLot 
    {

        public int NoParkinglots { get; }

        //Declare List
        public List<Curve> parkingSpaces = new List<Curve>();
        public List<Curve> notParkingSpaces = new List<Curve>();
        public int parkingspaceNO;
        private List<Curve> CenterLines = new List<Curve>();
        private List<Curve> parkingEnds = new List<Curve>();
        private List<Curve> parkingLinesLeft = new List<Curve>();
        private List<Curve> parkingLinesRight = new List<Curve>();

        //Debugger
        public List<Box> debug = new List<Box>();



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


            //Make boundarybox and starterline
            Curve innerBounderyCrv = bounderyCurve.Offset(plane, (localRules.BoothDepth + localRules.ManeuveringArea), 0.5, CurveOffsetCornerStyle.Sharp)[0];
            Box boundarybox;
            innerBounderyCrv.GetBoundingBox(plane, out boundarybox);
            debug.Add(boundarybox);
            Point3d[] boxCorners = boundarybox.GetCorners();
            Line startLine = new Line(boxCorners[0], boxCorners[1]);
            Line endLine = new Line(boxCorners[3], boxCorners[2]);

            // Direction of inner booths
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
                curve.DivideByLength(localRules.BoothWidth, true, false, out Point3d[] midpoints);
                if (midpoints.Length == 0)
                {
                    continue;
                }
                foreach (Point3d point in midpoints)
                {
                    //One way
                    Line parkingLine = new Line(point, boothDir);
                    parkingLinesLeft.Add(parkingLine.ToNurbsCurve());
                    //Opposite way
                    boothDir.Reverse();
                    parkingLine = new Line(point, boothDir);
                    parkingLinesRight.Add(parkingLine.ToNurbsCurve());
                    //Return Vector
                    boothDir.Reverse();
                }

                PointCloud startPTsLeft = new PointCloud();
                PointCloud startPTsRight = new PointCloud();
                PointCloud allStartPT = new PointCloud();

                //Check for collision with driving path

                List<Curve> finalParkingLinesLeft = new List<Curve>();
                List<Curve> finalParkingLinesRight = new List<Curve>();
                for (int i = 0; i < parkingLinesLeft.Count; i++)
                {
                    CurveIntersections crvEvent = Intersection.CurveCurve(innerBounderyCrv, parkingLinesLeft[i], 0.0001, 0.0000);
                    if (crvEvent.Count > 0)
                    {
                        parkingEnds.Add(parkingLinesLeft[i]);

                    }
                    else
                    {
                        finalParkingLinesLeft.Add(parkingLinesLeft[i]);
                        startPTsLeft.Add(parkingLinesLeft[i].PointAtStart);
                        allStartPT.Add(parkingLinesLeft[i].PointAtStart);
                    }
                }

                for (int i = 0; i < parkingLinesRight.Count; i++)
                {
                    CurveIntersections crvEvent = Intersection.CurveCurve(innerBounderyCrv, parkingLinesRight[i], 0.0001, 0.0000);
                    if (crvEvent.Count > 0)
                    {
                        parkingEnds.Add(parkingLinesRight[i]);

                    }
                    else
                    {
                        finalParkingLinesRight.Add(parkingLinesRight[i]);
                        startPTsRight.Add(parkingLinesRight[i].PointAtStart);
                        allStartPT.Add(parkingLinesRight[i].PointAtStart);
                    }
                }




                try
                {
                    //Fix one end
                    Point3d crvEnd = curve.PointAtEnd;
                    Curve endCurve = createEnds(innerBounderyCrv, startPTsLeft, startPTsRight, finalParkingLinesLeft, finalParkingLinesRight, crvEnd);
                    notParkingSpaces.Add(endCurve);
                    Curve splitCurveOne = splitend(curve, allStartPT, crvEnd);

                    //Fix the other end
                    Point3d crvStart = curve.PointAtStart;
                    endCurve = createEnds(innerBounderyCrv, startPTsLeft, startPTsRight, finalParkingLinesLeft, finalParkingLinesRight, crvStart);
                    notParkingSpaces.Add(endCurve);
                    notParkingSpaces.Add(splitend(splitCurveOne, allStartPT, crvStart));

                }
                catch
                {

                }


                parkingSpaces.AddRange(finalParkingLinesLeft);
                parkingSpaces.AddRange(finalParkingLinesRight);

                parkingspaceNO = parkingSpaces.Count;

                

            }


            //parkingSpaces.AddRange(CenterLines);
        }

        private Curve splitend(Curve curve, PointCloud allStartPT, Point3d crvEnd)
        {
            //split ends of centerline
            int ptIndex = allStartPT.ClosestPoint(crvEnd);
            curve.ClosestPoint(allStartPT.PointAt(ptIndex), out double centerT);
            Curve[] splitCenterCurves = curve.Split(centerT);
            Curve splitCenter;
            if (splitCenterCurves[0].GetLength() < splitCenterCurves[1].GetLength())
            {
                splitCenter = splitCenterCurves[1];
            }
            else
            {
                splitCenter = splitCenterCurves[0];
            }
            return splitCenter;
        }

        private static Curve createEnds(Curve innerBounderyCrv, PointCloud startPTsLeft, PointCloud startPTsRight, List<Curve> finalParkingLinesLeft, List<Curve> finalParkingLinesRight, Point3d crvEnd)
        {
            List<double> CurveT = new List<double>();
            CurveIntersections intersectionsOne;
            Curve lineOne;
            Curve lineTwo;
            CurveIntersections intersectionsTwo;

            //Find end Left at make intersection with inner Crv
            closeIntersect(innerBounderyCrv, startPTsLeft, finalParkingLinesLeft, crvEnd, out lineOne, out intersectionsOne);

            //Find end Right at make intersection with inner Crv
            closeIntersect(innerBounderyCrv, startPTsRight, finalParkingLinesRight, crvEnd, out lineTwo, out intersectionsTwo);

            //Find intersections and make polyline
            CurveT.Add(intersectionsOne[0].ParameterA);
            CurveT.Add(intersectionsTwo[0].ParameterA);

            //Split the innerboundarycurve
            Curve[] splitCurves = innerBounderyCrv.Split(CurveT);

            //Add the curve together
            List<Curve> curves = new List<Curve>();
            if (splitCurves[0].GetLength() < splitCurves[1].GetLength())
            {
                curves.Add(splitCurves[0]);
            }
            else
            {
                curves.Add(splitCurves[1]);
            }

            curves.Add(new Line(lineOne.PointAtEnd, innerBounderyCrv.PointAt(CurveT[0])).ToNurbsCurve());
            curves.Add(new Line(lineTwo.PointAtEnd, innerBounderyCrv.PointAt(CurveT[1])).ToNurbsCurve());
            Curve[] joinedCurve = Curve.JoinCurves(curves);
            Curve endCurve = Curve.CreateFilletCornersCurve(joinedCurve[0], 1, 0.1, 1);
            return endCurve;
        }

        private static void closeIntersect(Curve innerBounderyCrv, PointCloud startPTsLeft, List<Curve> finalParkingLinesLeft, Point3d crvEnd, out Curve lineOne, out CurveIntersections intersectionsOne)
        {
            int indexOne = startPTsLeft.ClosestPoint(crvEnd);
            lineOne = finalParkingLinesLeft[indexOne];

            Vector3d vectorone = lineOne.TangentAtEnd;
            vectorone.Rotate(RhinoMath.ToRadians(90), new Vector3d(0, 0, 1));
            Point3d movedPTone = lineOne.PointAtStart + vectorone;
            vectorone.Rotate(RhinoMath.ToRadians(180), new Vector3d(0, 0, 1));
            Point3d movedPTTwo = lineOne.PointAtStart + vectorone;
            PointCloud directionPT = new PointCloud();
            directionPT.Add(movedPTone);
            directionPT.Add(movedPTTwo);
            
            int indexdir = directionPT.ClosestPoint(crvEnd);
            if (indexdir == 0)
            {
                vectorone.Rotate(RhinoMath.ToRadians(180), new Vector3d(0, 0, 1));
            }
            vectorone = Vector3d.Multiply(vectorone, 10);
            intersectionsOne = Intersection.CurveCurve(innerBounderyCrv, new Line(lineOne.PointAtEnd, vectorone).ToNurbsCurve(), 0.01, 0.01);
        }
    }


}
