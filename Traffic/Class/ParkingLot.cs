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
        public List<Parkingspace> Spaces = new List<Parkingspace>();
        public List<Curve> notParkingSpaces = new List<Curve>();
        public int parkingspaceNO;
        private List<Curve> CenterLines = new List<Curve>();
        private List<Curve> parkingEnds = new List<Curve>();
        private List<Parkingspace> parkingLinesLeft = new List<Parkingspace>();
        private List<Parkingspace> parkingLinesRight = new List<Parkingspace>();

        //DEBUG
        public List<Curve> debug = new List<Curve>();



        public ParkingLot(Curve bounderyCurve, LocalRules localRules, Plane plane, List<GeometryBase> obstacles)
        {
            CreateOuterParkingLot(bounderyCurve, localRules, plane);
            CreateInnerParkinglots(bounderyCurve, localRules, plane);
            clearObstacles(obstacles);
        }


        //Create Parkinglots along edge
        private void CreateOuterParkingLot(Curve boundaryCurve, LocalRules localRules, Plane plane)
        {
            
            //Change Curve offset
            BoundingBox bx = boundaryCurve.GetBoundingBox(Plane.WorldXY);
            Point3d[] bxPT = bx.GetCorners();
            Vector3d outsideVector = bxPT[0] - bxPT[2];
            outsideVector.Unitize();
            Point3d outsidePT = bxPT[0] + outsideVector;
            Curve[] offsetBoundary = boundaryCurve.Offset(outsidePT, Vector3d.ZAxis, -localRules.BoothDepth, 0.01, CurveOffsetCornerStyle.Sharp);

            //Sometimes offset fails. This checks for that and uses alternative method, less precise method
            if (offsetBoundary[0].GetLength() < (localRules.BoothDepth*2) + Math.Sqrt((Math.Pow(localRules.BoothDepth, 2) * 2)) + 1)
            {
                offsetBoundary = boundaryCurve.Offset(Plane.WorldXY, localRules.BoothDepth, 0.01, CurveOffsetCornerStyle.Sharp);
                //Check direction
                if (offsetBoundary[0].Contains(boundaryCurve.PointAtStart, Plane.WorldXY, 0.01) == PointContainment.Inside)
                {
                    offsetBoundary = boundaryCurve.Offset(Plane.WorldXY, -localRules.BoothDepth, 0.01, CurveOffsetCornerStyle.Sharp);
                }
            }

            List<Curve> explodeOffsetBoundary = new List<Curve>();
            foreach (var curve in offsetBoundary)
            {
                if(curve is PolylineCurve)
                {
                    explodeOffsetBoundary.AddRange(curve.DuplicateSegments());
                }
                else
                {
                    explodeOffsetBoundary.Add(curve);
                }
            }

            Curve[] offsetBoundaryjoined;
            if (offsetBoundary.Length > 2)
            {
                offsetBoundaryjoined = Curve.JoinCurves(offsetBoundary, 0.5);
            }
            else
            {
                offsetBoundaryjoined = offsetBoundary;
            }

            

            //Make Outer parkingplots

            foreach (var curve in explodeOffsetBoundary)
            {
                List<Parkingspace> tempSpaces = new List<Parkingspace>();
                double[] divpar = curve.DivideByLength(localRules.BoothWidth, true, false, out Point3d[] points);

                //Create Booths
                for (int i = 0; i < points.Length; i++)
                {

                    Vector3d Dir = curve.TangentAt(divpar[i]);
                    Dir.Rotate(Math.PI / 2, Vector3d.ZAxis);
                    Dir.Unitize();
                    if (boundaryCurve.ClosedCurveOrientation() == CurveOrientation.CounterClockwise)
                    {

                        Dir.Reverse();
                    }
                    Dir = Vector3d.Multiply(localRules.BoothDepth, Dir);

                    // Create new parking
                    Plane boothPlane = new Plane(points[i], curve.TangentAt(divpar[i]), Dir);
                    Parkingspace parkingspace = new Parkingspace(boothPlane, localRules, false);

                    
                    tempSpaces.Add(parkingspace);



                }
                 
                List<int> spacesToCull = new List<int>();

                // Check if Parkinglot fullfill criterias
                if (boundaryCurve.IsPeriodic != true)
                {
                    for (int j = 0; j < tempSpaces.Count; j++)
                    {
                        Curve boxCurves = tempSpaces[j].ManeuveringBox.ToNurbsCurve();

                        //Check for maneuveringarea
                        if (Intersection.CurveCurve(offsetBoundaryjoined[0], boxCurves, 0.0, 0.001).Count > 1)
                        {

                            spacesToCull.Add(j);
                            continue;
                        }
                        //Check if booths exceed the ends
                        if (offsetBoundaryjoined[0].Contains(tempSpaces[j].Curves.PointAtStart, plane, 0.001) != PointContainment.Coincident)
                        {
                            spacesToCull.Add(j);
                            continue;
                        }
                        if (offsetBoundaryjoined[0].Contains(tempSpaces[j].Curves.PointAtEnd, plane, 0.001) != PointContainment.Coincident)
                        {
                            spacesToCull.Add(j);
                            continue;
                        }
                    }
                    //Check if end booths overlap >180 degress corners
                    for (int i = 5; i < (tempSpaces.Count); i++)
                    {
                        if (spacesToCull.Contains(i))
                        {
                            continue;
                        }
                        if (boundaryCurve.Contains(tempSpaces[i].Curves.PointAtLength(localRules.BoothDepth), Plane.WorldXY, 0.001) != PointContainment.Coincident)
                        {
                            spacesToCull.Add(i);
                            continue;
                        }
                        if (boundaryCurve.Contains(tempSpaces[i].Curves.PointAtLength(tempSpaces[i].Curves.GetLength()-localRules.BoothDepth), Plane.WorldXY, 0.001) != PointContainment.Coincident)
                        {
                            spacesToCull.Add(i);
                            continue;
                        }
                    }
                }

                //Remove Parkinglots that doesn't fullfill criterias
                spacesToCull.Sort();
                spacesToCull.Reverse();
                foreach (int index in spacesToCull)
                {
                    tempSpaces.RemoveAt(index);
                }
                Spaces.AddRange(tempSpaces);
            }
        }

        //Create inner Parkinglots
        private void CreateInnerParkinglots(Curve boundaryCurve, LocalRules localRules, Plane plane)
        {

            //Create offset curve with outer booth and driveway distance
            BoundingBox bx = boundaryCurve.GetBoundingBox(Plane.WorldXY);
            Point3d[] bxPT = bx.GetCorners();
            Vector3d outsideVector = bxPT[0] - bxPT[2];
            outsideVector.Unitize();
            Point3d outsidePT = bxPT[0] + outsideVector;
            Curve[] innerBounderyCrv = boundaryCurve.Offset(outsidePT, Vector3d.ZAxis, -(localRules.BoothDepth + localRules.ManeuveringArea), 0.5, CurveOffsetCornerStyle.Sharp);
            
            //Sometimes offset fails. This checks for that and uses alternative method, less precise method
            if (innerBounderyCrv[0].GetLength() < ((localRules.BoothDepth + localRules.ManeuveringArea) * 2) + Math.Sqrt((Math.Pow(localRules.BoothDepth + localRules.ManeuveringArea, 2) * 2)) + 1)
            {
                innerBounderyCrv = boundaryCurve.Offset(Plane.WorldXY, localRules.BoothDepth + localRules.ManeuveringArea, 0.01, CurveOffsetCornerStyle.Sharp);
                //Check direction
                if (innerBounderyCrv[0].Contains(boundaryCurve.PointAtStart, Plane.WorldXY, 0.01) == PointContainment.Inside)
                {
                    innerBounderyCrv = boundaryCurve.Offset(Plane.WorldXY, -(localRules.BoothDepth + localRules.ManeuveringArea), 0.01, CurveOffsetCornerStyle.Sharp);
                }
            }
            debug.AddRange(innerBounderyCrv);
            //Create BB and centerlines
            innerBounderyCrv[0].GetBoundingBox(plane, out Box boundarybox);
            Point3d[] boxCorners = boundarybox.GetCorners();
            Line startLine = new Line(boxCorners[0], boxCorners[1]);
            Line endLine = new Line(boxCorners[3], boxCorners[2]);

            // Direction of inner booths
            Vector3d boothDir = startLine.UnitTangent;
            boothDir.Unitize();
            boothDir = Vector3d.Multiply(localRules.BoothDepth, boothDir);

            //Copy center lines according to distancerules
            List<Curve> copyLines = new List<Curve>();
            
            for (double i = localRules.BoothDepth; i < startLine.Length;)
            {
                Point3d startPT = startLine.PointAtLength(i);
                Point3d endPT = endLine.PointAtLength(i);
                copyLines.Add(new Line(startPT, endPT).ToNurbsCurve());
                i = i + (localRules.BoothDepth * 2) + localRules.ManeuveringArea;
            }


            //Find inner curves
            foreach (Curve crvline in copyLines)
            {
                var curveEvent = Intersection.CurveCurve(innerBounderyCrv[0], crvline, 0, 0);
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

                        if (innerBounderyCrv[0].Contains(curve.PointAt(midT), Plane.WorldXY, 0).Equals(PointContainment.Inside))
                        {
                            CenterLines.Add(curve);
                        }
                    }

                }
            }

            //Parking Booths
            foreach (Curve curve in CenterLines)
            {
                curve.DivideByLength(localRules.BoothWidth, true, false, out Point3d[] points);
                if (points == null)
                {
                    continue;
                }
                foreach (Point3d point in points)
                {
                    //One way
                    Plane boothPlane = new Plane(point, curve.TangentAt(0.1), boothDir);
                    Parkingspace parkingspace = new Parkingspace(boothPlane, localRules, true);
                    parkingLinesLeft.Add(parkingspace);
                    //Opposite way
                    boothDir.Reverse();
                    boothPlane = new Plane(point, curve.TangentAt(0.1), boothDir);
                    parkingspace = new Parkingspace(boothPlane, localRules, true);
                    parkingLinesLeft.Add(parkingspace);
                    //Return Vector
                    boothDir.Reverse();
                }

                PointCloud startPTsLeft = new PointCloud();
                PointCloud startPTsRight = new PointCloud();
                PointCloud allStartPT = new PointCloud();

                //Check for collision with driving path

                List<Parkingspace> finalParkingLinesLeft = new List<Parkingspace>();
                List<Parkingspace> finalParkingLinesRight = new List<Parkingspace>();
                for (int i = 0; i < parkingLinesLeft.Count; i++)
                {
                    CurveIntersections crvEvent = Intersection.CurveCurve(innerBounderyCrv[0], parkingLinesLeft[i].Curves, 0.0001, 0.0000);
                    if (crvEvent.Count > 0)
                    {
                        parkingEnds.Add(parkingLinesLeft[i].Curves);

                    }
                    else
                    {
                        finalParkingLinesLeft.Add(parkingLinesLeft[i]);
                        startPTsLeft.Add(parkingLinesLeft[i].Curves.PointAtStart);
                        allStartPT.Add(parkingLinesLeft[i].Curves.PointAtStart);
                    }
                }

                for (int i = 0; i < parkingLinesRight.Count; i++)
                {
                    CurveIntersections crvEvent = Intersection.CurveCurve(innerBounderyCrv[0], parkingLinesRight[i].Curves, 0.0001, 0.0000);
                    if (crvEvent.Count > 0)
                    {
                        parkingEnds.Add(parkingLinesRight[i].Curves);

                    }
                    else
                    {
                        finalParkingLinesRight.Add(parkingLinesRight[i]);
                        startPTsRight.Add(parkingLinesRight[i].Curves.PointAtStart);
                        allStartPT.Add(parkingLinesRight[i].Curves.PointAtStart);
                    }
                }




                //try
                //{
                //    //Fix one end
                //    Point3d crvEnd = curve.PointAtEnd;
                //    Curve endCurve = createEnds(innerBounderyCrv, startPTsLeft, startPTsRight, finalParkingLinesLeft, finalParkingLinesRight, crvEnd);
                //    notParkingSpaces.Add(endCurve);
                //    Curve splitCurveOne = splitend(curve, allStartPT, crvEnd);

                //    //Fix the other end
                //    Point3d crvStart = curve.PointAtStart;
                //    endCurve = createEnds(innerBounderyCrv, startPTsLeft, startPTsRight, finalParkingLinesLeft, finalParkingLinesRight, crvStart);
                //    notParkingSpaces.Add(endCurve);
                //    notParkingSpaces.Add(splitend(splitCurveOne, allStartPT, crvStart));

                //}
                //catch
                //{

                //}


                Spaces.AddRange(finalParkingLinesLeft);
                Spaces.AddRange(finalParkingLinesRight);

                parkingLinesLeft.Clear();
                parkingLinesRight.Clear();

                



            }
            parkingspaceNO = Spaces.Count;
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

        private static Curve createEnds(Curve innerBounderyCrv, PointCloud startPTsLeft, PointCloud startPTsRight, List<Parkingspace> finalParkingLinesLeft, List<Parkingspace> finalParkingLinesRight, Point3d crvEnd)
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

        private static void closeIntersect(Curve innerBounderyCrv, PointCloud startPTsLeft, List<Parkingspace> finalParkingLinesLeft, Point3d crvEnd, out Curve lineOne, out CurveIntersections intersectionsOne)
        {
            int indexOne = startPTsLeft.ClosestPoint(crvEnd);
            lineOne = finalParkingLinesLeft[indexOne].Curves;

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

        //Obstacle detection
        private void clearObstacles(List<GeometryBase> obstacles)
        {
            foreach (var obstacle in obstacles)
            {
                if(obstacle is Curve)
                {
                    Curve crvObstacle = (Curve)obstacle;
                    List<int> cullList = new List<int>();
                    for (int i = 0; i < Spaces.Count; i++)
                    {
                        if(Intersection.CurveCurve(crvObstacle, Spaces[i].Curves, 0.001, 0.001).Count != 0)
                        {
                            cullList.Add(i);
                            continue;
                        }
                        if (Intersection.CurveCurve(crvObstacle, Spaces[i].ManeuveringBox.ToNurbsCurve(), 0.001, 0.001).Count != 0)
                        {
                            cullList.Add(i);
                            continue;
                        }
                        if (crvObstacle.Contains(Spaces[i].Curves.PointAtEnd, Plane.WorldXY, 0.001) == PointContainment.Inside)
                        {
                            cullList.Add(i);
                            continue;
                        }

                    }
                    cullList.Reverse();
                    foreach (int index in cullList)
                    {
                        Spaces.RemoveAt(index);
                    }
                }
                if (obstacle is Brep)
                {
                    Brep brepObstacle = (Brep)obstacle;
                    List<int> cullList = new List<int>();
                    for (int i = 0; i < Spaces.Count; i++)
                    {
                        Intersection.CurveBrep(Spaces[i].Curves, brepObstacle, 0.001, 0.001, out double[] insections);
                        if (insections.Length != 0)
                        {
                            cullList.Add(i);
                            continue;
                        }
                        Intersection.CurveBrep(Spaces[i].ManeuveringBox.ToNurbsCurve(), brepObstacle, 0.001, 0.001, out insections);
                        if (insections.Length != 0)
                        {
                            cullList.Add(i);
                            continue;
                        }
                        if (brepObstacle.IsPointInside(Spaces[i].Curves.PointAtEnd, 0.001, true))
                        {
                            cullList.Add(i);
                            continue;
                        }
                    }
                    cullList.Reverse();
                    foreach (int index in cullList)
                    {
                        Spaces.RemoveAt(index);
                    }
                }
                if (obstacle is Mesh)
                {
                    Mesh meshObstacle = (Mesh)obstacle;
                    List<int> cullList = new List<int>();
                    for (int i = 0; i < Spaces.Count; i++)
                    {
                        if (Intersection.MeshPolyline(meshObstacle, Spaces[i].Box.ToPolyline().ToPolylineCurve(), out int[] notused).Length != 0)
                        {
                            cullList.Add(i);
                            continue;
                        }
                        if (Intersection.MeshPolyline(meshObstacle, Spaces[i].ManeuveringBox.ToPolyline().ToPolylineCurve(), out notused).Length != 0)
                        {
                            cullList.Add(i);
                            continue;
                        }
                        if (meshObstacle.IsPointInside(Spaces[i].Curves.PointAtEnd, 0.001, true))
                        {
                            cullList.Add(i);
                            continue;
                        }

                    }
                    cullList.Reverse();
                    foreach (int index in cullList)
                    {
                        Spaces.RemoveAt(index);
                    }
                }
            }
        }
    }


}
