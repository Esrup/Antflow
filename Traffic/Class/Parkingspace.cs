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
    public class Parkingspace
    {
        public Rectangle3d Box;
        public Curve Curves;
        public Point3d Center;
        public Rectangle3d ManeuveringBox;

        public Parkingspace(Plane parkingspacePlane, LocalRules localRules, Boolean Flip)
        {
            createParkingspace(parkingspacePlane, localRules, Flip);
        }

        private void createParkingspace(Plane parkingspacePlane, LocalRules localRules, Boolean Flip)
        {
            //Create Parking rectangle
            Box = new Rectangle3d(parkingspacePlane, localRules.BoothWidth, localRules.BoothDepth);
            //Create ManeuveringArea
            Vector3d reverseVector = parkingspacePlane.YAxis;
            reverseVector.Reverse();
            parkingspacePlane = new Plane(parkingspacePlane.Origin, parkingspacePlane.XAxis, reverseVector);
            if (Flip)
            {
                reverseVector.Unitize();
                reverseVector = Vector3d.Multiply(localRules.ManeuveringArea+localRules.BoothDepth, -parkingspacePlane.YAxis);
                parkingspacePlane.Translate(reverseVector);
            }
            ManeuveringBox = new Rectangle3d(parkingspacePlane, localRules.BoothWidth, localRules.ManeuveringArea);

            //Create Parking Curves
            Center = Box.Center;
            Polyline polylineBox = Box.ToPolyline();
            List<Curve> boxSegments = new List<Curve>(); 
            boxSegments.Add(polylineBox.SegmentAt(1).ToNurbsCurve());
            boxSegments.Add(polylineBox.SegmentAt(3).ToNurbsCurve());
            if (Flip)
            {
                boxSegments.Add(polylineBox.SegmentAt(0).ToNurbsCurve());
            }
            else
            {
                boxSegments.Add(polylineBox.SegmentAt(2).ToNurbsCurve());
            }
            
            Curves = Curve.JoinCurves(boxSegments)[0];

            


        }
    }
}
