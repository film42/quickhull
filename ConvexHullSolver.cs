using System;
using System.Collections.Generic;
using System.Text;
using System.Drawing;
using System.Linq;

namespace _2_convex_hull {
  class ConvexHullSolver {

    public void Solve(Graphics g, List<PointF> pointList) {
      // Get the Convex Hull for the points in pointList
      var newPoints = ConvexHull(pointList);

      // Draw the Hull using Polygon
      var pen = new Pen(Color.Orange, 2);
      g.DrawPolygon(pen, newPoints.ToArray());
    }

    private List<PointF> ConvexHull(List<PointF> pointList) {
      // If we're less than 3, then we're gonna base case. Return.
      if (pointList.Count < 3) return pointList;
      // Create a new hull list to build our convex hull with.
      var hull = new List<PointF>();
      
      // Reference Indicies
      int smallestXValueIndex = -1;
      int largestXValueIndex = -1;
      // We're storing Values because PointF is a non-null type.
      double smallestXValue = double.MaxValue;
      double largestXValue = double.MinValue;

      // Iterate through points to find the MAX and MIN X values.
      for (int i = 0; i < pointList.Count; i++) {
        var current = pointList[i];
        // Save the smallest X value and associated Index
        if (current.X < smallestXValue) {
          smallestXValue = current.X;
          smallestXValueIndex = i;
        }
        // Save the largest X value and associated Index
        if (current.X > largestXValue) {
          largestXValue = current.X;
          largestXValueIndex = i;
        }
      }

      // Grab the smallest and largest X points from Point list
      var smallestXPoint = pointList[smallestXValueIndex];
      var largestXPoint = pointList[largestXValueIndex];
      // Add initial points to the new hull object
      hull.Add(smallestXPoint);
      hull.Add(largestXPoint);
      // Remove the hull points from pointList so no conflicts occur
      pointList.Remove(smallestXPoint);
      pointList.Remove(largestXPoint);

      // Create Hulls for our new division
      var upperList = new List<PointF>();
      var lowerList = new List<PointF>();

      // Split them according to their point location
      for (int i = 0; i < pointList.Count; i++) {
        var current = pointList[i];
        // Negative cross products will be added to the upper list
        if (ValidPointLocation(smallestXPoint, largestXPoint, current) == false) 
          upperList.Add(current);
        // Positive values will be added to the lower list
        else lowerList.Add(current);
      }

      // Divide and Conquer the two new hulls
      var upperHull = HullProcessor(smallestXPoint, largestXPoint, lowerList, hull);
      var lowerHull = HullProcessor(largestXPoint, smallestXPoint, upperList, hull);

      // Return the concat'd hulls
      return MergeHulls(ref upperHull, ref lowerHull);
    }

    private List<PointF> HullProcessor(PointF min, PointF max, List<PointF> pts, List<PointF> hull) {
      // The index out MAX is now our new index.
      int insertAt = hull.IndexOf(max);
      // If there's none left, we've hit our base case
      if (pts.Count == 0) return hull;
      // If there's one, then we add it to the hull.
      if (pts.Count == 1) {
        var point = pts[0];
        // Append new point to hull and return it.
        hull.Insert(insertAt, point);
        return hull;
      }

      // There are no more weird cases, so let's calculate the furthest distance
      // point, and then divide based on it.
      double furthestDistance = int.MinValue;
      int furthestPointIndex = -1;
      for (int i = 0; i < pts.Count; i++) {
        var current = pts[i];
        // Distance from line Min-Max to point Current
        double distance = Distance(min, max, current);
        // Save the new point of furthest
        if (distance > furthestDistance) {
          furthestDistance = distance;
          furthestPointIndex = i;
        }
      }

      // Add the new furthest point
      var furthestPoint = pts[furthestPointIndex];
      // Remove the point from PTS to ensure we're not factoring when splitting.
      pts.Remove(furthestPoint);
      // Add new point to hull at point given
      hull.Insert(insertAt, furthestPoint);
      // Create new lists to divie into
      var upperList = new List<PointF>();
      var lowerList = new List<PointF>();
      // Determine who's to the left of Min,furthestPoint
      for (int i = 0; i < pts.Count; i++) {
        var current = pts[i];
        // Valid points from line Min-FurthestPoint to Current
        if (ValidPointLocation(min, furthestPoint, current))
          upperList.Add(current);
      }

      // Determine who's to the left of fP, Max
      for (int i = 0; i < pts.Count; i++) {
        var current = pts[i];
        // Valid points from line furthestPoint-Max to Current
        if (ValidPointLocation(furthestPoint, max, current))
          lowerList.Add(current);
      }

      // Divide and Conquer for each Hull
      var upperHull = HullProcessor(min, furthestPoint, upperList, hull);
      var lowerHull = HullProcessor(furthestPoint, max, lowerList, hull);

      // Return the new Concat list of both Hulls
      return MergeHulls(ref upperHull, ref lowerHull);
    }

    private List<PointF> MergeHulls(ref List<PointF> upper, ref List<PointF> lower) {
      // Super basic Concat, but kept seperate in case there is something more efficient out there
      upper.Concat(lower);
      return upper;
    }

    private bool ValidPointLocation(PointF min, PointF max, PointF cur) {
      // Calculate the cross product of the 3 points provided.
      double product = ((max.X - min.X) * (cur.Y - min.Y)) - ((max.Y - min.Y) * (cur.X - min.X));
      // If the product is Greater than 0, then it is in a valid location.
      if (product > 0) return true;
      else return false;
    }

    private double Distance(PointF min, PointF max, PointF cur) {
      // Credit goes to wikipedia for Distance from a Line
      // Difference from Max to Mins - Xs
      double x = max.X - min.X;
      // Difference from Max to Min - Ys
      double y = max.Y - min.Y;
      // The result come from diff in x * diff in min and cur subtracted
      // by the product of y and distance from Min to Currents - Xs.
      double res = (x * (min.Y - cur.Y)) - (y * (min.X - cur.X));
      // Check for pos or neg here, negate it negative.
      if (res < 0) res = -res;
      return res;
    }

  }

}
