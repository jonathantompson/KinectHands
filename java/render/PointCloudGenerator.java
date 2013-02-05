
package render;
import java.util.*;

public class PointCloudGenerator
{
   Random R = new Random(0);
   Matrix m = new Matrix();
   Matrix mTmp = new Matrix();

   /*
   Convert a geometric object into a point cloud,
   given a certain total number of points.
   For each triangular face (after chopping
   polygons down to triangles) create point cloud
   vertices within that triangle.  The average
   number of vertices is proportional to the area
   of the triangle.
   */

   public Geometry toPointCloud(Geometry g, int npts) {
      m.copy(g.getMatrix());
      Geometry pc = new Geometry();
      double area = computeArea(g, m);
      if (area > 0)
         addPoints(pc, npts / area, npts, g, m);
      return pc;
   }

   double computeArea(Geometry g, Matrix m) {
      double area = 0;
      if (g.faces != null && g.vertices != null) {
         transformVerticesToTmp(g.vertices, m);
         for (int f = 0 ; f < g.faces.length ; f++) {
            int face[] = g.faces[f];
            for (int i = 1 ; i < face.length - 1 ; i++)
               area += triangleArea(tmp[face[0]], tmp[face[i]], tmp[face[i+1]]);
         }
      }
      for (int i = 0 ; g.child(i) != null ; i++) {
         mTmp.copy(m);
	 m.preMultiply(g.child(i).getMatrix());
         area += computeArea(g.child(i), m);
         m.copy(mTmp);
      }
      return area;
   }

   double triangleArea(double a[], double b[], double c[]) {
      for (int i = 0 ; i < 3 ; i++) {
         u[i] = b[i] - a[i];
         v[i] = c[i] - a[i];
      }
      Vec.cross(u, v, w);
      return 0.5 * Vec.norm(w);
   }

   int addPoints(Geometry pc, double density, int npts, Geometry g, Matrix m) {
      pc.getMatrix().copy(g.getMatrix());

      pc.vertices = new double[npts][6];
      double g_v[][] = g.vertices;

      int n = 0;
      if (g.faces != null && g_v != null) {
         transformVerticesToTmp(g_v, m);
         for (int f = 0 ; f < g.faces.length ; f++) {
            int face[] = g.faces[f];
            for (int i = 1 ; i < face.length - 1 ; i++) {
               double expected = triangleArea(tmp[face[0]], tmp[face[i]], tmp[face[i+1]]) * density;
               int np = (int)(2 * expected * R.nextDouble());
               np = Math.min(np, npts - n);
               n = addPointsInTriangle(g_v[face[0]], g_v[face[i]], g_v[face[i+1]], pc.vertices, n, np);
            }
         }
      }
      for (int i = 0 ; g.child(i) != null ; i++) {
         mTmp.copy(m);
	 m.preMultiply(g.child(i).getMatrix());
         n += addPoints(pc.add(), density, npts - n, g.child(i), m);
         m.copy(mTmp);
      }
      return n;
   }

   int addPointsInTriangle(double d[], double e[], double f[], double dst[][], int n, int np) {
      while (np-- > 0)
         addPointInTriangle(d, e, f, dst[n++]);
      return n;
   }

   void addPointInTriangle(double a[], double b[], double c[], double dst[]) {
      double U = R.nextDouble(), V = R.nextDouble();
      if (U + V > 1) {
         U = 1 - U;
         V = 1 - V;
      }
      for (int i = 0 ; i < 3 ; i++) {
         u[i] = b[i] - a[i];
         v[i] = c[i] - a[i];
	 dst[i] = a[i] + U * u[i] + V * v[i];
      }
      Vec.cross(u, v, w);
      Vec.normalize(w);
      for (int i = 0 ; i < 3 ; i++)
         dst[3 + i] = w[i];
   }

   void transformVerticesToTmp(double src[][], Matrix m) {
      if (src.length > tmp.length)
	 tmp = new double[src.length + 100][3];
      for (int i = 0 ; i < src.length ; i++)
         m.transform(src[i][0], src[i][1], src[i][2], tmp[i]);
   }

   double tmp[][] = new double[1000][3];
   double u[] = new double[3], v[] = new double[3], w[] = new double[3];
}

