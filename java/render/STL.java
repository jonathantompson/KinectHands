
package render;
import java.util.*;
import java.io.*;

public class STL
{
   public static boolean saveSTL(Geometry geometry, String name) {
/*
      Geometry geometries[] = new Geometry[geometry.nChildren()];
      for (int n = 0 ; n < geometries.length ; n++)
         (geometries[n] = new Geometry()).bake(geometry.child(n));
*/
      Geometry geometries[] = new Geometry[count(geometry)];
      bake(geometry, geometries, 0);

      String str = toSTL(geometries).toString();
      try {
         File dir = new File(System.getProperty("user.dir"));
         FileOutputStream fout = new FileOutputStream(new File(dir, name + ".stl"));
         fout.write(str.getBytes());
         fout.close();
      } catch (Exception e) { return false; }
      return true;
   }

   static int count(Geometry g) {
      int ng = g.vertices != null ? 1 : 0;
      for (int n = 0 ; g.child(n) != null ; n++)
         ng += count(g.child(n));
      return ng;
   }

   static int bake(Geometry g, Geometry gs[], int ng) {
      if (g.vertices != null)
         (gs[ng++] = new Geometry()).bake(g);
      for (int n = 0 ; g.child(n) != null ; n++)
         ng = bake(g.child(n), gs, ng);
      return ng;
   }

   static double normal[] = new double[3];

   public static StringBuffer toSTL(Geometry geometries[]) {
      StringBuffer sb = new StringBuffer();
      sb.append("solid OBJECT\n");

      for (int n = 0 ; n < geometries.length ; n++)
         toSTL(geometries[n], sb);

      sb.append("endsolid OBJECT\n");
      return sb;
   }

   public static StringBuffer toSTL(Geometry geometry) {
      StringBuffer sb = new StringBuffer();
      sb.append("solid OBJECT\n");

      toSTL(geometry, sb);

      sb.append("endsolid OBJECT\n");
      return sb;
   }

   static void toSTL(Geometry geometry, StringBuffer sb) {
      int faces[][] = geometry.faces;
      double vertices[][] = geometry.vertices;

      for (int f = 0 ; f < faces.length ; f++) {
         int face[] = faces[f];

         if (face.length == 3)
            triangle(vertices[face[0]], vertices[face[1]], vertices[face[2]], sb);
         else {
            triangle(vertices[face[0]], vertices[face[1]], vertices[face[2]], sb);
            triangle(vertices[face[2]], vertices[face[3]], vertices[face[0]], sb);
         }
      }
   }

   static void triangle(double a[], double b[], double c[], StringBuffer sb) {
      for (int j = 0 ; j < 3 ; j++)
         normal[j] = a[3 + j] + b[3 + j] + c[3 + j];
      Vec.normalize(normal);

      sb.append("   facet normal");
      for (int j = 0 ; j < 3 ; j++) {
         sb.append(" ");
         sb.append(normal[j]);
      }
      sb.append("\n");

      sb.append("      outer loop\n");
      vertex(a, sb);
      vertex(b, sb);
      vertex(c, sb);
      sb.append("      endloop\n");

      sb.append("   endfacet\n");
   }

   static void vertex(double v[], StringBuffer sb) {
      sb.append("         vertex");
      for (int j = 0 ; j < 3 ; j++) {
         sb.append(" ");
         sb.append(v[j]);
      }
      sb.append("\n");
   }
}

