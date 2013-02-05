
package render;
import java.util.*;

public class Obj {
   static int HUGE = 100000, N = 32, n_v = 0, n_f = 0, i_v = 0, i_f = 0;

   static double v[][];
   static int f[][];

   static double lo[] = new double[3];
   static double hi[] = new double[3];
   static double avg[] = new double[3];

   public static Geometry newObj(String data) {
      n_v = n_f = 0;

      StringTokenizer st = new StringTokenizer(data, "\n");
      while (st.hasMoreTokens()) {
         String s = st.nextToken();
         if (s.length() > 1)
            if (s.charAt(0) == 'v' && s.charAt(1) == ' ')
               n_v++;
            else if (s.charAt(0) == 'f' && s.charAt(1) == ' ')
               n_f++;
      }

      v = new double[n_v][6];
      f = new int[n_f][];

      i_v = i_f = 0;
      st = new StringTokenizer(data, "\n");

      int line = 0;
      while (st.hasMoreTokens())
         parseLine(st.nextToken());

      Geometry g = new Geometry();
      g.vertices = v; 
      g.faces = f;

      g.computePolyhedronNormals();
      for (int n = 0 ; n < n_v ; n++)
         for (int j = 0 ; j < 3 ; j++)
            g.vertices[n][3+j] *= -1;

      return g;
   }

   public static void normalizeSize(Geometry g) {
      double v[][] = g.vertices;

      for (int j = 0 ; j < 3 ; j++) {
         lo[j] = HUGE;
         hi[j] = -HUGE;
      }

      for (int n = 0 ; n < v.length ; n++)
         for (int j = 0 ; j < 3 ; j++) {
            lo[j] = Math.min(lo[j], v[n][j]);
            hi[j] = Math.max(hi[j], v[n][j]);
         }

      double radius = 0.5 * Math.max(hi[0]-lo[0], Math.max(hi[1]-lo[1], hi[2]-lo[2]));

      for (int j = 0 ; j < 3 ; j++)
         avg[j] = (lo[j] + hi[j]) / 2;

      for (int n = 0 ; n < v.length ; n++)
         for (int j = 0 ; j < 3 ; j++)
            g.vertices[n][j] = (g.vertices[n][j] - avg[j]) / radius;
   }

   static void parseLine(String s) {
      StringTokenizer st = new StringTokenizer(s);
      if (st.hasMoreTokens()) {
         String key = st.nextToken();
         if (key.equals("v"))
            parseXYZ(v[i_v++], st);
         else if (key.equals("f"))
            f[i_f++] = parseFace(st);
      }
   }

   static void parseXYZ(double v[], StringTokenizer st) {
      for (int i = 0 ; i < 3 ; i++)
         v[i] = (new Double(st.nextToken())).doubleValue();
   }

   static int fTmp[] = new int[100];

   static int[] parseFace(StringTokenizer st) {
      int n = 0;
      for ( ; st.hasMoreTokens() ; n++)
         fTmp[n] = (new Integer(st.nextToken())).intValue() - 1;

      int[] f = new int[n];
      for (int i = 0 ; i < n ; i++)
         f[i] = fTmp[i];
      return f;
   }
}


