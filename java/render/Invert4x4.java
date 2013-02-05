
package render;

public class Invert4x4 {
   static double a[][] = new double[4][4];
   static double b[][] = new double[4][4];
   static double c[] = new double[4];
   static int index[] = new int[4];

   public static void invert(double src[][], double dst[][]) {
      gaussian(src, a);

      for (int i = 0 ; i < 4 ; i++)
      for (int j = 0 ; j < 4 ; j++)
          b[i][i] = i == j ? 1 : 0;

      for (int i = 0 ; i < 3 ; i++)
         for (int j = i + 1 ; j < 4 ; j++)
            for (int k = 0 ; k < 4 ; k++)
               b[index[j]][k] -= a[index[j]][i] * b[index[i]][k];

      for (int i = 0 ; i < 4 ; i++) {
         dst[4-1][i] = b[index[4-1]][i] / a[index[4-1]][4-1];
         for (int j = 2 ; j >= 0 ; j--) {
            dst[j][i] = b[index[j]][i];
            for (int k = j + 1 ; k < 4 ; k++)
               dst[j][i] -= a[index[j]][k] * dst[k][i];
            dst[j][i] /= a[index[j]][j];
         }
      }
   }

   private static void gaussian(double src[][], double dst[][]) {

      for (int i = 0 ; i < 4 ; i++)
      for (int j = 0 ; j < 4 ; j++)
         dst[i][j] = src[i][j];

      for (int i = 0 ; i < 4 ; i++)
          index[i] = i;

      for (int i = 0 ; i < 4 ; i++) {
         double c1 = 0;
         for (int j = 0 ; j < 4 ; j++) {
            double c0 = Math.abs(dst[i][j]);
            if (c0 > c1)
               c1 = c0;
         }
         c[i] = c1;
      }

      int k = 0;
      for (int j = 0 ; j < 3 ; j++) {
         double p1 = 0;
         for (int i = j ; i < 4 ; i++) {
            double p0 = Math.abs(dst[index[i]][j]);
            p0 /= c[index[i]];
            if (p0 > p1) {
               p1 = p0;
               k = i;
            }
         }

         int itmp = index[j];
         index[j] = index[k];
         index[k] = itmp;
         for (int i = j + 1 ; i < 4 ; i++) {
            double pj = dst[index[i]][j] / dst[index[j]][j];
            dst[index[i]][j] = pj;
            for (int l = j + 1 ; l < 4 ; l++)
               dst[index[i]][l] -= pj * dst[index[j]][l];
         }
      }
   }
}

