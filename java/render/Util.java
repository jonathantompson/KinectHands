
package render;
import java.awt.*;
import java.io.*;
import java.net.*;

public class Util
{
   public static double lerp(double t, double a, double b) { return a + t * (b - a); }

   public static double alerp(double t, double a, double b) {
      while (a - b >  Math.PI) a -= 2 * Math.PI;
      while (a - b < -Math.PI) a += 2 * Math.PI;
      return a + t * (b - a);
   }

   public static double sCurve(double t) { return t < 0 ? 0 : t > 1 ? 1 : t * t * (3 - t - t); }

   public static double currentTime() { return System.currentTimeMillis() / 1000.0; }

   public static void computeRay(double a[], double b[], double ray[]) {
      for (int i = 0 ; i < 3 ; i++)
         ray[i] = a[i];
      double x = b[0] - a[0], y = b[1] - a[1], z = b[2] - a[2];
      double d = Math.sqrt(x * x + y * y + z * z);
      ray[3] = x / d;
      ray[4] = y / d;
      ray[5] = z / d;
   }

   public static double rayIntersectPlane(double ray[], double plane[], double dst[]) {
      double A = plane[3], B = 0;
      for (int i = 0 ; i < 3 ; i++) {
         A += plane[i] * ray[i];
         B += plane[i] * ray[i + 3];
      }
      double t = - A / B;
      for (int i = 0 ; i < 3 ; i++)
         dst[i] = ray[i] + t * ray[i + 3];
      return t;
   }

   public static int stringWidth(Graphics g, String s) {
      return g == null ? 0 : g.getFontMetrics().stringWidth(s);
   }

   static final double LOG_HALF = Math.log(0.5);

   public static double bias(double a, double b) {
      if (a < .0001)
         return 0;
      else if (a > .9999)
         return 1;
      else if (b < .0001)
         return 0;
      else if (b > .9999)
         return 1;
      else
         return Math.pow(a, Math.log(b) / LOG_HALF);
   }

   public static double round(double value) {
      return (int)(10000 * value + 0.5) / 10000.0;
   }

   public static int getAscii(int key, boolean shift) {
      if (key == 222)
         return shift ? '"' : '\'';

      if (key >= 'A' && key <= 'Z' && shift)
	 return key;

      if (key >= 'A' && key <= 'Z')
         return key + 'a' - 'A';

      if (shift)
         switch (key) {
         case '`': return '~';
         case '1': return '!';
         case '2': return '@';
         case '3': return '#';
         case '4': return '$';
         case '5': return '%';
         case '6': return '^';
         case '7': return '&';
         case '8': return '*';
         case '9': return '(';
         case '0': return ')';
         case '-': return '_';
         case '=': return '+';
         case '[': return '{';
         case ']': return '}';
         case '\\':return '|';
         case ';': return ':';
         case '\'':return '"';
         case ',': return '<';
         case '.': return '>';
         case '/': return '?';
         }

      return key;
   }

   public static boolean save(String fileName, String text) {
      try {
         File dir = new File(System.getProperty("user.dir"));
         FileOutputStream fout = new FileOutputStream(new File(dir, fileName));
         fout.write(text.getBytes());
         fout.close();
         return true;
      } catch (Exception e) { return false; }
   }

   public static String load(String fileName) {
      try {
         File dir  = new File(System.getProperty("user.dir"));
         File file = new File(dir, fileName);
         URL  url  = file.toURI().toURL();
         return load(url);
      } catch (Exception e) { return null; }
   }

   public static String load(URL url) {
      try {
         return load(url.openStream());
      } catch (Exception e) { return null; }
   }

   public static String load(InputStream in) {
      try {
         ByteArrayOutputStream out = new ByteArrayOutputStream();
         byte buf[] = new byte[1024];
         while (true) {
            int n = in.read(buf);
            if (n < 0)
               break;
            out.write(buf, 0, n);
         }
         return new String(out.toByteArray());
      } catch (Exception e) { return null; }
   }
}

