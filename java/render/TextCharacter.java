
package render;
import java.io.BufferedInputStream;

public class TextCharacter extends Geometry
{
   public TextCharacter(int ch, java.applet.Applet applet) {
      super();

      if (fontMaterial == null) {
         fontMaterial = Material.texturedMaterial(fileName);
         if (fontMaterial != null)
            fontMaterial.setAmbient(1,1,1).setTransparency(0.01);
      }

      Geometry g = add();
      g.setMaterial(fontMaterial);
      g.mesh(2, 2);
      g.getMatrix().scale(.55,1,1);

      add(g);

      int n = ch - ' ', I = n % 16, J = n / 16;

      for (int i = 0 ; i < 3 ; i++)
      for (int j = 0 ; j < 3 ; j++) {
         double v[] = g.vertices[i + 3 * j];
         v[6] = 51.0 / 1024 * (I + v[6]);
         v[7] = 93.0 / 1024 * (J + (1 - v[7]));
      }
   }

   static String fileName = "font51x93texture.gif";
   static Material fontMaterial = null;
}

