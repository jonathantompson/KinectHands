
package render;
import java.io.BufferedInputStream;

public class TexturedMesh extends Geometry
{
   public TexturedMesh(String fileName) {
      this(fileName, 2, 2);
   }

   public TexturedMesh(String fileName, int m, int n) {
      super();

      Texture texture = null;

      for (int i = 0 ; i < nTextures ; i++)
         if (fileName.equals(names[i])) {
            texture = textures[i];
            break;
         }

      if (texture == null) {
         try {
            texture = new Texture(getClass().getResource("../" + fileName), "TexturedMesh");
         } catch (Exception e) { System.err.println(e); }
         names[nTextures] = fileName;
         textures[nTextures] = texture;
         nTextures++;
      }

      Material material = new Material().setTexture(texture);
      setMaterial(material);
      mesh(m, n);
   }

   public TexturedMesh(Texture texture, int m, int n) {
      super();
      Material material = new Material().setTexture(texture);
      setMaterial(material);
      mesh(m, n);
   }

   static String names[] = new String[30];
   static Texture textures[] = new Texture[30];
   static int nTextures = 0;
}

