
package render;

public interface UserEvent
{
   public static int MOUSE_EVENT = 0;
   public static int   KEY_EVENT = 1;

   public int      type      = 0;
   public int      key       = 0;
   public int      modifiers = 0;
   public int      button    = 0;
   public int      time      = 0;
   public int      x         = 0;
   public int      y         = 0;
   public double[] point     = {0, 0, 0};
   public double   u         = 0;
   public double   v         = 0;
   public Geometry geometry  = null;
}

