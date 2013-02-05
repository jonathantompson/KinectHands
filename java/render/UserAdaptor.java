
package render;

public class UserAdaptor implements UserListener
{
   public boolean mouseDown(UserEvent e) { return false; }
   public boolean mouseDrag(UserEvent e) { return false; }
   public boolean mouseUp  (UserEvent e) { return false; }
   public boolean mouseMove(UserEvent e) { return false; }
   public boolean keyDown  (UserEvent e) { return false; }
   public boolean keyUp    (UserEvent e) { return false; }
}

