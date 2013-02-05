
package render;

public interface UserListener
{
   public boolean mouseDown(UserEvent e);
   public boolean mouseDrag(UserEvent e);
   public boolean mouseUp  (UserEvent e);
   public boolean mouseMove(UserEvent e);
   public boolean keyDown  (UserEvent e);
   public boolean keyUp    (UserEvent e);
}

