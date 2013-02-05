
package render;
import java.awt.*;

///////////////// DISPLAY SPEECH AND THOUGHT BUBBLES ///////////////////

public class TextBubble
{
   public final static int NARRATION = 0;
   public final static int SPEECH    = 1;
   public final static int THOUGHT   = 2;

   static int W = 0, fontHeight, border, shadowX, shadowY;
   static Font font = null, italicFont = null;
   static Color shadowColor = new Color(0,0,0,80);

   public static void draw(Graphics g, int type, int w, int tX, int tY, String text, boolean editing) {

      if (w != W) {
	 W = w;
         fontHeight = W / 24;
         font = new Font("Helvetica", Font.BOLD, fontHeight);
         italicFont = new Font("Helvetica", Font.ITALIC | Font.BOLD, fontHeight);
	 border = fontHeight;
	 shadowX = border/2;
	 shadowY = border/4;
      }
      int sW = Math.max(2*border, border + g.getFontMetrics(font).stringWidth(text));
      int sH = fontHeight + border;
      int sX = tX < W/3 ? 0 : tX < 2*W/3 ? (W - sW)/2 : W - sW;
      int sY = 0;

      drawTextBubble(g, type, sX, sY, sW, sH, text, editing);

      switch (type) {
      case SPEECH : drawSpeechStem (g, sX, sY, sW, sH, tX, tY); break;
      case THOUGHT: drawThoughtStem(g, sX, sY, sW, sH, tX, tY); break;
      }
   }

   static Color narrationColor = new Color(255,235,200);

   static void drawTextBubble(Graphics g, int type, int x, int y, int w, int h, String text, boolean editing) {
      g.setColor(shadowColor);
      g.fillRoundRect(x+shadowX,y+shadowY,w,h,border,border);
      g.setColor(type==NARRATION ? narrationColor : Color.white);
      g.fillRoundRect(x,y,w,h,border,border);
      g.setColor(Color.black);
      g.drawRoundRect(x,y,w,h,border,border);
      Font f = type==NARRATION ? italicFont : font;
      g.setFont(f);
      g.drawString(text, x + border/2, y + border/2 + fontHeight*9/10+1);
      if (editing) {
         g.setColor(Color.red);
         int tw = g.getFontMetrics(f).stringWidth(text);
         g.fillRect(x + border/2 + tw, y + border/2, 2, h-border);
      }
   }

   static int X[] = new int[3], Y[] = new int[3];
   static void drawSpeechStem(Graphics g, int x, int y, int w, int h, int tx, int ty) {
      X[0] = x + w/2-border/2;
      X[1] = tx;
      X[2] = x + w/2+border/2;
      Y[0] = y + h;
      Y[1] = ty;
      Y[2] = y + h;
      g.setColor(Color.white);
      g.fillPolygon(X,Y,3);
      g.setColor(Color.black);
      g.drawLine(X[0],Y[0],X[1],Y[1]);
      g.drawLine(X[1],Y[1],X[2],Y[2]);
   }

   static void drawThoughtStem(Graphics g, int x, int y, int w, int h, int tx, int ty) {
      for (int i = 7 ; i > 0 ; i -= 2) {
         int b = border/2 * (16-i) / 16;
	 int X = x+w/2 + i * i * (tx - (x+w/2)) / 8 / 8;
	 int Y = y+h + (int)(Math.pow(i/8.,.8) * (ty - (y+h)));
	 g.setColor(Color.white);
	 g.fillOval(X-b,Y-b,2*b,2*b);
	 g.setColor(Color.black);
	 g.drawOval(X-b,Y-b,2*b,2*b);
      }
   }
}

