
package render;
import java.awt.*;

public class Signals
{
   Signal signal[] = new Signal[100];
   Graphics g;
   int nSignals = 0;
   Color whiteScrim = new Color(255, 255, 255, 128);
   Color blackScrim = new Color(  0,   0,   0, 192);
   int fontHeight = 16, newFontHeight = fontHeight, width;
   Font font;

   public Signals(String data[]) {
      this(0, 0, 25, data);
   }

   public Signals(int x, int y, int h, String data[]) {
      this.x = x;
      this.y = y;
      fontHeight = (h - 2) * 2 / 3;
      newFontHeight = fontHeight;
      if (data != null)
         for (int n = 0 ; n < data.length ; n++)
            addSignal(data[n]);
   }

   public int getX() {
      return x;
   }

   public int getY() {
      return y;
   }

   public int getWidth() {
      return width;
   }

   public int getHeight() {
      return nSignals * fontHeight * 3 / 2 - 2;
   }

   public int getNumSignals() {
      return nSignals;
   }

   public int getFontSize() {
      return fontHeight;
   }

   public Font getFont() {
      return font;
   }

   public void setX(int x) {
      this.x = x;
   }

   public void setY(int y) {
      this.y = y;
   }

   public void setFontSize(int fontSize) {
      newFontHeight = fontSize;
   }

   public int addSignal(String infoString) {
      signal[nSignals++] = new Signal(infoString);
      computeWidth();
      return nSignals - 1;
   }

   public int indexOf(String label) {
      for (int n = 0 ; n < nSignals ; n++)
         if (signal[n].getLabel().equals(label))
	    return n;
      return -1;
   }

   public Signal get(int i) { return signal[i]; }

   public int identifySignal(int key) {
      for (int n = 0 ; n < nSignals ; n++)
         if (signal[n].isHotKey(key))
	    return n;
      return -1;
   }

   public int tryHotKey(int key) {
      for (int n = 0 ; n < nSignals ; n++) {
         int state = signal[n].tryHotKey(key);
	 if (state >= 0)
	    return state;
      }
      return -1;
   }

   public boolean mouseDown(int x, int y) {
      for (int n = 0 ; n < nSignals ; n++) {
         if (isWithin(n, x, y)) {
	    selected = n;
	    return true;
         }
      }
      selected = -1;
      return false;
   }

   public boolean mouseDrag(int x, int y) {
      return selected >= 0;
   }

   public boolean mouseUp(int x, int y) {
      if (selected >= 0 && isWithin(selected, x, y)) {
         Signal s = signal[selected];
         if (s.size() == 0)
	    s.setState(! s.getState());
         else
	    s.select((s.getSelected() + 1) % s.size());
         return true;
      }
      return false;
   }

   public int signalY(int n) { return y + n * fontHeight * 3 / 2; }

   boolean isWithin(int n, int x, int y) {
      return x >= this.x && x < width && y >= signalY(n) && y < signalY(n + 1) - 2;
   }

   void computeWidth() { 
      if (g != null) {
         width = 3 * fontHeight;
         for (int n = 0 ; n < nSignals ; n++) {
            Signal s = signal[n];
	    String str = s.getLabel();
            int stringWidth = Util.stringWidth(g, str);
	    if (s.isMultistate())
	       stringWidth += (1 + s.size()) * fontHeight;
            width = Math.max(width, stringWidth + fontHeight);
         }
      }
   }

   public void draw(Graphics g) {

      this.g = g;
      if (font == null || newFontHeight != fontHeight) {
         font = new Font("Helvetica", Font.BOLD, fontHeight = newFontHeight);
         g.setFont(font);
         computeWidth();
      }
      g.setFont(font);

      for (int n = 0 ; n < nSignals ; n++) {
         Signal s = signal[n];
         g.setColor(s.isMultistate() && s.getSelected() > 0 || s.getState() ? Color.white : whiteScrim);
         int dy = fontHeight;
         g.fillRect(x, signalY(n), width, signalY(n + 1) - signalY(n) - 2);
         g.setColor(Color.black);
	 String str = s.getLabel();
	 if (s.isMultistate())
	    str += ":";
         g.drawString(str, x + 5, signalY(n) + dy);
	 if (s.isMultistate()) {
	    int xx = x + width - dy * 3 / 4;
	    for (int i = s.size() - 1 ; i >= 0 ; i--)
	       if (s.getHotKey(i) > 0) {
	          drawHotKey(g, i == s.getSelected(), s.getHotKey(i), xx, signalY(n) + dy);
		  xx -= fontHeight - 1;
               }
         }
         else
	    drawHotKey(g, s.getState(), s.getHotKey(), x + width - dy * 3 / 4, signalY(n) + dy);
      }
   }

   void drawHotKey(Graphics g, boolean isSelected, int ch, int x, int y) {
      g.setColor(isSelected ? Color.black : blackScrim);
      String cs = "" + (char)ch;
      int w = Util.stringWidth(g, cs);
      if (w > 10)
        w += 3;
      x -= w / 2;
      g.drawString(cs, x, y);
      if (isSelected)
         g.drawString(cs, x + 1, y);
   }

   public void update(double elapsed) {
      for (int i = 0 ; i < nSignals ; i++)
         signal[i].update(elapsed);
   }

   int x = 0, y = 0, selected = -1;
}

