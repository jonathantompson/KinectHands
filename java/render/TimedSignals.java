
package render;

import java.awt.*;

public class TimedSignals extends Signals
{
   public TimedSignals(String signalData[]) {
      this(0, 0, 25, signalData);
   }

   public TimedSignals(int x, int y, int signalHeight, String signalData[]) {
      super(x, y, signalHeight, signalData);
      this.signalHeight = signalHeight;
   }

   public void setRunningTime(double t) {
      runningTime = Math.max(0, Math.min(60, t));
      setStateAtFrame((int)(10 * runningTime));
   }

   public void setPause(int f, boolean state) {
      pauseData[f] = state;
   }

   public boolean isPause(int f) {
      return pauseData[f];
   }

   public void setTransitionFlash(boolean state) {
      isTransitionFlash = state;
   }

   public void setCommandMode(boolean state) {
      isCommandMode = state;
   }

   public boolean isControlKey(int key) {
      switch (key) {
      case 'y' - offset:
         mustRedo = true;
         return true;
      case 'z' - offset:
         mustUndo = true;
         return true;
      }
      return false;
   }

   public boolean isHotKey(int key) {
      int state = tryHotKey(key);
      if (state >= 0) {
         int s = identifySignal(key);
	 if (s >= 0)
	    setFrameData(s, state);
         return true;
      }
      return false;
   }

   public boolean isTransition() {
      if (isRunning())
         for (int f = f0 ; f < f1 ; f++)
            for (int s = 0 ; s < getNumSignals() ; s++)
               if (getFrameData(f, s) >= 0)
                  return true;
      return false;
   }

   public int getFrameData(int f, int s) { return f < 0 || f >= frameData.length ? -1 : frameData[f][s]; }
   public boolean getPauseData(int f) { return f < 0 || f >= pauseData.length ? false : pauseData[f]; }
   public double getRunningTime() { return runningTime; }
   public int getSignalBorder() { return signalBorder; }

   public boolean isDeleteMode() { return isDeleteMode; }
   public boolean isRunning() { return isRunning; }

   public void setDeleteMode(boolean state) { isDeleteMode = state; }

   public void setFrameData(int s, int value) {
      if (isCommandMode) {
         frameData[0][s] = value;
         for (int f = 1 ; f < frameData.length ; f++)
            frameData[f][s] = -1;
      }
      else
	 setFrameData((int)(getRunningTime() * 10), s, value);
      isDamage = true;
   }

   public void setPauseData(int f, boolean value) {
      if (f >= 0 && f < pauseData.length) {
         pauseData[f] = value;
         isDamage = true;
      }
   }

   public void setFrameData(int f, int s, int value) {
      if (f >= 0 && f < frameData.length) {
         frameData[f][s] = value;
         isDamage = true;
      }
   }

   public void setSignalBorder(int value) { signalBorder = value; }

   public void setRunning(boolean state) { isRunning = state; }

   public void clear() {
      for (int s = 0 ; s < getNumSignals() ; s++)
         for (int f = 0 ; f < frameData.length ; f++) {
	    frameData[f][s] = -1;
	    pauseData[f] = false;
         }
      isDamage = true;
   }

   public int parse(String str) {
      if (str == null)
         return -1;

      clear();
      int n = 0;
      for (int s = 0 ; s < getNumSignals() ; s++) {
         int lp = str.indexOf('(', n);
         int rp = str.indexOf(')', n);
	 if (lp < 0 || rp < 0)
	    return -1;

         if (rp > lp + 1) {
            String events[] = str.substring(lp+1, rp).split(",");
	    for (int i = 0 ; i < events.length ; i++) {
	       String e = events[i];
	       int eq = e.indexOf('=');
	       int f = Integer.parseInt(e.substring(0, eq));
	       int v = Integer.parseInt(e.substring(eq+1, e.length()));
	       setFrameData(f, s, v);
	    }
	 }
         n = rp + 1;
      }
      if (str.length() > n) {
         String pauses[] = str.substring(n+1, str.length()).split(",");
	 for (int j = 0 ; j < pauses.length ; j++)
	    pauseData[Integer.parseInt(pauses[j])] = true;
      }
      return 0;
   }

   public String toString() {
      String str = "";
      if (frameData != null) {
         for (int s = 0 ; s < getNumSignals() ; s++) {
            String st = "";
	    for (int f = 0 ; f < frameData.length ; f++)
	       if (frameData[f][s] >= 0)
	          st += (st.length() == 0 ? "" : ",") + f + "=" + frameData[f][s];
            str += "(" + st + ")";
         }
	 for (int f = 0 ; f < pauseData.length ; f++)
	    if (pauseData[f])
	       str += "," + f;
      }
      return str;
   }

   public void setStateAtFrame(int fr) {
      for (int s = 0 ; s < getNumSignals() ; s++) {
         Signal signal = get(s);
         int sy0 = signalY(s);
         int sy1 = signalY(s+1);
         int f = fr;
         for ( ; f >= 0 ; f--)
            if (frameData[f][s] >= 0) {
               signal.select(frameData[f][s]);
               break;
            }
         if (f < 0)
            signal.select(0);
      }
   }

   public boolean isDamage() { return isDamage; }

   int f0 = 0, f1 = 0;

   public void update(double elapsed) {
      if (frameData == null) {
         frameData = new int[10 * 60 + 1][getNumSignals()];
         pauseData = new boolean[frameData.length];
	 clear();
      }

      if (mustUndo) {
         mustUndo = false;
         nhDecr();
         if (parse(history[nh]) >= 0)
	    isDamage = false;
         else
            nhIncr();
      }

      if (mustRedo) {
         mustRedo = false;
         nhIncr();
         if (parse(history[nh]) >= 0)
	    isDamage = false;
         else
	    nhDecr();
      }

      if (isRunning) {
	 f0 = (int)(10 * runningTime);
         setRunningTime(runningTime += elapsed);
	 f1 = (int)(10 * runningTime);
	 if (f1 > f0 && f1 < sw)
	    for (int f = f0 + 1 ; f <= f1 ; f++)
	        if (pauseData[f]) {
		   setRunningTime(f / 10.);
		   isRunning = false;
		}
      }

      if (getWidth() > 0) {
         sx = getX() + getWidth();
         sy = getY();
         sw = 600;
         sh = super.getHeight();

         fbx = getX();
	 fby = sy + sh;
	 fbw = getWidth();
	 fbh = pbs;
      }
      super.update(elapsed);
      if (isDamage)
         saveHistory();
      isDamage = false;
   }

   void nhIncr() { nh = (nh + 1) % NH; }
   void nhDecr() { nh = (nh + NH - 1) % NH; }

   int NH = 1000;
   int nh = 0;
   String history[] = new String[NH];
   final static int offset = 'a' - 1;
   boolean mustRedo = false;
   boolean mustUndo = false;

   public void clearHistory() {
      for (int n = 0 ; n < history.length ; n++)
         history[n] = null;
   }

   public void saveHistory() {
      nhIncr();
      history[nh] = toString();
   }

   public int getHeight() {
      return super.getHeight() + pbs;
   }

   int pbs = 12; // PAUSE BUTTON SIZE
   int pauseAtMouse = -1;
   int pauseAtMouseDown;

   public boolean mouseDown(int x, int y) {
      isInFlashButton = x >= fbx && x < fbx + fbw && y >= fby && y < fby + fbh;
      if (isInFlashButton)
         return true;

      isInTimeline = x >= sx && x < sx + sw && y >= sy && y < sy + sh + pbs;
      if (isInTimeline)
         return timelineMouseDown(x, y);
      return super.mouseDown(x, y);
   }

   public boolean mouseDrag(int x, int y) {
      if (isInFlashButton)
         return true;
      if (isInTimeline)
         return timelineMouseDrag(x, y);
      return super.mouseDrag(x, y);
   }

   public boolean mouseUp(int x, int y) {
      if (isInFlashButton) {
         if (x >= fbx && x < fbx + fbw && y >= fby && y < fby + fbh)
	    isTransitionFlash = ! isTransitionFlash;
	 isInFlashButton = false;
         return true;
      }
      if (isInTimeline) {
         isInTimeline = false;
         return timelineMouseUp(x, y);
      }
      return super.mouseUp(x, y);
   }

   boolean timelineMouseDown(int x, int y) {
      if (isPauseControl = y - sy >= sh) {

         // AT MOUSE DOWN, SEE WHETHER THERE IS A PAUSE AT THE MOUSE

         pauseAtMouse = findPauseAt(x - sx);
	 if (pauseAtMouse >= 0) {
	    pauseData[pauseAtMouse] = false;
	    pauseAtMouse = x - sx;
	    pauseData[pauseAtMouse] = true;
	    pauseAtMouseDown = pauseAtMouse;
	 }

	 // IF NOT, CREATE ONE

	 else {
	    pauseAtMouse = x - sx;
	    pauseData[pauseAtMouse] = true;
	    pauseAtMouseDown = -1;
	 }

	 isDamage = true;
         return true;
      }

      if (isCommandMode) {
         cxLo = cxHi = x;
	 return true;
      }

      if (x >= cxLo && x < cxHi) {
         cx = x;
         return true;
      }

      ss = (y - sy) * getNumSignals() / sh;
      sss = isDeleteMode ? 0 : 1;
      if (sss > 0 && get(ss).isMultistate())
         sss += (y - signalY(ss)) * 2 / (signalY(ss+1) - signalY(ss));
      sxLo = sxHi = x;
      return true;
   }

   boolean timelineMouseDrag(int x, int y) {
      if (isPauseControl) {

         // DRAG A PAUSE TO MOVE IT IN TIME

         if (pauseAtMouse >= 0) {
	    pauseData[pauseAtMouse] = false;
	    pauseAtMouse = x - sx;
	    pauseData[pauseAtMouse] = true;
	    isDamage = true;
	 }
         return true;
      }

      // CREATE THE COMMAND STRIP

      if (isCommandMode) {
         cxLo = Math.max(sx, Math.min(sx + sw, Math.min(cxLo, x)));
         cxHi = Math.max(sx, Math.min(sx + sw, Math.max(cxHi, x)));
	 return true;
      }

      // MOVE DATA IN THE COMMAND STRIP

      if (x >= cxLo && x < cxHi) {
         int dx = x - cx;

         if (dx < 0)
            for (int f = cxLo - sx ; f < cxHi - sx ; f++)
	       copyData(f, f + dx);
	 else
            for (int f = cxHi - sx - 1 ; f >= cxLo - sx ; f--)
	       copyData(f, f + dx);

	 cx += dx;
	 cxLo = Math.max(sx, Math.min(sx + sw, cxLo + dx));
	 cxHi = Math.max(sx, Math.min(sx + sw, cxHi + dx));
	 isDamage = true;
         return true;
      }

      sxLo = Math.max(sx, Math.min(sx + sw, Math.min(sxLo, x)));
      sxHi = Math.max(sx, Math.min(sx + sw, Math.max(sxHi, x)));
      return true;
   }

   void copyData(int fSrc, int fDst) {
      setPauseData(fDst, getPauseData(fSrc));
      for (int s = 0 ; s < getNumSignals() ; s++)
         setFrameData(fDst, s, getFrameData(fSrc, s));
   }

   boolean timelineMouseUp(int x, int y) {
      if (isPauseControl) {

         // CLICK ON A PAUSE TO DELETE IT

         if (pauseAtMouse >= 0 && pauseAtMouse == pauseAtMouseDown) {
	    pauseData[pauseAtMouse] = false;
	    isDamage = true;
         }

         pauseAtMouse = -1;
         return true;
      }

      if (isCommandMode) {
	 return true;
      }

      return true;
   }

   int findPauseAt(int f) {
      for (int ff = f - pbs / 2 ; ff < f + pbs ; ff++)
         if (ff >= 0 && ff < sw && pauseData[ff])
	    return ff;
      return -1;
   }

   public void stepToPause(int incr) {
      int f = (int)(10 * runningTime);
      switch (incr) {
      case -1:
	 for (f -= pbs ; f > 0 && ! pauseData[f] ; f--)
	    ;
         break;
      case 1:
	 for (f += pbs ; f < sw - 1 && ! pauseData[f] ; f++)
	    ;
         break;
      }
      setRunningTime(f / 10.0);
      setRunning(false);
   }

   public void draw(Graphics g) {
      super.draw(g);

      for (int s = 0 ; s < getNumSignals() ; s++) {
         g.setColor(s % 2 == 0 ? timelineBgColor1 : timelineBgColor2);
         g.fillRect(sx, signalY(s) - 1, sw, signalY(s+1) - signalY(s) + 1);
      }

      int count = 0;
      for (int x = sx ; x < sx + sw ; x += 10) {
         g.setColor(count % 10 == 0 ? Color.black : timelineLineColor);
         g.fillRect(x, sy, 1, sh);
         count++;
      }

      g.setColor(Color.black);
      int ww = getWidth();
      g.drawRect(sx - ww, sy, sw + ww, sh);
      g.drawRect(sx - ww, sy + sh, sw + ww, 1);

      g.setColor(pauseStripColor);
      g.fillRect(sx, sy + sh, sw, pbs);

      for (int x = sx ; x < sx + sw ; x++)
         if (pauseData[x - sx]) {
            g.setColor(pauseColor);
	    g.fillRect(x, sy, 1, sh);
	    g.fillRect(x - pbs / 2, sy + sh, pbs, pbs);
	    if (x - sx == pauseAtMouse) {
               g.setColor(pausePressedColor);
	       g.fillRect(x - pbs / 2 + 2, sy + sh + 2, pbs - 4, pbs - 4);
	    }
	 }

      g.setColor(currentFrameColor);
      int rx = sx + (int)(runningTime * 10);
      g.drawLine(rx, sy, rx, sy + sh);

      if (frameData == null)
         return;

      // CHANGES SPECIFIED BY DRAGGING WITHIN TIMELINE ONLY HAPPEN AFTER MOUSE UP

      if (! isInTimeline && ss >= 0) {
         int fLo = sxLo - sx, fHi = sxHi - sx;
         if (fLo == fHi)
            setRunningTime(fLo / 10.0);
         else {
            int endState = getState(fHi, ss);
            for (int f = fLo ; f < fHi ; f++)
               setFrameData(f, ss, -1);

            int f0 = prevTransitionFrame(fLo, ss);
	    if (f0 < 0 || getFrameData(f0, ss) != sss)
               setFrameData(fLo, ss, sss);

            if (endState != sss) {
               setFrameData(fHi, ss, endState);

	       int f1 = nextTransitionFrame(fHi, ss);
	       if (f1 >= 0 && getFrameData(f1, ss) == endState)
	          setFrameData(f1, ss, -1);
            }
            setStateAtFrame((int)(10 * runningTime));
         }
         ss = -1;
      }

      for (int s = 0 ; s < getNumSignals() ; s++) {
         int sy0 = signalY(s);
         int sy1 = signalY(s+1);
         for (int f = 0 ; f < frameData.length ; f++)
            if (frameData[f][s] > 0) {
               int f1 = f;
               for ( ; f1 < frameData.length - 2 ; f1++)
                  if (frameData[f1+1][s] >= 0)
                     break;
               g.setColor(selectedColor);
               if (! get(s).isMultistate())
                   g.fillRect(sx + f, sy0, f1 - f + 1, (sy1 - sy0) - 2);
               else {
	          int n = frameData[f][s] - 1;
		  int ns = get(s).size() - 1;
                  g.fillRect(sx + f, sy0 + n * (sy1 - sy0) / ns, f1 - f + 1, (sy1 - sy0) / ns);
               }
               f = f1;
            }
      }

      if (isInTimeline && ss >= 0) {
         g.setColor(isDeleteMode ? deleteColor : dragColor);
         int dy = signalY(ss+1) - signalY(ss);
         if (! isDeleteMode && get(ss).isMultistate())
            g.fillRect(sxLo, signalY(ss) + (sss - 1) * dy / 2, sxHi - sxLo, dy / 2);
         else
            g.fillRect(sxLo, signalY(ss), sxHi - sxLo, dy - 1);
      }

      if (cxHi > cxLo) {
         g.setColor(commandSelectColor);
         g.fillRect(cxLo, sy, cxHi - cxLo, sh);
      }

      if (isTransitionFlash && isTransition()) {
         g.setColor(timelineFlashColor);
         g.fillRect(sx, sy + sh, sw, pbs);
      }

      g.setColor(isTransitionFlash ? Color.white : Color.black);
      g.fill3DRect(fbx, fby, fbw, fbh, ! isInFlashButton);
      g.setColor(isTransitionFlash ? Color.black : Color.gray);
      g.drawString("flash", fbx + 2, fby + fontHeight);
   }

   int fbx, fby, fbw, fbh;

   public int getState(int s) {
      return getState((int)(10 * runningTime), s);
   }

   int getState(int f, int s) {
      for ( ; f >= 0 ; f--)
         if (frameData[f][s] >= 0)
	    return frameData[f][s];
      return 0;
   }

   int prevTransitionFrame(int ff, int s) {
      for (int f = ff - 1 ; f >= 0 ; f--)
         if (getFrameData(f, s) >= 0)
	    return f;
      return -1;
   }

   int nextTransitionFrame(int ff, int s) {
      for (int f = ff + 1 ; f < frameData.length ; f++)
         if (getFrameData(f, s) >= 0)
	    return f;
      return -1;
   }

   int frameData[][];
   boolean pauseData[];
   boolean isRunning;
   boolean isDeleteMode;
   boolean isPauseControl;
   double runningTime = 0;
   int signalBorder = 12;
   int signalHeight = 29;
   int sx, sy, sw, sh;
   boolean isInTimeline = false;
   boolean isInFlashButton = false;
   int ss = -1, sss = 0, sxLo, sxHi, cxLo, cxHi, cx;
   boolean isDamage = false;
   boolean isCommandMode = false;
   boolean isTransitionFlash = false;

   Color timelineBgColor1 = new Color(255, 255, 255, 32);
   Color timelineBgColor2 = new Color(180, 180, 180, 32);
   Color timelineLineColor = new Color(0, 0, 0, 64);
   Color timelineFlashColor = new Color(255, 255, 255);
   Color selectedColor = new Color(200, 200, 200, 128);
   Color dragColor = new Color(200, 200, 200, 64);
   Color deleteColor = new Color(0, 0, 0, 64);
   Color pauseColor = new Color(255, 0, 0);
   Color pausePressedColor = new Color(96, 0, 0);
   Color pauseStripColor = new Color(255, 0, 0, 32);
   Color currentFrameColor = new Color(0, 128, 255);
   Color commandSelectColor = new Color(255, 0, 0, 64);
}

