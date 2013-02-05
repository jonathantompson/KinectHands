
package render;

public class Signal
{
   boolean state;
   int hotKey;
   int selectedState;
   double value;
   double rate = 1.0;
   String label;
   States states;
   Signal signal[];

   public Signal(String infoString) {
      int i = infoString.indexOf(':');
      if (i >= 0) {
         String info[] = infoString.split(":");
	 String statesInfo[] = info[1].split(",");
	 States states = new States();
	 for (int n = 0 ; n < statesInfo.length; n++)
	    states.addState(statesInfo[n]);
         init(info[0], states);
      }
      else
         init(infoString, null);
   }

   void init(String str, States states) {
      label = InfoString.getLabel(str);
      hotKey = InfoString.getHotKey(str);
      this.states = states;
      if (isMultistate()) {
         signal = new Signal[states.size()];
         for (int i = 0 ; i < signal.length ; i++)
            signal[i] = new Signal(states.getInfoString(i));
         signal[0].setState(true);
         signal[0].setValue(1);
      }
   }

   public int indexOf(String label) {
      for (int n = 0 ; n < size() ; n++)
         if (InfoString.getLabel(states.getInfoString(n)).equals(label))
	    return n;
      return -1;
   }

   public boolean isMultistate() { return states != null; }
   public int size() { return isMultistate() ? states.size() : 0; }

   public void setRate(double rate) {
      this.rate = rate;
      if (isMultistate())
         for (int i = 0 ; i < signal.length ; i++)
            signal[i].setRate(rate);
   }

   public void setState(boolean state) {
      this.state = state;
   }

   public int getSelected() { return selectedState; }

   public void select(int n) {
      if (isMultistate()) {
         selectedState = n;
         for (int i = 0 ; i < signal.length ; i++)
            signal[i].setState(i == n);
      }
      else
         setState(n > 0);
   }

   public String getLabel() { return label; }
   public String getLabel(int n) { return isMultistate() ? signal[n].getLabel() : ""; }
   public int getHotKey() { return hotKey; }
   public int getHotKey(int n) { return isMultistate() ? signal[n].getHotKey() : 0; }

   public boolean getState() { return state; }

   public double getValue() { return Util.sCurve(value); }
   public double getValue(int n) { return isMultistate() ? signal[n].getValue() : 0; }

   public void setValue(double value) {
      this.value = value;
   }

   public void update(double elapsed) {
      elapsed = Math.max(0, Math.min(0.1, elapsed));
      if (isMultistate())
         for (int i = 0 ; i < signal.length ; i++)
            signal[i].update(elapsed);
      else if (state)
         value = Math.min(1, value + rate * elapsed);
      else
         value = Math.max(0, value - rate * elapsed);
   }

   public boolean isHotKey(int key) {
      if (size() == 0) {
         if (getHotKey() == key)
	    return true;
      }
      else {
         for (int i = 0 ; i < size() ; i++)
            if (getHotKey(i) == key)
	       return true;
      }
      return false;
   }

   public int tryHotKey(int key) {
      if (size() == 0) {
         if (getHotKey() == key) {
            setState(! getState());
            return getState() ? 1 : 0;
         }
      }
      else {
         for (int i = 0 ; i < size() ; i++)
            if (getHotKey(i) == key) {
               select(i > 0 && getSelected() == i ? 0 : i);
               return getSelected();
            }
      }
      return -1;
   }
}

