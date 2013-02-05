
package render;
import java.util.*;

public class States
{
   int nStates = 0;
   ArrayList infoStrings = new ArrayList();

   public int size() {
      return nStates;
   }

   public String getInfoString(int n) { return (String)infoStrings.get(n); }

   public int addState(String infoString) {
      infoStrings.add(infoString);
      return nStates++;
   }
}

