
package render;

public class InfoString
{
   public static String getLabel(String str) {
     if (str == null) return "";
     int i = str.indexOf("]");
     return i == -1 ? str : str.substring(i+1, str.length());
   }

   public static int getHotKey(String str) {
     if (str == null) return 0;
     int i = str.indexOf(']');
     return i == -1 ? 0 : str.charAt(i - 1);
   }
}

