
package imageprocessor;

import java.awt.*;
import java.io.*;
import util.*;

public class ImageWriter
{

   static public void write(String name, Image image) {
      try {
         (new JpegEncoder(image, 100, new FileOutputStream(name))).Compress();
      } catch (Exception e) { System.err.println(e); }
   }

   static String leftPad(int value, int len0, int c) {
      String s = "" + value;
      int len = s.length();
      return ( len==len0+1 ? "" + (char)c + (char)c + (char)c :
               len==len0+2 ? "" + (char)c + (char)c :
               len==len0+3 ? "" + (char)c : ""
             ) + s;
   }
}

