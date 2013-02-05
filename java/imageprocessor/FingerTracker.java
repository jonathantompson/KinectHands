package imageprocessor;

import java.awt.*;
import java.util.Arrays;

public class FingerTracker {
   
	ImageProcessor ip;
	Effects e;
	public FindContours fc;
	int w;
	int h;
	int numfingers;
	int[] tmp;
	int meltFactor = 40;
	double[] screenx;
	double[] screeny;
	double[] normalx;
	double[] normaly;
	int[] contour;
	boolean printVerbose = true;
	double FINGER_RADIUS = 15.0;				// perimeter of a fingertip
	double ROUNDNESS_THRESHOLD = 1.33;			// minimum allowable value for 
   // ratio of area / perimeter of fingertip
   int numFingersDesired;
   
	public FingerTracker(int w, int h) {
		this.w = w;
		this.h = h;
		ip = new ImageProcessor(w, h);
		e = new Effects(w, h);
		fc = new FindContours(w, h);
		fc.setThreshold(128);
		
		screenx = new double[w * h];
		screeny = new double[w * h];
		normalx = new double[w * h];
		normaly = new double[w * h];
		contour = new int[w * h];
		tmp = new int[w * h];
      numFingersDesired = 2;		
	}
   
	public void setMeltFactor(int value) {
	   meltFactor = value;
	}
	
   private int findLeftMostIndexOnContour(int k) {
      double best = 1000;
      int besti = 0;
      for (int i = 0; i < fc.getContourLength(k); i++) {
         double x = fc.getContourX(k,i);
         if (x < best) {
            best = x;
            besti = i;
         }
      }
      return besti;
   }
   
   private int findTopMostIndexOnContour(int k) {
      double best = 1000;
      int besti = 0;
      for (int i = 0; i < fc.getContourLength(k); i++) {
         double y = fc.getContourY(k,i);
         if (y < best) {
            best = y;
            besti = i;
         }
      }
      return besti;
   }
   
   public void setNumFingersDesired(int num) {
      numFingersDesired = num;
   }
   
	public void update(int[] pix) {
            
      numfingers = 0;
      int numcontours = fc.findIsoline(pix);      
      if (numFingersDesired <= 2) {
         int x1 = 0;
         int y1 = 0;
         int i = 0;
         for (; i < pix.length; i++) {
            if (pix[i] > 0) {
               x1 = i % w;
               y1 = i / w;
               screenx[numfingers] = x1;
               screeny[numfingers] = y1;               
               numfingers++;
               break;
            }
         }
         
         if (numfingers < numFingersDesired) {
            for (; i < pix.length; i++) {
               int x2 = i % w;
               int y2 = i / w;
               if (pix[i] > 0 && Math.abs(x2 - x1) > 150) {
                  screenx[numfingers] = x2;
                  screeny[numfingers] = y2;               
                  numfingers++;
                  break;
               }
            }            
         }
         return;
      }
      

		e.blur(pix, 5);
		for (int i = 0; i < pix.length; i++) {
			int x = i % w;
			int y = i / w;
			if (x < 1 || x > (w - 2) || y < 1 || y > (h - 2)) {
				pix[i] = 0;
			}
		}
      int bigenoughcontours = 0;
      for (int k = 0; k < numcontours; k++) {
         int l = fc.getContourLength(k);
         if (l > 100) {
            bigenoughcontours++;
         }
      }
      
      numcontours = bigenoughcontours;
      
      if (numcontours == 1) {
         int pointer = findTopMostIndexOnContour(0);
         screenx[numfingers] = fc.getContourX(0, pointer);
         screeny[numfingers] = fc.getContourY(0, pointer);
         numfingers++;
      } else if (numcontours >= 2) {
         
         int uk = 0;
         int pk = 0;
         int ul = 0;
         int pl = 0;
         for (int k = 0; k < numcontours; k++) {
            int l = fc.getContourLength(k);
            if (l > ul) {
               pl = ul;
               pk = uk;
               uk = k;
               ul = l;
            } else if (l > pl) {
               pl = l;
               pk = k;
            }
         }
         double com0 = fc.getBBCenterX(uk);
         double com1 = fc.getBBCenterX(pk);
         int left = uk;
         int right = pk;
         if (com1 < com0) {
            left = pk;
            right = uk;
         }
         int lefti = findTopMostIndexOnContour(left);
         int righti = findTopMostIndexOnContour(right);         
         screenx[numfingers] = fc.getContourX(left, lefti);
         screeny[numfingers] = fc.getContourY(left, lefti);
         numfingers++;
         screenx[numfingers] = fc.getContourX(right, righti);
         screeny[numfingers] = fc.getContourY(right, righti);
         numfingers++;  
      } else {
         
         for (int i = 0 ; i < meltFactor ; i++)
            fc.meltContours();
         
         int spanoffset = 0;
         int window = (int)FINGER_RADIUS;
         double[] tips = fc.findRoundedCorners(window);
         numfingers = 0;
         for (int k = 0; k < numcontours; k++) {
            int l = fc.getContourLength(k);
            for (int i = 0; i < l; i++) {
               if (tips[fc.getValidIndex(k, i)] <= ROUNDNESS_THRESHOLD) {
                  spanoffset = i + 1;
                  break;
               }
            }
            int span = 0;
            for (int i = spanoffset; i < l + spanoffset; i++) {
               double roundness = tips[fc.getValidIndex(k, i)];
               
               if (roundness > ROUNDNESS_THRESHOLD) {
                  span++;
               } else {
                  if (span > 0) {
                     int tip = (i - 1) - span/2;
                     int lo = tip - window;
                     int hi = tip + window;
                     if (fc.measureDistance(k, lo, hi) < 2 * FINGER_RADIUS) {
                        double cx = 0;
                        double cy = 0;
                        for (int j = lo; j <= hi; j++) {
                           cx += fc.getContourX(k,j);
                           cy += fc.getContourY(k,j);
                        }
                        cx = cx / (2 * window + 1);
                        cy = cy / (2 * window + 1);
                        screenx[numfingers] = cx;
                        screeny[numfingers] = cy;
                        contour[numfingers] = k;
                        
                        double nx = fc.measureNormalX(k, tip - 2 * window, tip + 2 * window);
                        double ny = fc.measureNormalY(k,  tip - 2 * window, tip + 2 * window);
                        double nl = Math.sqrt(nx * nx + ny * ny);
                        if (nl == 0) {
                           normalx[numfingers] = 0;
                           normaly[numfingers] = 0;
                        } else {
                           normalx[numfingers] = nx / nl;
                           normaly[numfingers] = ny / nl;
                        }
                        numfingers++;
                     }
                     
                  }
                  span = 0;
               }
            }
         }
      }
   }
   
   public double getFingerX(int i) {
      return screenx[i];
   }
   
   public double getFingerY(int i) {
      return screeny[i];
   }
   
   public double getFingerNormalX(int i) {
      return normalx[i];
   }
   
   public double getFingerNormalY(int i) {
      return normaly[i];
   }
   
   public int getContour(int i) {
      return contour[i];
   }
   
   public void sortByY() {
      for (int i = 0; i < numfingers; i++) {
         tmp[i] = i;
      }
      ip.sortArrayWithAuxArray(screeny, tmp, numfingers);
      ip.remapArray(screenx, tmp, numfingers);
      ip.remapArray(normalx, tmp, numfingers);
      ip.remapArray(normaly, tmp, numfingers);		
   }
   
   public int getNumFingers() {
      return numfingers;
   }
   
   public void setPrintVerbose(boolean val) {
      printVerbose = val;
   }
   
   public void out(String s) {
      if (printVerbose) {
         System.out.println(s);
      }
   }
   
}
