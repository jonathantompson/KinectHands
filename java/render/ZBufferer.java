//<pre>
//Copyright 2001 Ken Perlin

/*
  To do:
     option to create a list of zbuffered px,py,z.
*/

package render;

import java.util.*;

public class ZBufferer {
   public static int bgZ = - (1 << 31);

   int bgColor = pack(0,0,0);
   int vertexDepth = 6;
   double transparency = 0;
   int a[], b[], c[] = new int[8], d[];
   int zbuffer[];
   int pix[];
   int bgPix[];
   int W = 0, H = 0, left, top, right, bottom;
   boolean meshOnly;
   boolean showMesh;
   int sparkleColor, sparkleSize = 32;
   int speckleColor, speckleSize = 40;
   Geometry s;
   Random R = new Random(0);
   int frame = 0;

   public ZBufferer(int pix[], int W, int H) {
      this.pix = pix;
      this.W = W;
      this.H = H;
      zbuffer = new int[W * H];
   }

   public void setDirtyArea(int left, int top, int right, int bottom) {
      this.left   = Math.max(0, left);
      this.top    = Math.max(0, top);
      this.right  = Math.min(W, right);
      this.bottom = Math.min(H, bottom);
   }

   public void setBg(int bgSrc[]) {
      bgPix = bgSrc;
   }

   public int[] getBg() {
      return bgPix;
   }

   public void setBgColor(int r, int g, int b) {
      bgColor = pack(r, g, b);
   }

   public void setMeshOnly(boolean tf) {
      meshOnly = tf;
   }

   public void setShowMesh(boolean tf) {
      showMesh = tf;
   }

   void copy(int src[], int dst[], int i, int n) {
      for (int k = 0 ; k < n ; k++)
         dst[i + k] = src[i + k];
   }

   public boolean isBg(int i) {
      return zbuffer[i] == bgZ;
   }

   public void startFrame() {
      frame++;

      if (bgPix != null) {
         int l = Math.max(0, Math.min(W, left));
         int r = Math.max(0, Math.min(W, right));
         int t = Math.max(0, Math.min(H, top));
         int b = Math.max(0, Math.min(H, bottom));
         for (int y = 0 ; y < t ; y++)
            copy(bgPix, pix, xy2i(0, y), W);
         for (int y = t ; y < b ; y++)
            copy(bgPix, pix, xy2i(0, y), l);
         for (int y = t ; y < b ; y++)
            copy(bgPix, pix, xy2i(r, y), W - r);
         for (int y = b ; y < H ; y++)
            copy(bgPix, pix, xy2i(0, y), W);
      }

      for (int y = top ; y < bottom ; y++) {
         int i = xy2i(left, y);
         for (int x = left ; x < right ; x++) {
            zbuffer[i] = bgZ;
            pix[i] = bgPix == null ? bgColor : bgPix[i];
            i++;
         }
      }

      nParticles = 0;
   }

   public void setGeometry(Geometry s) {
      this.s = s;
      vertexDepth = s.vertexDepth;
      transparency = s.material == null ? 0 : s.material.transparency;
   }

   public void setSparkleColor(int color) {
      sparkleColor = color;
   }

   public void setSparkleSize(int size) {
      sparkleSize = size;
   }

   public int nParticles = 0;
   public int particleX[] = new int[100000];
   public int particleY[] = new int[100000];

   public void fillPoint(int[] A) {
      int x = A[0] >> NB;
      int y = A[1] >> NB;
      int z = A[2];
      int r = A[3];
      int g = A[4];
      int b = A[5];

      // NEED TO IMPLEMENT STREAK EFFECT

      if (s.isStreak) {
         int s = (int)Math.sqrt(r * r + g * g + b * b);
	 if (s >= 1) {
            for (int j = -s ; j <= s ; j++) {
	       int dx = j * r / s;
	       int dy = j * g / s;
	       int dz = j * b / s;
	       int xx = x + dx;
	       int yy = y + dy;
	       int zz = z + dz;

	       if (xx >= left && xx < right && yy >= top && yy < bottom) {
	          int ii = xx + W * yy;
	          if (zz > zbuffer[ii]) {
	             zbuffer[ii] = zz;
                     blend(ii, zz, 255, 128, 255, A[6]);
	          }
	       }
            }
         }
	 return;
      }

      if (sparkleColor == 0 || R.nextInt(3000) != 0) {
         if (x >= 0 && x < W - 1 && y >= 0 && y < H - 1) {
            int i = x + W * y;
            if (z > zbuffer[i]) {
	       particleX[nParticles] = x;
	       particleY[nParticles] = y;
	       nParticles++;
/*
               zbuffer[i] = z;

	       int xx = A[0] >> NB - 8;
	       int yy = A[1] >> NB - 8;
               blend(xx / 256., yy / 256., z, r, g, b, 255);
*/
               //pix[i] = pack(r, g, b);
            }
         }
         return;
      }

      r = sparkleColor >> 16 & 255;
      g = sparkleColor >>  8 & 255;
      b = sparkleColor       & 255;
      int s = sparkleSize/2 + R.nextInt(3*sparkleSize/2);

      for (int xx = x - s ; xx <= x + s ; xx++)
      for (int yy = y - 2 ; yy <= y + 2 ; yy++) {
         double yyy = yy + (xx - x) / 6.0;
         if (xx >= 0 && xx < W-1 && yyy >= 0 && yyy < H-1) {
            int rr = (xx - x) * (xx - x) + 6 * s * (yy - y) * (yy - y);
            if (rr < s * s) {
               int alpha = 256 - (rr << 8) / (s * s);
               blend(xx, yyy, z, r, g, b, alpha * alpha >> 8);
            }
         }
      }

      for (int xx = x - 2 ; xx <= x + 2 ; xx++)
      for (int yy = y - s ; yy <= y + s ; yy++) {
         double xxx = xx - (yy - y) / 6.0;
         if (xxx >= 0 && xxx < W-1 && yy >= 0 && yy < H-1) {
            int rr = 6 * s * (xx - x) * (xx - x) + (yy - y) * (yy - y);
            if (rr < s * s) {
               int alpha = 256 - (rr << 8) / (s * s);
               blend(xxx, yy, z, r, g, b, alpha * alpha >> 8);
            }
         }
      }
   }

   public void setPixel(int i, int rgb, int z) {
      if (z > zbuffer[i]) {
         pix[i] = rgb;
         zbuffer[i] = z;
      }
   }

   void blend(double x, double y, int z, int r, int g, int b, int alpha) {
       int ix = (int)x;
       int iy = (int)y;
       int i = ix + W * iy;
       int fx = (int)(256 * (x % 1.0));
       int fy = (int)(256 * (y % 1.0));
       blend(i        , z, r, g, b, alpha * (255 - fx) * (255 - fy) >> 16);
       blend(i + 1    , z, r, g, b, alpha *        fx  * (255 - fy) >> 16);
       blend(i     + W, z, r, g, b, alpha * (255 - fx) *        fy  >> 16);
       blend(i + 1 + W, z, r, g, b, alpha *        fx  *        fy  >> 16);
   }

   void blend(int i, int z, int r, int g, int b, int alpha) {
       if (true || z > zbuffer[i]) {
          int pr = pix[i] >> 16 & 255;
          int pg = pix[i] >>  8 & 255;
          int pb = pix[i]       & 255;
          pix[i] = pack(pr + ((r - pr) * alpha >> 8),
                        pg + ((g - pg) * alpha >> 8),
                        pb + ((b - pb) * alpha >> 8));
      }
   }

   public void fillTriangle(int[] A, int[] B, int[] C) {

      int y0 = A[1] < B[1] ? (A[1] < C[1] ? 0 : 2) : (B[1] < C[1] ? 1 : 2);
      int y2 = A[1] > B[1] ? (A[1] > C[1] ? 0 : 2) : (B[1] > C[1] ? 1 : 2);
      int y1 = 3 - (y0 + y2);
      if (y0 == y2)
              return;

      a = y0 == 0 ? A : y0 == 1 ? B : C; // FIRST VERTEX IN Y SCAN
      b = y1 == 0 ? A : y1 == 1 ? B : C; // MIDDLE VERTEX IN Y SCAN
      d = y2 == 0 ? A : y2 == 1 ? B : C; // LAST VERTEX IN Y SCAN

      // SPLIT TOP-TO-BOTTOM EDGE

      double t = (double) (b[1] - a[1]) / (d[1] - a[1]);
      for (int i = 0; i < vertexDepth; i++)
         c[i] = (int) (a[i] + t * (d[i] - a[i]));

      if (b[0] < c[0]) {
         fillTrapezoid(a, a, b, c);
         fillTrapezoid(b, c, d, d);
      }
      else {
         fillTrapezoid(a, a, c, b);
         fillTrapezoid(c, b, d, d);
      }
   }

   protected void fillTrapezoid(int A[], int B[], int C[], int D[]) {

      int zb[] = zbuffer, px[] = pix; // LOCAL ARRAYS CAN BE FASTER

      int yLo = A[1] >> NB;
      int yHi = C[1] >> NB;
      if (yHi < 0 || yLo >= H)
         return;

      int deltaY = yHi - yLo;
      if (deltaY <= 0)
         return;

      int xL = A[0];
      int zL = A[2];
      int rL = A[3];
      int gL = A[4];
      int bL = A[5];

      for (int i = 6; i < vertexDepth; i++)
         pixelL[i] = A[i];

      int dxL = (C[0] - A[0]) / deltaY;
      int dzL = (C[2] - A[2]) / deltaY;
      int drL = (C[3] - A[3]) / deltaY;
      int dgL = (C[4] - A[4]) / deltaY;
      int dbL = (C[5] - A[5]) / deltaY;

      for (int i = 6; i < vertexDepth; i++)
         dpixelL[i] = (C[i] - A[i]) / deltaY;

      int xR = B[0];
      int zR = B[2];
      int rR = B[3];
      int gR = B[4];
      int bR = B[5];

      for (int i = 6; i < vertexDepth; i++)
         pixelR[i] = B[i];

      int dxR = (D[0] - B[0]) / deltaY;
      int dzR = (D[2] - B[2]) / deltaY;
      int drR = (D[3] - B[3]) / deltaY;
      int dgR = (D[4] - B[4]) / deltaY;
      int dbR = (D[5] - B[5]) / deltaY;

      for (int i = 6; i < vertexDepth; i++)
         dpixelR[i] = (D[i] - B[i]) / deltaY;

      int ixL, ixR, deltaX, z, r, g, b, dz = 0, dr = 0, dg = 0, db = 0;
/*
      if (vertexDepth > 6)
         dpixel = new int[vertexDepth];
*/
      boolean isOpaque = (transparency == 0);
      int opacity = (int) ((1 - Math.abs(transparency)) * (1 << NB));
      int a0, r0, g0, b0, packed;

      if (yLo < 0) {
         xL -= dxL * yLo;
         zL -= dzL * yLo;
         rL -= drL * yLo;
         gL -= dgL * yLo;
         bL -= dbL * yLo;

         if (vertexDepth > 6) {
            for (int i = 6; i < vertexDepth; i++)
               pixelL[i] -= dpixelL[i] * yLo;

         }

         xR -= dxR * yLo;
         zR -= dzR * yLo;
         rR -= drR * yLo;
         gR -= dgR * yLo;
         bR -= dbR * yLo;

         for (int i = 6; i < vertexDepth; i++)
            pixelR[i] -= dpixelR[i] * yLo;

         yLo = 0;
      }
      yHi = Math.min(yHi, H);

      for (int y = yLo; y < yHi; y++) {

         ixL = xL >> NB;
         ixR = xR >> NB;

         deltaX = ixR - ixL;
         z = zL;
         r = rL;
         g = gL;
         b = bL;

         if (vertexDepth > 6) {
            pixel[1] = y;
            pixel[2] = z;
            pixel[3] = r;
            pixel[4] = g;
            pixel[5] = b;
            for (int i = 6; i < vertexDepth; i++)
               pixel[i] = pixelL[i];
         }

         if (deltaX > 0) {
            dz = (zR - zL) / deltaX;
            dr = (rR - rL) / deltaX;
            dg = (gR - gL) / deltaX;
            db = (bR - bL) / deltaX;
            for (int i = 6; i < vertexDepth; i++)
               dpixel[i] = (pixelR[i] - pixelL[i]) / deltaX;
         }

         if (ixL < 0) {
            z -= dz * ixL;
            r -= dr * ixL;
            g -= dg * ixL;
            b -= db * ixL;

            if (vertexDepth > 6) {
               pixel[1] = y;
               pixel[2] = z;
               pixel[3] = r;
               pixel[4] = g;
               pixel[5] = b;
               for (int i = 6; i < vertexDepth; i++)
                  pixel[i] -= dpixel[i] * ixL;
            }
            ixL = 0;
         }

         ixR = Math.min(ixR, W);

         int i = xy2i(ixL, y);
         int op = opacity;
         for (int ix = ixL; ix < ixR; ix++) {
            if (z > zb[i]) {
               pixel[0] = ix;
               if (isOpaque) {
                  if (vertexDepth > 6) {

                     int rc = r >> NB, gc = g >> NB, bc = b >> NB;

                     packed = computeTexture(pixel, deltaX, deltaY);
		     int at = packed >> 24 & 255;
                     int rt = packed >> 16 & 255;
                     int gt = packed >>  8 & 255;
                     int bt = packed       & 255;

		     if (at == 255) {
                        px[i] = pack(rc * rt >> 8,
			             gc * gt >> 8,
			             bc * bt >> 8);
                     }
                     else {
                        packed = px[i];
                        r0 = packed >> 16 & 255;
                        g0 = packed >>  8 & 255;
                        b0 = packed       & 255;

                        int r1 = rc * rt >> 8;
                        int g1 = gc * gt >> 8;
                        int b1 = bc * bt >> 8;

                        px[i] = pack(r0 + (at * (r1 - r0) >> 8),
                                     g0 + (at * (g1 - g0) >> 8),
                                     b0 + (at * (b1 - b0) >> 8));
		     }
                  }
                  else {
                     specklePixel(ix, y, 255, z);
                     px[i] = pack(r >> NB, g >> NB, b >> NB);
                  }

               } else {
                  packed = px[i];
                  r0 = packed >> 16 & 255;
                  g0 = packed >>  8 & 255;
                  b0 = packed       & 255;
                  op = opacity;
                  int rc = r >> NB, gc = g >> NB, bc = b >> NB;
                  if (vertexDepth > 6) {
                     int c = computeTexture(pixel, deltaX, deltaY);
                     op = (c >> 24 & 255) * op / 255;
                     rc = c >> 16 & 255;
                     gc = c >>  8 & 255;
                     bc = c       & 255;
                  }
                  specklePixel(ix, y, op, z);
                  px[i] = pack(r0 + (op * (rc - r0) >> NB),
                               g0 + (op * (gc - g0) >> NB),
                               b0 + (op * (bc - b0) >> NB));
               }
               if (showMesh)
                  if (ix == ixL || ix == ixR - 1)
                     px[i] = pack(0, 0, 0);
               zb[i] = z;
               if (meshOnly)
                  break;
            }
            z += dz;
            r += dr;
            g += dg;
            b += db;

            if (vertexDepth > 6) {
               pixel[0] = ix;
               pixel[2] = z;
               pixel[3] = r;
               pixel[4] = g;
               pixel[5] = b;
            }

            for (int k = 6; k < vertexDepth; k++)
               pixel[k] += dpixel[k];
            i++;
         }

         xL += dxL;
         zL += dzL;
         rL += drL;
         gL += dgL;
         bL += dbL;

         for (int k = 6; k < vertexDepth; k++)
            pixelL[k] += dpixelL[k];

         xR += dxR;
         zR += dzR;
         rR += drR;
         gR += dgR;
         bR += dbR;

         for (int k = 6; k < vertexDepth; k++)
            pixelR[k] += dpixelR[k];
      }
   }

   public void setSpeckleColor(int color) {
      speckleColor = color;
   }

   void specklePixel(int x, int y, int opacity, int z) {
      if (speckleColor != 0 &&
          Math.random() * 255 < opacity &&
          Noise.noise(.2 * x, .2 * y, .1 * frame) > 0.2) {
         int sx = x + (int)(speckleSize * (Math.random() - 0.5));
         int sy = y + (int)(speckleSize * (Math.random() - 0.5));
         if (sx >= 0 && sx < W && sy >= 0 && sy < H) {
            int j = xy2i(sx, sy);
            pix[j] = speckleColor;
            zbuffer[j] = z;
         }
      }
   }

   protected int computeTexture(int pixel[], int deltaX, int deltaY) {
      return s.material == null ? 0 : s.material.computePixel(pixel, deltaX, deltaY, NB);
   }

   protected int xy2i(int x, int y) {
      return y * W + x;
   }

   protected static int pack(int r, int g, int b) {
      return r << 16 | g << 8 | b | 0xff000000;
   }

   protected int[] pixel   = new int[8];
   protected int[] pixelL  = new int[8];
   protected int[] pixelR  = new int[8];

   protected int[] dpixel  = new int[8];
   protected int[] dpixelL = new int[8];
   protected int[] dpixelR = new int[8];

   protected static final int NB = 14;
}

