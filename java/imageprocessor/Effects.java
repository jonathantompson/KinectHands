package imageprocessor;

import java.util.*;

public class Effects {
   public Effects(int w, int h) {
      this.w = w;
      this.h = h;
      line = new int[2 * Math.max(w, h)];
      for (int d = 0 ; d < tmp.length ; d++)
         tmp[d] = new int[w * h >> 2 * d];
      for (int i = 0 ; i < rgb.length ; i++)
         rgb[i] = new int[w * h];
   }
   
   public void toComponent(int dst[], int src[], int c) {
      int shift = 8 * (2 - c);
      for (int i = 0 ; i < dst.length ; i++)
         dst[i] = src[i] >> shift & 255;
   }
   
   public void toARGB(int dst[], int r[], int g[], int b[]) {
      for (int i = 0 ; i < dst.length ; i++)
         dst[i] = 255 << 24 | r[i] << 16 | g[i] << 8 | b[i];
   }
   
   public void toARGB(int dst[]) {
      for (int i = 0 ; i < dst.length ; i++) {
         int c = dst[i] & 255;
         dst[i] = 255 << 24 | c << 16 | c << 8 | c;
      }
   }
   
   public void toValue(int dst[]) {
      for (int i = 0 ; i < dst.length ; i++) {
         int c = dst[i], r = c >> 16 & 255, g = c >> 8 & 255, b = c & 255;
         dst[i] = r + g + g + b >> 2;
      }
   }
   
   public void horizontalFlip(int dst[]) {
      for (int y = 0 ; y < h ; y++) {
         int i0 = w * y, i1 = i0 + w - 1;
         for (int x = 0 ; x < w / 2 ; x++) {
            int tmp = dst[i0];
            dst[i0++] = dst[i1];
            dst[i1--] = tmp;
         }
      }
   }
   
   public void createZShiftMap(int dst[], int z[]) {
      int i = 0;
      for (int y = 0 ; y < h ; y++)
         for (int x = 0 ; x < w ; x++)
            dst[i] = (y << 4) - (x >> 4) << 16
            | (x << 4) - Math.max(0, 230 - z[i++]) * 3 / 2 + 115 + (y >> 4);
   }
   
   public void createTranslationMap(int dst[], double dx, double dy) {
      int idx = (int)(dx * 16);
      int idy = (int)(dy * 16);
      int i = 0;
      for (int y = 0 ; y < h ; y++)
         for (int x = 0 ; x < w ; x++)
            dst[i++] = (y << 4) + idy << 16 | (x << 4) + idx;
   }
   
   public void map(int dst[], int src[], int map[]) {
      int alpha[] = tmp[deres];
      copy(alpha, 0);
      copy(dst, 0);
      for (int i = 0 ; i < src.length ; i++)
         if (src[i] > 0) {
            int xx = map[i] & 65535;
            int yy = map[i] >> 16 & 65535;
            int x = xx >> 4, dx = xx & 15;
            int y = yy >> 4, dy = yy & 15;
            int j = x + w * y;
            if (j >= 0 && j < dst.length - w - 1) {
               int c = src[i];
               
               int a00 = (16 - dx) * (16 - dy), a10 = dx  * (16 - dy),
               a01 = (16 - dx) *       dy , a11 = dx  *       dy ;
               
               dst[j        ] += c * a00;
               dst[j + 1    ] += c * a10;
               dst[j     + w] += c * a01;
               dst[j + 1 + w] += c * a11;
               
               alpha[j        ] += a00;
               alpha[j + 1    ] += a10;
               alpha[j     + w] += a01;
               alpha[j + 1 + w] += a11;
            }
         }
      for (int i = 0 ; i < src.length ; i++)
         if (alpha[i] > 0)
            dst[i] /= alpha[i];
   }
   
   public void verticalFlip(int dst[]) {
      for (int x = 0 ; x < w ; x++) {
         int i0 = x, i1 = x + w * (h - 1);
         for (int y = 0 ; y < h / 2 ; y++) {
            int tmp = dst[i0];
            dst[i0] = dst[i1];
            dst[i1] = tmp;
            i0 += w;
            i1 -= w;
         }
      }
   }
   
   /*
    Given a silhouette, grow outward
    breadth-first, to compute distances
    from the silhouette.
    */
   
   public void grow(int dst[]) {
      int count = 0;
      for (int i = 0 ; i < dst.length ; i++) {
         if (dst[i] > 0 && dst[i] < 255)
            count++;
      }
      System.err.println(count);
   }
   
   public void findSilhouette(int dst[]) {
      for (int y = 0 ; y < h - 1 ; y++) {
         int i = w * y;
         for (int x = 0 ; x < w - 1 ; x++) {
            int a = dst[i], b = dst[i+1], c = dst[i+w], d = dst[i+w+1];
            if (Math.max(a, Math.max(b, Math.max(c, d))) < 128)
               dst[i] = 0;
            else if (Math.min(a, Math.min(b, Math.min(c, d))) > 128)
               dst[i] = 255;
            else
               dst[i] = a + b + c + d >> 2;
            i++;
         }
      }
   }
   
   public void scale(int dst[], double sx, double sy, double x0, double y0) {
      if (sx > 1.0)
         scaleUpX(tmp[deres], dst, sx, x0);
      else
         scaleDownX(tmp[deres], dst, sx, x0);
      
      if (sy > 1.0)
         scaleUpY(dst, tmp[deres], sy, y0);
      else
         scaleDownY(dst, tmp[deres], sy, y0);
   }
   
   public void blur(int dst[], int radius, int deres) {
      for (int i = 0 ; i < deres ; i++)
         dnRes(tmp[i+1], i == 0 ? dst : tmp[i]);
      blur(deres == 0 ? dst : tmp[deres], radius);
      for (int i = deres-1 ; i >= 0 ; i--)
         upRes(i == 0 ? dst : tmp[i], tmp[i+1]);
   }
   
   public void blurRGB(int dst[], int radius) {
       blurRGB(dst, radius, 0);
   }

   public void blurRGB(int dst[], int radius, int deres) {
      for (int i = 0 ; i < 3 ; i++) {
         toComponent(rgb[i], dst, i);
         blur(rgb[i], radius, deres);
      }
      toARGB(dst, rgb[0], rgb[1], rgb[2]);
   }

   public void addGlow(int dst[], int mask[], int radius, int color, boolean keepMask, int intensity) {
      dnRes(tmp[1], mask);
      dnRes(tmp[2], tmp[1]);
      blur (tmp[2], radius);
      if (intensity > 1)
         for (int i = 0 ; i < tmp[2].length ; i++)
            tmp[2][i] *= intensity;
      upRes(tmp[1], tmp[2]);
      upRes(tmp[0], tmp[1]);
      if (! keepMask)
         mask(tmp[0], mask, 0);
      comp(dst, color, tmp[0]);
   }

   public void dnRes() {
      w >>= 1;
      h >>= 1;
      deres++;
   }

   public void upRes() {
      --deres;
      w <<= 1;
      h <<= 1;
   }

   public void dnRes(int dst[], int src[]) {
      int w2 = w >> 1, h2 = h >> 1;
      dnRes(dst, w2, h2, src, w, h);
      dnRes();
   }

   public void upRes(int dst[], int src[]) {
      int w2 = w, h2 = h;
      upRes();
      upRes(dst, w, h, src, w2, h2);
   }

   public static void dnRes(int dst[], int w2, int h2, int src[], int w, int h) {
      for (int x = 0 ; x < w ; x += 2)
      for (int y = 0 ; y < h ; y += 2) {
         int x2 = x >> 1, y2 = y >> 1;
         int i = x + w * y;
         dst[x2 + w2 * y2] = src[i] + src[i + 1] + src[i + w] + src[i + w + 1] >> 2;
      }
   }

   public static void upRes(int dst[], int w, int h, int src[], int w2, int h2) {
      int dy = 0;
      for (int y = h - 2 ; y >= 0 ; y -= 2) {
         int y2 = y >> 1;

         int x  = w-2;
         int x2 = x >> 1;
         int i  = x  + w  * y ;
         int i2 = x2 + w2 * y2;

         int dx = 0;
         while (x >= 0) {
            int s00 = src[i2], s10 = src[i2 + dx], s01 = src[i2 + dy], s11 = src[i2 + dx + dy];

            dst[i        ] = s00;
            dst[i + 1    ] = s00 + s10 >> 1;
            dst[i     + w] = s00 + s01 >> 1;
            dst[i + 1 + w] = s00 + s10 + s01 + s11 >> 2;

            x-=2;
            x2--;
            i-=2;
            i2--;
            dx = 1;
         }
         dy = w2;
      }
   }

   public void blur(int dst[], int radius) {
      for (int n = 0 ; n < 2 ; n++) {
         blurX(dst, radius);
         blurY(dst, radius);
      }
   }
   
   public int labelComponents(int dst[], int colors[]) {
      int cMax = 0;
      int n = findComponents(dst, tmp[deres]), c;
      for (int i = 0 ; i < dst.length ; i++)
         if ((c = tmp[deres][i]) >= 0 && c < colors.length)
            dst[i] = colors[c];
      return n;
   }
   
   public int findCentroids2(int dst[], int xy[][]) {
      int n = 0;
      copy(tmp[deres], dst);
      while (findCentroid2(tmp[deres], xy[n]))
         n++;
      
      return n;
   }
   
   public int findCentroids(int dst[], int xy[][]) {
      int sx = 0, sy = 0, sw = 0, i = 0;
      for (int y = 0 ; y < h ; y++)
         for (int x = 0 ; x < w ; x++) {
            sx += dst[i] * x;
            sy += dst[i] * y;
            sw += dst[i];
            i++;
         }
      if (sw > w * h / 2) {
         xy[0][0] = sx / sw;
         xy[0][1] = sy / sw;
         return 1;
      }
      return 0;
   }
   
   public void comp(int dst[], int color, int mask[]) {
      for (int i = 0 ; i < dst.length ; i++)
         dst[i] = mix(dst[i], color, mask[i]);
   }
   
   public void comp(int dst[], int src[], int mask[]) {
      for (int i = 0 ; i < dst.length ; i++)
         dst[i] = mix(dst[i], src[i], mask[i]);
   }
   
   public void comp(int dst[], int src[], int mask) {
      for (int i = 0 ; i < dst.length ; i++)
         dst[i] = mix(dst[i], src[i], mask);
   }
   
   public void fill(int dst[], int x, int y, int width, int height, int value) {
      for (int _x = x ; _x < x + width ; _x++)
         for (int _y = y ; _y < y + height ; _y++)
            dst[_x + w * _y] = value;
   }
   
   static public void copy(int dst[], int value) {
      Arrays.fill(dst, 0, dst.length, value);
   }
   
   static public void copy(int dst[], int src[]) {
      System.arraycopy(src, 0, dst, 0, dst.length);
   }
   
   static public void hicon(int dst[]) {
      for (int i = 0 ; i < dst.length ; i++)
         dst[i] = dst[i] >= 128 ? 255 : 0;
   }
   
   static public void mask(int dst[], int mask[], int mode) {
      for (int i = 0 ; i < dst.length ; i++)
         if ((mode > 0) != (mask[i] > 0))
            dst[i] = 0;
   }
   
   public void fillRect(int dst[], int x, int y, int width, int height, int color) {
      int xLo = Math.max(0, Math.min(w, x         ));
      int xHi = Math.max(0, Math.min(w, x + width ));
      int yLo = Math.max(0, Math.min(h, y         ));
      int yHi = Math.max(0, Math.min(h, y + height));
      for (y = yLo ; y < yHi ; y++) {
         int i0 = w * y + xLo;
         int i1 = w * y + xHi;
         Arrays.fill(dst, i0, i1, color);
      }
   }
   
   public void fillOval(int dst[], int x, int y, int width, int height, int color) {
      int xLo = Math.max(0, Math.min(w, x         ));
      int xHi = Math.max(0, Math.min(w, x + width ));
      int yLo = Math.max(0, Math.min(h, y         ));
      int yHi = Math.max(0, Math.min(h, y + height));
      double xm = 0.5 * (xHi + xLo);
      double ym = 0.5 * (yHi + yLo);
      double xr = 0.5 * (xHi - xLo);
      double yr = 0.5 * (yHi - yLo);
      for (y = yLo ; y < yHi ; y++) {
         double t = (y - ym) / yr;
         double u = Math.sqrt(1 - t * t);
         int i0 = w * y + (int)(xm + u * (xLo - xm));
         int i1 = w * y + (int)(xm + u * (xHi - xm));
         Arrays.fill(dst, i0, i1, color);
      }
   }
   
   ////////////////// PRIVATE METHODS AND DATA //////////////////
   
   int findComponents(int img[], int id[]) {
      copy(id, -1);
      int n = 0;
      while (findComponent(img, id, n))
         n++;
      return n;
   }
   
   boolean findComponent(int img[], int id[], int n) {
      int i = 0;
      for (int y = 0 ; y < h ; y++)
         for (int x = 0 ; x < w ; x++) {
            if (img[i] > 0 && id[i] == -1) {
               fillComponent(img, id, n, x, y, i);
               return true;
            }
            i++;
         }
      return false;
   }
   
   void fillComponent(int img[], int id[], int n, int x, int y, int i) {
      if (! (img[i] > 0 && id[i] == -1))
         return;
      
      int i0 = i, iLo = w * y;
      while(i0 > iLo && img[i0-1] > 0 && id[i] == -1)
         i0--;
      
      int i1 = i, iHi = iLo + w;
      while(i1 < iHi && img[i1] > 0 && id[i] == -1)
         i1++;
      
      Arrays.fill(id, i0, i1, n);
      
      if (y > 0)
         for (int ii = i0 ; ii < i1 ; ii++)
            fillComponent(img, id, n, x + ii - i, y - 1, ii - w);
      
      if (y < h - 1)
         for (int ii = i0 ; ii < i1 ; ii++)
            fillComponent(img, id, n, x + ii - i, y + 1, ii + w);
   }
   
   boolean findCentroid2(int img[], int xy[]) {
      for (int y = 0 ; y < h ; y++)
         for (int x = 0 ; x < w ; x++)
            if (img[x + w * y] > 0) {
               findPeak(img, x, y, xy);
               int s = img[xy[0] + w * xy[1]];
               clearArea(img, xy[0], xy[1]);
               return s >= 200;
            }
      return false;
   }
   
   void findPeak(int img[], int x, int y, int xy[]) {
      int xm = x > 0 ? -1 : 0, xp = x < w-1 ? 1 : 0;
      int ym = y > 0 ? -w : 0, yp = y < h-1 ? w : 0;
      
      int i = x + w * y;
      int s = img[i], smx = img[i + xm], spx = img[i + xp], smy = img[i + ym], spy = img[i + yp];
      
      if      (smx > Math.max(s, Math.max(spx, Math.max(smy, spy)))) findPeak(img, x - 1, y, xy);
      else if (spx > Math.max(s, Math.max(smx, Math.max(smy, spy)))) findPeak(img, x + 1, y, xy);
      else if (smy > Math.max(s, Math.max(spy, Math.max(smx, spx)))) findPeak(img, x, y - 1, xy);
      else if (spy > Math.max(s, Math.max(smy, Math.max(smx, spx)))) findPeak(img, x, y + 1, xy);
      else {
         xy[0] = x;
         xy[1] = y;
      }
   }
   
   void clearArea(int img[], int x, int y) {
      if (x >= 0 && x < w && y >= 0 && y < h) {
         int i = x + w * y;
         if (img[i] != 0) {
            int iLo = w * y, iHi = iLo + w;
            
            while (i > iLo && img[i-1] > 0)
               i--;
            while (i < iHi && img[i] > 0)
               img[i++] = 0;
            
            clearArea(img, x, y - 1);
            clearArea(img, x, y + 1);
         }
      }
   }
   
   public int mix(int a, int b, int c) {
      if (c <= 0)
         return a;
      if (c >= 255)
         return b;
      
      int ar = a >> 16 & 255, ag = a >> 8 & 255, ab = a & 255;
      int br = b >> 16 & 255, bg = b >> 8 & 255, bb = b & 255;
      c += c >> 7;
      return 255 << 24 | ar + (c * (br - ar) >> 8) << 16
                       | ag + (c * (bg - ag) >> 8) <<  8
                       | ab + (c * (bb - ab) >> 8)     ;
   }
   
   int rL(int radius, int x       ) { return Math.min(radius, x); }
   int rR(int radius, int x, int w) { return Math.min(radius, w - 1 - x); }
   int rT(int radius, int x       ) { return Math.min(radius, x); }
   int rB(int radius, int x, int h) { return Math.min(radius, h - 1 - x); }
   
   public void blurX(int dst[], int radius) {
      blurX(dst, radius, w, h);
   }

   public void blurY(int dst[], int radius) {
      blurY(dst, radius, w, h);
   }

   public void blurX(int dst[], int radius, int w, int h) {
      for (int y = 0 ; y < h ; y++) {
         int j = w * y;
         for (int x = 0 ; x < w ; x++) {
            if (dst[j] != 0) {
               
               int x0 = x;
               int i0 = x + w * y;
               int i1 = w + w * y;
               while (dst[i1 - 1] == 0)
                  i1--;
               int x1 = x0 + i1 - i0;
               
               Arrays.fill(line, x0 - rL(radius, x0), x1 + rR(radius, x1, w) + 1, 0);
               for (int i = i0 ; i < i1 ; i++) {
                  int d = dst[i];
                  line[x - rL(radius, x   )    ] += d;
                  line[x + rR(radius, x, w) + 1] -= d;
                  x++;
               }
               int sum = 0, i = (x0 - rL(radius, x0)) + w * y;
               for (x = x0 - rL(radius, x0) ; x < x1 + rR(radius, x1, w) + 1 ; x++) {
                  dst[i] = (sum += line[x]) / (rL(radius, x) + rR(radius, x, w) + 1);
                  i++;
               }
               break;
            }
            j++;
         }
      }
   }
   
   public void blurY(int dst[], int radius, int w, int h) {
      for (int x = 0 ; x < w ; x++) {
         int j = x;
         for (int y = 0 ; y < h ; y++) {
            if (dst[j] != 0) {
               
               int y0 = y;
               int i0 = x + w * y0;
               int i1 = x + w * h;
               while (dst[i1-w] == 0)
                  i1-=w;
               int y1 = (i1 - i0) / w + y0;
               
               Arrays.fill(line, y0 - rT(radius, y0), y1 + rB(radius, y1, h) + 1, 0);
               for (int i = i0 ; i < i1 ; i+=w) {
                  int d = dst[i];
                  line[y - rT(radius, y   )    ] += d;
                  line[y + rB(radius, y, h) + 1] -= d;
                  y++;
               }
               int sum = 0, i = x + w * (y0 - rT(radius, y0));
               for (y = y0 - rT(radius, y0) ; y < y1 + rB(radius, y1, h) + 1 ; y++) {
                  dst[i] = (sum += line[y]) / (rT(radius, y) + rB(radius, y, h) + 1);
                  i+=w;
               }
               break;
            }
            j+= w;
         }
      }
   }
   
   void scaleUpX(int dst[], int src[], double s, double xFixed) {
      copy(dst, 0);
      for (int x = 0 ; x < w ; x++) {
         double sx0 = Math.max(0, Math.min(w - 1, (x - 0.5 - xFixed) / s + xFixed));
         double sx1 = Math.max(0, Math.min(w - 1, (x + 0.5 - xFixed) / s + xFixed));
         if (sx0 < sx1) {
            int ix0 = (int)sx0;
            int ix1 = (int)sx1;
            double fx0 = sx0 - ix0;
            double fx1 = sx1 - ix1;
            if (ix0 == ix1) {
               double fx = (fx0 + fx1) / 2;
               int a = (int)(256 * fx         );
               int b = (int)(256 * (fx1 - fx0));
               int di = x, si = ix0;
               for (int y = 0 ; y < h ; y++) {
                  dst[di] = ilerp(a, src[si], src[si + 1]) * b >> 8;
                  di += w;
                  si += w;
               }
            }
            else {
               double f0 = (fx0 + 1) / 2;
               double f1 =  fx1      / 2;
               int a = (int)(256 * f0       );
               int b = (int)(256 * f1       );
               int c = (int)(256 * (1 - fx0));
               int d = (int)(256 * fx1      );
               int di = x, si = ix0;
               if (ix0 < w - 2)
                  for (int y = 0 ; y < h ; y++) {
                     dst[di] = ilerp(a, src[si    ], src[si + 1]) * c +
                     ilerp(b, src[si + 1], src[si + 2]) * d >> 8;
                     di += w;
                     si += w;
                  }
            }
         }
      }
   }
   
   void scaleUpY(int dst[], int src[], double s, double yFixed) {
      copy(dst, 0);
      for (int y = 0 ; y < h ; y++) {
         double sy0 = Math.max(0, Math.min(h - 1, (y - 0.5 - yFixed) / s + yFixed));
         double sy1 = Math.max(0, Math.min(h - 1, (y + 0.5 - yFixed) / s + yFixed));
         if (sy0 < sy1) {
            int iy0 = (int)sy0;
            int iy1 = (int)sy1;
            double fy0 = sy0 - iy0;
            double fy1 = sy1 - iy1;
            if (iy0 == iy1) {
               double fy = (fy0 + fy1) / 2;
               int a = (int)(256 * fy         );
               int b = (int)(256 * (fy1 - fy0));
               int di = w * y, si = w * iy0;
               for (int x = 0 ; x < w ; x++) {
                  dst[di] = ilerp(a, src[si], src[si + w]) * b >> 8;
                  di++;
                  si++;
               }
            }
            else {
               double f0 = (fy0 + 1) / 2;
               double f1 =  fy1      / 2;
               int a = (int)(256 * f0       );
               int b = (int)(256 * f1       );
               int c = (int)(256 * (1 - fy0));
               int d = (int)(256 * fy1      );
               int di = w * y, si = w * iy0;
               if (si < w * (h - 2))
                  for (int x = 0 ; x < w ; x++) {
                     dst[di] = ilerp(a, src[si    ], src[si + w    ]) * c +
                     ilerp(b, src[si + w], src[si + w + w]) * d >> 8;
                     di++;
                     si++;
                  }
            }
         }
      }
   }
   
   void scaleDownX(int dst[], int src[], double s, double xFixed) {
      copy(dst, 0);
      for (int x = 0 ; x < w ; x++) {
         double dx0 = Math.max(0, Math.min(w - 1, (x - 0.5 - xFixed) * s + xFixed));
         double dx1 = Math.max(0, Math.min(w - 1, (x + 0.5 - xFixed) * s + xFixed));
         if (dx0 < dx1) {
            int ix0 = (int)dx0;
            int ix1 = (int)dx1;
            double fx0 = dx0 - ix0;
            double fx1 = dx1 - ix1;
            if (ix0 == ix1) {
               double fx = (fx0 + fx1) / 2;
               int a = (int)(256 * fx         );
               int b = (int)(256 * (fx1 - fx0));
               int si = x, di = ix0;
               if (si < w - 1)
                  for (int y = 0 ; y < h ; y++) {
                     dst[di] += ilerp(a, src[si], src[si + 1]) * b >> 8;
                     si += w;
                     di += w;
                  }
            }
            else {
               double f0 = (fx0 + 1) / 2;
               double f1 =  fx1      / 2;
               int a = (int)(256 * f0       );
               int b = (int)(256 * f1       );
               int c = (int)(256 * (1 - fx0));
               int d = (int)(256 * fx1      );
               int si = x, di = ix0;
               if (di < w - 1 && si < w - 2)
                  for (int y = 0 ; y < h ; y++) {
                     dst[di    ] += ilerp(a, src[si    ], src[si + 1]) * c >> 8;
                     dst[di + 1] += ilerp(b, src[si + 1], src[si + 2]) * d >> 8;
                     si += w;
                     di += w;
                  }
            }
         }
      }
   }
   
   void scaleDownY(int dst[], int src[], double s, double yFixed) {
      copy(dst, 0);
      for (int y = 0 ; y < h ; y++) {
         double dy0 = Math.max(0, Math.min(h - 1, (y - 0.5 - yFixed) * s + yFixed));
         double dy1 = Math.max(0, Math.min(h - 1, (y + 0.5 - yFixed) * s + yFixed));
         if (dy0 < dy1) {
            int iy0 = (int)dy0;
            int iy1 = (int)dy1;
            double fy0 = dy0 - iy0;
            double fy1 = dy1 - iy1;
            if (iy0 == iy1) {
               double fy = (fy0 + fy1) / 2;
               int a = (int)(256 * fy         );
               int b = (int)(256 * (fy1 - fy0));
               int si = w * y, di = w * iy0;
               if (si < w * (h - 1))
                  for (int x = 0 ; x < w ; x++) {
                     dst[di] += ilerp(a, src[si], src[si + w]) * b >> 8;
                     si++;
                     di++;
                  }
            }
            else {
               double f0 = (fy0 + 1) / 2;
               double f1 =  fy1      / 2;
               int a = (int)(256 * f0       );
               int b = (int)(256 * f1       );
               int c = (int)(256 * (1 - fy0));
               int d = (int)(256 * fy1      );
               int si = w * y, di = w * iy0;
               if (di < w * (h - 1) && si < w * (h - 2))
                  for (int x = 0 ; x < w ; x++) {
                     dst[di    ] += ilerp(a, src[si    ], src[si + w    ]) * c >> 8;
                     dst[di + w] += ilerp(b, src[si + w], src[si + w + w]) * d >> 8;
                     si++;
                     di++;
                  }
            }
         }
      }
   }
   
   int w = 0, h = 0, deres = 0;
   int line[], tmp[][] = new int[6][], rgb[][] = new int[3][];
   
   static double lerp(double t, double a, double b) { return a + t * (b - a); }
   static int ilerp(int t, int a, int b) { return a + (t * (b - a) >> 8); }

   public static void shrink(int src[], int dst[], int wd, int hd, int scale) {
      switch (scale) {
      case 1: copy    (dst, src)        ; break;
      case 2: shrink2 (src, dst, wd, hd); break;
      case 3: shrink3b(src, dst, wd, hd); break;
      }
   }

   public static void shrink2(int src[], int dst[], int wd, int hd) {
      int ws = 2 * wd;
      int hs = 2 * hd;
/*
      for (int xs = 0 ; xs < ws ; xs += 2)
      for (int ys = 0 ; ys < hs ; ys += 2) {
         int xd = xs >> 1, yd = ys >> 1;
         int is = xs + ws * ys;
         dst[xd + wd * yd] = avg(src[is], src[is + 1], src[is + ws], src[is + ws + 1]);
      }
*/
      for (int xd = 0 ; xd < wd ; xd++)
      for (int yd = 0 ; yd < hd ; yd++) {
         int xs = 2 * xd, ys = 2 * yd;
         int is = xs + ws * ys;
         dst[xd + wd * yd] = avg(src[is], src[is + 1], src[is + ws], src[is + ws + 1]);
      }
   }

   public static void shrink3(int src[], int dst[], int wd, int hd) {
      int ws = 3 * wd;
      int hs = 3 * hd;
      for (int xd = 0 ; xd < wd ; xd++)
      for (int yd = 0 ; yd < hd ; yd++) {
         int xs = 3 * xd, ys = 3 * yd;
         int is = xs + ws * ys;
         dst[xd + wd * yd] = avg(src[is       ], src[is        + 1], src[is        + 2],
	                         src[is +   ws], src[is +   ws + 1], src[is +   ws + 2],
	                         src[is + 2*ws], src[is + 2*ws + 1], src[is + 2*ws + 2]);
      }
   }

   public static void shrink3b(int src[], int dst[], int wd, int hd) {
      int ws = 3 * wd;
      int hs = 3 * hd;
      for (int xd = 0 ; xd < wd ; xd++)
      for (int yd = 0 ; yd < hd ; yd++) {
         int xs = 3 * xd, ys = 3 * yd;
         int is = xs + ws * ys;
         int s0 = src[is], s1 = src[is + 1], s2 = src[is + 2];
	 is += ws;
         int s3 = src[is], s4 = src[is + 1], s5 = src[is + 2];
	 is += ws;
         int s6 = src[is], s7 = src[is + 1], s8 = src[is + 2];
	 int r = ( (s0 >> 16 & 255) + (s1 >> 16 & 255) + (s2 >> 16 & 255) +
	           (s3 >> 16 & 255) + (s4 >> 16 & 255) + (s5 >> 16 & 255) +
	           (s6 >> 16 & 255) + (s7 >> 16 & 255) + (s8 >> 16 & 255) ) / 9;
	 int g = ( (s0 >>  8 & 255) + (s1 >>  8 & 255) + (s2 >>  8 & 255) +
	           (s3 >>  8 & 255) + (s4 >>  8 & 255) + (s5 >>  8 & 255) +
	           (s6 >>  8 & 255) + (s7 >>  8 & 255) + (s8 >>  8 & 255) ) / 9;
	 int b = ( (s0       & 255) + (s1       & 255) + (s2       & 255) +
	           (s3       & 255) + (s4       & 255) + (s5       & 255) +
	           (s6       & 255) + (s7       & 255) + (s8       & 255) ) / 9;
         dst[xd + wd * yd] = 0xff000000 | r << 16 | g << 8 | b;
      }
   }

   public static void enlarge(int src[], int dst[], int ws, int hs, int wd, int hd) {
      for (int y = 0 ; y < hs ; y++) {
         if (2 * y >= hd)
            return;
         int i1 = ws * y;
         int i1y = y < hs-1 ? ws : 0;
         int i2 = wd * (2 * y) + (wd - 2 * ws) / 2;
         for (int x = 0 ; x < ws ; x++) {
            int i1x = x < ws-1 ? 1 : 0;
            dst[i2] = src[i1];
            dst[i2 +  1] = avg(src[i1], src[i1 + i1x]);
            dst[i2 + wd] = avg(src[i1], src[i1 + i1y]);
            dst[i2 + wd + 1] = avg(avg(src[i1], src[i1 + i1x]), avg(src[i1 + i1y], src[i1 + i1x + i1y]));

            i1++;
            i2 += 2;
         }
      }
   }

   static int mask2 = createRGBMask(2);
   static int mask4 = createRGBMask(4);
   static int mask8 = createRGBMask(8);

   public static int avg(int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8, int p9) {
      return 0xff000000 | avg(p1, avg(p2, p3, p4, p5, p6, p7, p8, p9));
   }

   public static int avg(int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8) {
      return 0xff000000 | (p1 & mask8) + (p2 & mask8) + (p3 & mask8) + (p4 & mask8) +
                          (p5 & mask8) + (p6 & mask8) + (p7 & mask8) + (p8 & mask8) >> 3;
   }

   public static int avg(int p1, int p2, int p3, int p4) {
      return 0xff000000 | (p1 & mask4) + (p2 & mask4) + (p3 & mask4) + (p4 & mask4) >> 2;
   }

   public static int avg(int p1, int p2) {
      return 0xff000000 | (p1 & mask2) + (p2 & mask2) >> 1;
   }

   static int createRGBMask(int n) {
      int m = 256 - n;
      return m << 16 | m << 8 | m;
   }
}


