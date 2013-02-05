package imageprocessor;

import java.util.Arrays;
import java.awt.*;

public class FindContours {

	double TAU = 3.14159 * 2;
	public double[] px;
	public double[] py;
	public double[] tx;		// tmp
	public double[] ty;		// tmp
	public double[] tn;		// tmp
	public double[] vx;		// tmp
	public double[] vy;		// tmp
	public double[] minx;
	public double[] miny;
	public double[] maxx;
	public double[] maxy;
	
	public int[] cd;	
	public double[] cx;	
	public double[] cy;
	public double[] cu;
	public int[] cl;
	public int[] co;
	public double[] cu2;	
	public double[] tips;
	
	public int[] pointsx;
	public int[] pointsy;
	double[] dx;
	double[] dy;
	public double[] an;
	public double[] as;
	double[] tmp;
	
	double hx;
	double hy;
	
	public int n = 0;
	int MAX_POINTS = 5000;
	double EDGE_LENGTH = 2;	// actually edge length squared
	int[] img;
	int w;
	int h;
	double STEP_LENGTH = 0.25;
	double step = STEP_LENGTH;
	int threshold = 128;
	
	public int numContours = 0;
	
	boolean printVerbose = true;
	
	public boolean flag;
	
	public FindContours() {
		this(640, 480);
	}

	public FindContours(int w, int h) {
		
		this.w = w;
		this.h = h;
		
		cd = new int[w * h];	
		cx = new double[w * h];	
		cy = new double[w * h];
		cu = new double[w * h];
		cl = new int[w * h];
		co = new int[w * h];
		cu = new double[w * h];
		tips = new double[w * h];
		minx = new double[w * h];
		miny = new double[w * h];
		maxx = new double[w * h];
		maxy = new double[w * h];
		
		px = new double[MAX_POINTS];
		py = new double[MAX_POINTS];
		tx = new double[MAX_POINTS];
		ty = new double[MAX_POINTS];
		tn = new double[MAX_POINTS];
		vx = new double[w * h];
		vy = new double[w * h];
		
		
		dx = new double[MAX_POINTS];
		dy = new double[MAX_POINTS];
		an = new double[MAX_POINTS];
		as = new double[MAX_POINTS];
		cu = new double[MAX_POINTS];
		tmp = new double[MAX_POINTS];
				
		pointsx = new int[MAX_POINTS];
		pointsy = new int[MAX_POINTS];
		
	}
	
	
	public void setImage(int[] buf) {
		img = buf;
	}
	
	public void setThreshold(int t) {
		threshold = t;
	}
		
	public int[] getPointsX() {
		for (int i = 0; i <= n; i++) {
			pointsx[i] = (int)px[i];
		}
		return pointsx;
	}
	
	public int[] getPointsY() {
		for (int i = 0; i <= n; i++) {
			pointsy[i] = (int)py[i];
		}
		return pointsy;
	}
	
	public double getHeadX() {
		return hx;
	}
	
	public double getHeadY() {
		return hy;
	}
		
	double[] top = new double[5];
	double[] bot = new double[5];
	double[] rgt = new double[5];
	double[] lft = new double[5];
	
	public double sample(double x, double y, int[] img) {
	
		// compute query rect
		// for each of 4 possible intersecting rects
		//		compute area of overlap
		//		multiply weight times pixel value of target times area
		//		add result to accumulator
		// return accumulator
		//
		// overlap rects are numbered 0, 1, 2, 3 going left->right & top->bottom
		
		top[0] = y;
		bot[0] = y + 1;
		lft[0] = x;
		rgt[0] = x + 1;
		
		top[1] = (int)top[0];
		top[2] = (int)top[0];
		top[3] = (int)top[0] + 1.0;
		top[4] = (int)top[0] + 1.0;

		bot[1] = (int)bot[0];
		bot[2] = (int)bot[0];
		bot[3] = (int)bot[0] + 1.0;
		bot[4] = (int)bot[0] + 1.0;
		
		lft[1] = (int)lft[0];
		lft[2] = (int)lft[0] + 1.0;
		lft[3] = (int)lft[0];
		lft[4] = (int)lft[0] + 1.0;

		rgt[1] = (int)rgt[0];
		rgt[2] = (int)rgt[0] + 1.0;
		rgt[3] = (int)rgt[0];
		rgt[4] = (int)rgt[0] + 1.0;
		
		double acc = 0;					// accumulator
		double areatot = 0;
		for (int i = 1; i < 5; i++) {		
			double t = top[0] > top[i] ? top[0] : top[i];
			double b = bot[0] > bot[i] ? bot[i] : bot[0];
			double l = lft[0] > lft[i] ? lft[0] : lft[i];
			double r = rgt[0] > rgt[i] ? rgt[i] : rgt[0];
			
			if ( t < b  && l < r ) {
				double area = (b - t) * (r - l);
				//int idx = (int)(top[i]) * w + (int)lft[i];
				int idx = ixy(lft[i], top[i]);
				acc += area * img[idx];
			}
		}
		return acc;
	}

	
	public int lk(int i) {
		if (i > n) i = i - n - 1;
		if (i < 0) i = i + n + 1;
		return i;
	}
		
	
	public double t(int A, int B) {
		if (A-B == 0) return 0;
		return ((double)(A - threshold)) / ((double)(A - B));
		//return ((A - threshold)) / ((A - B));
	}

	final int CASE0		=	0x00000000; 
	final int CASE1		=	0x00000001;
	final int CASE2		=	0x00000010;
	final int CASE3		=	0x00000011;
	final int CASE4		=	0x00000100;
	final int CASE5		=	0x00000101;
	final int CASE6		=	0x00000110;
	final int CASE7		=	0x00000111;
	final int CASE8		=	0x00001000;
	final int CASE9		=	0x00001001;
	final int CASE10	=	0x00001010;
	final int CASE11	=	0x00001011;
	final int CASE12	=	0x00001100;
	final int CASE13	=	0x00001101;
	final int CASE14	=	0x00001110;
	final int CASE15	=	0x00001111;
	final int WHITE		=	1;
	final int BLACK		=	1;

	public int findIsoline(int[] in) {
		int vi = 0;
		int b0, b1, b2, b3;
		for (int x = 0; x < w - 1; x++) {
			for (int y = 0; y < h - 1; y++) {
				b0 = x == 0			||	y == 0			||	in[ixy(x + 0, y + 0)] < threshold ? 0x00001000 : 0x00000000;
				b1 = x == (w - 1)	||	y == 0			||	in[ixy(x + 1, y + 0)] < threshold ? 0x00000100 : 0x00000000;
				b2 = x == (w - 1)	||	y == (h - 1)	||	in[ixy(x + 1, y + 1)] < threshold ? 0x00000010 : 0x00000000;
				b3 = x == 0			||	y == (h - 1)	||	in[ixy(x + 0, y + 1)] < threshold ? 0x00000001 : 0x00000000;
				cd[ixy(x, y)] = b0 | b1 | b2 | b3;
			}
		}
				
		int next = -1;
		int i = 0, x = 0, y = 0;
		double tox = 0, fromx = 0;
		double toy = 0, fromy = 0;
		int fromedge = -1;
		int toedge = -1;
		int code = 0;
		double avg = 0;
		int contournum = -1;
		int length = 0;
		while (i < w * h) {
			
			fromedge = toedge;			
			if (next < 0) next = ++i;
			x = next % w;
			y = next / w;
			if (x >= (w - 1) || y >= (h - 1)) {
				next = -1;
				continue;
			}
			code = cd[next];
			
			switch (code) {
				case CASE0:									// CASE 0
					toedge = -1;
					break;
				case CASE1:									// CASE 1
					cd[next] = 0;
					//out ("case 1");
					toedge = 2;
					break;
				case CASE2:									// CASE 2
					cd[next] = 0;
					//out ("case 2");
					toedge = 1;
					break;
				case CASE3:									// CASE 3
					cd[next] = 0;
					toedge = 1;
					//out ("case 3");
					
					break;
				case CASE4:									// CASE 4
					cd[next] = 0;
					toedge = 0;
					//out ("case 4");
					
					break;
				case CASE5:									// CASE 5, saddle
					avg = 0.25 * (double)(in[ixy(x + 0, y + 0)] + in[ixy(x + 1, y + 0)] + in[ixy(x + 0, y + 1)] + in[ixy(x + 1, y + 1)]);
					if (avg > threshold) {
						if (fromedge == 3) {						// treat as case 1, then switch code to case 4
							toedge = 2;								
							cd[next] = CASE4;
						} else {									// treat as case 4, then switch code to case 1
							toedge = 0;								
							cd[next] = CASE1;
						}
					} else {
						if (fromedge == 3) {						// treat as case 7, then switch code to case 13
							toedge = 0;								
							cd[next] = CASE13;
						} else {									// treat as case 13, then switch code to case 7
							toedge = 2;								
							cd[next] = CASE7;
						}
					}
					//out ("case 5");
					
					break;
				case CASE6:									// CASE 6
					cd[next] = 0;
					toedge = 0;
					//out ("case 6");
					
					break;
				case CASE7:									// CASE 7
					cd[next] = 0;
					toedge = 0;
					//out ("case 7");
					
					break;
				case CASE8:									// CASE 8
					cd[next] = 0;
					toedge = 3;
					//out ("case 8");
					
					break;
				case CASE9:									// CASE 9
					cd[next] = 0;
					toedge = 2;
					//out ("case 9");
					
					break;
				case CASE10:									// CASE 10, saddle
					avg = 0.25 * (double)(in[ixy(x + 0, y + 0)] + in[ixy(x + 1, y + 0)] + in[ixy(x + 0, y + 1)] + in[ixy(x + 1, y + 1)]);
					if (avg > threshold) {
						if (fromedge == 0) {						// treat as case 8, then switch code to case 2
							toedge = 3;								
							cd[next] = CASE2;
						} else {									// treat as case 2, then switch code to case 8
							toedge = 1;								
							cd[next] = CASE8;
						}
					} else {
						if (fromedge == 2) {						// treat as case 14, then switch code to case 11
							toedge = 3;								
							cd[next] = CASE11;
						} else {									// treat as case 11, then switch code to case 14
							toedge = 1;								
							cd[next] = CASE14;
						}
					}
					//out ("case 10");
					
					break;
				case CASE11:									// CASE 11
					cd[next] = 0;
					toedge = 1;
					//out ("case 11");
					
					break;
				case CASE12:									// CASE 12
					cd[next] = 0;
					toedge = 3;
					//out ("case 12");
					
					break;
				case CASE13:									// CASE 13
					cd[next] = 0;
					toedge = 2;
					//out ("case 13");
					
					break;
				case CASE14:									// CASE 14
					cd[next] = 0;
					toedge = 3;
					//out ("case 14");
					
					break;
				case CASE15:									// CASE 15
					toedge = -1;
					break;
				default:
					//out("Uh oh, unknown case");
					break;
			}
			
			if (fromedge == -1 && toedge > -1) {					// starting a new contour
				contournum++;
			}
			
			switch (toedge) {			
				case 0: 
					cx[vi] = x + t(in[ixy(x + 0, y + 0)], in[ixy(x + 1, y + 0)]);
					cy[vi++] = (double)y;
					next = ixy(x + 0, y - 1);
					cl[contournum] = ++length;			
					break;
				case 1:
					cx[vi] = (double)x + 1;
					cy[vi++] = y + t(in[ixy(x + 1, y + 0)], in[ixy(x + 1, y + 1)]);
					next = ixy(x + 1, y + 0);
					cl[contournum] = ++length;			
					break;
				case 2:
					cx[vi] = x + t(in[ixy(x + 0, y + 1)], in[ixy(x + 1, y + 1)]);
					cy[vi++] = (double)(y + 1);
					next = ixy(x, y + 1);
					cl[contournum] = ++length;			
					break;
				case 3:
					cx[vi] = (double)x;
					cy[vi++] = y + t(in[ixy(x + 0, y + 0)], in[ixy(x + 0, y + 1)]);
					next = ixy(x - 1, y + 0);
					cl[contournum] = ++length;			
					break;
				default:
					next = -1;
					length = 0;
					break;
			}
						
		}
		
		numContours = contournum + 1;
		int sum = 0;
		for (i = 0; i < numContours; i++) {
			co[i] = sum;
			sum += cl[i];
		}
		
		computeBoundingBoxes();
		
		return numContours;
	}
	
	
	public int[] getContourLengths() {
		return cl;
	}
	
	public int getContourLength(int contour) {
		return cl[contour];
	}
	

	public int getNumContours() {
		return numContours;
	}
	


	public int getLastIndex() {
		int nc = getNumContours();
		if (nc > 0) {
			return co[nc - 1] + cl[nc - 1];
		}
		return 0;
	}
	
	public void setContourX(int contour, int v, double x) {	
		int o = co[contour];
		cx[wrap(o + v, o, o + cl[contour])] = x;
	}
	
	public void setContourY(int contour, int v, double y) {	
		int o = co[contour];
		cy[wrap(o + v, o, o + cl[contour])] = y;
	}



	public double getContourX(int contour, int v) {	
		int o = co[contour];
		return cx[wrap(o + v, o, o + cl[contour])];
	}

	public double getContourY(int contour, int v) {
		int o = co[contour];
		return cy[wrap(o + v, o, o + cl[contour])];
	}
	
	public double getContourAngle(int contour, int v) {
		int o = co[contour];
		return cu[wrap(o + v, o, o + cl[contour])];
	}
	
	public int getValidIndex(int contour, int v) {
		int o = co[contour];
		return wrap(o + v, o, o + cl[contour]);
	}
	
	public double getBBMinX(int contour) {	return minx[contour];	}
	public double getBBMaxX(int contour) {	return maxx[contour];	}
	public double getBBMinY(int contour) {	return miny[contour];	}
	public double getBBMaxY(int contour) {	return maxy[contour];	}

	
	public double measureArea(int contour, int first, int last) {
		double area = 0;
		if (getValidIndex(contour, first) == getValidIndex(contour, last)) last = last - 1;
		int n = last - first + 1;
		double w = 0, h = 0;
		for (int i = first; i < last ; i++) {
			w = getContourX(contour, i + 1) - getContourX(contour, i);
			h = (getContourY(contour, i + 1) + getContourY(contour, i)) / 2.0;
			area += w * h;
		}
		w = getContourX(contour, first) - getContourX(contour, last);
		h = (getContourY(contour, first) + getContourY(contour, last)) / 2.0;
		area += w * h;
		return area;
	}
	
	public double measureArea(int contour) {
		return measureArea(contour, 0, getContourLength(contour));
	}

	public double measureMeanX(int contour) {
		double mean = 0.0;
		int l = getContourLength(contour);
		for (int i = 0; i < l; i++)
			mean += getContourX(contour, i);
		return mean / l;
	}
	
	public double measureMeanY(int contour) {
		double mean = 0.0;
		int l = getContourLength(contour);
		for (int i = 0; i < l; i++)
			mean += getContourY(contour, i);
		return mean / l;
	}


	public double measurePerimeter(int contour, int first, int last) {
		if (getValidIndex(contour, first) == getValidIndex(contour, last)) last = last - 1;
		double perim = 0;
		for (int i = first; i < last ; i++) {
			perim += measureLength(contour, i);
		}
		double dx = getContourX(contour, first) - getContourX(contour, last);
		double dy = getContourY(contour, first) - getContourY(contour, last);
		perim += Math.sqrt(dx * dx + dy * dy);
		return perim;
	}
	
	public double measurePerimeter(int contour) {
		return measurePerimeter(contour, 0, getContourLength(contour));
	}
	
	


	
	public double measureNormalX(int contour, int i) {
		double ret = getContourY(contour, i) - getContourY(contour, i + 1);
		ret = ret / measureLength(contour, i);
		return ret;
	}
	
	public double measureNormalY(int contour, int i) {
		double ret = getContourX(contour, i + 1) - getContourX(contour, i);
		ret = ret / measureLength(contour, i);
		return ret;
	}
	
	public double measureNormalY(int contour, int first, int last) {
		double ret = 0;
		for (int i = first; i < last ; i++) {
			ret += measureNormalY(contour, i);
		}
		return ret;
	}
	
	public double measureNormalX(int contour, int first, int last) {
		double ret = 0;
		for (int i = first; i < last ; i++) {
			ret += measureNormalX(contour, i);
		}
		return ret;
	}


	
	public double measureAngleChange(int contour, int first, int last) {
		double sum = 0;
		for (int i = first; i <= last; i++) {
			sum += measureAngle(contour, i);
		}
		return sum;
	}

	
	public int wrap(int i, int lo, int hi) {
		int l = hi - lo;
		int d = i - lo;
		int w = 0;
		if (d < 0) w = hi - ((-d) % l);
		else w = lo + d % l;
		if (w == hi) w = lo;
		if (w < lo) {
			out("went below lo");
		} else if (w >= hi) {
			out("went above hi");
		}
		

		return w;		
	}
	
	public double measureDistance(int contour, int first, int second) {
		double dx = getContourX(contour, first) - getContourX(contour, second);
		double dy = getContourY(contour, first) - getContourY(contour, second);
		return Math.sqrt(dx * dx + dy * dy);
	}
	
	
	// return length from i to i + 1
	//			i		to		i + 1
	public double measureLength(int contour, int i) {
		
		double aftx = 0, afty = 0, aftl = 0;
		
		int lo = co[contour];
		int n = cl[contour];
		int hi = lo + n;
		
		int v1 = wrap(lo + i+0, lo, hi);
		int v2 = wrap(lo + i+1, lo, hi);

		aftx = cx[v2] - cx[v1];
		afty = cy[v2] - cy[v1];
		aftl = Math.sqrt(aftx * aftx + afty * afty);
		
		return aftl;
	}

	// return the relative angle change in radians
	// about the point i (assuming ccw is positive)
	public double measureAngle(int contour, int i) {
		
		double befx = 0, befy = 0, aftx = 0, afty = 0, aftl = 0, befl = 0, dot;
		double rads;

		befx = getContourX(contour, i + 0) - getContourX(contour, i - 1);
		befy = getContourY(contour, i + 0) - getContourY(contour, i - 1);
		aftx = getContourX(contour, i + 1) - getContourX(contour, i + 0);
		afty = getContourY(contour, i + 1) - getContourY(contour, i + 0);

		befl = Math.sqrt(befx * befx + befy * befy); 
		befx = befx / befl;
		befy = befy / befl;
		aftl = Math.sqrt(aftx * aftx + afty * afty);
		aftx = aftx / aftl;
		afty = afty / aftl;		
		
		dot = befx * aftx + befy * afty;
		if (dot > 1.0) dot = 1.0;
		if (dot < 0) dot = 0;
		rads = Math.acos(dot);
		if (Double.isNaN(rads)) {
			out("oops: " + befx  + " " + befy + " " + befl + " " + aftx + " " + afty + " " + aftl);
			out("dot-product: " + (befx * aftx + befy * afty));
		}
		if (aftx * befy - afty * befx < 0) rads = rads * -1;
		return rads;
	}
	
	
	public void measureAllAngles(int w) {
		
		for (int k = 0; k < numContours; k++) {
			int o = co[k];
			int l = cl[k];
			if (l > 2 * w + 1) {
				for (int i = 0; i < l; i++) {
					cu[o + i] = measureAngle(k, i);
				}
				
				for (int i = 0; i < l; i++) {
					double perimeter = 0;
					int v1 = wrap(o + i, o, o + l);			
					cu2[v1] = 0;
					for (int j = -w; j <= w; j++) {
						int v = wrap(o + i + j, o, o + l);
						perimeter += measureLength(k, i + j);
						cu2[v1] += cu[v];
					}
					cu2[v1] = cu2[v1] / perimeter;
				}
				System.arraycopy(cu2, o, cu, o, l);			
				
			}
		}
	}
	
	public void measureAllCurvatures() {
		
		for (int k = 0; k < numContours; k++) {
			int o = co[k];
			int l = cl[k];
			for (int i = 0; i < l; i++) {
				cu[o + i] = measureAngle(k, i) / measureLength(k, i);
			}
		}
	}

   public double measureCurvature(int contour, int i) {
		return measureAngle(contour, i) / measureLength(contour, i);
	}


	public void meltContours() {
		double[] tx = vx;
		double[] ty = vy;
				
		for (int k = 0; k < numContours; k++) {
			int o = co[k];
			int l = cl[k];
						
			for (int i = 0; i < l; i++) {
				int v0 = wrap(o + i - 1, o, o + l);
				int v1 = wrap(o + i, o, o + l);
				int v2 = wrap(o + i + 1, o, o + l);
				vx[v1] = (cx[v0] + cx[v1] + cx[v2]) / 3.0;
				vy[v1] = (cy[v0] + cy[v1] + cy[v2]) / 3.0;
			}
		}
		System.arraycopy(vx, 0, cx, 0, getLastIndex());
		System.arraycopy(vy, 0, cy, 0, getLastIndex());
	}

	public double[] findAreas(int window) {
	
		for (int k = 0; k < numContours; k++) {
			int l = getContourLength(k);
			for (int i = 0; i < l; i++) {
				int lo = i - window;
				int hi = i + window;
				tips[getValidIndex(k, i)] = measureArea(k, lo, hi);
			}
		}
		return tips;
	}
	
	public double[] findRoundedCorners(int window) {
	
		for (int k = 0; k < numContours; k++) {
			int l = getContourLength(k);
			for (int i = 0; i < l; i++) {
				int lo = i - window;
				int hi = i + window;
				tips[getValidIndex(k, i)] = measureArea(k, lo, hi) / measurePerimeter(k, lo, hi);
			}
		}
		return tips;
	}

	public int ixy(int x, int y) {
		return x + y * w;
	}
	
	public int ixy(double x, double y) {
		return (int)x + (int)y * w;
	}
	
	public int getNPoints() {
		return n + 1;
	}
	
	public int getMaxContour() {
		int maxlength = 0;
		int idx = 0;
		for (int k = 0; k < numContours; k++) {
			int l = getContourLength(k);
			if (l > maxlength) {
				maxlength = l;
				idx = k;
			}
		}
		return idx;
	}
	
	public void deleteContour(int k) {
		cl[k] = 0;
	}
	
	/* POLYGON HIT TESTING ROUTINES */
	
	public void computeBoundingBoxes() {
		for (int k = 0; k < getNumContours(); k++) {
			int o = co[k];
			minx[k] = cx[o];
			miny[k] = cy[o];
			maxx[k] = cx[o];
			maxy[k] = cy[o];
			for (int i = 1; i < getContourLength(k); i++) {
				int j = o + i;
				if (cx[j] < minx[k]) 
					minx[k] = cx[j];
				if (cx[j] > maxx[k]) 
					maxx[k] = cx[j];
				if (cy[j] < miny[k]) 
					miny[k] = cy[j];
				else if (cy[j] > maxy[k]) 
					maxy[k] = cy[j];
			}
		}
	}
	
	public boolean contains(int k, double x, double y) {
	
		boolean inside = false;
		int l = getContourLength(k);
		for (int i = 0, j = -1; i < l; j = i++) {
			double xi = getContourX(k, i);
			double yi = getContourY(k, i);
			double xj = getContourX(k, j);
			double yj = getContourY(k, j);
			if ((yj > y) != (yi > y)) {
				if (yi == yj) {
					if (x < xi) inside = !inside;
				} else if (x < xi + (y - yi) * (xi - xj) / (yi - yj)) {
					inside = !inside;
				}
			}
		}
		return inside;
	}
	
	
	public boolean containsContour(int k1, int k2) {
		if (bbIntersect(k1, k2)) {
		
			double minx = getBBMinX(k2);
			double maxx = getBBMaxX(k2);
			double miny = getBBMinY(k2);
			double maxy = getBBMaxY(k2);

			if (contains(k1, minx, miny) && contains(k1, maxx, miny) && contains(k1, maxx, maxy) && contains(k1, minx, maxy))
				return true;
		}
		return false;
	}


	public boolean containsBoundingBox(int k, double minx, double miny, double maxx, double maxy) {

		if (contains(k, minx, miny) && contains(k, maxx, miny) && contains(k, maxx, maxy) && contains(k, minx, maxy))
			return true;

		return false;
	}
	
	public boolean contains(double[] polyx, double[] polyy, double x, double y) {
		boolean inside = false;
		int l = polyx.length;
		for (int i = 0, j = l - 1; i < l; j = i++) {
			double xi = polyx[i];
			double yi = polyy[i];
			double xj = polyx[j];
			double yj = polyy[j];
			if ((yj > y) != (yi > y)) {
				if (yi == yj) {
					if (x < xi) inside = !inside;
				} else if (x < xi + (y - yi) * (xi - xj) / (yi - yj)) {
					inside = !inside;
				}
			}
		}
		return inside;
	}

	
	
	// intersects contour k1 with contour k2
	public boolean bbIntersect(int k1, int k2) {
		double minx1 = getBBMinX(k1);
		double maxx1 = getBBMaxX(k1);
		double miny1 = getBBMinY(k1);
		double maxy1 = getBBMaxY(k1);
		double minx2 = getBBMinX(k2);
		double maxx2 = getBBMaxX(k2);
		double miny2 = getBBMinY(k2);
		double maxy2 = getBBMaxY(k2);
		
		double lt = minx1 > minx2 ? minx1 : minx2;
		double rt = maxx1 < maxx2 ? maxx1 : maxx2;
		double tp = miny1 > miny2 ? miny1 : miny2;
		double bt = maxy1 < maxy2 ? maxy1 : maxy2;
				
		return (lt < rt && tp < bt);
	}
	
	public boolean bbContainsBB(int k1, int k2) {
		double minx1 = getBBMinX(k1);
		double maxx1 = getBBMaxX(k1);
		double miny1 = getBBMinY(k1);
		double maxy1 = getBBMaxY(k1);
		double minx2 = getBBMinX(k2);
		double maxx2 = getBBMaxX(k2);
		double miny2 = getBBMinY(k2);
		double maxy2 = getBBMaxY(k2);
		
		return (minx1 <= minx2 && maxx1 >= maxx2 && miny1 <= miny2 && maxy1 >= maxy2);
	}
	
	
	public double bbArea(int k) {
		double w = getBBMaxX(k) - getBBMinX(k);
		double h = getBBMaxY(k) - getBBMinY(k);
		return w * h;
	}

	public double getBBCenterX(int k) {
		return (getBBMinX(k) + getBBMaxX(k)) / 2;
	}

	public double getBBCenterY(int k) {
		return (getBBMinY(k) + getBBMaxY(k)) / 2;
	}
	
	
		
	/*********************************************/
	/* DRAWING FUNCTIONS                         */
	
	public void drawContourDashed(Graphics2D g, int contour) {
		double cx = 0, cy = 0;
		for (int i = 0; i < getContourLength(contour); i++) {
			cx += getContourX(contour, i);
			cy += getContourY(contour, i);
		}
		cx = cx / getContourLength(contour);
		cy = cy / getContourLength(contour);
		
		
		double s = 1.0;
		for (int i = 0; i < getContourLength(contour); i++) {
			int xi = (int)((getContourX(contour, i) - cx) * s + cx);
			int yi = (int)((getContourY(contour, i) - cy) * s + cy);
			int xj = (int)((getContourX(contour, i - 1) - cx) * s + cx);
			int yj = (int)((getContourY(contour, i - 1) - cy) * s + cy);
			int tx = (xi + xj) / 2;
			int ty = (yi + yj) / 2;
			int nx = (int)(10 * (getContourY(contour, i) - getContourY(contour, i - 1)));
			int ny = (int)(10 * (getContourX(contour, i - 1) - getContourX(contour, i)));
			
			if (i % 2 == 0) {
					g.setColor(Color.red);
			} else {
					g.setColor(Color.cyan);
			}
			g.drawLine(xi, yi, xj, yj);
			g.setColor(Color.yellow);
			//g.drawLine(tx, ty, tx + nx, ty + ny);
			
			//g.drawLine((int)getContourX(contour, i), (int)getContourY(contour, i), (int)getContourX(contour, i), (int)getContourY(contour, i));
		}
	}
	
	public void drawContour(Graphics2D g, int contour) {
		double cx = 0, cy = 0;
		for (int i = 0; i < getContourLength(contour); i++) {
			cx += getContourX(contour, i);
			cy += getContourY(contour, i);
		}
		cx = cx / getContourLength(contour);
		cy = cy / getContourLength(contour);
		
		double s = 1.0;
		g.setColor(Color.red);
		for (int i = 0; i < getContourLength(contour); i++) {
			int xi = (int)((getContourX(contour, i) - cx) * s + cx);
			int yi = (int)((getContourY(contour, i) - cy) * s + cy);
			int xj = (int)((getContourX(contour, i - 1) - cx) * s + cx);
			int yj = (int)((getContourY(contour, i - 1) - cy) * s + cy);
			int tx = (xi + xj) / 2;
			int ty = (yi + yj) / 2;
			int nx = (int)(10 * (getContourY(contour, i) - getContourY(contour, i - 1)));
			int ny = (int)(10 * (getContourX(contour, i - 1) - getContourX(contour, i)));
			
			g.drawLine(xi, yi, xj, yj);
			//g.drawLine(tx, ty, tx + nx, ty + ny);
		}
	}

	
	public void drawBB(Graphics2D g, int contour) {
		int x = (int)getBBMinX(contour);
		int y = (int)getBBMinY(contour);
		int w = (int)getBBMaxX(contour) - x;
		int h = (int)getBBMaxY(contour) - y;
		g.drawRect(x, y, w, h);
	}

	public void drawBB(Graphics2D g, int contour, Color c) {
		int x = (int)getBBMinX(contour);
		int y = (int)getBBMinY(contour);
		int w = (int)getBBMaxX(contour) - x;
		int h = (int)getBBMaxY(contour) - y;
		g.setColor(c);		
		g.drawRect(x, y, w, h);
	}

	
	
	public void drawContours(Graphics2D g) {
		for (int k = 0; k < getNumContours(); k++) {
			drawContour(g, k);
		}
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