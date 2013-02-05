package imageprocessor;

import java.awt.*;
import java.awt.image.*;
import java.util.Hashtable;
import java.util.ArrayList;
import java.util.Arrays;

public class ImageProcessor {

	
	public int width = 640;
	public int height = 480;
	public int N = width * height;
	public int[] buf;
	public int[] buf2;
	public int[] X;
	public int[] Y;
	public double[] tmpD;
	
	public float TAU = (float)(Math.PI * 2.0);

	/* Constructor */
	public ImageProcessor(int width, int height) {
		
		setDimensions(width, height);
		
	}
		
	public void setDimensions(int width, int height) {
		out("Resizing ImageProcessor dimensions from (" + this.width + ", " + this.height + ") to (" + width + ", " + height + ")..."); 
		long tic = System.nanoTime();
		this.width = width;
		this.height = height;
		this.N = width * height;
		this.buf = new int[N];
		this.buf2 = new int[N];
		this.X = new int[N];
		this.Y = new int[N];
		this.tmpD = new double[N];
		
		for (int i = 0; i < N; i++) {
			X[i] = i % width;
			Y[i] = i / width;
		}
		long toc = System.nanoTime();
		out("...took " + (double)(toc - tic) / 1000000 + "ms.");
	}
	
	
	public void cvtPaddingValid(int src[], int margin) {
		int r = margin;
		int minX = r - 1;
		int maxX = width - minX;
		int minY = minX;
		int maxY = height - minY;
		for (int i = 0; i < src.length; i++) {
			int x = i % width;
			int y = i / width;
			if (x > minX && x < maxX && y > minY && y < maxY) {
				src[i] = 1;
			} else {
				src[i] = 0;
			}
		}
	}
		
	public void cvtIndex(int[] src) {
		for (int i = 0; i < src.length; i++) {
			src[i] = i;
		}
	}
	
	public void cvtDataToRGB(int[] src) {
		
		int min = Integer.MAX_VALUE;
		int max = Integer.MIN_VALUE;
		
		for (int i = 0; i < src.length; i++) {
			if (src[i] < min) {
				min = src[i];
			}
			if (src[i] > max) {
				max = src[i];
			}
		}
		
		cvtDataToRGB(src, min, max);
		
	}
	
	public void cvtDataToRGB(int[] src, int lo, int hi) {
		
		for (int i = 0; i < src.length; i++) {
			int s = Math.min(hi, Math.max(lo, src[i]));
			int val = 255 & (int)(255.0f * (float)(s - lo) / (float)(hi - lo));
			src[i] = 0xFF000000 | val << 16 | val << 8 | val;
		}
		
	}

	public void cvtDataToRGB(float[] src, int[] dst, float lo, float hi) {
		
		for (int i = 0; i < src.length; i++) {
			float s = Math.min(hi, Math.max(lo, src[i]));
			int val = 255 & (int)(255.0f * (float)(s - lo) / (float)(hi - lo));
			dst[i] = 0xFF000000 | val << 16 | val << 8 | val;
		}
		
	}

	public void cvtDataToRGB(float[] src, int[] dst) {
		
		float min = Float.MAX_VALUE;
		float max = Float.MIN_VALUE;
		
		for (int i = 0; i < src.length; i++) {
			if (src[i] < min) {
				min = src[i];
			}
			if (src[i] > max) {
				max = src[i];
			}
		}
		cvtDataToRGB(src, dst, min, max);
		
	}
	
	public void cvtRGBToGray(int[] src, int[] dst) {
		
		for (int i = 0; i < src.length; i++) {
			int r = src[i] >> 16 & 255;
			int g = src[i] >> 8 & 255;
			int b = src[i] & 255;
			dst[i] = (r + b + g) / 3;
		}
	}
	
	public void cvtBinaryThreshold(int[] src, int min, int loFill, int hiFill) {
		for (int i = 0; i < src.length; i++) {
			if (src[i] < min) 
				src[i] = loFill;
			else {
				src[i] = hiFill;
			}
			
		}		
	}
	
	public void cvtBinaryThreshold(int[] src, int min) {
		cvtBinaryThreshold(src, min, 0, 1);
	}	
	
	public void cvtThreshold(int[] src, int min, int fill) {
		for (int i = 0; i < src.length; i++) {
			if (src[i] < min) src[i] = fill;
		}		
	}
	
	public void cvtThreshold(int[] src, int min) { cvtThreshold(src, min, 0); }
	
	public void cvtScale(int[] src, float a, float b) {
		for (int i = 0; i < src.length; i++) {
			src[i] = (int)((float)src[i] * a + b);
		}
	}
	
	
	public void cvtMask(int[] src, int[] mask) {
		
		for (int i = 0; i < src.length; i++) {
			if (mask[i] == 0) {
				src[i] = 0;
			}
		}
	}
	
	
	int[] pixSpread = new int[N];
	public void cvtSpread(int[] src, int[] dst, int size, int mask) {
		
		int[] tmp = getTemporaryBuffer();
		
		int r = size / 2;
		int minX = r - 1;
		int maxX = width - minX;
		int minY = minX;
		int maxY = height - minY;
		
		int avg;
		int x, y;
		for (int i = 0; i < src.length; i++) {
			x = i % width;
			y = i / width;
			avg = 0;
			
			if (src[i] != mask && x > minX && x < maxX && y > minY && y < maxY) {
				for (int kx = -r; kx < r; kx++) {
					for (int ky = -r; ky < r; ky++) {
						int idx = (y + ky) * width + (x + kx);
						avg += src[idx];
					}
				}
			}
			tmp[i] = avg / (size);
		}
		
		if (tmp != dst) {
			System.arraycopy(tmp, 0, dst, 0, tmp.length);
		}
	}
	
	public void cvtSpread(int[] src, int[] dst, int size) {
		cvtSpread(src, dst, size, -1);
	}
	

	
	/* Flood Fill With Controlled Bleeding */
	int[] pixCC = new int[N];
	int[] pixFill = new int[N];
	int[] pixCCFlag = new int[N];
	public void cvtMaskFromFloodFill(int[] src, int[] dst, int seed, int threshold, int fillValue) {
	
		if (seed >= 0 && seed < src.length) {
			int[] fill = pixFill;
			int[] cc = pixCC;
			int[] flag = pixCCFlag;
			if (src == dst) {
				if (pixCC.length != src.length) {
					out("Resizing cvtConnectComponents tmp array from " + pixCC.length + " to " + src.length);
					pixCC = new int[src.length];
					pixCCFlag = new int[src.length];
					pixFill = new int[src.length];
					cc = pixCC;			
					fill = pixFill;
					flag = pixCCFlag;
				}
				System.arraycopy(src, 0, fill, 0, src.length);
			} else {
				fill = dst;
			}
			Arrays.fill(cc, 0);			
			Arrays.fill(flag, 0);
			
			int f = 0;		// next open slot in stack
			int k = 0;
			int width = this.width;
			
			cc[seed] = fillValue;
			flag[f++] = seed;
			int n = src.length;
			int start = src[seed];
			while (f > 0) {
				int c = flag[--f];
				int p = c - 1;
				if (c % width != 0 && Math.abs(src[p] - start) < threshold && cc[p] == 0) {
					cc[p] = 1;
					fill[p] = fillValue;
					flag[f++] = p;
				}
				p = c + 1;
				if ((c + 1) % width != 0 && Math.abs(src[p] - start) < threshold && cc[p] == 0) {
					cc[p] = 1;
					fill[p] = fillValue;
					flag[f++] = p;
				}
				p = c - width;
				if (c > width && Math.abs(src[p] - start) < threshold && cc[p] == 0) {
					cc[p] = 1;
					fill[p] = fillValue;
					flag[f++] = p;
				}
				p = c + width;
				if (c < n - width && Math.abs(src[p] - start) < threshold && cc[p] == 0) {
					cc[p] = 1;
					fill[p] = fillValue;
					flag[f++] = p;
				}
			}

			if (src == dst) {
				System.arraycopy(fill, 0, dst, 0, fill.length);
			}
		}
	}	

	
	
	
	/* Connected Components w/o Mask */
	public int cvtConnectedComponents(int[] src, int[] dst, int threshold) {
		
		int[] cc = pixCC;
		int[] flag = pixCCFlag;
		if (src == dst) {
			if (pixCC.length != src.length) {
				out("Resizing cvtConnectComponents tmp array from " + pixCC.length + " to " + src.length);
				pixCC = new int[N];
				pixCCFlag = new int[N];
			}
			cc = pixCC;
			flag = pixCCFlag;
		} else {
			cc = dst;
		}
		
		Arrays.fill(cc, -1);
		Arrays.fill(flag, 0);
		
		int f = 0;		// next open slot in stack
		int k = -1;
		int n = cc.length;
		int width = this.width;
		if (n > 0) {
			for (int i = 0; i < n; i++) {				
				if (cc[i] == -1) {		// start new component at i
					f = 0;				
					cc[i] = ++k;
					flag[f++] = i;
					
					while (f > 0) {
						int c = flag[--f];
						int p = c - 1;
						if (c % width != 0 && Math.abs(src[p] - src[c]) < threshold && cc[p] == -1) {
							cc[p] = cc[c];
							flag[f++] = p;
						}
						p = c + 1;
						if ((c + 1) % width != 0 && Math.abs(src[p] - src[c]) < threshold && cc[p] == -1) {
							cc[p] = cc[c];
							flag[f++] = p;
						}
						p = c - width;
						if (c > width && Math.abs(src[p] - src[c]) < threshold && cc[p] == -1) {
							cc[p] = cc[c];
							flag[f++] = p;
						}
						p = c + width;
						if (c < n - width && Math.abs(src[p] - src[c]) < threshold && cc[p] == -1) {
							cc[p] = cc[c];
							flag[f++] = p;
						}
					}
				}
			}
		}
		
		if (cc != dst) {
			System.arraycopy(cc, 0, dst, 0, cc.length);
		}
		
		out("cvtConnectedComponents found " + k + " components");
		return k + 1;
		
	}	
	
	
	/* Connected Components w/ Mask */
	// this still has components starting at 1
	// should change to match above
	public int cvtConnectedComponents(int[] src, int[] dst, int threshold, int[] mask) {
		
		int[] cc = pixCC;
		int[] flag = pixCCFlag;
		if (src == dst) {
			if (pixCC.length != src.length) {
				out("Resizing cvtConnectComponents tmp array from " + pixCC.length + " to " + src.length);
				pixCC = new int[N];
				pixCCFlag = new int[N];
			}
			cc = pixCC;
			flag = pixCCFlag;
		} else {
			cc = dst;
		}
		
		Arrays.fill(cc, 0);
		Arrays.fill(flag, 0);
		
		int f = 0;		// next open slot in stack
		int k = 0;
		int n = cc.length;
		int width = this.width;
		if (n > 0) {
			
			for (int i = 0; i < n; i++) {				
				if (mask[i] > 0 && cc[i] == 0) {		// start new component at i
					f = 0;				
					cc[i] = ++k;
					flag[f++] = i;
					
					while (f > 0) {
						int c = flag[--f];
						if (mask[c] > 0) {
							int p = c - 1;
							if (c % width != 0 && Math.abs(src[p] - src[c]) < threshold && cc[p] == 0) {
								cc[p] = cc[c];
								flag[f++] = p;
							}
							p = c + 1;
							if ((c + 1) % width != 0 && Math.abs(src[p] - src[c]) < threshold && cc[p] == 0) {
								cc[p] = cc[c];
								flag[f++] = p;
							}
							p = c - width;
   							if (c > width && Math.abs(src[p] - src[c]) < threshold && cc[p] == 0) {
								cc[p] = cc[c];
								flag[f++] = p;
							}
							p = c + width;
							if (c < n - width && Math.abs(src[p] - src[c]) < threshold && cc[p] == 0) {
								cc[p] = cc[c];
								flag[f++] = p;
							}
						}
					}
				}
			}
		}
		
		if (cc != dst) {
			System.arraycopy(cc, 0, dst, 0, cc.length);
		}
		
		//out("cvtConnectedComponents found " + k + " components");
		return k + 1;
		
	}
	
	public void cvtConnectedComponentStats(int[] src, int[] x, int[] y, int[] mass) {
		
		Arrays.fill(x, 0);
		Arrays.fill(y, 0);
		Arrays.fill(mass, 0);
		int maxc = x.length;
		
		for (int i = 0; i < src.length; i++) {
			int c = src[i];
			if (c < maxc) {
				x[c] += i % width;
				y[c] += i / width;
				mass[c] += 1;	
			}
		}
		
		for (int i = 0; i < maxc; i++) {
			if (mass[i] > 0) {
				x[i] = x[i] / mass[i];
				y[i] = y[i] / mass[i];
			}
		}
		
	}
	
	public void cvtThresholdComponents(int[] src, int[] dst, int[] Masses, int lo_mass, int hi_mass) {
		int[] OMasses = getTemporaryBuffer();
		System.arraycopy(Masses, 0, OMasses, 0, N);
		
		// first make sure 0th component is largest
		// if not, swap it so it is
		int maximum_mass_index = 0;
		int maximum_mass_found = 0;
		int num_components = 0;
		for (int i = 0; i < Masses.length; i++) {
			if (Masses[i] > maximum_mass_found) {
				maximum_mass_found = Masses[i];
				maximum_mass_index = i;
			}
			num_components++;
		}
		
		if (maximum_mass_index != 0) {
			Masses[maximum_mass_index] = Masses[0];
			Masses[0] = maximum_mass_found;
		}
		
		// then run through all components and add small ones to 0th component
		int idx = 0;
		for (int i = 0; i < N; i++) {
			idx = src[i];
			if (OMasses[idx] < lo_mass || OMasses[idx] > hi_mass) {
				Masses[idx] -= 1;
				Masses[0] += 1;
				dst[i] = 0;
			} else {
				dst[i] = src[i];
			}
		}
	}
	
	public void cvtSortLabels(int[] src, int[] masses, int[] labels) {
		
		// compute masses
		int num_components = 0;
		for (int i = 0; i < src.length; i++) {
			int idx = src[i];
			if (idx > num_components) {
				num_components = idx;
			}
			masses[idx]++;
		}
		num_components++;
		
		/*
		String m = "";
		for (int i = 0; i < num_components + 1; i++) {
			m += masses[i] + " ";
		}
		out("Here it is " + m);
		
		*/
		
		// setup indices matrix
		for (int i = 0; i < num_components; i++) {
			labels[i] = i;
		}
		
		sortArrayWithAuxArray(masses, labels);
		
		  
		
		
		
		/*	
		String s = "";
		for (int i = 0; i < num_components; i++) {
			s += " " + masses[i];
		}
		out(s);
		*/
		
		
	}
	
	public void sortArrayWithAuxArray(int[] keys, int[] aux) {
		sortArrayWithAuxArray(keys, aux, keys.length);
	}
	
	public void sortArrayWithAuxArray(double[] keys, int[] aux) {
		sortArrayWithAuxArray(keys, aux, keys.length);
	}


	public void sortArrayWithAuxArray(int[] keys, int[] aux, int n) {
	
		int tmp = 0;
		for (int i = 0; i < n - 1; i++) {
			for (int j = 0; j < n - i - 1; j++) {
				if (keys[j] > keys[j+1]) {
					tmp = keys[j];
					keys[j] = keys[j+1];
					keys[j+1] = tmp;
					
					tmp = aux[j];
					aux[j] = aux[j+1];
					aux[j+1] = tmp;
				}
			}
			if (keys[i] == 0) break;
		}
	}

	public void sortArrayWithAuxArray(double[] keys, int[] aux, int n) {
	
		double tmpK = 0;
		int tmpA;
		for (int i = 0; i < n - 1; i++) {
			for (int j = 0; j < n - i - 1; j++) {
				if (keys[j] > keys[j+1]) {
					tmpK = keys[j];
					keys[j] = keys[j+1];
					keys[j+1] = tmpK;
					
					tmpA = aux[j];
					aux[j] = aux[j+1];
					aux[j+1] = tmpA;
				}
			}
			if (keys[i] == 0) break;
		}
	}

	public void sortArrayWithAuxArray(int[] keys, double[] aux, int n) {
	
		int tmpK = 0;
		double tmpA;
		for (int i = 0; i < n - 1; i++) {
			for (int j = 0; j < n - i - 1; j++) {
				if (keys[j] > keys[j+1]) {
					tmpK = keys[j];
					keys[j] = keys[j+1];
					keys[j+1] = tmpK;
					
					tmpA = aux[j];
					aux[j] = aux[j+1];
					aux[j+1] = tmpA;
				}
			}
			if (keys[i] == 0) break;
		}
	}
	
	
	public void sortArrayWithAuxArray(double[] keys, double[] aux, int n) {
	
		double tmpK = 0;
		double tmpA;
		for (int i = 0; i < n - 1; i++) {
			for (int j = 0; j < n - i - 1; j++) {
				if (keys[j] > keys[j+1]) {
					tmpK = keys[j];
					keys[j] = keys[j+1];
					keys[j+1] = tmpK;
					
					tmpA = aux[j];
					aux[j] = aux[j+1];
					aux[j+1] = tmpA;
				}
			}
			if (keys[i] == 0) break;
		}
	}

	

	public void remapArray(double[] data, int[] idx, int n) {
	
		for (int i = 0; i < n; i++) {
			tmpD[i] = data[idx[i]];
		}
		System.arraycopy(tmpD, 0, data, 0, n);
	}
	
	
	public void cvtSubtract(int[] A, int B[], int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = A[i] - B[i];
		}
	}

	public void cvtAdd(int[] A, int B[], int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = A[i] + B[i];
		}
	}
	
	public void cvtMultiply(int[] A, int B[], int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = A[i] * B[i];
		}
	}

	public void cvtDivide(int[] A, int B[], int[] dst) {
		for (int i = 0; i < N; i++) {
			if (B[i] != 0) {
				dst[i] = A[i] / B[i];
			} else {
				dst[i] = A[i];
			}
		}
	}

	public void cvtDivideUnsafe(int[] A, int B[], int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = A[i] / B[i];
		}
	}
	
	public void cvtDivideIfAboveThreshold(int[] A, int B[], int[] dst, int threshold) {
		for (int i = 0; i < N; i++) {
			if (B[i] > threshold) {
				dst[i] = A[i] / B[i];
			} else {
				dst[i] = A[i];
			}
		}
	}

	public void cvtSq(int[] src, int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = src[i] * src[i];
		}
	}

	public void cvtSqrt(int[] src, int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = (int)Math.sqrt(src[i]);
		}
	}
	
	
	public void cvtCombine(int[] A, float a, int B[], float b, int[] dst) {
		for (int i = 0; i < N; i++) {
			dst[i] = (int)(A[i] * a + B[i] * b);
		}
	}
		
	public void cvtMax(int[] A, int[] B, int dst[]) {
		for (int i = 0; i < N; i++) {
			if (A[i] > B[i]) {
				dst[i] = A[i];
			} else {
				dst[i] = B[i];
			}
		}
	}
	
	public void cvtMin(int[] A, int[] B, int dst[]) {
		for (int i = 0; i < N; i++) {
			if (A[i] > B[i]) {
				dst[i] = B[i];
			} else {
				dst[i] = A[i];
			}
		}
	}
	
	/* Linear Scale By XY coordinate */
	// for each index i=(x,y) set dst[i] = f(x,y) * src[i]
	// where f(x,y) = a * x + b * y + c
	public void cvtLinearScaleXY(int[] src, int[] dst, float a, float b, float c) {
		int[] out = getTemporaryBuffer();
		for (int i = 0; i < N; i++) {
			int x = X[i];
			int y = Y[i];
			float f = a * X[i] + b * Y[i] + c;
			out[i] = (int)(src[i] * f);
		}
		if (out != dst) {
			System.arraycopy(out, 0, dst, 0, N);
		}		
	}
	
	
	public void cvtRotate90CCW(int[] src, int[] dst) {
		
		int[] mat = getTemporaryBuffer();
		
		int j = 0;
		int c = 0;
		int r = 0;
		for (int i = 0; i < N; i++) {
			j = r * width + c;
			mat[i] = src[j];
			r = r + 1;
			if (r >= height) {
				c++;
				r = 0;
			}
		}
		
		System.arraycopy(mat, 0, dst, 0, N);
	}
	
	
	
	public void cvtFlipHorizontal(int[] src, int[] dst) {
		for (int x = 0; x < width / 2; x++) {
			for (int y = 0; y < height; y++) {
				int left = y * width + x;
				int right = y * width + (width - x - 1);
				int tmp = src[left];
				dst[left] = src[right];
				dst[right] = tmp;
			}
		}
	}

	public void cvtFlipVertical(int[] src, int[] dst) {
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height / 2; y++) {
				int top = y * width + x;
				int bottom = (height - y - 1) * width + x;
				int tmp = src[top];
				dst[top] = src[bottom];
				dst[bottom] = tmp;
			}
		}
	}
	
	
	public void cvtFlip180(int[] src, int[] dst) {

		if (src == dst) {
			for (int i = 0, j = N-1; i < N / 2; i++, j--) {
				int tmp = src[i];
				src[i] = src[j];
				src[j] = tmp;
			}
		} else {
			for (int i = 0, j = N-1; i < N; i++, j--) {
				dst[j] = src[i];
			}
		}
	}
	
	public void cvtAverage(int[] src, int[] dst, int size, int mask) {
		int[] tmp = getTemporaryBuffer();
		
		int r = size / 2;
		int minX = r - 1;
		int maxX = width - minX;
		int minY = minX;
		int maxY = height - minY;
		
		int avg;
		int x, y;
		int s2 = size * size;
		
		for (int i = 0; i < src.length; i++) {
			x = i % width;
			y = i / width;
			avg = 0;
			
			if (src[i] != mask && x > minX && x < maxX && y > minY && y < maxY) {
				for (int kx = -r; kx < r; kx++) {
					for (int ky = -r; ky < r; ky++) {
						int idx = (y + ky) * width + (x + kx);
						avg += src[idx];
					}
				}
				tmp[i] = avg / s2;
			}
		}
			
		 if (tmp != dst) {
			 System.arraycopy(tmp, 0, dst, 0, tmp.length);
		 }	
	}
		
	public void cvtNormalize(int[] src, int[] dst, int size, float scale, int mask) {
		
		int[] tmp = getTemporaryBuffer();
		
		cvtAverage(src, tmp, size, mask);
		for (int i = 0; i < N; i++) {
			dst[i] = (int)(scale * (float)src[i] / (1 + tmp[i]));
			dst[i] = dst[i] * dst[i];
		}
		//cvtSubtract(src, tmp, dst);
		//cvtMultiply(dst, dst, dst);
		//cvtSubtract(src, tmp, tmp);
		//cvtDivide(src, tmp, dst);
		//cvtScale(src, 30, 0);
	}
	
	public void cvtNormalize(int[] src, int[] dst, int size, float scale) {
		cvtNormalize(src, dst, size, scale, -1);
	}
	
	public void cvtAdaptiveThreshold(int[] src, int[] reference, int[] dst, float factor, int mask) {
		for (int i = 0; i < N; i++) {
			if (src[i] != mask && src[i] > factor * reference[i]) {
				dst[i] = src[i];
			} else {
				dst[i] = 0;
			}
		}
	}
	
	public void cvtAdaptiveThreshold(int[] src, int[] reference, int[] dst, float factor) {
		cvtAdaptiveThreshold(src, reference, dst, factor, -1);
	}
	
	public void cvtAdaptiveThreshold(int[] src, int[] dst, int size, float threshold, int mask) {
		
		int[] tmp = getTemporaryBuffer();
		
		int r = size / 2;
		int minX = r - 1;
		int maxX = width - minX;
		int minY = minX;
		int maxY = height - minY;
		
		int avg;
		int x, y;
		int s2 = size * size;
		for (int i = 0; i < src.length; i++) {
			x = i % width;
			y = i / width;
			avg = 0;
			
			if (src[i] != mask && x > minX && x < maxX && y > minY && y < maxY) {
				for (int kx = -r; kx < r; kx++) {
					for (int ky = -r; ky < r; ky++) {
						int idx = (y + ky) * width + (x + kx);
						avg += src[idx];
					}
				}
			}
			avg = avg / s2;
			if (src[i] > threshold * avg) {
				tmp[i] = (int)(src[i]);
			} else {
				tmp[i] = 0;
			}
		}
		
		if (tmp != dst) {
			System.arraycopy(tmp, 0, dst, 0, tmp.length);
		}
	}
	
	public void cvtAdaptiveThreshold(int[] src, int[] dst, int size, float threshold) {
		cvtAdaptiveThreshold(src, dst, size, threshold, -1);
	}
	
	public void cvtErode(int[] src, int[] dst, int size, int mask) {
		
		int[] tmp = getTemporaryBuffer();
		int r = size / 2;
		int minX = r - 1;
		int maxX = width - minX;
		int minY = minX;
		int maxY = height - minY;
		
		int x, y;
		int avg;
		int s2 = size * size;
		for (int i = 0; i < src.length; i++) {
			int max = src[i];
			x = i % width;
			y = i / width;
			avg = 0;
			if (src[i] != mask && x > minX && x < maxX && y > minY && y < maxY) {
				for (int kx = -r; kx < r; kx++) {
					for (int ky = -r; ky < r; ky++) {
						int idx = (y + ky) * width + (x + kx);
						avg += src[idx];
						if (src[idx] > max) max = src[idx];
					}
				}
			}
			if (src[i] >= max) {
				tmp[i] = avg / s2;
			} else {
				tmp[i] = src[i];
			}
			
		}
		
		if (tmp != dst) {
			System.arraycopy(tmp, 0, dst, 0, tmp.length);
		}
	}
	
	public void cvtErode(int[] src, int[] dst, int size) {
		cvtErode(src, dst, size, -1);
	}
	
	float[] kernel1DGaussian;	
	public float[] ker1DGaussian(int radius) {
		int size = 2 * radius + 1;
		float[] mat = kernel1DGaussian;
		double var = 3 * radius * radius;
		if (mat != null && mat.length == size) {
			return mat;
		} else {
			float sum = 0;
			mat = new float[size];
			for (int i = 0; i < size; i++) {
				mat[i] = (float)Math.exp((double)((radius - i) * (i - radius)) / var);
				sum += mat[i];
			}
			for (int i = 0; i < size; i++) {
				mat[i] = mat[i] / sum;
			}			
			kernel1DGaussian = mat;
		}
		out(Arrays.toString(mat));
		return mat;
	}
		
	public void cvtDilate(int[] src, int[] dst, int size, int mask) {
		
		int[] tmp = getTemporaryBuffer();
		int r = size / 2;
		int minX = r - 1;
		int maxX = width - minX;
		int minY = minX;
		int maxY = height - minY;
		
		int x, y;
		int avg;
		int s2 = size * size;
		
		for (int i = 0; i < src.length; i++) {
			int min = src[i];
			int max = src[i];
			x = i % width;
			y = i / width;
			avg = 0;
			
			if (src[i] != mask && x > minX && x < maxX && y > minY && y < maxY) {
				for (int kx = -r; kx < r; kx++) {
					for (int ky = -r; ky < r; ky++) {
						int idx = (y + ky) * width + (x + kx);
						//avg += src[idx];
						if (src[idx] < min) min = src[idx];
						if (src[idx] > max) max = src[idx];
					}
				}
			}
			if (src[i] == min) {
				tmp[i] = max;
			} else {
				tmp[i] = src[i];
			}
			
		}
		
		if (tmp != dst) {
			System.arraycopy(tmp, 0, dst, 0, tmp.length);
		}
	}
	
	public void cvtDilate(int[] src, int[] dst, int size) {
		cvtDilate(src, dst, size, -1);
	}
	
	
	/* float kernel */
	public void cvtConvolve1DHorizontal(int[] src, int[] dst, float[] ker) {		
		int[] mat = getTemporaryBuffer();
		int size = ker.length;
		int radius = size / 2;
		for (int i = 0; i < N; i++) {
			int x = X[i];
			if (x > radius && x < width - radius) {
				for (int j = 0, k = -radius; j < size; j++, k++) {
					mat[i] += src[i + k] * ker[j];
				}
			} else {
				mat[i] = src[i];
			}
		}		
		if (mat != dst) System.arraycopy(mat, 0, dst, 0, N);
	}
	
	/* int kernel */
	public void cvtConvolve1DHorizontal(int[] src, int[] dst, int[] ker) {		
		int[] mat = getTemporaryBuffer();
		int size = ker.length;
		int radius = size / 2;
		for (int i = 0; i < N; i++) {
			int x = X[i];
			if (x > radius && x < width - radius) {
				for (int j = 0, k = -radius; j < size; j++, k++) {
					mat[i] += src[i + k] * ker[j];
				}
			} else {
				mat[i] = src[i];
			}
		}		
		if (mat != dst) System.arraycopy(mat, 0, dst, 0, N);
	}
	
	
	/* float kernel */
	public void cvtConvolve1DVertical(int[] src, int[] dst, float[] ker) {		
		int[] mat = getTemporaryBuffer();
		int size = ker.length;
		int radius = size / 2;		
		for (int i = 0; i < N; i++) {
			int y = Y[i];
			if (y > radius && y < height - radius) {
				for (int j = 0, k = -radius; j < size; j++, k++) {
					mat[i] += src[i + k * width] * ker[j];
				}
			} else {
				mat[i] = src[i];
			}
		}
		if (mat != dst) {
			System.arraycopy(mat, 0, dst, 0, N);
		}
	}
	
	/* int kernel */
	public void cvtConvolve1DVertical(int[] src, int[] dst, int[] ker) {		
		int[] mat = getTemporaryBuffer();
		int size = ker.length;
		int radius = size / 2;		
		for (int i = 0; i < N; i++) {
			int y = Y[i];
			if (y > radius && y < height - radius) {
				for (int j = 0, k = -radius; j < size; j++, k++) {
					mat[i] += src[i + k * width] * ker[j];
				}
			} else {
				mat[i] = src[i];
			}
		}
		if (mat != dst) {
			System.arraycopy(mat, 0, dst, 0, N);
		}
	}
	
	public void cvtConvolveSeperable(int[] src, int[] dst, float[] kerH, float[] kerV) {
		cvtConvolve1DHorizontal(src, dst, kerH);
		cvtConvolve1DVertical(dst, dst, kerV);
	}
	
	public void cvtGaussian2D(int[] src, int[] dst, int size) {
		int radius = size / 2;
		float[] ker = ker1DGaussian(radius);
		cvtConvolveSeperable(src, dst, ker, ker);
	}
	
	int[] zeromean;
	public void cvtLocalContrastNormalization(int[] src, int[] dst, int threshold) {
		
		if (zeromean == null) {
			zeromean = new int[N];
		}
		
		cvtGaussian2D(src, zeromean, 3);
		System.arraycopy(zeromean, 0, dst, 0, N);
		cvtSq(zeromean, zeromean);
		cvtGaussian2D(zeromean, zeromean, 3);
		cvtSqrt(zeromean, zeromean);
		cvtDivideIfAboveThreshold(dst, zeromean, dst, threshold);
		
	}

	
	public void cvtCopy(int[] src, int[] dst) {
		System.arraycopy(src, 0, dst, 0, N);
	}
	
	public void cvtRetrieveNonZeroes(int[] src, int[] dst) {
	
		int blank = 0;
		
		for (int i = 0; i < src.length; i++) {
		
			if	(src[i] != 0)	{
					dst[blank++] = i;
			}
		}
	}
	
	
	/********************************************
	 Setup Buffer
	********************************************/
	public int[] getTemporaryBuffer() {
		if (buf.length != N) {
			out("Resizing buffer tmp array from " + buf.length + " to " + N);
			buf = new int[N];
		} else {
			Arrays.fill(buf, 0);
		}
		return buf;
	}
						  
						  
	public int[] getTemporaryBuffer2() {
		if (buf2.length != N) {
			out("Resizing buffer tmp array from " + buf2.length + " to " + N);
			buf2 = new int[N];
		} else {
			Arrays.fill(buf2, 0);
		}
			return buf2;
	}
						  
	
	/********************************************
	 TIMESAVING FUNCTIONS
	 *********************************************/
	public void out(String msg) {
		System.out.println(msg);
	}
	
	public int dist2(int x1, int x2, int y1, int y2) {
		return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
	}
}
