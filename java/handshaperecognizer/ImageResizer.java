package handshaperecognizer;

import java.util.Arrays;

public class ImageResizer {

	int width = 1024;
	int height = 1024;

	static long[] tmp = new long[640 * 480];

	public static void scaleByHalf(int[] src, int sx, int sy, int sw, int sh, int ss, int[] dst, int ds) {

		int dw = sw / 2;
		int dh = sh / 2;
		for (int x = 0; x < dw; x++) {
			for (int y = 0; y < dh; y++) {
				int di = x + y * ds;
				int s0 = sx + (2 * x) + (sy + (2 * y)) * ss;
				int s1 = s0 + 1;
				int s2 = s0 + ss;
				int s3 = s0 + ss + 1;
				dst[di] = src[s0] + src[s1] + src[s2] + src[s3];

			}
		}
	}

	public void scaleBilinear(int[] src, int sx, int sy, int sw, int sh, int ss, int[] dst, int dw, int dh, int ds) {

		double scaleX = dw / sw;
		double scaleY = dh / sh;

		for (int x = 0; x < dw; x++)
			for (int y = 0; y < dh; y++)
				dst[x + y * ds] = 0;

		for (int x = sx; x < sx + sw; x++) {
			double dx = (x - sx) * scaleX;
			for (int y = sy; y < sy + sh; y++) {
				double frac = dx - (int)dx;

			}
		}
	}

	public static int integral(int[] src, int left, int top, int right, int bottom, int stride) {
		int tl = left + top * stride;
		int tr = right + top * stride;
		int bl = left + bottom * stride;
		int br = right + bottom * stride;

		int a = src[tl];
		int b = src[tr];
		int c = src[bl];
		int d = src[br];
		return (a + d) - (b + c);
	}
	
	public static long integral(long[] src, int l, int t, int r, int b, int stride) {
		long A, B, C;
		long D = src[r + b * stride];

		if (l <= 0 && t <= 0)
			return D;
		
		if (l <= 0) {
			B = src[r + (t - 1) * stride];
			return D - B;
		}

		if (t == 0) {
			C = src[(l - 1) + b * stride];
			return D - C;		
		}
		
		A = src[(l - 1) + (t - 1) * stride];
		B = src[r + (t - 1) * stride];
		C = src[(l - 1) + b * stride];
		
		return (A + D) - (B + C);
	}
	
	public static void downsizeUsingSummedArea(int[] src, int sx, int sy, int sw, int sh, int ss, 
			int[] dst, int dx, int dy, int dw, int dh, int ds) {

		// PER PIXEL IN DST
		// 	COMPUTE ALL 9 AREAS
		// 	SUM THEM
		// 	SCALE BY AREA

		double upScaleX = ((double)sw) / dw;
		double upScaleY = ((double)sh) / dh;

		int src_image_height = src.length / ss;
		integralImage(src, ss, src_image_height);

		for (int x = dx; x < dx + dw; x++) {
			double sl = (x - dx) * upScaleX + sx;
			double sr = (x + 1 - dx) * upScaleX + sx;
			if (sl >= 0 && sr <= ss - 2) {
				for (int y = dy; y < dy + dh; y++) {
					double st = (y - dy) * upScaleY + sy;
					double sb = (y + 1 - dy) * upScaleY + sy;
					int di = x + y * ds;
					if (st >= 0 && sb <= src_image_height - 2) {
						
						double A = sampleBilerp(src, sl, st, ss);
						double B = sampleBilerp(src, sr, st, ss);
						double C = sampleBilerp(src, sl, sb, ss);
						double D = sampleBilerp(src, sr, sb, ss);
						
						double sum = (A + D) - (B + C);
						double area = (sr - sl) * (sb - st);
						dst[di] = (int)(sum * area);
					} else {
						dst[di] = 0;						
					}
				}
			} else {
				for (int y = dy; y < dy + dh; y++) {
					int di = x + y * ds;
					dst[di] = 0;
				}
			}
		}
	}
	
	public static double lerp(int a, int b, double t) {
		return (1 - t) * a + t * b;
	}
	
	public static double sampleBilerp(int[] a, double px, double py, int stride) {
		
		int fpx = (int)Math.floor(px);
		int cpx = (int)Math.ceil(px);
		int fpy = (int)Math.floor(py);
		int cpy = (int)Math.ceil(py);
		
		double u = px - fpx;
		double v = py - fpy;
		
		double 	value = a[fpx + fpy * stride] * (1-u)*(1-v);
				value += a[cpx + fpy * stride] * (u)*(1-v);
				value += a[fpx + cpy * stride] * (1-u) * v;
				value += a[cpx + cpy * stride] * u * v;
		
		return value;
	}

	public static void integralImage(int[] im, int width, int height) {
		for (int x = 1; x < width; x++) {
			for (int y = 0; y < height; y++) {
				int i1 = (x - 1) + y * width;
				int i2 = x + y * width;
				im[i2] = im[i1] + im[i2];
			}
		}

		for (int x = 0; x < width; x++) {
			for (int y = 1; y < height; y++) {
				int i1 = x + (y - 1) * width;
				int i2 = x + y * width;
				im[i2] = im[i1] + im[i2];
			}
		}
	}

	public static void integralImage(long[] im, int width, int height) {
		for (int x = 1; x < width; x++) {
			for (int y = 0; y < height; y++) {
				int i1 = (x - 1) + y * width;
				int i2 = x + y * width;
				im[i2] = im[i1] + im[i2];
			}
		}

		for (int x = 0; x < width; x++) {
			for (int y = 1; y < height; y++) {
				int i1 = x + (y - 1) * width;
				int i2 = x + y * width;
				im[i2] = im[i1] + im[i2];
			}
		}
	}


	public static void copyRect(int[] src, int sx, int sy, int sw, int sh, int ss, int[] dst, int dx, int dy, int ds) {

		for (int x0 = sx, x1 = dx; x0 < sx + sw; x0++, x1++) {
			for (int y0 = sy, y1 = dy; y0 < sy + sh; y0++, y1++) {
				int si = x0 + y0 * ss;
				int di = x1 + y1 * ds;
				dst[di] = src[si];
			}
		}

	}


	public static void byte2ARGB(int src[]) {
		for (int i = 0; i < src.length; i++) {
			int v = src[i] & 255;
			src[i] = 0xff000000 | v << 16 | v << 8 | v;
		}
	}

	public static void stretchGrays(int src[]) {
		int max = max(src);
		int min = min(src);
		for (int i = 0; i < src.length; i++) {
			src[i] = (int)(255.0 * ((double)(src[i] - min)) / (max - min));
		}
		byte2ARGB(src);
	}

	public static int max(int src[]) {
		int max = Integer.MIN_VALUE;
		for (int i = 0; i < src.length; i++) {
			if (src[i] > max) max = src[i];
		}
		return max;
	}

	public static int min(int src[]) {
		int min = Integer.MAX_VALUE;
		for (int i = 0; i < src.length; i++) {
			if (src[i] < min) min = src[i];
		}
		return min;
	}

	public static void killAlpha(int[] a) {
		for (int i = 0; i < a.length; i++) {
			a[i] = a[i] | 0xff000000;
		}
	}
	
	static void out(String msg) {
		System.out.println(msg);
	}
}
