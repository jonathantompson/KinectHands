package rfclassifier;

import java.util.Iterator;
import java.util.Map;
import java.util.Random;
import java.util.HashMap;
import java.util.Arrays;

public class WLPixelGroup extends WeakLearner {

	public long dim;
	public long threshold;
	public long mask;

	int MIN_DIMENSION;
	int MAX_DIMENSION;
	int MAX_GROUPS;
	double THRESHOLD;

	long savedDim;
	long savedThreshold;
	long savedMask;

	static Random random = new Random();

	public static long[] masks;
	public static int NUM_MASKS;
	public int numTests = 1;

	public WLPixelGroup() {
		
	}
	
	public boolean compare(int[] datum) {
		return (datum[(int)dim] > threshold);
	}

	static final long m1  = 0x5555555555555555L; //binary: 0101...
	static final long m2  = 0x3333333333333333L; //binary: 00110011..
	static final long m4  = 0x0f0f0f0f0f0f0f0fL; //binary:  4 zeros,  4 ones ...
	static final long m8  = 0x00ff00ff00ff00ffL; //binary:  8 zeros,  8 ones ...
	static final long m16 = 0x0000ffff0000ffffL; //binary: 16 zeros, 16 ones ...
	static final long m32 = 0x00000000ffffffffL; //binary: 32 zeros, 32 ones
	static final long hff = 0xffffffffffffffffL; //binary: all ones
	static final long h01 = 0x0101010101010101L; //the sum of 256 to the power of 0,1,2,3...

	static final long fMask = 0xf000000000000000L;
	static final long tMask = 0x00000000000000ffL;
	
	static int RADIUS = 2;
	
	long popcount_3(long x) {
		x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
		x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits 
		x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits 
		return (x * h01)>>56;  //returns left 8 bits of x + (x<<8) + (x<<16) + (x<<24) + ... 
  	}
	
	public long countPixel(long[] datum, int x, int y) {
		return (datum[y] >> x) & 1;
	}
	
	public boolean compare(long[] datum) {

		int cnt = 0;
		int x = (int)(dim % 64);
		int y = (int)(dim / 64);
		for (int r = 0; r < RADIUS; r++) {
			for (int c = 0; c < RADIUS; c++) {
				cnt += countPixel(datum, x + c, y + r);
			}
		}
		if ((mask & 1) == 0) {
			return (cnt > threshold);
		}
		return (cnt < threshold);
	}

	public void randomize() {
		dim = (int)(Math.random() * (4095 - 64 * RADIUS));
		threshold = (int)(Math.random() * RADIUS * RADIUS);
		mask = (int)(Math.random() * 2);
	}

	public void save() {
		savedDim = dim;
		savedMask = mask;
		savedThreshold = threshold;
	}

	public void useSaved() {
		dim = savedDim;
		mask = savedMask;
		threshold = savedThreshold;
	}
 
	public String toString() {
		return "dim: " + dim + " threshold: " + threshold + " negate: " + mask;
	}
}
