package rfclassifier;

import java.util.Iterator;
import java.util.Map;
import java.util.Random;
import java.util.HashMap;
import java.util.Arrays;

public class WLPackedLearnedMasks extends WeakLearner {

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
//	public static void learnMasks(long[][] data) {
//
//		HashMap<Long, Integer>map = new HashMap<Long, Integer>();
//
//		for (int i = 0; i < data.length; i++) {
//			for (int j = 0; j < data[0].length; j++) {
//				long key = data[i][j];
//				if (map.containsKey(key)) {
//					int count = map.get(key);
//					map.put(key, count + 1);
//				} else {
//					map.put(key, 1);
//				}
//			}
//		}
//
//		System.out.println("Hash size: " + map.size());
//		NUM_MASKS = map.size();
//		masks = new long[NUM_MASKS];
//		int i = 0; 
//		for (Long key : map.keySet()) {
//			masks[i] = key;
//			i++;
//		}
//	}
	
	public static void learnMasks(long[][] data) {
		masks = new long[63];
		for (int i = 0; i < masks.length; i++) {
			masks[i] = 0x000000000000000fL << i;
		}
		NUM_MASKS = masks.length;
	}

	public WLPackedLearnedMasks() {
		
	}
	
	public WLPackedLearnedMasks (int MIN_DIMENSION, int MAX_DIMENSION) {
		this.MIN_DIMENSION = MIN_DIMENSION;
		this.MAX_DIMENSION = MAX_DIMENSION;
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
	
	long popcount_3(long x) {
		x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
		x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits 
		x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits 
		return (x * h01)>>56;  //returns left 8 bits of x + (x<<8) + (x<<16) + (x<<24) + ... 
	}
	
	public boolean compare(long[] datum) {

		long maskTmp = mask;
		long dimTmp = dim;
		long thresholdTmp = threshold;
		
		for (int i = 0; i < numTests; i++) {
			byte x = (byte)(maskTmp & 0xffL);
			byte y = (byte)(dimTmp & 0xffL);
			byte n = (byte)(thresholdTmp & 1L);
			byte t = (byte)((thresholdTmp >> 1) & 127);
			maskTmp = maskTmp >> 8;
			dimTmp = dimTmp >> 8;
			thresholdTmp = thresholdTmp >> 8;
			long val;
			//out("in " + i + ": " + x + " " + y + " " + t + " " + n);
			if (n == 0)
				val = datum[y] & (15 << x);
			else
				val = (~datum[y]) & (15 << x);
			if (popcount_3(val) < t)
				return false;
		}
		return true;		
	}

	public void randomize() {
		mask = 0;
		dim = 0;
		threshold = 0;
		for (int i = 0; i < numTests; i++) {
			byte x = (byte)(Math.random() * 64);
			byte y = (byte)(Math.random() * 64);
			byte t = (byte)(Math.random() * 5);
			byte n = (byte)(Math.random() * 2);
			//out("in " + i + ": " + x + " " + y + " " + t + " " + n);
			mask = (mask << 8) | x;
			dim = (dim << 8) | y;
			threshold = ((threshold << 8)) | ((t << 1) | n);
		}		
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
		long maskTmp = mask;
		long dimTmp = dim;
		long thresholdTmp = threshold;
		String s = "";
		for (int i = 0; i < numTests; i++) {
			byte x = (byte)(maskTmp & 0xffL);
			byte y = (byte)(dimTmp & 0xffL);
			byte n = (byte)(thresholdTmp & 1L);
			byte t = (byte)((thresholdTmp >> 1) & 127);
			maskTmp = maskTmp >> 8;
			dimTmp = dimTmp >> 8;
			thresholdTmp = thresholdTmp >> 8;
			s += "(" + x + " " + y + " " + t + " " + n + ")";
		}
		return s;
	}
}
