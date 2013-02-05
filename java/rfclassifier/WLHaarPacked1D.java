package rfclassifier;

import java.util.Random;

public class WLHaarPacked1D extends WeakLearner {

	int dim;
	double threshold;
	long mask;
	
	int MIN_DIMENSION;
	int MAX_DIMENSION;
	double THRESHOLD;
	
	int savedDim;
	double savedThreshold;
	long savedMask;
	
	static Random random = new Random();
	
	public WLHaarPacked1D (int MIN_DIMENSION, int MAX_DIMENSION) {
		this.MIN_DIMENSION = MIN_DIMENSION;
		this.MAX_DIMENSION = MAX_DIMENSION;
	}
	
	public boolean compare(int[] datum) {
		return (datum[dim] > threshold);
	}
	
	static final long m1  = 0x5555555555555555L; //binary: 0101...
	static final long m2  = 0x3333333333333333L; //binary: 00110011..
	static final long m4  = 0x0f0f0f0f0f0f0f0fL; //binary:  4 zeros,  4 ones ...
	static final long m8  = 0x00ff00ff00ff00ffL; //binary:  8 zeros,  8 ones ...
	static final long m16 = 0x0000ffff0000ffffL; //binary: 16 zeros, 16 ones ...
	static final long m32 = 0x00000000ffffffffL; //binary: 32 zeros, 32 ones
	static final long hff = 0xffffffffffffffffL; //binary: all ones
	static final long h01 = 0x0101010101010101L; //the sum of 256 to the power of 0,1,2,3...
	 
	public boolean compare(long[] datum) {
		long x = datum[dim] & mask;
	    x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
	    x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits 
	    x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits 
	    return (x * h01)>>56 < threshold;  //returns left 8 bits of x + (x<<8) + (x<<16) + (x<<24) + ... 
	}
	
	public void randomize() {
		dim = (int)(Math.random() * (MAX_DIMENSION - MIN_DIMENSION + 1) + MIN_DIMENSION);
		mask = random.nextLong();
		threshold = (int)(Math.random() * (MAX_DIMENSION - MIN_DIMENSION + 1) + MIN_DIMENSION);
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
		return "dim: " + dim + " mask: " + mask + " threshold: " + threshold;
	}
}
