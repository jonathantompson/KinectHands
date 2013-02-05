package rfclassifier;

public class WLDimensionalThreshold extends WeakLearner {

	long dim;
	double threshold;
	
	long MIN_DIMENSION;
	long MAX_DIMENSION;
	double MIN_THRESHOLD;
	double MAX_THRESHOLD;
	
	long savedDim;
	double savedThreshold;
	
	public WLDimensionalThreshold (long MIN_DIMENSION, long MAX_DIMENSION, double MIN_THRESHOLD, double MAX_THRESHOLD) {
		this.MIN_DIMENSION = MIN_DIMENSION;
		this.MAX_DIMENSION = MAX_DIMENSION;
		this.MIN_THRESHOLD = MIN_THRESHOLD;
		this.MAX_THRESHOLD = MAX_THRESHOLD;
	}
	
	public boolean compare(int[] datum) {
		return (datum[(int)dim] > threshold);
	}
	
	public boolean compare(long[] datum) {
		int x = (int)(dim % 64);
		int y = (int)(dim / 64);
		int v = (int)((datum[y] >> x) & 1);
		return (v == threshold);
	}

	public void randomize() {
		dim = (int)(Math.random() * (MAX_DIMENSION - MIN_DIMENSION + 1) + MIN_DIMENSION);
		threshold = (int)(Math.random() * (MAX_THRESHOLD - MIN_THRESHOLD)) + MIN_THRESHOLD;
		//out(this.toString());
	}
	
	public void save() {
		savedDim = dim;
		savedThreshold = threshold;
	}
	
	public void useSaved() {
		dim = savedDim;
		threshold = savedThreshold;
	}
	
	
	public String toString() {
		return "dim: " + dim + " threshold: " + threshold;
	}
}
