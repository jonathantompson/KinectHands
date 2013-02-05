package rfclassifier;

public class WLDimensionalFixedThreshold extends WeakLearner {

	int dim;
	double threshold;
	
	int MIN_DIMENSION;
	int MAX_DIMENSION;
	double THRESHOLD;
	
	int savedDim;
	double savedThreshold;
	
	public WLDimensionalFixedThreshold (int MIN_DIMENSION, int MAX_DIMENSION, int THRESHOLD) {
		this.MIN_DIMENSION = MIN_DIMENSION;
		this.MAX_DIMENSION = MAX_DIMENSION;
		threshold = THRESHOLD;
	}
	
	public boolean compare(int[] datum) {
		return (datum[dim] > threshold);
	}
	
	public boolean compare(long[] datum) {
		return (datum[dim] > threshold);
	}
	
	public void randomize() {
		dim = (int)(Math.random() * (MAX_DIMENSION - MIN_DIMENSION + 1) + MIN_DIMENSION);
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
