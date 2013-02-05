package rfclassifier;

public class WLHaar extends WeakLearner {

	int NUM_BOXES = 2;

	double[][] boxes;
	double[][] saved_boxes;
	
	double threshold;
	double saved_threshold;
	
	int MAX_WIDTH;
	int MAX_HEIGHT;
	double MIN_THRESHOLD;
	double MAX_THRESHOLD;
		
	final int TL = 0;
	final int TR = 1;
	final int BL = 2;
	final int BR = 3;
	final int POLARITY = 4;
	final int AREA = 5;
	final int BOX_MEMBERS = AREA + 1;
	
	public WLHaar (int MAX_WIDTH, int MAX_HEIGHT, double MIN_THRESHOLD, double MAX_THRESHOLD) {
		this.MAX_WIDTH = MAX_WIDTH;
		this.MAX_HEIGHT = MAX_HEIGHT;
		this.MIN_THRESHOLD = MIN_THRESHOLD;
		this.MAX_THRESHOLD = MAX_THRESHOLD;
		boxes = new double[NUM_BOXES][BOX_MEMBERS];
		saved_boxes = new double[NUM_BOXES][BOX_MEMBERS];
		randomize();
		save();
	}
	
	public boolean compare(int[] datum) {		
		double sum = 0;
		for (int i = 0; i < NUM_BOXES; i++) {
			sum += sum(datum, boxes[i]) * boxes[i][POLARITY] / boxes[i][AREA];
		}
		return sum > threshold;
	}

	public boolean compare(long[] datum) {		
		double sum = 0;
		for (int i = 0; i < NUM_BOXES; i++) {
			sum += sum(datum, boxes[i]) * boxes[i][POLARITY] / boxes[i][AREA];
		}
		return sum > threshold;
	}

	
	public double sum(int[] datum, double[] box) {
		int a = datum[(int)box[TL]];
		int b = datum[(int)box[TR]];
		int c = datum[(int)box[BL]];
		int d = datum[(int)box[BR]];
		return ((a + d) - (b + c));
	}

	public double sum(long[] datum, double[] box) {
		long a = datum[(int)box[TL]];
		long b = datum[(int)box[TR]];
		long c = datum[(int)box[BL]];
		long d = datum[(int)box[BR]];
		return ((a + d) - (b + c));
	}

	public void randomize() {

		int i = 0;
		while (i < NUM_BOXES) {
			int l = randi(1, MAX_WIDTH - 1);
			int r = randi(l, MAX_WIDTH - 1);
			int t = randi(1, MAX_HEIGHT - 1);
			int b = randi(t, MAX_HEIGHT - 1);

			double area = (r - l + 1) * (b - t + 1);
			if (area != 0) {
				boxes[i][TL] = (l - 1) + (t - 1) * MAX_WIDTH;
				boxes[i][TR] = r + (t - 1) * MAX_WIDTH;
				boxes[i][BL] = (l - 1) + b * MAX_WIDTH;
				boxes[i][BR] = r + b * MAX_WIDTH;			
				boxes[i][AREA] = area;
				boxes[i][POLARITY] = Math.random() > 0.5 ? 1 : -1;
				i++;
			}
		}
		threshold = randf(MIN_THRESHOLD, MAX_THRESHOLD);
	}
	
	public void save() {
		int size = boxes[0].length;
		for (int i = 0; i < NUM_BOXES; i++) {
			System.arraycopy(boxes[i], 0, saved_boxes[i], 0, size);
		}
		saved_threshold = threshold;
	}
	
	public void useSaved() {
		int size = saved_boxes[0].length;
		for (int i = 0; i < NUM_BOXES; i++) {
			System.arraycopy(saved_boxes[i], 0, boxes[i], 0, size);
		}
		threshold = saved_threshold;
	}
	
	public static void cumSum2D(int[] im, int width, int height) {
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
	
	public String toString() {
		String s = "";
		for (int i = 0; i < NUM_BOXES; i++) {
			s += "Box #" + i + " tl: " + boxes[i][TL] + " tr: " + boxes[i][TR];
			s += " bl: " + boxes[i][BL] + " br: " + boxes[i][BR];
			s += " area: " + boxes[i][AREA] + " sign: " + boxes[i][POLARITY];
			if (i < NUM_BOXES - 1) {
				s += "\n";
			}
		}
		return s;
	}
	
	public int randi(int low, int hi) {
		return (int)(Math.random() * (hi - low + 1) + low);
	}
	
	public double randf(double low, double hi) {
		return Math.random() * (hi - low) + low;
	}
}
