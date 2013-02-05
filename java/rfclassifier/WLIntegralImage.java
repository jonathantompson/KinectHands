package rfclassifier;

public class WLIntegralImage extends WeakLearner {

	int tl, tr, bl, br;	
	int saved_tl, saved_tr, saved_bl, saved_br;
	int l, r, t, b;
	int saved_l, saved_r, saved_t, saved_b;
	int area;
	int saved_area;
	
	double threshold;
	
	int MAX_WIDTH;
	int MAX_HEIGHT;
	double MIN_THRESHOLD;
	double MAX_THRESHOLD;
	
	double saved_threshold;
	
	public WLIntegralImage (int MAX_WIDTH, int MAX_HEIGHT, double MIN_THRESHOLD, double MAX_THRESHOLD) {
		this.MAX_WIDTH = MAX_WIDTH;
		this.MAX_HEIGHT = MAX_HEIGHT;
		this.MIN_THRESHOLD = MIN_THRESHOLD;
		this.MAX_THRESHOLD = MAX_THRESHOLD;
		randomize();
		save();
	}
	
	public boolean compare(int[] datum) {
		int a = datum[tl];
		int b = datum[tr];
		int c = datum[bl];
		int d = datum[br];

		int val = ((a + d) - (b + c)) / area;
			
		return ((a + d) - (b + c)) > threshold;
	}

	public boolean compare(long[] datum) {
		long a = datum[tl];
		long b = datum[tr];
		long c = datum[bl];
		long d = datum[br];

		long val = ((a + d) - (b + c)) / area;
			
		return ((a + d) - (b + c)) > threshold;
	}

	
	public int sumFast(int[] datum) {
		int a = datum[tl];
		int b = datum[tr];
		int c = datum[bl];
		int d = datum[br];
		return (a + d) - (b + c);
	}

	public int sumSlow(int[] datum) {
		int sum = 0;
		for (int x = l; x <= r; x++) {
			for (int y = t; y <= b; y++) {
				//sum += datum[x + y * MAX_WIDTH];
				sum += 1;
			}
		}
		return sum;
	}
	
	public void randomize() {
		l = randi(1, MAX_WIDTH - 1);
		r = randi(l, MAX_WIDTH - 1);
		t = randi(1, MAX_HEIGHT - 1);
		b = randi(t, MAX_HEIGHT - 1);

		area = (r - l + 1) * (b - t + 1);
		if (area == 0) {
			out("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!area is zero: " + toString());
		}
		tl = (l - 1) + (t - 1) * MAX_WIDTH;
		tr = r + (t - 1) * MAX_WIDTH;
		bl = (l - 1) + b * MAX_WIDTH;
		br = r + b * MAX_WIDTH;
		
		threshold = randf(MIN_THRESHOLD, MAX_THRESHOLD);
	}
	
	public void save() {
		saved_tl = tl;
		saved_tr = tr;
		saved_bl = bl;
		saved_br = br;
		saved_l = l;
		saved_r = r;
		saved_t = t;
		saved_b = b;
		saved_area = area;
		saved_threshold = threshold;
	}
	
	public void useSaved() {
		tl = saved_tl;
		tr = saved_tr;
		bl = saved_bl;
		br = saved_br;
		l = saved_l;
		r = saved_r;
		t = saved_t;
		b = saved_b;
		area = saved_area;
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
		return "tl: " + tl + " tr: " + tr + " bl: " + bl + " br: " + br + " threshold: " + threshold;
	}
	
	public int randi(int low, int hi) {
		return (int)(Math.random() * (hi - low + 1) + low);
	}
	
	public double randf(double low, double hi) {
		return Math.random() * (hi - low) + low;
	}
}
