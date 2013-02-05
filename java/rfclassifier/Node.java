package rfclassifier;

import java.util.Arrays;

class Node {
	public WeakLearner wl;
	public Node left = null;
	public Node right = null;
	public int set[];
	public double entropy;
	public int flatIndex;
	public Node parent;
	public long[][] data;
	public int[] labels;
	public int depth;
	public double[] distribution;
	public int size;
 	public int NUM_LABELS;
	public Tree myTree;
	public int WEAKLEARNER_GENERATIONS;
	public boolean isLeft;
	public double bestGain;
	public int NUMBER;
	
	public Node(int setSize, Node p, long[][] trainData, int[] trainLabels, int NUM_LABELS, Tree myTree) {
		size = setSize;
		this.NUM_LABELS = NUM_LABELS;
		this.myTree = myTree;
		set = new int[setSize];
		data = trainData;
		labels = trainLabels;
		parent = p;
		if (p == null) 
			depth = 0;
		else 
			depth = p.depth + 1;
		distribution = new double[NUM_LABELS];
		wl = myTree.generateWeakLearner();
		WEAKLEARNER_GENERATIONS = myTree.WEAKLEARNER_GENERATIONS;
	}

	public void forget() {
		size = 0;
		Arrays.fill(distribution, 0);
	}

	public boolean learn() {

		left = new Node(size, this, data, labels, NUM_LABELS, myTree);
		right = new Node(size, this, data, labels, NUM_LABELS, myTree);
		left.isLeft = true;

		double bestgain = 0;
		boolean cont = false;
		int reps = 0;

		int weakLearnerGenerations = WEAKLEARNER_GENERATIONS;
		double prevBestGain = 0;

		do {
			for (int w = 0; w < weakLearnerGenerations; w++) {
				wl.randomize();
				separate();			
				double gain = informationGain();
				if (gain > bestgain) {
					bestgain = gain;
					wl.save();
				}
			}
			if (prevBestGain != bestgain && entropy > 0 && (bestgain / entropy) < .01 && reps < 10) {
				out("bestgain: " + bestgain + " size: " + size + " entropy: " + entropy);
				out(wl.toString());
				printDistribution();
				prevBestGain = bestgain;
				out("reps: " + reps++);
				weakLearnerGenerations *= 1.5;
				cont = true;
			} else {
				cont = false;
			}
		} while(cont);

		wl.useSaved();
		separate();

		if (bestgain > 0) {
			left.entropy = left.shannonEntropy();
			right.entropy = right.shannonEntropy();
			bestGain = bestgain;
			return true;
		} else {
			left = null;
			right = null;
			return false;
		}

	}

	public void separate() {
		left.forget();
		right.forget();
		for (int i = 0; i < size; i++) {
			int idx = set[i];
			if (wl.compare(data[idx])) {
				left.set[left.size] = idx;
				left.size++;
			} else {
				right.set[right.size] = idx;
				right.size++;
			}
		}
		left.computeDistribution();
		right.computeDistribution();
	}

	public void computeDistribution() {
		for (int i = 0; i < size; i++) {
			int bin = labels[set[i]];
			distribution[bin]++;
		}
		normalizeDistribution();
	}

	public double shannonEntropy() {
		double entropy = 0;
		normalizeDistribution();
		for (int k = 0; k < distribution.length; k++) {
			double p = distribution[k];
			double e = p * Math.log(p);
			if (!Double.isNaN(e)) {
				entropy += e;
			}
		}
		//out(Arrays.toString(distribution));
		//out("entropy:" + (-entropy));
		return -entropy;
	}

	public void normalizeDistribution() {
		double sum = 0;
		for (int k = 0; k < distribution.length; k++)
			sum += distribution[k];

		if (sum == 0) return;

		for (int k = 0; k < distribution.length; k++)
			distribution[k] = distribution[k] / sum;
	}

	public double informationGain() {
		double h1 = ((double)left.size / (double)size) * left.shannonEntropy();
		double h2 = ((double)right.size / (double)size) * right.shannonEntropy();
		return entropy - (h1 + h2);
	}

	void out(String m) {
		if (RFTrainer.useVerbose)
			System.out.println(m);
	}

	void printDistribution() {
		double[] d = distribution;
		switch (d.length) {
		case 2: 
			System.out.printf("(%d)\t%2.2f\t%2.2f\n", size, d[0] * 100, d[1] * 100);
			break;
		case 3:
			System.out.printf("(%d)\t%2.2f\t%2.2f\t%2.2f\n", size, d[0] * 100, d[1] * 100, d[2] * 100);			
			break;
		case 4:
			System.out.printf("(%d)\t%2.2f\t%2.2f\t%2.2f\t%2.2f\n", size, d[0] * 100, d[1] * 100, d[2] * 100, d[3] * 100);			
			break;
		case 5:
			System.out.printf("(%d)\t%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\n", size, d[0] * 100, d[1] * 100, d[2] * 100, d[3] * 100, d[4] * 100);			
			break;
		}
	}
}