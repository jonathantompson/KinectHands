package rfclassifier;

import java.io.BufferedInputStream;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public class RFClassifier implements RFClassifierInterface {


	public static int MAX_MATCHES = 3000;

	public int[] matches;
	public int numMatches;
	
	ArrayList<Tree> trees;

	public double[] distribution_tmp;
	public int NUM_CATEGORIES;
	
	public RFClassifier() {
		matches = new int[MAX_MATCHES];
	}
	
	public int match(long[] datum) {
		out("match not implemented.");
		return -1;
	};

	public int match(long[] datum, double[] distribution) {
		Arrays.fill(matches, -1);
		int cnt = 0;
		classify(datum, distribution);
		for (int i = 0; i < trees.size(); i++) {
			Node n = trees.get(i).eval2Leaf(datum);
			for (int k = 0; k < NUM_CATEGORIES; k++)
				distribution[k] += n.distribution[k];
			for (int k = 0; k < Math.min(n.size, 3); k++) {
				if (cnt < matches.length) {					
					matches[cnt++] = n.set[k];
				} else {
					//out("Max matches reached on tree: " + i);
					break;
				}
			}
		}
		normalizeDistribution(distribution);
		//printDistribution(distribution);
		return cnt;
	}
	
	void printDistribution(double[] d) {
		switch (d.length) {
		case 2: 
			System.out.printf("%2.2f\t%2.2f\n", d[0] * 100, d[1] * 100);
			break;
		case 3:
			System.out.printf("%2.2f\t%2.2f\t%2.2f\n", d[0] * 100, d[1] * 100, d[2] * 100);			
			break;
		case 4:
			System.out.printf("%2.2f\t%2.2f\t%2.2f\t%2.2f\n", d[0] * 100, d[1] * 100, d[2] * 100, d[3] * 100);			
			break;
		case 5:
			System.out.printf("%2.2f\t%2.2f\t%2.2f\t%2.2f\t%2.2f\n", d[0] * 100, d[1] * 100, d[2] * 100, d[3] * 100, d[4] * 100);			
			break;
		}
	}	
	
	public static void normalizeDistribution(double[] distribution) {
		double sum = 0;
		for (int k = 0; k < distribution.length; k++)
			sum += distribution[k];

		if (sum == 0) return;

		for (int k = 0; k < distribution.length; k++)
			distribution[k] = distribution[k] / sum;
	}
	
	
	public double[] classify(long[] datum, double[] distribution) {
		Arrays.fill(distribution, 0);
		for (int i = 0; i < trees.size(); i++) {
			Node n = trees.get(i).eval2Leaf(datum);
			for (int k = 0; k < distribution.length; k++) {
				distribution[k] += n.distribution[k];
			}
		}
		normalizeDistribution(distribution);
		return distribution;
	}

	public int categorize(double[] distribution) {
		int bestLabel = -1;
		double bestProbability = 0;
		for (int k = 0; k < NUM_CATEGORIES; k++) {
			if (distribution[k] > bestProbability) {
				bestProbability = distribution[k];
				bestLabel = k;
			}
		}
		return bestLabel;
	}

	public double evaluate(long[][] data, int[] labels, int size) {
		if (distribution_tmp == null) {
			distribution_tmp = new double[NUM_CATEGORIES];
		}
		double numCorrectlyLabeled = 0;
		for (int i = 0; i < size; i++) {
			double[] distribution = classify(data[i], distribution_tmp);
			int predictedLabel = categorize(distribution);
			if (predictedLabel == labels[i]) {
				numCorrectlyLabeled++;
			}
		}
		return numCorrectlyLabeled / size;		
	}

	public Tree readTreeFromFile(DataInputStream in) {
		try {
			Tree tree = new Tree();
			
			// READ NUM CLASSES
			tree.NUM_CLASSES = in.readInt();
			NUM_CATEGORIES = tree.NUM_CLASSES;
			
			// READ NUM NODES
			int NUM_NODES = in.readInt();
			tree.nodes = new Node[NUM_NODES];

			// READ EACH NODE IN ORDER OF
			tree.maxLeafSize = 0;
			for (int i = 0; i < NUM_NODES; i++) {

				tree.nodes[i] = new Node();
				Node n = tree.nodes[i];
				n.left = in.readInt();					// INT: LEFT INDEX
				n.right = in.readInt();					// INT: RIGHT INDEX
				n.wlDim = in.readLong();				// LONG: DUMMY DIM
				n.wlThreshold = in.readLong();			// LONG: DUMMY THRESHOLD
				n.wlMask = in.readLong();				// LONG: DUMMY MASK
				n.size = in.readInt();					// INT: SET SIZE
				if (n.size > 0) {
					n.set = new int[n.size];
					for (int k = 0; k < n.size; k++) {
						n.set[k] = in.readInt();
					}
					n.distribution = new double[tree.NUM_CLASSES];
					for (int k = 0; k < n.distribution.length; k++) {
						n.distribution[k] = in.readDouble();
					}
					if (n.size > tree.maxLeafSize) tree.maxLeafSize = n.size;
				}
				
			}
			
			return tree;
		} catch (IOException fioe) {
			out("error reading tree from file.");
			return null;
		}

	}

	public int readFromFile(String filename) {
		int success = 0;
		try {
			DataInputStream in = new DataInputStream(new BufferedInputStream(
					new FileInputStream(filename)));

			int NUM_TREES = in.readInt();
			trees = new ArrayList<Tree>();
			MAX_MATCHES = 0;
			for (int i = 0; i < NUM_TREES; i++) {
				Tree nextTree = readTreeFromFile(in);
				if (nextTree == null) {
					success = -1;
					break;
				} else {
					trees.add(nextTree);
					MAX_MATCHES += nextTree.maxLeafSize;
					//out("MAX LEAF SIZE FOR TREE #" + i + " is " + nextTree.maxLeafSize);
				}
			}
			if (MAX_MATCHES > 3000) MAX_MATCHES = 10;
 			matches = new int[MAX_MATCHES];
			//out("MAX_MATCHES = " + MAX_MATCHES);

		} catch (FileNotFoundException e) {
			out("Couldn't read from file named: " + filename);
			success = -1;
		} catch (IOException ioe) {
			out("Error flushing...");
			success = -1;
		}

		if (success == 0) {
			out("Read forest file named " + filename);
		}

		return success;
		
	}

	public class Tree {

		int maxLeafSize = 0;
		int NUM_CLASSES = 0;
		
		Node[] nodes;
		//WLPackedLearnedMasks wl = new WLPackedLearnedMasks();
		WLPixelGroup wl = new WLPixelGroup();
		
		public Node eval2Leaf(long[] datum) {
			int idx = 0;
			do {
				Node n = nodes[idx];

				if (n.left == -1) {
					return n;
				}

				wl.dim = n.wlDim;
				wl.threshold = n.wlThreshold;
				wl.mask = n.wlMask;
				if (wl.compare(datum)) {
					idx = n.left;
				} else {
					idx = n.right;
				}
			} while (idx < nodes.length);

			return null;
		}
	}

	public class Node {

		int[] set;
		int size;
		int left;
		int right;
		long wlDim;
		long wlThreshold;
		long wlMask;
		double[] distribution;
	

	}

	static void out(String msg) {
		System.out.println(msg);
	}
}
