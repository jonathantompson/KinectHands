package rfclassifier;

import handshaperecognizer.HandShapeDictionary;

import java.awt.Graphics2D;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collection;

import canvasframe.*;
import java.awt.Color;
import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

public class RFTrainer implements RFClassifierInterface {

	public long[][] trainData;
	public int[] trainLabels;

	public static int MAX_TREE_DEPTH = 3;
	public static int NUM_TREES = 1;
	public static int NUM_CATEGORIES = 3;
	public static int WEAKLEARNER_GENERATIONS = 1000;
	public static int MAX_MATCHES = 5000;

	public ArrayList<Tree> trees;
	double[] distribution_tmp;

	public static boolean useVerbose = false;
	
	public int TRAIN_DATA_SIZE;
	public int PACKED_IMAGE_SIZE = 64;
	

	public RFTrainer(long[][] data, int[] labels, int TRAIN_DATA_SIZE) {
		
//		int r = 9;
//		trainData = new long[r * TRAIN_DATA_SIZE][PACKED_IMAGE_SIZE];
//		trainLabels = new int[r * TRAIN_DATA_SIZE];
//		for (int i = 0; i < TRAIN_DATA_SIZE; i++) {
//			System.arraycopy(data[i], 0, trainData[r * i + 0], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 1], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 2], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 3], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 4], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 5], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 6], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 7], 0, PACKED_IMAGE_SIZE);
//			System.arraycopy(data[i], 0, trainData[r * i + 8], 0, PACKED_IMAGE_SIZE);
//			shiftPackedRight(trainData[r * i + 1]);
//			shiftPackedLeft(trainData[r * i + 2]);
//			shiftPackedUp(trainData[r * i + 3]);
//			shiftPackedDown(trainData[r * i + 4]);
//			shiftPackedRight(trainData[r * i + 5]); shiftPackedDown(trainData[r * i + 5]);
//			shiftPackedRight(trainData[r * i + 6]); shiftPackedUp(trainData[r * i + 6]);
//			shiftPackedLeft(trainData[r * i + 7]); shiftPackedDown(trainData[r * i + 7]);
//			shiftPackedLeft(trainData[r * i + 8]); shiftPackedUp(trainData[r * i + 8]);
//			trainLabels[r * i + 0] = labels[i];
//			trainLabels[r * i + 1] = labels[i];
//			trainLabels[r * i + 2] = labels[i];
//			trainLabels[r * i + 3] = labels[i];
//			trainLabels[r * i + 4] = labels[i];
//			trainLabels[r * i + 5] = labels[i];
//			trainLabels[r * i + 6] = labels[i];
//			trainLabels[r * i + 7] = labels[i];
//			trainLabels[r * i + 8] = labels[i];
//		}
//		this.TRAIN_DATA_SIZE = r * TRAIN_DATA_SIZE;
		trainData = data;
		trainLabels = labels;
		this.TRAIN_DATA_SIZE = TRAIN_DATA_SIZE;
	}

	public void shiftPackedRight(long[] tiny) {
		for (int i = 0; i < PACKED_IMAGE_SIZE; i++)
			tiny[i] = (tiny[i] >> 1) | (tiny[i] & 0x8000000000000000L);
	}

	public void shiftPackedLeft(long[] tiny) {
		for (int i = 0; i < PACKED_IMAGE_SIZE; i++)
			tiny[i] = (tiny[i] << 1) | (tiny[i] & 0x0000000000000008L);
	}

	public void shiftPackedUp(long[] tiny) {
		for (int i = 1; i < PACKED_IMAGE_SIZE; i++)
			tiny[i - 1] = tiny[i];
		tiny[63] = tiny[62];
	}

	public void shiftPackedDown(long[] tiny) {
		for (int i = 0; i < PACKED_IMAGE_SIZE - 1; i++)
			tiny[i+1] = tiny[i];
		tiny[0] = tiny[1];
	}
	public int[] matches = new int[MAX_MATCHES];

	public void train() {
		out("Allocating space for " + NUM_TREES + " trees...");
		trees = new ArrayList<Tree>();
		for (int i = 0; i < NUM_TREES; i++) {
			Tree t = new Tree(MAX_TREE_DEPTH, TRAIN_DATA_SIZE, NUM_CATEGORIES, WEAKLEARNER_GENERATIONS);
			trees.add(t);
			out("Training tree #: " + i + "...");
			trees.get(i).train(trainData, trainLabels, TRAIN_DATA_SIZE);
			double trainAccuracy = evaluate(trainData, trainLabels, TRAIN_DATA_SIZE);
			out("Training Accuracy on " + (i + 1) + " trees: " + trainAccuracy);
			trees.get(i).freeInternalMemory();
		}
	}

	public int match(long[] datum, double[] distribution) {
		out("match with two arguments not implemented.");
		return -1;
	}

	public int match(long[] datum) {
		Arrays.fill(matches, -1);
		int cnt = 0;
		for (int i = 0; i < trees.size(); i++) {
			Node n = trees.get(i).eval2Leaf(datum);
			for (int k = 0; k < n.size; k++) {
				if (cnt < matches.length) {
					matches[cnt++] = n.set[k];
				} else {
					out("Max matches reached on tree: " + i);
					break;
				}
			}
		}
		return cnt;
	}


	public double[] classify(long[] datum) {
		if (distribution_tmp == null) {
			distribution_tmp = new double[NUM_CATEGORIES];
		}
		Arrays.fill(distribution_tmp, 0);
		for (int i = 0; i < trees.size(); i++) {
			double[] classification = trees.get(i).classify(datum);
			for (int k = 0; k < NUM_CATEGORIES; k++) {
				distribution_tmp[k] += classification[k];
			}
		}
		normalizeDistribution(distribution_tmp);
		return distribution_tmp;
	}

	public double[] classifyWithMultiply(long[] datum) {
		if (distribution_tmp == null) {
			distribution_tmp = new double[NUM_CATEGORIES];
		}
		Arrays.fill(distribution_tmp, 1);
		for (int i = 0; i < trees.size(); i++) {
			double[] classification = trees.get(i).classify(datum);
			for (int k = 0; k < NUM_CATEGORIES; k++) {
				distribution_tmp[k] *= classification[k];
			}
		}
		//normalizeDistribution(distribution_tmp);
		return distribution_tmp;
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
		double numCorrectlyLabeled = 0;
		for (int i = 0; i < size; i++) {
			double[] distribution = classify(data[i]);
			int predictedLabel = categorize(distribution);
			if (predictedLabel == labels[i]) {
				numCorrectlyLabeled++;
			}
		}
		return numCorrectlyLabeled / size;		
	}

	public static void main(String[] args) {


		long[][] trainData = new long[100][2];
		int[] trainLabels = new int[100];

		long[][] testData = new long[1000][2];
		int[] testLabels = new int[1000];

		generateFakeData(trainData, trainLabels);
		generateFakeData(testData, testLabels);

		RFTrainer.MAX_TREE_DEPTH = 3;
		RFTrainer.NUM_TREES = 1;
		RFTrainer.NUM_CATEGORIES = 3;
		RFTrainer.WEAKLEARNER_GENERATIONS = 1000;
		RFTrainer rf = new RFTrainer(trainData, trainLabels, trainData.length);

		CanvasFrame view = new CanvasFrame(null, 0, 480, 480);
		renderData(view, trainData, trainLabels);

		rf.train();
		double trainAccuracy = rf.evaluate(trainData, trainLabels, trainData.length);
		out("Finished training");
		out("Training Accuracy: " + trainAccuracy);
		out("Classifiying testset...");
		Graphics2D g = view.getGraphics2D();
		for (int i = 0; i < testLabels.length; i++) {
			double[] distribution = rf.classify(testData[i]);
			int predictedCategory = rf.categorize(distribution);
			if (predictedCategory == testLabels[i]) {
				g.setColor(Color.black);
			} else {
				g.setColor(Color.red);
			}
			g.drawRect((int)testData[i][0], (int)testData[i][1], 1, 1);
		}
		out("Finished classifying testset.");
		double testAccuracy = rf.evaluate(testData, testLabels, testData.length);
		out("Test Accuracy: " + testAccuracy);
		view.update();

	}

	public static void permute(int[] a) {
		for (int i = 0; i < a.length - 1; i++) {
			int k = (int)(Math.random() * (a.length - i) + i);
			int tmp = a[i];
			a[i] = a[k];
			a[k] = tmp;
		}
	}

	public static void renderData(CanvasFrame view, long[][] data, int[] labels) {
		Graphics2D g = view.getGraphics2D();
		g.setColor(Color.white);
		g.fillRect(0,  0, 480, 480);

		for (int i = 0; i < labels.length; i++) {
			if (labels[i] == 0) {
				g.setColor(Color.green);
			} else if (labels[i] == 1){
				g.setColor(Color.blue);
			} else {
				g.setColor(Color.orange);
			}
			g.drawRect((int)data[i][0], (int)data[i][1], 1, 1);
		}

		view.update();

	}

	public static void generateFakeData(long[][] data, int[] labels) {
		for (int k = 0; k < labels.length; k++) {
			int x, y, c;
			c = (int)(Math.random() * 3);
			if (c == 0) {
				x = (int)(Math.random() * 50 + 50);
				y = (int)(Math.random() * 50 + 50);
			} else if (c == 1){
				x = (int)(Math.random() * 100 + 260);
				y = (int)(Math.random() * 100 + 100);
			} else {
				x = (int)(Math.random() * 100 + 0);
				y = (int)(Math.random() * 100 + 300);
			}
			data[k][0] = x;
			data[k][1] = y;
			labels[k] = c;
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

	public boolean saveTreeToFile(Tree tree, DataOutputStream out) {
		ArrayList<Node> flattenedTree = tree.flatten();
		try {

			// WRITE NUM CATEGORIES
			out.writeInt(NUM_CATEGORIES);
			// WRITE NUM NODES
			out.writeInt(flattenedTree.size());

			// WRITE EACH NODE IN ORDER OF ITS _FLATINDEX_
			for (int i = 0; i < flattenedTree.size(); i++) {

				Node n = flattenedTree.get(i);

				boolean isLeaf = n.left == null;
				
				if (isLeaf) {
					out.writeInt(-1);					// INT: LEFT INDEX
					out.writeInt(-1);					// INT: RIGHT INDEX
					out.writeLong(0);					// LONG: DUMMY DIM
					out.writeLong(0);					// LONG: DUMMY THRESHOLD
					out.writeLong(0);					// LONG: DUMMY MASK
					out.writeInt(n.size);				// INT: SET SIZE

					for (int k = 0; k < n.size; k++) {
						out.writeInt(n.set[k]);
					}
					
					for (int k = 0; k < n.distribution.length; k++) {
						out.writeDouble(n.distribution[k]);
					}
					
				} else {
					WLPixelGroup wl = (WLPixelGroup)n.wl;
					out.writeInt(n.left.flatIndex);		// INT: LEFT INDEX
					out.writeInt(n.right.flatIndex);	// INT: RIGHT INDEX
					out.writeLong(wl.dim);				// LONG: WL DIM
					out.writeLong(wl.threshold);		// LONG: WL THRESHOLD
					out.writeLong(wl.mask);				// LONG: WL MASK;
					out.writeInt(0);					// INT: SET SIZE					
				}
			}
		} catch (IOException fioe) {
			out("error writing.");
			return false;
		}

		return true;

	}

	public int readFromFile(String filename) {
		out("readFromFile is not implemented.");
		return -1;	
	}
	
	public int saveToFile(String filename) {
		int success = 0;
		try {
			DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
					new FileOutputStream(filename)));

			out.writeInt(trees.size());
			
			for (int i = 0; i < trees.size(); i++) {
				saveTreeToFile(trees.get(i), out);
			}

			out.flush();

		} catch (FileNotFoundException e) {
			out("Couldn't write to file named: " + filename);
			success = -1;
		} catch (IOException ioe) {
			out("Error flushing...");
			success = -1;
		}

		if (success == 0) {
			out("Wrote DB to file named " + filename);
		}

		return success;


	}

	static void out(String msg) {
		//		if (RFClassifier.useVerbose)
		System.out.println(msg);
	}

}
