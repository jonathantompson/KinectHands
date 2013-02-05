package rfclassifier;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;

class Tree {

	double GO_LEFT_THRESHOLD = 0;

	public int MAX_NODES;
	public int numNodes;
	public int MAX_DEPTH;
	public int NUM_CATEGORIES;
	public int WEAKLEARNER_GENERATIONS;
	public Node root;
	public Stack<Node> trainStack;

	public Tree(int MAX_DEPTH, int TRAINING_DATA_LENGTH, int NUM_CATEGORIES, int WEAKLEARNER_GENERATIONS) {
		this.MAX_DEPTH = MAX_DEPTH;
		this.NUM_CATEGORIES = NUM_CATEGORIES;
		this.WEAKLEARNER_GENERATIONS = WEAKLEARNER_GENERATIONS;
		MAX_NODES = (int)Math.pow(2,MAX_DEPTH) + 1;
		numNodes = 0;
		trainStack = new Stack<Node>();
	}

	public WeakLearner generateWeakLearner() {
		//		return new WLHaar(64, 64, -255, 255);
		//		return new WLIntegralImage(64, 64, 0, 255);
		//		return new WLDimensionalFixedThreshold(0, 63, 128);
		//		return new WLHaarPacked1D(0, 63);
//		return new WLPackedLearnedMasks(0, 63);
//		return new WLTinyImage();
		//		return new WLDimensionalThreshold(0, 1, 0, 480);
//				return new WLDimensionalThreshold(0, 4095, 0, 2);
		return new WLPixelGroup();
	}

	public double[] classify(long[] data) {

		Node curNode = root;

		do {
			if (curNode.left == null && curNode.right == null) {
				return curNode.distribution;
			} else {
				if (curNode.wl.compare(data)) {
					curNode = curNode.left;
				} else {
					curNode = curNode.right;
				}
			}
		} while (curNode != null);
		out("classify found no leaves!, should fail...");
		return null;
	}

	public Node eval2Leaf(long[] data) {

		Node curNode = root;		
		do {
			if (curNode.left == null && curNode.right == null) {
				return curNode;
			} else {
				if (curNode.wl.compare(data)) {
					curNode = curNode.left;
				} else {
					curNode = curNode.right;
				}
			}
		} while (curNode != null);
		out("classify found no leaves!, should fail...");
		return null;
	}


	public void train(long[][] data, int[] labels, int DATA_SIZE) {

		this.root = new Node(DATA_SIZE, null, data, labels, NUM_CATEGORIES, this);
		for (int i = 0; i < DATA_SIZE; i++) {
			root.set[i] = i;
		}
 		root.size = DATA_SIZE;
		numNodes = 1;
		root.computeDistribution();
		root.entropy = root.shannonEntropy();
		int nodesTrained = 0;
		boolean stop = false;
		trainStack.push(root);
		root.NUMBER = numNodes;
		while (!trainStack.empty() && !stop) {

			Node n = trainStack.pop();
			boolean attemptSplit = true;

			if (attemptSplit && n.entropy == 0) {
				//out("skipping: node entropy is too low (" + n.entropy + ")");
				//out(Arrays.toString(n.distribution));
				attemptSplit = false;
				if (n.size > 1000) out("BIG LEAF.");
				out("LEAF.\t\tdepth: " + n.depth + " size: " + n.size);
				n.printDistribution();
			}

			if (attemptSplit) {
				boolean split = n.learn();
				if (split) {					
					if (n.depth < MAX_DEPTH - 1) {
						//						if (n.size < 30 && (n.right.size < 10 || n.left.size < 10)) {
						//							n.right = null;
						//							n.left = null;
						//							out("Bad split... Making parent a leaf.");
						//						} else 
							out("SPLIT. (" + n.depth + ")\t\tleft size: " + n.left.size + " right size: " + n.right.size);
							out("Feature: " + n.wl.toString());
							out("Gain: " + n.bestGain);
							n.printDistribution();
							n.left.printDistribution();
							n.right.printDistribution();
							n.right.NUMBER = numNodes++;
							n.left.NUMBER = numNodes++;
							trainStack.push(n.right);
							trainStack.push(n.left);
					}
				} else {
					out("LEAF.\t\tdepth: " + n.depth + " size: " + n.size);
					n.printDistribution();
				}

				nodesTrained++;
			}
		}

		out("Finished training tree.  Num nodes is " + nodesTrained);
	}

	public void freeInternalMemory() {
		Stack<Node> dfs = new Stack<Node>();
		dfs.push(root);
		while (!dfs.empty()) {
			Node n = dfs.pop();
			if (n.left != null) {
				dfs.push(n.left);
			}
			if (n.right != null) {
				dfs.push(n.right);
			}
			if (n.right != null & n.left != null) {
				n.set = null;
				n.distribution = null;
				n.labels = null;
				n.data = null;
			}
		}
	}
	public ArrayList<Node> flatten() {
		ArrayList<Node> nodes = new ArrayList<Node>();
		Stack<Node> treeStack = new Stack<Node>();
		treeStack.push(root);
		while (!treeStack.empty()) {
			Node n = treeStack.pop();
			n.flatIndex = nodes.size();
			nodes.add(n);
			if (n.left != null) {
				treeStack.push(n.left);
			}
			if (n.right != null) {
				treeStack.push(n.right);
			}
		}
		return nodes;
	}

	void out(String m) {
		//		if (RFClassifier.useVerbose)
		System.out.println(m);
	}

}