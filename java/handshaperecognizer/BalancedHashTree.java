package handshaperecognizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Stack;

public class BalancedHashTree {

	long[][] data;
	int MAX_LEVELS = 10;
	int TARGET_SIZE = 128;
	int[] map;
	final int NUM_DIMENSIONS = 4096;
	Stack<Node> todo = new Stack<Node>();
	ArrayList<Node> nodes = new ArrayList<Node>();
	int NUM_IMAGES;
	Node root;
	
	public BalancedHashTree(long[][] data, int NUM_IMAGES) {
		this.data = data;		
		this.NUM_IMAGES = NUM_IMAGES;
		if (NUM_IMAGES > 0)
			balance();
	}

	public void balance() {

		int[] allIndices = new int[NUM_IMAGES];
		boolean[] rootKey = new boolean[NUM_DIMENSIONS];
		for (int i = 0; i < allIndices.length; i++) {
			allIndices[i] = i;
			rootKey[i] = false;
		}
		root = new Node(null, allIndices, rootKey);
		todo.push(root);
		int numDataPointsAccountedFor = 0;
		int leafs = 0;
		while(!todo.empty()) {
			Node node = todo.pop();
			boolean didSplit = false;
			if (node.size > TARGET_SIZE) {
				//out("Splitting node #" + nodes.size() + " with size: " + node.size);
				didSplit = node.trySplit();
			} else {
				leafs++;
				node.isLeaf = true;
				out("Leaf has size: " + node.size);
				numDataPointsAccountedFor += node.size;
			}
			if (didSplit) {
				nodes.add(node);
				todo.push(node.left);
				todo.push(node.right);
			}
		}
		out(numDataPointsAccountedFor + " of " + NUM_IMAGES + " accounted for");
		out("Leafs: " + leafs);
	}

	public boolean goLeft(int index, int dim) {
		return data[index][dim] < 128;
	}

	public boolean goLeft(int[] img, int dim) {
		return img[dim] < 128;
	}

	public int[] evaluate(int[] probe) {
		Node node = root;
		while (!node.isLeaf) {
			if (goLeft(probe, node.splitDim))
				node = node.left;
			else
				node = node.right;
		}
		return node.indices;
	}

	void out(String msg) {
		System.out.println(msg);
	}

 
	class Node {

		int splitDim = -1;
		int[] indices;
		boolean[] key;
		Node left;
		Node right;
		int size;
		int height;
		Node parent;
		boolean isLeaf;

		public Node(Node parent, int[] indices, boolean[] key) {
			
			this.parent = parent;
			if (parent == null)
				height = 0;
			else 
				height = parent.height;

			this.key = new boolean[NUM_DIMENSIONS];
			System.arraycopy(key, 0, this.key, 0, NUM_DIMENSIONS);
			this.indices = new int[indices.length];
			System.arraycopy(indices, 0, this.indices, 0, indices.length);
			size = indices.length;
		}

		public boolean trySplit() {
			int bestD = -1;
			int bestPopulation = 0;
			for (int d = 0; d < NUM_DIMENSIONS; d++) {
				if (!key[d]) {
					int population = 0;
					for (int i = 0; i < indices.length; i++) {
						if (goLeft(indices[i], d)) 
							population++;
					}
					if (Math.abs(2 * population - size) < Math.abs(2 * bestPopulation - size)) {
						bestD = d;
						bestPopulation = population;
					}
				}
			}
			if (bestD == -1) {
				return false;
			}
			splitDim = bestD;
			key[bestD] = true;
			int numLeft = bestPopulation;
			int numRight = size - numLeft;
			int[] indicesLeft = new int[numLeft];
			int[] indicesRight = new int[numRight];
			for (int i = 0, l = 0, r = 0; i < size; i++) {
				if (goLeft(indices[i], bestD)) {
					indicesLeft[l++] = indices[i];
				} else {
					indicesRight[r++] = indices[i];
				}
			}
			left = new Node(this, indicesLeft, key);
			right = new Node(this, indicesRight, key);
			return true;
		}

	}
}
