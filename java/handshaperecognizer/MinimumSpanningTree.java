package handshaperecognizer;

import java.util.Arrays;
import java.util.Stack;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

public class MinimumSpanningTree {

	double[][] W;
	int N[];
	int[] P;
	double[] D;
	int n;
	int[][] tree;
	int[] T;
	int root;
	int[] order;
	
	public MinimumSpanningTree(double[][] weights, int root) {
		W = weights;
		n = W.length;
		N = new int[n];		
		D = new double[n];
		P = new int[n];
		T = new int[n];
		order = new int[n];
		this.root = root;
	}

	
	public int nearestNodeOutsideTree() {
		int best = -1;
		double bestD = Double.POSITIVE_INFINITY;
		for (int i = 0; i < n; i++) {
			if (D[i] < bestD) {
				best = i;
				bestD = D[i];
			}
		}
		return best;		
	}
	
	public void addNodeToTree(int v) {
		P[v] = N[v];
		for (int i = 0; i < n; i++) {
			if (N[i] != -1 && W[v][i] < D[i]) {
				D[i] = W[v][i];
				N[i] = v;
			}
		}
		N[v] = -1;
		D[v] = Double.POSITIVE_INFINITY;
	}
	
	public void create() {
		for (int i = 0; i < n; i++) {
			D[i] = W[root][i];
			N[i] = root;
			P[i] = -1;
		}
		N[root] = -1;
		D[root] = Double.POSITIVE_INFINITY;
				
		for (int i = 1; i < n; i++) {			
			int next = nearestNodeOutsideTree();
			addNodeToTree(next);
		}
		out("P: " + Arrays.toString(P));
		out("N: " + Arrays.toString(N));
		out("D: " + Arrays.toString(D));		
	}
	
	void makeTree() {

		int[] tmpEdges = new int[n];
		int[][] tmpTree = new int[n][];
		boolean[] reachable = new boolean[n];
		Arrays.fill(reachable, false);
		ArrayBlockingQueue<Integer> q = new ArrayBlockingQueue<Integer>(n);

		for (int i = 0; i < n; i++) {
			if (P[i] == -1) {
				q.offer(i);
				reachable[i] = true;
			}			
		}
		int orderPosition = 0;
		int leafs = 0;
		while (!q.isEmpty()) {
			int parent = q.remove();
			order[orderPosition++] = parent;
			Arrays.fill(tmpEdges, -1);
			int cnt = 0;
			for (int i = 0; i < n; i++) {
				if (P[i] == parent) {
					if (reachable[i])
						out("UH OH, there is a cycle!");
					reachable[i] = true;
					q.offer(i);
					tmpEdges[cnt++] = i;
				}
			}
			if (cnt > 0) {
				tmpTree[parent] = new int[cnt];
				System.arraycopy(tmpEdges, 0, tmpTree[parent], 0, cnt);
			} else {
				tmpTree[parent] = null;
				leafs++;
			}			
		}

		tree = tmpTree;
//		tree = new int[n - leafs][];
//		for (int i = 0, cnt = 0; i < n; i++) {
//			if (tmpTree[order[i]] != null) {
//				tree[cnt++] = tmpTree[order[i]];
//			}
//		}
		out("Topological order: " + Arrays.toString(order));
		out("Reachable: " + Arrays.toString(reachable));
		for (int i = 0; i < n; i++) {
			if (!reachable[i]) out("Node " + i + " is not reachable");
		}

	}
	
	public double treeWeight() {
		double sum = 0;
		for (int i = 0; i < tree.length; i++)
			if (tree[i] != null)
				for (int j = 0; j < tree[i].length; j++) {
					int c = tree[i][j];
					sum += W[c][P[c]];
				}
		return sum;
	}
	
	public int getParent(int v) {
		return P[v];
	}
	
	public int[] getChildren(int v) {
		for (int i = 0; i < tree.length; i++) {
			if (tree[i][0] == v) return tree[i];
		}
		return null;
	}
		
	public void printTree() {
		
		for (int i = 0; i < tree.length; i++) {
			out("Node " + P[tree[i][0]] + ": " + Arrays.toString(tree[i]));
//			out("Node " + i + ": " + Arrays.toString(tree[i]));
		}
	}
	
	static void out(String msg) {
		System.out.println(msg);
	}
	
	static double[][] generateGraph(int n) {
		//[170, 74, 53, 191, 43, 188, 147, 33, 25, 151]
		int[] V = new int[n];
		
		double[][] W = new double[n][n];
		
		for (int i = 0; i < n; i++) {
			V[i] = (int)(Math.random() * 4096);
		}
		out("Vertices: " + Arrays.toString(V));
		
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				
				if (i == j) {
					//W[i][j] = Double.POSITIVE_INFINITY;
				} else {
					double xi = V[i] % 64;
					double yi = V[i] / 64;
					double xj = V[j] % 64;
					double yj = V[j] / 64;					
					W[i][j] = Math.random(); // (xi - xj) * (xi - xj) + (yi - yj) * (yi - yj);
				}
				
			}
		}
		return W;
	}

	public static void main(String[] args) {
		int n = 10;
		double[][] W = generateGraph(n);
		MinimumSpanningTree mst = new MinimumSpanningTree(W,n-1);
		mst.create();
		mst.makeTree();
		mst.printTree();
		out("root: " + (n-1));
		out("weight: " + mst.treeWeight());
	}
	
	
	
}
