package handshaperecognizer;

import java.util.Arrays;

public class Matcher {

	public static final double		WORST_SCORE = Double.MIN_VALUE;

	public double 		bestScore;
	public int 			bestShapeIndex;
	public int 			bestExamplarIndex;
	public HandShape 	bestShape;
	public int[]		image;
	public int[] 		matchHistory;
	
	public Matcher() {
		image = new int[HandShapeDictionary.UNPACKED_IMAGE_SIZE];
		matchHistory = new int[5];
	}

	public void shiftHistory() {
		for (int i = matchHistory.length - 1; i > 0; i--) {
			matchHistory[i] = matchHistory[i - 1];
		}
	}
	
	public boolean updateBestShapeIndex(int newBest) {
		shiftHistory();
		matchHistory[0] = newBest;
		if (matchHistory[1] != newBest) return false;
		if (matchHistory[2] != newBest) return false;
		
		bestShapeIndex = newBest;
		return true;
	}
	
	public void reset() {
		bestScore = WORST_SCORE;
		bestShapeIndex = 0;
		bestExamplarIndex = 0;
		bestShape = null;
	}
	
	public double dist2(int[] v1, int[] v2) {
		double dd = 0;
		for (int i = 0; i < v1.length; i++) {
			double d = (v1[i] - v2[i]);
			dd += d * d;
		}
		return dd;
	}
	
	public double distContour(int[] v1, int[] v2) {
		double dd = 0;
		for (int i = 0; i < v1.length; i++) {
			double d = Math.random();
			dd += d * d + (128 - v1[i]) * (128 - v1[i])  + (128 - v2[i]) * (128 - v2[i]);
		}
		return dd;
	}


	public double dotprod (int[] v1, int[] v2) {
		double dp = 0;
		double v1Sum = 0;
		double v2Sum = 0;
		for (int i = 0; i < v1.length; i++) {
			v1Sum += v1[i];
			v2Sum += v2[i];
			dp += (v1[i] - 128) * (v2[i] - 128);
		}
		double v1Norm = (v1Sum - 128 * v1.length);
		v1Norm *= v1Norm;
		double v2Norm = (v2Sum - 128 * v2.length);
		v2Norm *= v2Norm;
		dp = dp / (v1Norm * v2Norm);
		return dp;
	}

	void out(String msg) {
		System.out.println(msg);
	}
}
