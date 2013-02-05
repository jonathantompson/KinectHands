package handshaperecognizer;

import oniclient.*;

import java.util.Arrays;
import java.awt.Graphics2D;
import java.io.File;
import java.io.FilenameFilter;
import java.util.Map;
import java.util.SortedMap;
import java.util.Stack;
import java.util.TreeMap;
import java.io.*;
import oniclient.HandTracker;
import rfclassifier.*;

public class HandShapeDictionary implements HandTrackerListener {

	public Map<String, HandShape> shapes = new TreeMap<String, HandShape>();
	public Matcher matcherLH = new Matcher();
	public Matcher matcherRH = new Matcher();
	public Matcher matcher = matcherLH;
	
	public HandShapeDictionary() {};

	//	public BalancedHashTree bht;
	public final static int MAX_EXEMPLARS_PER_HANDSHAPE = 100 * 100;
	public final static int TINY_IMAGE_UNIT = 64;
	public final static int PACKED_IMAGE_SIZE = 64;
	public final static int UNPACKED_IMAGE_SIZE = PACKED_IMAGE_SIZE * PACKED_IMAGE_SIZE;

	public int NUM_IMAGES;
	public String hand_shape_directory_name;
	public int[][] handScreenBounds = new int[2][4];
	HandTracker handTracker;
	public int PIXEL_WIDTH_OF_HAND_BOX_AT_500MM = 336;
	double[] shapeScores = new double[13];
	public int[] shapeIndices = new int[13];
	public long[][] allData;
	public int[] allLabels;
	public double[] allScores;
	public int[] allSortedIndices;
	int[] tmpMask = new int[640 * 480];
	RFClassifier rf;
	public static boolean useRF = true;	
	public static boolean useVerboseLogging = true;
	public boolean needToCollectImages = true;
	Sorter sorter;
	double[] intraShapeMatchScore;
	int[] sortedShapeIndices;
	int[] tinyTmp = new int[UNPACKED_IMAGE_SIZE];
	int[] tinyTmp2 = new int[UNPACKED_IMAGE_SIZE];
	public boolean useLeftHandForCapture = true;

	public byte[][] allDataBinary;

	public long[] packed = new long[TINY_IMAGE_UNIT];
	public long[] packedTmp = new long[TINY_IMAGE_UNIT];
	public byte[] bProbe = new byte[UNPACKED_IMAGE_SIZE];
	public short[][] diffIndices;
	public double[] treeScores;
	public long[][] packedWeights;
	MinimumSpanningTree mst;

	public static int MAX_IMAGES;

	public HandShapeDictionary(String dirname) {

		NUM_IMAGES = 0;
		File[] files = getHandShapeFileHandlesFromDirectory(dirname);

		if (files == null) {
			System.err.println("Couldn't find directory: " + dirname);
		}
		if (files == null || files.length == 0) {
			createNewShape();
		} else {
			for (int i = 0; i < files.length; i++) {
				HandShape newshape = new HandShape();
				newshape.readFromFile(files[i], MAX_EXEMPLARS_PER_HANDSHAPE);
				shapes.put(newshape.name, newshape);
				NUM_IMAGES += newshape.length;
			}
		}

		MAX_IMAGES = shapes.size() * MAX_EXEMPLARS_PER_HANDSHAPE;

		hand_shape_directory_name = dirname;
		out("HandShapeLibrary: I read " + NUM_IMAGES + " from directory " + dirname);

		sorter = new Sorter();
		sorter.ascending = false;

		allocateMemory();

		if (useRF) {
			allLabels = getAllLabelsAsArray();
			allData = getAllImagesAsArray();

//			RFTrainer.NUM_CATEGORIES = numShapes();
//			RFTrainer.NUM_TREES = 25;
//			RFTrainer.MAX_TREE_DEPTH = 12;
//			RFTrainer.WEAKLEARNER_GENERATIONS = 100000;
//			WLPackedLearnedMasks.learnMasks(allData);
//			out("NUM IMAGES IS " + NUM_IMAGES);
//			RFTrainer rft = new RFTrainer(allData, allLabels, NUM_IMAGES);
//			rft.train();
//			rft.saveToFile(dirname + "/trees.rf");
//
//			double trainAccuracy = rft.evaluate(allData, allLabels, NUM_IMAGES);
//			out("Finished training.");
//			out("Accuracy on training set: " + trainAccuracy);

			rf = new RFClassifier();
			rf.readFromFile(dirname + "/rf_new.tree");

			printHandShapeNames();
		}		
	}

	public void allocateMemory() {
		MAX_IMAGES = shapes.size() * MAX_EXEMPLARS_PER_HANDSHAPE;
		allScores = new double[MAX_IMAGES];
		allSortedIndices = new int[MAX_IMAGES];

		if (allData != null) {
			long[][] newData = new long[MAX_IMAGES][PACKED_IMAGE_SIZE];
			int[] newLabels = new int[MAX_IMAGES];
			for (int i = 0; i < NUM_IMAGES; i++) {
				System.arraycopy(allData[i], 0, newData[i], 0, PACKED_IMAGE_SIZE);
				newLabels[i] = allLabels[i];
			}
			allData = newData;
			allLabels = newLabels;
		} else {
			allData = new long[MAX_IMAGES][PACKED_IMAGE_SIZE];
			allLabels = new int[MAX_IMAGES];
		}

	}

	public String createNewShape() {
		String newName = null;
		for (int i = 97; i <= 122; i++) {
			String key = String.valueOf((char)i);
			if (!shapes.containsKey(key)) {
				newName = key;
				break;
			}
		}
		if (newName != null) {
			HandShape newShape = new HandShape(newName, PACKED_IMAGE_SIZE);
			shapes.put(newName, newShape);
			out("Created new shape named: " + newName);
			allocateMemory();
		} else {
			out("Couldn't create new shape because all 26 have been used");
		}
		return newName;
	}

	public File[] getHandShapeFileHandlesFromDirectory(String dirname) {

		File dir = new File(dirname);

		return dir.listFiles(new FilenameFilter() { 
			public boolean accept(File dir, String filename)
			{ return filename.endsWith(".bin"); }
		} );

	}

	public boolean isZero(long[] tiny) {
		for (int i = 0; i < tiny.length; i++) {
			if (tiny[i] != 0) return false;
		}
		return true;
	}

	public boolean isZero(int[] tiny) {
		for (int i = 0; i < tiny.length; i++) {
			if (tiny[i] != 0) return false;
		}
		return true;
	}

	public int numWithValue(int[] tiny, int value) {
		int count = 0;
		for (int i = 0; i < tiny.length; i++) {
			if (tiny[i] == value) count++;
		}
		return count;
	}

	public int numWithValueAtMost(int[] tiny, int value) {
		int count = 0;
		for (int i = 0; i < tiny.length; i++) {
			if (tiny[i] <= value) count++;
		}
		return count;
	}

	public int lookupIndexFromLetter(String name) {
		name = name.toLowerCase();
		if (name.equals("a")) return 0;
		if (name.equals("b")) return 1;
		if (name.equals("c")) return 2;
		if (name.equals("d")) return 3;
		if (name.equals("e")) return 4;
		if (name.equals("f")) return 5;
		if (name.equals("g")) return 6;
		if (name.equals("h")) return 7;
		if (name.equals("i")) return 8;
		if (name.equals("j")) return 9;
		if (name.equals("k")) return 10;
		if (name.equals("l")) return 11;
		if (name.equals("m")) return 12;
		return -1;
	}

	public boolean captureTiny(String shapeName, int[] tiny, int position) {
		boolean wasAdded = false;
		HandShape shape = shapes.get(shapeName);
		long[][] tinyImages = shape.getImagesArray();
		int[][] tinyImagesInt = shape.getImagesArrayInt();
		packTiny(tiny, packed);
		if (position >= 0 && position < MAX_EXEMPLARS_PER_HANDSHAPE) {
			if (shape.length < MAX_EXEMPLARS_PER_HANDSHAPE) {
				boolean differentOrFirst = position == 0 || scoreTiny(packed, tinyImages[position -1]) > 0;
				if (differentOrFirst) {
					boolean positionIsBlack = isZero(tinyImages[position]);
					wasAdded = true;
					System.arraycopy(packed, 0, tinyImages[position], 0, PACKED_IMAGE_SIZE);
					unpackTiny(packed, tinyImagesInt[position]);
					shape.length++;
					out("Captured tiny image to shape " + shape.name + " position: " + position);
					packTiny(tiny, allData[NUM_IMAGES]);
					allLabels[NUM_IMAGES] = lookupIndexFromLetter(shape.name);
					if (positionIsBlack)
						NUM_IMAGES++;
					//					needToCollectImages = true;
				} else {
					out("Skipping capture, image is same as last.");
				}
			}
		}
		return wasAdded;
	}

	public static void binarizeTiny(int[] tiny, byte[] bTiny) {
		for (int i = 0; i < tiny.length; i++)
			bTiny[i] = tiny[i] < 128 ? (byte)0 : (byte)255;
	}

	public static void packTiny(int[] tiny, long[] packed) {
		int i = 0;
		for (int r = 0; r < TINY_IMAGE_UNIT; r++) {
			long a = 0L;
			for (int c = 0; c < TINY_IMAGE_UNIT - 1; c++) {
				a = a | ((tiny[i] < 128) ? 0L : 1L);
				a = a << 1;
				i++;				
			}
			packed[r] = a | ((tiny[i] < 128) ? 0L : 1L);
			i++;
		}
		//out("packed: " + i);
	}

	public static void unpackTiny(long[] packed, int[] tiny) {
		int i = UNPACKED_IMAGE_SIZE - 1;
		for (int r = TINY_IMAGE_UNIT - 1; r >= 0; r--) {
			long row = packed[r];
			for (int c = TINY_IMAGE_UNIT - 1; c >= 0; c--) {
				tiny[i--] = (int)(row & 1) * 255;
				row = row >> 1;
			}
		}
	}

	public void unpackManySorted(int[][] dst, int start, int end) {
		for (int i = start, j = 0; i < end; i++) {
			unpackTiny(allData[allSortedIndices[i]], dst[j++]);
		}
	}

	//types and constants used in the functions below

	final long m1  = 0x5555555555555555L; //binary: 0101...
	final long m2  = 0x3333333333333333L; //binary: 00110011..
	final long m4  = 0x0f0f0f0f0f0f0f0fL; //binary:  4 zeros,  4 ones ...
	final long m8  = 0x00ff00ff00ff00ffL; //binary:  8 zeros,  8 ones ...
	final long m16 = 0x0000ffff0000ffffL; //binary: 16 zeros, 16 ones ...
	final long m32 = 0x00000000ffffffffL; //binary: 32 zeros, 32 ones
	final long hff = 0xffffffffffffffffL; //binary: all ones
	final long h01 = 0x0101010101010101L; //the sum of 256 to the power of 0,1,2,3...

	//	//This is a naive implementation, shown for comparison,
	//	//and to help in understanding the better functions.
	//	//It uses 24 arithmetic operations (shift, add, and).
	//	long popcount_1(long x) {
	//	    x = (x & m1 ) + ((x >>  1) & m1 ); //put count of each  2 bits into those  2 bits 
	//	    x = (x & m2 ) + ((x >>  2) & m2 ); //put count of each  4 bits into those  4 bits 
	//	    x = (x & m4 ) + ((x >>  4) & m4 ); //put count of each  8 bits into those  8 bits 
	//	    x = (x & m8 ) + ((x >>  8) & m8 ); //put count of each 16 bits into those 16 bits 
	//	    x = (x & m16) + ((x >> 16) & m16); //put count of each 32 bits into those 32 bits 
	//	    x = (x & m32) + ((x >> 32) & m32); //put count of each 64 bits into those 64 bits 
	//	    return x;
	//	}
	//	
	//	
	//	//This uses fewer arithmetic operations than any other known  
	//	//implementation on machines with slow multiplication.
	//	//It uses 17 arithmetic operations.
	//	long popcount_2(long x) {
	//	    x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
	//	    x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits 
	//	    x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits 
	//	    x += x >>  8;  //put count of each 16 bits into their lowest 8 bits
	//	    x += x >> 16;  //put count of each 32 bits into their lowest 8 bits
	//	    x += x >> 32;  //put count of each 64 bits into their lowest 8 bits
	//	    return x & 0x7fL;
	//	}

	long popcount_3(long x) {
		x -= (x >> 1) & m1;             //put count of each 2 bits into those 2 bits
		x = (x & m2) + ((x >> 2) & m2); //put count of each 4 bits into those 4 bits 
		x = (x + (x >> 4)) & m4;        //put count of each 8 bits into those 8 bits 
		return (x * h01)>>56;  //returns left 8 bits of x + (x<<8) + (x<<16) + (x<<24) + ... 
	}

	// LOW numbers are better
	public short scoreTiny(long[] packed, long[] probe) {
		short score = 0;
		for (int i = 0; i < packed.length; i++) {
			long xor = packed[i] ^ probe[i];
			score += popcount_3(xor);
		}
		return score;
	}

	// LOW numbers are better
	public short scoreTinyWeighted(long[] packed, long[] weights, long[] probe) {
		short score = 0;
		for (int i = 0; i < packed.length; i++) {
			long xor = packed[i] ^ probe[i];
			long wei = xor & weights[i];
			//			long weiSub = ~(packed[i] & weights[i]) & probe[i];
			//			long weiSub = xor & weights[i]; 
			//			score += popcount_3(xor) + popcount_3(weiAdd); //popcount_3(xor) + popcount_3(weiAdd) - popcount_3(weiSub);
			score += popcount_3(xor) + popcount_3(wei); 
		}
		return score;
	}


	// LOW numbers are better
	public short scoreTinyBinary(byte[] bTiny, byte[] probe) {
		short score = 0;
		for (int i = 0; i < bTiny.length; i++) {
			if (bTiny[i] != probe[i]) score++;
		}
		return score;
	}

	public void scoreProbeByTree(int[] probe) {		
		packTiny(probe, packed);
		binarizeTiny(probe, bProbe);
		int[][] tree = mst.tree;

		// CALCULATE TEMPLATE SCORES FIRST
		int[] rootEdges = tree[mst.order[0]];
		for (int i = 0; i < rootEdges.length; i++) {
			int childIndex = rootEdges[i];
			treeScores[childIndex] = scoreTiny(allData[childIndex], packed);
		}

		// CALCULATE NON-TEMPLATE SCORES, IN ORDER
		for (int i = rootEdges.length + 1; i < tree.length; i++) {
			int childIndex = mst.order[i];
			int parentIndex = mst.P[childIndex];
			double iScore = treeScores[parentIndex];
			short[] indices = diffIndices[childIndex];
			if (indices != null) {
				for (int k = 0; k < indices.length; k++) {
					short d = diffIndices[childIndex][k];
					iScore += bProbe[d] == allDataBinary[parentIndex][d] ?  -1 : 1;
				}
			}
			treeScores[childIndex] = iScore;				
		}
	}

	public boolean isUnique(long[] probe) {
		for (int i = 0; i < NUM_IMAGES; i++) {
			if (scoreTiny(probe, allData[i]) <= 5) {
				return false;
			}
		}
		return true;
	}

	public void collectImages() {
		if (needToCollectImages) {
			needToCollectImages = false;
			out("Dictionary: entering recognition mode.");
			out("Collecting images.");
			getAllImagesAsArray(allData);
			getAllLabelsAsArray(allLabels);			
		}
	}

	public void scoreAllFast(int[] probe) {
		packTiny(probe, packed);
		for (int i = 0; i < NUM_IMAGES; i++) {
			allScores[i] =  scoreTiny(allData[i], packed);
		}
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

	long[][] probeShifts = new long[8][64];
	final int LEFT = 0;
	final int RIGHT = 1;
	final int UP = 2;
	final int DOWN = 3;
	final int UPLEFT = 4;
	final int UPRIGHT = 5;
	final int DOWNLEFT = 6;
	final int DOWNRIGHT = 7;

	public long[][] createOffsets(long[] probe) {

		System.arraycopy(probe, 0, probeShifts[LEFT], 0, PACKED_IMAGE_SIZE);
		shiftPackedLeft(probeShifts[LEFT]);
		System.arraycopy(probe, 0, probeShifts[RIGHT], 0, PACKED_IMAGE_SIZE);		
		shiftPackedRight(probeShifts[RIGHT]);
		System.arraycopy(probe, 0, probeShifts[UP], 0, PACKED_IMAGE_SIZE);		
		shiftPackedUp(probeShifts[UP]);
		System.arraycopy(probe, 0, probeShifts[DOWN], 0, PACKED_IMAGE_SIZE);		
		shiftPackedDown(probeShifts[DOWN]);
		System.arraycopy(probeShifts[LEFT], 0, probeShifts[UPLEFT], 0, PACKED_IMAGE_SIZE);		
		shiftPackedUp(probeShifts[UPLEFT]);
		System.arraycopy(probeShifts[RIGHT], 0, probeShifts[UPRIGHT], 0, PACKED_IMAGE_SIZE);		
		shiftPackedUp(probeShifts[UPRIGHT]);
		System.arraycopy(probeShifts[LEFT], 0, probeShifts[DOWNLEFT], 0, PACKED_IMAGE_SIZE);		
		shiftPackedDown(probeShifts[DOWNLEFT]);
		System.arraycopy(probeShifts[RIGHT], 0, probeShifts[DOWNRIGHT], 0, PACKED_IMAGE_SIZE);		
		shiftPackedDown(probeShifts[DOWNRIGHT]);

		return probeShifts;

	}

	public void scoreAllFastButCareful(int[] probe) {
		scoreAllFast(probe);
		//		scoreProbeByTree(probe);
		//		System.arraycopy(treeScores, 0, allScores, 0, NUM_IMAGES);
		//		sorter.ascending = true;
		//		sorter.quickSort(allScores, allSortedIndices);
		String s = "";
		packTiny(probe, packed);
		for (int i = 0; i < 10; i++)
			s += allScores[i] + "\t";			
		out("Top 10 (binary): \t\t\t" + s);

		int numToImprove = 100;

		s = "";
		long[][] probeShifts = createOffsets(packed);
		for (int i = 0; i < numToImprove; i++) {
			long min = UNPACKED_IMAGE_SIZE;
			int idx = allSortedIndices[i];
			for (int j = 0; j < probeShifts.length; j++) {
				long newScore = scoreTiny(probeShifts[j], allData[idx]);
				if (newScore < min) min = newScore;
			}
			if (min < allScores[i]) {
				allScores[i] = min;
			}
		}

		s = "";
		for (int i = 0; i < 10; i++)
			s += allScores[i] + "\t";			
		out("Top 10 (shifts): \t\t\t" + s);

		double worstScore = 0;
		for (int i = 0; i < numToImprove; i++) {
			if (allScores[i] > worstScore) {
				worstScore = allScores[i];
			}
		}

		//		for (int i = 0; i < numToImprove; i++) {
		//			int idx = allSortedIndices[i];
		//			allScores[i] = scoreSlow(probe, allData[idx]);
		//		}
		sorter.ascending = true;
		sorter.quickSort(allScores, allSortedIndices, 0, numToImprove - 1);
		for (int i = 0; i < numToImprove; i++) {
			allScores[i] = 1 - (allScores[i] / (UNPACKED_IMAGE_SIZE));
		}
	}

	public double scoreSlow(int[] tiny, int[] probe) {
		double diff = 0;
		for (int i = 0; i < probe.length; i++) {
			double d = (tiny[i] - probe[i]);
			diff += d * d;
		}		
		return diff / (UNPACKED_IMAGE_SIZE * UNPACKED_IMAGE_SIZE * 255 * 255);
	}


	public Matcher simpleMatch(int[] probe, int hand) {
		if (hand == 0) {
			matcher = matcherLH;
		} else {
			matcher = matcherRH;
		}
		scoreAllFast(probe);
		for (int i = 0; i < 100; i++) {
			allScores[i] =  1 - scoreTiny(allData[i], packed) / (double)UNPACKED_IMAGE_SIZE;
		}
		//out("NUM_IMAGES: " + NUM_IMAGES);
		Arrays.fill(shapeScores, 0);
		Arrays.fill(shapeIndices, 0);
		for (int i = 0; i < 100; i++) {

			int label = allLabels[i];
			if (allScores[i] > shapeScores[label]) {
				shapeScores[label] = allScores[i];
				shapeIndices[label] = i;
			}
		}
		System.out.printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", shapeScores[0], shapeScores[1], shapeScores[2], shapeScores[3], shapeScores[4]);
		double max = 0;
		int best = 0;
		int best_exemplar = 0;
		for (int i = 0; i < shapeScores.length; i++) {
			if (shapeScores[i] > max) {
				max = shapeScores[i];
				best = i;
				best_exemplar = shapeIndices[i];
			}
		}
		matcher.bestShape = getHandShapeFromIndex(best);
		matcher.bestShapeIndex = best;
		matcher.bestExamplarIndex = best_exemplar;
		matcher.bestScore = max;

		return matcher;
	}


	public Matcher match(int[] probe, int hand) {
		
		if (useRF) {
			return matchRF(probe, hand);
		}
		
		collectImages();
		//scoreAllFastButCareful(probe);
		scoreAllFast(probe);
		//scoreAllBHT(probe);
		//out("NUM_IMAGES: " + NUM_IMAGES);
		for (int i = 0; i < NUM_IMAGES; i++)
			allSortedIndices[i] = i;
		sorter.ascending = true;		
		sorter.quickSort(allScores, allSortedIndices, 0, NUM_IMAGES - 1);		
		for (int i = 0; i < 100; i++) {
			allScores[i] =  1 - scoreTiny(allData[i], packed) / (double)UNPACKED_IMAGE_SIZE;
		}

		Arrays.fill(shapeScores, 0);
		Arrays.fill(shapeIndices, 0);
		for (int i = 0; i < 100; i++) {
			//			int idx = i;
			int idx = allSortedIndices[i];
			int label = allLabels[idx];
			if (allScores[i] > shapeScores[label]) {
				shapeScores[label] = allScores[i];
				shapeIndices[label] = idx;
			}
		}
		System.out.printf("%.3f, %.3f, %.3f, %.3f, %.3f\n", shapeScores[0], shapeScores[1], shapeScores[2], shapeScores[3], shapeScores[4]);
		double max = 0;
		int best = 0;
		int best_exemplar = 0;
		for (int i = 0; i < shapeScores.length; i++) {
			if (shapeScores[i] > max) {
				max = shapeScores[i];
				best = i;
				best_exemplar = shapeIndices[i];
			}
		}
		matcher.bestShape = getHandShapeFromIndex(best);
		matcher.bestShapeIndex = best;
		matcher.bestExamplarIndex = best_exemplar;
		matcher.bestScore = max;
		if (matcher.bestExamplarIndex >= 0 && matcher.bestExamplarIndex < NUM_IMAGES)
			unpackTiny(allData[best_exemplar], matcher.image);
		return matcher;
	}


	double[] distribution;
	public Matcher matchRF(int[] probe, int hand) {
		if (hand == 0) {
			matcher = matcherLH;
		} else {
			matcher = matcherRH;
		}
	
		if (distribution == null) {
			distribution = new double[numShapes()];
		}
		
		packTiny(probe, packed);
		int numMatched = rf.match(packed, distribution);
		int category = rf.categorize(distribution);

//		for (int i = 0; i < numMatched; i++) {
//			int idx = rf.matches[i];
//			allSortedIndices[i] = idx;
//			allScores[i] = scoreTiny(packed, allData[idx]);
//		}
//		sorter.ascending = true;		
//		sorter.quickSort(allScores, allSortedIndices, 0, numMatched - 1);		
//
//		for (int i = 0; i < numMatched; i++) {
//			allScores[i] = 1 - allScores[i] / (double)UNPACKED_IMAGE_SIZE;
//		}
//
//		Arrays.fill(shapeScores, 0);
//		Arrays.fill(shapeIndices, 0);
//		for (int i = 0; i < numMatched; i++) {
//			int idx = allSortedIndices[i];
//			int label = allLabels[idx];
//			if (allScores[i] > shapeScores[label]) {
//				shapeScores[label] = allScores[i];
//				shapeIndices[label] = idx;
//			}
//		}
//		double max = 0;
//		int best = 0;
//		int best_exemplar = 0;
//		for (int i = 0; i < shapeScores.length; i++) {
//			if (shapeScores[i] > max) {
//				max = shapeScores[i];
//				best = i;
//				best_exemplar = shapeIndices[i];
//			}
//		}
		if (matcher.updateBestShapeIndex(category)) {
			matcher.bestShape = getHandShapeFromIndex(category);
			matcher.bestExamplarIndex = category * MAX_EXEMPLARS_PER_HANDSHAPE;
			matcher.bestScore = distribution[category];
//			String s = "";
//			for (int i = 0; i < 20; i++) {
//				s += matcher.bestShape.name + " ";
//			}
//			out(s);
			if (matcher.bestExamplarIndex >= 0 && matcher.bestExamplarIndex < NUM_IMAGES)
				unpackTiny(allData[category * MAX_EXEMPLARS_PER_HANDSHAPE], matcher.image);
		} else {
//			if (matcher.bestShape != null) {
//				String s = "";
//				for (int i = 0; i < 20; i++) {
//					s += matcher.bestShape.name + " ";
//				}
//				out("STICKING TO: " + s);
//			}
		}
		return matcher;
	}

	//	public void scoreAllBHT(int[] probe) {
	//		
	//		int[] matches = bht.evaluate(probe);
	//		int numToImprove = matches.length;
	//		packTiny(probe, packed);
	//		for (int i = 0; i < numToImprove; i++) {
	//			int idx = matches[i];
	//			allScores[i] = scoreTiny(packed, allData[idx]);
	//			allSortedIndices[i] = idx;
	//		}
	//		sorter.ascending = true;
	//		sorter.quickSort(allScores, allSortedIndices, 0, numToImprove - 1);
	//
	//		double worstScore = 0;
	//		for (int i = 0; i < numToImprove; i++) {
	//			if (allScores[i] > worstScore) {
	//				worstScore = allScores[i];
	//			}
	//		}
	//
	//		for (int i = 0; i < numToImprove; i++) {
	//			allScores[i] = 1 - (allScores[i] / worstScore);
	//		}
	//	}



	//	public Matcher match(int[] probe) {		
	//
	//		if (useRF) {
	//			matcher.reset();
	//			double[] distribution = rf.classify(probe);
	//			Arrays.fill(shapeScores, 0);
	//			for (int i = 0; i < distribution.length; i++) {
	//				shapeScores[i] = ((int)(distribution[i] * 100.0));
	//			}
	//			int winningLabel = rf.categorize(distribution);
	//			//out("WINNING LABEL :" + winningLabel);
	//			matcher.bestshape = getHandShapeFromIndex(winningLabel);
	//			matcher.bestscore = distribution[winningLabel];
	//			matcher.bestexamp = 0;
	//
	//		} else {
	//			matcher.reset();
	//			Arrays.fill(shapeScores, 0);
	//			int k = 0;
	//			for (Map.Entry<String, HandShape> entry : shapes.entrySet()) {
	//				HandShape shape = entry.getValue();
	//				int[][] tinyImages = shape.getImagesArray();
	//				for (int i = 0; i < MAX_EXEMPLARS_PER_HANDSHAPE; i++) {
	//					double score = matcher.match(tinyImages[i], probe);
	//					if (score > matcher.bestscore) {
	//						matcher.bestshape = shape;
	//						matcher.bestscore = score;
	//						matcher.bestshape_index = i;
	//					}
	//					if (score > shapeScores[k]) {
	//						shapeScores[k] = score;
	//					}
	//				}
	//				k++;
	//			}
	//			for (int i = 0; i < shapeScores.length; i++)
	//				shapeScores[i] = (int)((1 - Math.sqrt((-shapeScores[i] + 1))) * 100.0);
	//			System.out.printf("%.2f, %.2f, %.2f, %.2f, %.2f\n", shapeScores[0], shapeScores[1], shapeScores[2], shapeScores[3], shapeScores[4]);
	//		}
	//
	//		return matcher;
	//	}

	// Write requested shape to file or warn
	public int writeShapeToFile(String name, String dirname) {
		int success = -1;
		HandShape saveShape = shapes.get(name);
		cleanupBlacks(saveShape);
		if (saveShape != null) {
			String fileName = saveShape.getRelativeFilename();
			if (dirname.endsWith("/"))
				success = saveShape.writeToFile(dirname + fileName);
			else
				success = saveShape.writeToFile(dirname + "/" + fileName);

		} else {
			out("Warning: attempted to write shape " + name + " failed (out of range)");
			success = -1;
		}
		return success;
	}

	// Do bounds check on requested shape number
	// return null if out of bounds
	public HandShape getShape(String name) {
		return shapes.get(name);
	}

	public int numShapes() {
		return shapes.size();
	}

	public double[] getScores() {
		return shapeScores;
	}

	int[] clientTinyIntTmp = new int[UNPACKED_IMAGE_SIZE];
	public int[] getTinyImage(String k, int i) {
		HandShape s = shapes.get(k);
		if (s != null) {
			unpackTiny(s.getTinyImage(i), clientTinyIntTmp);
		} else {
			//out("Shape named: " + k + " was not found, so couldn't load tiny image.");
		}
		return clientTinyIntTmp;
	}


	public int[] getBestTinyImageAfterMatch(int i) {
		unpackTiny(allData[shapeIndices[i]], clientTinyIntTmp);
		return clientTinyIntTmp;
	}

	public HandShape getHandShapeFromIndex(int i) {
		int cnt = 0;
		for (HandShape s: shapes.values()) {
			if (cnt++ == i) return s;
		}
		return null;
	}

	public void printHandShapeNames() {
		int cnt = 0;
		String str = "";

		for (String k: shapes.keySet()) {
			str += k + " ";
		}		
		out("HandShape names: " + str);
	}

	public void updateHandScreenBounds(int[] mask, int maskValue, int z, int[] aabb) {

		int cx = 0;
		int cy = 0;
		int cnt = 0;
		for (int i = 0; i < mask.length; i++) {
			if (mask[i] == maskValue) {
				cx += i % 640;
				cy += i / 640;
				cnt += 1;
			}
		}
		if (cnt != 0) {
			cx = cx / cnt;
			cy = cy / cnt;
		}

		int bw = (int)(PIXEL_WIDTH_OF_HAND_BOX_AT_500MM * (500.0 / z));
		aabb[0] = cx - bw / 2;		// left
		aabb[1] = aabb[0] + bw;		// right
		aabb[2] = cy - bw / 2;		// top
		aabb[3] = aabb[2] + bw;		// bottom
	}	

	public void deleteTiny(String name, int index) {
		HandShape s = shapes.get(name);
		s.deleteTiny(index);
		needToCollectImages = true;
	}

	public void removeDuplicates(String name) {
		out("Removing duplicates...");
		HandShape s = shapes.get(name);	
		long[][] images = s.getImagesArray();
		for (int i = 0; i < s.length; i++) {
			for (int j = i+1; j < s.length; j++) {
				if (scoreTiny(images[i], images[j]) == 0) {
					s.deleteTiny(j);
					out("removed: " + j);
					needToCollectImages = true;
				}
			}
		}
		cleanupBlacks(s);
	}

	public void getAllImagesAsArray(long[][] data) {
		int imagesLoaded = 0;
		for (HandShape s : shapes.values()) {
			long[][] images = s.getImagesArray();
			for (int i = 0; i < s.length; i++) {
				System.arraycopy(images[i], 0, data[imagesLoaded], 0, PACKED_IMAGE_SIZE);
				imagesLoaded++;
			}
		}
		out("I think imagesLoaded is " + imagesLoaded);
		NUM_IMAGES = imagesLoaded;
	}

	public long[][] getAllImagesAsArray() {
		long[][] data = new long[MAX_IMAGES][PACKED_IMAGE_SIZE];
		getAllImagesAsArray(data);
		return data;
	}

	public int[][] getAllImagesAsIntArray() {
		int[][] data = new int[MAX_IMAGES][UNPACKED_IMAGE_SIZE];
		for (HandShape s : shapes.values()) {
			long[][] images = s.getImagesArray();
			for (int i = 0; i < s.length; i++) {
				//System.arraycopy(images[i], 0, data[imagesLoaded], 0, IMAGE_SIZE);
				unpackTiny(images[i], data[i]);
			}
		}
		return data;
	}

	public void getAllLabelsAsArray(int[] labels) {
		int imagesLoaded = 0;
		int key = 0;
		for (HandShape s : shapes.values()) {
			for (int i = 0; i < s.length; i++) {
				labels[imagesLoaded] = key;
				imagesLoaded++;
			}
			key++;
		}
	}

	public int[] getAllLabelsAsArray() {
		int[] labels = new int[MAX_IMAGES];
		getAllLabelsAsArray(labels);
		return labels;
	}

	public void cleanupBlacks(HandShape s) {
		out("Removing black images from shape: " + s.name);
		out("Currently shape is size: " + s.length);
		long[][] im = s.getImagesArray();
		int[][] imInt = s.getImagesArrayInt();
		long[][] tmpIm = new long[im.length][PACKED_IMAGE_SIZE];
		int[][] tmpImInt = new int[im.length][UNPACKED_IMAGE_SIZE];
		int good = 0;
		int numBlacks = 0;
		for (int i = 0; i < im.length; i++) {
			if (!isZero(im[i])) {
				System.arraycopy(im[i], 0, tmpIm[good], 0, PACKED_IMAGE_SIZE);
				System.arraycopy(imInt[i], 0, tmpImInt[good++], 0, UNPACKED_IMAGE_SIZE);
			} else {
				numBlacks++;
			}
		}
		s.length = good;
		for (int i = 0; i < im.length; i++)
			if (i < good) {
				System.arraycopy(tmpIm[i], 0, im[i], 0, PACKED_IMAGE_SIZE);
				System.arraycopy(tmpImInt[i], 0, imInt[i], 0, UNPACKED_IMAGE_SIZE);
			} else {
				Arrays.fill(im[i], 0);
				Arrays.fill(imInt[i], 0);
			}
		NUM_IMAGES = NUM_IMAGES - numBlacks;
		out("Done.  Shape now has length: " + s.length);
	}

	/********************************************
	 TRAIN TINY IMAGE STUFF
	 *********************************************/


	public void makeTiny(int[] mask, int hand, int[] tinyHand) {

		int maskValue = 0;
		if (hand == HandTracker.LH)
			maskValue = 1;
		else if (hand == HandTracker.RH)
			maskValue = 2;

		int z = 500;
		if (handTracker != null) {
			z = handTracker.handDepth(hand);
		}
		for (int i = 0; i < mask.length; i++) {
			if (mask[i] == maskValue) tmpMask[i] = maskValue;
			else tmpMask[i] = 0;
		}
		updateHandScreenBounds(tmpMask, maskValue, z, handScreenBounds[hand]);


		int boxl = handScreenBounds[hand][0];
		int boxr = handScreenBounds[hand][1];
		int boxt = handScreenBounds[hand][2];
		int boxb = handScreenBounds[hand][3];
		int boxw = boxr - boxl + 1;
		int boxh = boxb - boxt + 1;
		ImageResizer.downsizeUsingSummedArea(tmpMask, boxl, boxt, boxw, boxh, 640, 
				tinyHand, 0, 0, TINY_IMAGE_UNIT, TINY_IMAGE_UNIT, TINY_IMAGE_UNIT);
		double min = Double.MAX_VALUE;
		double max = Double.MIN_VALUE;

		for (int i = 0; i < tinyHand.length; i++) {
			if (tinyHand[i] > max) max = tinyHand[i];
			if (tinyHand[i] < min) min = tinyHand[i];
		}

		for (int i = 0; i < tinyHand.length; i++) {
			tinyHand[i] = (int)(255 * (tinyHand[i] - min) / (max - min));
		}

		if (hand == HandTracker.RH) {
			for (int y = 0; y < TINY_IMAGE_UNIT; y++) {
				for (int x = 0; x < TINY_IMAGE_UNIT / 2; x++) {
					int x0 = TINY_IMAGE_UNIT - x - 1;
					int i = x + y * TINY_IMAGE_UNIT;
					int j = x0 + y * TINY_IMAGE_UNIT;
					int tmp = tinyHand[i];
					tinyHand[i] = tinyHand[j];
					tinyHand[j] = tmp;
				}
			}
		}
	}

	public void handleHandTrackerEvent(HandTrackerEvent e) {
		handTracker = e.getTracker();
	}


	void out(String msg) {
		if (useVerboseLogging) {
			System.out.println(msg);
		}
	}
}