package handshaperecognizer;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.Arrays;

import rfclassifier.RFClassifier;
import rfclassifier.RFTrainer;
import rfclassifier.WLIntegralImage;
import rfclassifier.WLPackedLearnedMasks;
import rfclassifier.WLTinyImage;
import canvasframe.CanvasFrame;

public class HandShapeRFClassifier {

	long[][] trainData;
	int[] trainLabels;

	long[][] validationData;
	int[] validationLabels;

	long[][] testData;
	int[] testLabels;

	public HandShapeRFClassifier() {

		long mask = 1 << 9;
		out("mask: " + ((mask >> 9) & 1));
		long dim = 0;
		long threshold = 0;
		
		out("packing...");

		String handshape_directory = "handshapes";
		HandShapeDictionary.useVerboseLogging = true;
		HandShapeDictionary.useRF = false;
		HandShapeDictionary hsl = new HandShapeDictionary(handshape_directory);

//		CanvasFrame view = new CanvasFrame(null, 0, 480, 480);
//		renderData(view, trainData, trainLabels);
				
		int NUM_CLASSES = hsl.numShapes();
		int CLASS_SIZE = 2000;
		int IMAGE_SIZE = HandShapeDictionary.PACKED_IMAGE_SIZE;
		int TRAINING_SIZE_PER_CLASS = (int)(0.85 * CLASS_SIZE);
		int VALIDATION_SIZE_PER_CLASS = (int)(0.05 * (CLASS_SIZE - TRAINING_SIZE_PER_CLASS));
		int TEST_SIZE_PER_CLASS = CLASS_SIZE - (TRAINING_SIZE_PER_CLASS + VALIDATION_SIZE_PER_CLASS);
		int TRAINING_SIZE = TRAINING_SIZE_PER_CLASS * NUM_CLASSES;
		int VALIDATION_SIZE = VALIDATION_SIZE_PER_CLASS * NUM_CLASSES;
		int TEST_SIZE = TEST_SIZE_PER_CLASS * NUM_CLASSES;
		trainData = new long[TRAINING_SIZE][IMAGE_SIZE];
		trainLabels = new int[TRAINING_SIZE];
		testData = new long[TEST_SIZE][IMAGE_SIZE];
		testLabels = new int[TEST_SIZE];
		validationData = new long[VALIDATION_SIZE][IMAGE_SIZE];
		validationLabels = new int[VALIDATION_SIZE];
		
		out("TOTAL IMAGES: " + (NUM_CLASSES * CLASS_SIZE));
		out("TRAINING SIZE: " + TRAINING_SIZE);
		out("VALIDATION SIZE: " + VALIDATION_SIZE);
		out("TEST SIZE: " + TEST_SIZE);

		int testIndex = 0;
		int validationIndex = 0;
		int trainIndex = 0;
		for (int k = 0; k < NUM_CLASSES; k++) {	
			int[] indices = randn(CLASS_SIZE);
			for (int i = 0; i < CLASS_SIZE; i++) {
				int idx = indices[i];
				long[][] shapeImages = hsl.getHandShapeFromIndex(k).getImagesArray();
				if (i < TRAINING_SIZE_PER_CLASS) {
					System.arraycopy(shapeImages[idx], 0, trainData[trainIndex], 0, IMAGE_SIZE);
					trainLabels[trainIndex] = k;
					trainIndex++;
				} else if (i < TRAINING_SIZE_PER_CLASS + VALIDATION_SIZE_PER_CLASS) {
					System.arraycopy(shapeImages[idx], 0, validationData[validationIndex], 0, IMAGE_SIZE);
					validationLabels[validationIndex] = k;
					validationIndex++;					
				} else {
					System.arraycopy(shapeImages[idx], 0, testData[testIndex], 0, IMAGE_SIZE);
					testLabels[testIndex] = k;
					testIndex++;
				}
			}
		}
		
		RFTrainer.NUM_CATEGORIES = NUM_CLASSES;
		RFTrainer.NUM_TREES = 25;
		RFTrainer.MAX_TREE_DEPTH = 20;
		RFTrainer.WEAKLEARNER_GENERATIONS = 100;
		RFTrainer rf = new RFTrainer(trainData, trainLabels, TRAINING_SIZE);
		RFTrainer.useVerbose = true;
		RFClassifier rfc = new RFClassifier();
		
		for (int g = 0; g < 1; g++) {
			out("Training new randomized forest #" + g);
			rf.train(); 
			out("Evaluating TRAINING SET...");
			double trainAccuracy = rf.evaluate(trainData, trainLabels, TRAINING_SIZE);
			out("Training Accuracy: " + trainAccuracy);
			
			out("Evaluating VALIDATION SET...");
			double validationAccuracy = rf.evaluate(validationData, validationLabels, VALIDATION_SIZE);
			out("Validation Accuracy: " + validationAccuracy);
			
			rf.saveToFile("rf_new.tree");
			
		}
		
		out("Evaluating TEST SET...");
		double testAccuracy = rf.evaluate(testData, testLabels, TEST_SIZE);
		out("Test Accuracy: " + testAccuracy);

		out("Evaluating saved tree on TEST SET...");
		rfc.readFromFile("rf_new.tree");		
		testAccuracy = rfc.evaluate(testData, testLabels, TEST_SIZE);
		out("Test Accuracy: " + testAccuracy);
		
	}
	
	
	public int[] randn(int n) {
		int[] a = new int[n];
		for (int i = 0; i < n; i++) a[i] = i;
		for (int i = 0; i < n - 1; i++) {
			int k = (int)(Math.random() * (n - i)) + i;
			int tmp = a[i];
			a[i] = a[k];
			a[k] = tmp;
		}
		return a;
	}
	
	public void out(String msg) {
		System.out.println(msg);
	}
	
	public static void main(String[] args) {

		HandShapeRFClassifier classifier = new HandShapeRFClassifier();
		
	}
}




