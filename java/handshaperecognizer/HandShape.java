package handshaperecognizer;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;

public class HandShape {
	int length;
	long[][] images;
	int[][] imagesInt;
	public String name;
	double scores[];
	double bestScore;
	int bestExemplarIndex;
	int MAX_LENGTH = HandShapeDictionary.MAX_EXEMPLARS_PER_HANDSHAPE;
	int PACKED_IMAGE_SIZE = 64;
	int UNPACKED_IMAGE_SIZE = PACKED_IMAGE_SIZE * PACKED_IMAGE_SIZE;

	public HandShape() {
	}

	public HandShape(String newName, int packed_image_size) {
		PACKED_IMAGE_SIZE = packed_image_size;
		UNPACKED_IMAGE_SIZE = PACKED_IMAGE_SIZE * PACKED_IMAGE_SIZE;
		name = newName;
		allocate(0);
	}

	public void allocate(int num_images) {
		images = new long[MAX_LENGTH][PACKED_IMAGE_SIZE];
		imagesInt = new int[MAX_LENGTH][UNPACKED_IMAGE_SIZE];
		length = num_images;
		scores = new double[MAX_LENGTH];
	}

	public void deleteTiny(int i) {
		if (i >= 0 && i < MAX_LENGTH) {
			Arrays.fill(images[i], 0);
			Arrays.fill(imagesInt[i], 0);
		}
	}

	public long[][] getImagesArray() {
		return images;
	}

	public int[][] getImagesArrayInt() {
		return imagesInt;
	}

	public long[] getTinyImage(int i) {
		return images[i];
	}

	public int[] getTinyImageInt(int i) {
		return imagesInt[i];
	}


	public int getBestExemplarIndex() {
		return bestExemplarIndex;		
	}

	public double getBestScore() {
		return bestScore;
	}

	public int writeToFile() {
		String fileName = getRelativeFilename();
		return writeToFile(fileName);
	}

	public int writeToFile(String filename) {

		int success = 0;
		try {
			DataOutputStream out = new DataOutputStream(new BufferedOutputStream(
					new FileOutputStream(filename)));

			try {

				long[][] db = getImagesArray();
				out.writeInt(this.length);
				for (int i = 0; i < this.length; i++) {
					for (int j = 0; j < PACKED_IMAGE_SIZE; j++) {
						out.writeLong(db[i][j]);
					}
				}
				out.flush();

			} catch (IOException fioe) {
				out("error writing.");
				success = -1;
			}
		} catch (FileNotFoundException e) {
			out("Couldn't write to file named: " + filename);
			success = -1;
		}

		if (success == 0) {
			out("Wrote DB to file named " + filename);
		}

		return success;
	}

	public void readFromFile(File file, int MAX_IMAGES) {
		DataInputStream in = null;
		long[][] db = null;
		int[][] dbInt = null;
		try {
			in = new DataInputStream(new
					BufferedInputStream(new FileInputStream(file)));
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}

		if (in != null) {
			try {

				int num_images = Math.min(in.readInt(), MAX_IMAGES);
				int image_size = PACKED_IMAGE_SIZE;
				out("Got db with " + num_images + ", each of size " + image_size);

				//allocate(num_images, image_size);				
				allocate(HandShapeDictionary.MAX_EXEMPLARS_PER_HANDSHAPE);				
				db = getImagesArray();
				dbInt = getImagesArrayInt();

				for (int i = 0; i < num_images; i++) {
					for (int j = 0; j < image_size; j++) {
						db[i][j] = in.readLong();
					}
					HandShapeDictionary.unpackTiny(db[i], dbInt[i]);
				}

				this.name = getHandShapeNameFromFilename(file.getName());
				this.length = num_images;
				out("Read shape: " + this.name + " (length: " + this.length + ")");
			} catch (EOFException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}		
		}

		if (db != null) {
			out("Read DB from file named " + file.getName() + ", and extracted name is " + this.name);			
		}

	}	

	public String getRelativeFilename() {
		return "handshape_" + name + ".bin";
	}

	public String getHandShapeNameFromFilename(String filename) {		
		return filename.substring(10,11);
	}

	void out(String msg) {
		if (HandShapeDictionary.useVerboseLogging)
			System.out.println(msg);
	}

}