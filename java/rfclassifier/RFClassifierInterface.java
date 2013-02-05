package rfclassifier;

import java.util.ArrayList;

public interface RFClassifierInterface {

	public ArrayList<Tree> trees = null;
	public static int MAX_MATCHES = 3000;
	public int[] matches = null;
	public int match(long[] datum);
	public int match(long[] datum, double[] distribution);

	public int readFromFile(String filename);

}
