package rfclassifier;

public abstract class WeakLearner {

	public abstract boolean compare(long[] datum);
	public abstract boolean compare(int[] datum);
	public abstract void randomize();
	public abstract void save();
	public abstract void useSaved();
	public abstract String toString();
	
	public static void out(String msg) {
		System.out.println(msg);
	}
}
