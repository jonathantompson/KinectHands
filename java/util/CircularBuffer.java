package util;

import java.util.Arrays;

public class CircularBuffer {
	int size = 30;
	double buffer[];
	int head = 0;
	int tail = 0;
	double sum = 0;
	public CircularBuffer(int size) {
		buffer = new double[size];
	}
	
	public void set(double val) {
		sum -= buffer[head];
		sum += val;
		buffer[head] = val;
		head++;
		if (head >= size) {
			head = 0;
		}
	}
	
	public double get() {
		return buffer[head];
	}
	
	public void fill(double val) {
		Arrays.fill(buffer, val);
		sum = val * size;
	}
	
	public void clear() {
		fill(0);
	}
	
	public double mean() {
		return sum / size;
	}
	
}
