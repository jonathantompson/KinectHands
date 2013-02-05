package oniclient;

import org.zeromq.ZMQ;

import com.sun.jna.Pointer;

import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.StringTokenizer;
import java.nio.FloatBuffer;
import java.nio.ByteBuffer;
import util.CircularBuffer;

public class KinectClient implements Runnable {
	ZMQ.Context context;
	ZMQ.Socket subscriber;

	String filter;
	public static int FL = 30;
	public static double FOV = 1.079378020917875;
	public static int NUM_HAND_FRAME_ELEMENTS = 30;
	static int NUM_RGB_PIXELS = 640 * 480;
	static int NUM_DEPTH_PIXELS = 640 * 480;
	static int NUM_MASK_PIXELS = 640 * 480;
	public int[] rgb = new int[NUM_RGB_PIXELS];
	public int[] depth = new int[NUM_DEPTH_PIXELS];
	public int[] mask = new int[NUM_MASK_PIXELS];
	HandTracker ht = new HandTracker();

	//------------------------------//
	// Gesture Related Variables
	//------------------------------//	
	ArrayList listenerList;
	FloatBuffer fBuf;
	CircularBuffer[] handWaveMovingAverage = new CircularBuffer[2];
	CircularBuffer[] topFingerMovingAverage = new CircularBuffer[2];
	double[][] longestAxis = new double[2][3];
	double[][] wasLongestAxis = new double[2][3];
	int MOVING_AVERAGE_SIZE = 100;
	//------------------------------//

	public void init() {
		context = ZMQ.context(1);

		//  Socket to talk to server
		System.out.println("Starting Java Kinect Client…");
		subscriber = context.socket(ZMQ.SUB);
		subscriber.connect("tcp://localhost:5557");

		//  Subscribe to zipcode, default is NYC, 10001
		subscriber.subscribe("".getBytes());

	}

	public HandTracker getHandTracker() {
		return ht;
	}

	public KinectClient() {
		listenerList = new ArrayList();
		handWaveMovingAverage[0] = new CircularBuffer(MOVING_AVERAGE_SIZE);
		handWaveMovingAverage[1] = new CircularBuffer(MOVING_AVERAGE_SIZE);
		topFingerMovingAverage[0] = new CircularBuffer(MOVING_AVERAGE_SIZE);
		topFingerMovingAverage[1] = new CircularBuffer(MOVING_AVERAGE_SIZE);
	}

	//------------------------------------------------------
	// LISTENER METHODS	
	//------------------------------------------------------
	public void addHandTrackerListener(HandTrackerListener l) 
	{
		listenerList.add(l);
	}

	public void removeHandTrackerListener(HandTrackerListener l) 
	{
		listenerList.remove(l);
	}
	
	public void clearHandTrackerListeners() 
	{
		synchronized(listenerList) {
			listenerList.clear();
		}
	}


	protected void fireHandTrackerEvent() 
	{
		HandTrackerEvent event = new HandTrackerEvent(this, ht);
		Iterator i = listenerList.iterator();
		while(i.hasNext())  {
			((HandTrackerListener)i.next()).handleHandTrackerEvent(event);
		}

	}

	public void run() {

		init();

		while (true) {
			try {
				boolean leftChanged = false;
				boolean rightChanged = false;
				byte[] key = subscriber.recv(0);
				byte[] frame = subscriber.recv(0);
				if (key[0] == 'v') {
					synchronized(rgb) {
						for (int i = 0; i < NUM_RGB_PIXELS; i++) {
							int r = frame[3*i+0] & 255;
							int g = frame[3*i+1] & 255;
							int b = frame[3*i+2] & 255;
							rgb[i] = 0xff000000 | r << 16 | g << 8 | b;
						}
					}
				} else if (key[0] == 'd') {
					synchronized(depth) {
						for (int i = 0; i < NUM_DEPTH_PIXELS; i++) {
							byte lo = frame[2*i + 0];
							byte hi = frame[2*i + 1];
							short value = (short)(hi << 8 | lo);
							//depth[i] = 0xff000000 | value;
							depth[i] = value;
						}
					}
				} else if (key[0] == 'm') {
					synchronized(mask) {
						for (int i = 0; i < NUM_MASK_PIXELS; i++) {
							mask[i] = frame[i];
						}
					}					
				} else if (key[0] == 'l') {
					leftChanged = pullHandFrame(HandTracker.LH, frame);
				} else if (key[0] == 'r') {
					rightChanged = pullHandFrame(HandTracker.RH, frame);						
				}
				if (leftChanged || rightChanged) {
					fireHandTrackerEvent();
				}
				try {
				    Thread.yield();
				} catch (Exception e) {
				    
				}
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	public void copyRGB(int[] dst) {
		synchronized(rgb) {
			System.arraycopy(rgb, 0, dst, 0, 640 * 480);
		}
	}

	public void copyDepth(int[] dst) {
		synchronized(depth) {
			System.arraycopy(depth, 0, dst, 0, 640 * 480);
		}
	}
	
	public void copyDepthAsRGBA(int[] dst) {
		synchronized(depth) {
			for (int i = 0; i < 640 * 480; i++) {
				dst[i] = 0xff000000 | depth[i];
			}
		}
		
	}

	public void copyMask(int[] dst) {
		synchronized(dst) {
			System.arraycopy(mask, 0, dst, 0, 640 * 480);
		}
	}

	//-------------------------------------------------------//
	// PRIVATE METHODS
	//-------------------------------------------------------//
	double[] xyz = new double[3];
	private boolean pullHandFrame(int hand, byte[] frame) {
		boolean changed = false;
		fBuf = ByteBuffer.wrap(frame,0,4*NUM_HAND_FRAME_ELEMENTS).order(ByteOrder.nativeOrder()).asFloatBuffer();
		changed = ht.pullHandFrame(hand, fBuf);
		if (changed) {
			System.arraycopy(longestAxis[hand], 0, wasLongestAxis[hand], 0, 3);
			ht.copyLongestAxis(hand, longestAxis[hand]);
			ht.copyFingerPosition(hand, xyz);
			topFingerMovingAverage[hand].set(xyz[1]);
			double diff = dist(longestAxis[hand], wasLongestAxis[hand]);
			handWaveMovingAverage[hand].set(diff);
			if (handWaveMovingAverage[hand].mean() > 0.25 && topFingerMovingAverage[hand].mean() > -3) {
				handWaveMovingAverage[hand].fill(Double.MIN_VALUE);
				if (hand == HandTracker.RH) {
					out("right hand waved.");
					ht.rightHandIsWaving = true;
				} else if (hand == HandTracker.LH) {
					out("left hand waved.");
					ht.leftHandIsWaving = true;
				}		
			} else {
				if (hand == HandTracker.RH) {
					ht.rightHandIsWaving = false;
				} else if (hand == HandTracker.LH) {
					ht.leftHandIsWaving = false;
				}
			}
		}
		return changed;
	}

	private double dist(double[] a, double[] b) {
		double dx = a[0] - b[0];
		double dy = a[1] - b[1];
		double dz = a[2] - b[2];
		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}

	private double norm(double[] a) {
		return Math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	}

	private double norm(double a, double b, double c) {
		return Math.sqrt(a*a + b*b + c*c);
	}

	private void out(String s) {
		System.out.println(s);
	}

	//-------------------------------------------------------//
	// STATIC METHODS
	//-------------------------------------------------------//
	public static void main(String[] args) {
		Thread t = new Thread(new KinectClient());
		t.start();		
	}

}
