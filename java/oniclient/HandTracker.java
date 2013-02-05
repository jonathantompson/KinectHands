package oniclient;

import render.*;
import java.nio.FloatBuffer;
import java.util.Arrays;

public class HandTracker {

	public static int LH = 0;
	public static int RH = 1;

	Object lock = new Object();

	static int NUM_FRAME_ELEMENTS = KinectClient.NUM_HAND_FRAME_ELEMENTS;

	float lhFrame[] = new float[NUM_FRAME_ELEMENTS];
	float rhFrame[] = new float[NUM_FRAME_ELEMENTS];
	float thFrame[] = new float[NUM_FRAME_ELEMENTS];

	static int SCALE_X =              0;
	static int SCALE_Y =              1;
	static int SCALE_Z =              2;
	static int AXIS0_X =              3;
	static int AXIS0_Y =              4;
	static int AXIS0_Z =              5;
	static int AXIS1_X =              6;
	static int AXIS1_Y =              7;
	static int AXIS1_Z =              8;
	static int AXIS2_X =              9;
	static int AXIS2_Y =              10;
	static int AXIS2_Z =              11;
	static int BOXCENTER_X =          12;
	static int BOXCENTER_Y =          13;
	static int BOXCENTER_Z =          14;
	static int HANDPOINTCLOUDCOM_X =  15;
	static int HANDPOINTCLOUDCOM_Y =  16;
	static int HANDPOINTCLOUDCOM_Z =  17;
	static int HIGHPOINT_X =          18;
	static int HIGHPOINT_Y =          19;
	static int HIGHPOINT_Z =          20;
	static int NUMPOINTS =            21;
	static int NUMFINGERS = 	  22;
	static int APPROXTHUMBANGLE =	  23;
	static int HANDPOINTCLOUDCOM_Z_ORIGINAL = 24;

			public boolean leftHandIsWaving;
	public boolean rightHandIsWaving;


	public HandTracker() {

	}

	//------------------------------------------------------
	// PUBLIC METHODS
	//------------------------------------------------------
	public boolean pullHandFrame(int hand, FloatBuffer fBuf) {
		float[] frame = getHandFrame(hand);
		System.arraycopy(frame, 0, thFrame, 0, frame.length);
		fBuf.get(frame, 0, NUM_FRAME_ELEMENTS);

		float sc = 25.4f;
		synchronized(lock) {
			
			frame[HANDPOINTCLOUDCOM_Z_ORIGINAL] = frame[HANDPOINTCLOUDCOM_Z];
			frame[SCALE_X] /= sc;
			frame[SCALE_Y] /= sc;
			frame[SCALE_Z] /= -sc;

			frame[AXIS0_Z] *= -1;
			frame[AXIS1_Z] *= -1;
			frame[AXIS2_Z] *= -1;

			frame[BOXCENTER_X] /= sc;
			frame[BOXCENTER_Y] /= sc;
			frame[BOXCENTER_Z] = KinectClient.FL - frame[BOXCENTER_Z] / sc;

			frame[HANDPOINTCLOUDCOM_X] /= sc;
			frame[HANDPOINTCLOUDCOM_Y] /= sc;
			frame[HANDPOINTCLOUDCOM_Z] = KinectClient.FL - frame[HANDPOINTCLOUDCOM_Z] / sc;

			frame[HIGHPOINT_X] /= sc;
			frame[HIGHPOINT_Y] /= sc;
			frame[HIGHPOINT_Z] = KinectClient.FL - frame[HIGHPOINT_Z] / sc;

			boolean changed = false;
			for (int i = 0; i < frame.length; i++) {
				if (frame[i] != thFrame[i]) {
					changed = true;
					break;
				}
			}

			return changed;
		}


	}

	public Matrix copyHandRotationMatrix(int hand, Matrix m) {

		m.identity();
		float[] frame = getHandFrame(hand);

		synchronized(lock) {
			m.set(0,0, frame[AXIS0_X]);
			m.set(1,0, frame[AXIS0_Y]);
			m.set(2,0, frame[AXIS0_Z]);

			m.set(0,1, frame[AXIS1_X]);
			m.set(1,1, frame[AXIS1_Y]);
			m.set(2,1, frame[AXIS1_Z]);

			m.set(0,2, frame[AXIS2_X]);
			m.set(1,2, frame[AXIS2_Y]);
			m.set(2,2, frame[AXIS2_Z]);
		}
		return m;
	}

	public Matrix copyHandMatrix(int hand, Matrix m) {
		m.identity();
		float[] frame = getHandFrame(hand);

		synchronized(lock) {
			m.set(0,0, frame[AXIS0_X]);
			m.set(1,0, frame[AXIS0_Y]);
			m.set(2,0, frame[AXIS0_Z]);

			m.set(0,1, frame[AXIS1_X]);
			m.set(1,1, frame[AXIS1_Y]);
			m.set(2,1, frame[AXIS1_Z]);

			m.set(0,2, frame[AXIS2_X]);
			m.set(1,2, frame[AXIS2_Y]);
			m.set(2,2, frame[AXIS2_Z]);

			m.set(0,3, frame[BOXCENTER_X]);
			m.set(1,3, frame[BOXCENTER_Y]);
			m.set(2,3, frame[BOXCENTER_Z]);
		}
		return m;
	}

	public Matrix copyHandMatrixScaled(int hand, Matrix m) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			return copyHandMatrix(hand, m).scale(frame[SCALE_X], frame[SCALE_Y], frame[SCALE_Z]);    
		}
	}

	public float[] copyFingerPosition(int hand, float[] xyz) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			xyz[0] = frame[HIGHPOINT_X];
			xyz[1] = frame[HIGHPOINT_Y];
			xyz[2] = frame[HIGHPOINT_Z];
		}
		return xyz;
	}

	public double[] copyFingerPosition(int hand, double[] xyz) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			xyz[0] = frame[HIGHPOINT_X];
			xyz[1] = frame[HIGHPOINT_Y];
			xyz[2] = frame[HIGHPOINT_Z];
		}
		return xyz;
	}


	public float[] copyHandCOM(int hand, float[] xyz) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			xyz[0] = frame[HANDPOINTCLOUDCOM_X];
			xyz[1] = frame[HANDPOINTCLOUDCOM_Y];
			xyz[2] = frame[HANDPOINTCLOUDCOM_Z];
		}
		return xyz;
	}

	public float[] copyHandBoxCenter(int hand, float[] xyz) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			xyz[0] = frame[BOXCENTER_X];
			xyz[1] = frame[BOXCENTER_Y];
			xyz[2] = frame[BOXCENTER_Z];
		}
		return xyz;
	}

	public float[] copyHandScale(int hand, float[] xyz) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			xyz[0] = frame[SCALE_X];
			xyz[1] = frame[SCALE_Y];
			xyz[2] = frame[SCALE_Z];
		}
		return xyz;
	}

	public double[] copyHandScale(int hand, double[] xyz) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			xyz[0] = frame[SCALE_X];
			xyz[1] = frame[SCALE_Y];
			xyz[2] = frame[SCALE_Z];
		}
		return xyz;
	}

	public double[] copyLongestAxis(int hand, double[] xyz) {

		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			int maxi = getMaxAxisIndex(hand);

			switch (maxi) {
			case 0:
				xyz[0] = frame[AXIS0_X];
				xyz[1] = frame[AXIS0_Y];
				xyz[2] = frame[AXIS0_Z];
				break;    	    		
			case 1:
				xyz[0] = frame[AXIS1_X];
				xyz[1] = frame[AXIS1_Y];
				xyz[2] = frame[AXIS1_Z];
				break;    	    		
			case 2:
				xyz[0] = frame[AXIS2_X];
				xyz[1] = frame[AXIS2_Y];
				xyz[2] = frame[AXIS2_Z];
				break;    	    		
			}
		}
		return xyz;

	}

	private int getMaxAxisIndex(int hand) {
		float[] frame = getHandFrame(hand);
		int maxi = 0;
		float maxs = Math.abs(frame[SCALE_X]);
		if (Math.abs(frame[SCALE_Y]) > maxs) {
			maxs = Math.abs(frame[SCALE_Y]);
			maxi = 1;
		}
		if (Math.abs(frame[SCALE_Z]) > maxs) {
			maxs = Math.abs(frame[SCALE_Z]);
			maxi = 2;
		}
		return maxi;
	}

	public int numPointsTrackedOnHand(int hand) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			return (int)frame[NUMPOINTS];
		}
	}

	// returns the original hand depth in mm
	// differs from HANDPOINTCLOUDCOM_Z in that it
	// wasn't transformed for display in renderer
	public int handDepth(int hand) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			return (int)frame[HANDPOINTCLOUDCOM_Z_ORIGINAL];
		}
	}

	public double percentBBoxTracked(int hand) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			double sx = frame[SCALE_X];
			double sy = frame[SCALE_Y];
			double sz = frame[SCALE_Z];
			double volume = Math.abs(8 * sx * sy * sz);
			if (volume == 0) {
				return 0;
			}
			return NUMPOINTS / volume;
		}
	}

	public int numFingersOnHand(int hand) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			return (int)frame[NUMFINGERS];
		}
	}
	
	public double thumbAngle(int hand) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			return frame[APPROXTHUMBANGLE];
		}
	}
	
	public boolean thumbOut(int hand) {
		return thumbAngle(hand) > 0.5;		
	}
	
	public boolean thumbIn(int hand) {
		return !thumbOut(hand);
	}

	public boolean isHandClenched(int hand) {
		return percentBBoxTracked(hand) > 0.4;
	}

	public boolean leftHandIsTracked() {
		synchronized(lock) {
			return handVolume(LH) > 0 && lhFrame[NUMPOINTS] > 10;    
		}
	}

	public boolean rightHandIsTracked() {
		synchronized(lock) {
			return handVolume(RH) > 0 && rhFrame[NUMPOINTS] > 10;    
		}
	}

	public double handVolume(int hand) {
		float[] frame = getHandFrame(hand);
		synchronized(lock) {
			double sx = frame[SCALE_X];
			double sy = frame[SCALE_Y];
			double sz = frame[SCALE_Z];
			double volume = Math.abs(8 * sx * sy * sz);
			return volume;
		}
	}

	public float[] getHandFrame(int hand) {
		if (hand == LH) 
			return lhFrame;

		return rhFrame;
	}

	//------------------------------------------------------
	// PRIVATE METHODS
	//------------------------------------------------------

	private void out(String msg) {
		System.out.println(msg);
	}



}
