package app;

import java.awt.event.*;
import java.util.Arrays;

import oniclient.*;
import canvasframe.*;
import handshaperecognizer.HandShape;
import handshaperecognizer.HandShapeDictionary;
import handshaperecognizer.Matcher;
import handshaperecognizer.ImageResizer;
import imageprocessor.*;
import java.awt.Graphics2D;
import java.awt.Color;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.EOFException;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

public class GestureTracker 
implements  Runnable, 
WindowListener, 
KeyListener, 
MouseListener, 
MouseMotionListener,
HandTrackerListener
{

	String handshape_directory = "handshapes";
	HandShapeDictionary hsl = new HandShapeDictionary(handshape_directory);

	int tiny_image_unit = hsl.TINY_IMAGE_UNIT;
	int selectedExemplar = 0;

	int dictionary_window_width = 640;
	int dictionary_window_height = 640;
	int tinyarray_rows = (int)Math.ceil((double)dictionary_window_height / (double)tiny_image_unit);
	int tinyarray_cols = (int)Math.ceil((double)dictionary_window_width / (double)tiny_image_unit);

	int MAX_IMAGES_PER_PAGE = tinyarray_rows * tinyarray_cols;
	int MAX_IMAGES_PER_HANDSHAPE = hsl.MAX_EXEMPLARS_PER_HANDSHAPE;
	int MAX_PAGES = MAX_IMAGES_PER_HANDSHAPE / MAX_IMAGES_PER_PAGE;
	
	CanvasFrame kinectView;
	CanvasFrame train;
	CanvasFrame tinyview;
	CanvasFrame sandbox;
	
	int[] pix = new int[640 * 480];
	int[] depth = new int[640 * 480];
	int[] mask = new int [640 * 480];
	int[] tmp = new int [dictionary_window_width * dictionary_window_height];
	int[] tmp2 = new int[320 * 240];
	int[] imageTiles = new int[dictionary_window_width * dictionary_window_height];

	KinectClient zmq;
	boolean showdepth = false;	
	boolean showvideo = true;
	boolean showmask = false;
	boolean saveshape = false;

	String selectedShape = "a";
	int page = 0;

	
	int[][] tinyArray = new int[MAX_IMAGES_PER_PAGE][tiny_image_unit * tiny_image_unit];
	int[] tinyHand = new int[tiny_image_unit * tiny_image_unit];

	enum Mode {EDIT, RECOGNIZE};
	Mode mode = Mode.EDIT;
	enum CaptureMode {CAPTURE_OFF, CAPTURE_CONTINUOUSLY, CAPTURE_ONCE, CAPTURE_CAUTIOUSLY};
	CaptureMode captureMode = CaptureMode.CAPTURE_OFF;
	
	int mouseX;
	int mouseY;
	boolean mouseDown = false;

	boolean toggleCaptureHand = false;
	
	public void run() {
		
		HandShapeDictionary.useVerboseLogging = true;
		hsl.printHandShapeNames();
		
		kinectView = new CanvasFrame(this, 0, 640, 480);
		train = new CanvasFrame(this, 0, dictionary_window_width, dictionary_window_height);
		train.setLocation(640, 0);
		tinyview = new CanvasFrame(this, 0, tiny_image_unit, tiny_image_unit);
		tinyview.setLocation(640, dictionary_window_height + 10);
//		sandbox = new CanvasFrame(this, 0, 320, 240);
//		sandbox.setLocation(640, 1000);
		
		zmq = new KinectClient();
		Thread t = new Thread(zmq);
		t.start();
	    zmq.addHandTrackerListener(hsl);
		while (true) {
			try {

				if (saveshape) {
					hsl.writeShapeToFile(selectedShape, handshape_directory);
					saveshape = false;
				}

				if (showdepth) {
					zmq.copyDepthAsRGBA(pix);    								
				} else if (showmask) {
					zmq.copyMask(pix);
				} else {
					zmq.copyRGB(pix);
				}

				zmq.copyDepth(depth);
				zmq.copyMask(mask);
				
				kinectView.copy(pix);

				// show live feed
				if (toggleCaptureHand) {
					hsl.useLeftHandForCapture = !hsl.useLeftHandForCapture;
					toggleCaptureHand = false;
				}
				int hand = HandTracker.LH;
				if (!hsl.useLeftHandForCapture) {
					hand = HandTracker.RH;
				}
				
				hsl.makeTiny(mask, hand, tinyHand);

				if (mode == Mode.EDIT) {
					if (!hsl.isZero(tinyHand))
						if (captureMode == CaptureMode.CAPTURE_CONTINUOUSLY || captureMode == CaptureMode.CAPTURE_ONCE) {
							boolean wasAdded = hsl.captureTiny(selectedShape, tinyHand, selectedExemplar);
							if (selectedExemplar == MAX_IMAGES_PER_HANDSHAPE - 1) {
								captureMode = CaptureMode.CAPTURE_OFF;
							}
							if (wasAdded)
								selectExemplar(selectedExemplar+1);
							if (captureMode == CaptureMode.CAPTURE_ONCE) 
								captureMode = CaptureMode.CAPTURE_OFF;
						} else if (captureMode == CaptureMode.CAPTURE_CAUTIOUSLY) {
							Matcher match = hsl.simpleMatch(tinyHand, hand);
							int label = hsl.lookupIndexFromLetter(selectedShape);
							double[] scores = hsl.getScores();
							double best_in_class = scores[label];
							out("best in class: " + best_in_class);
							if (match.bestShape != null && best_in_class < .99) {
								boolean wasAdded = hsl.captureTiny(selectedShape, tinyHand, selectedExemplar);
								if (wasAdded)
									selectExemplar(selectedExemplar+1);							
							} else {
								out("bestshape is null");
							}

							//							int k = 5;
//							boolean allTopK = true;
//							int[] indices = hsl.allSortedIndices;
//							for (int i = 0; i < 10; i++) {
//								if (hsl.allLabels[indices[i]] != match.bestshape_index) {
//									allTopK = false;
//									break;
//								}
//							}
//							if (match.bestshape != null) {
////								if (!match.bestshape.name.equals(selectedShape)) {
//								if (!allTopK) {
//									hsl.captureTiny(selectedShape, tinyHand, selectedExemplar);
//									selectExemplar(selectedExemplar+1);							
//								}
//							} else {
//								out("bestshape is null");
//							}
						}
						if (mouseDown) {
							int tx = (mouseX - tiny_image_unit / 2) / tiny_image_unit;
							int ty = (mouseY - tiny_image_unit / 2) / tiny_image_unit;
							int idx = page * MAX_IMAGES_PER_PAGE + tx + ty * (tinyarray_cols);
							selectExemplar(idx);
	 					}
						tinyArray = hsl.getShape(selectedShape).getImagesArrayInt();
						renderTinyArray(tinyArray);
				} else if (mode == Mode.RECOGNIZE) {
					hsl.matchRF(tinyHand, hand);					
					//hsl.scoreAllFastButCareful(tinyHand);
					hsl.unpackManySorted(tinyArray, 0, MAX_IMAGES_PER_PAGE);
					renderTinyArrayFromIndices(tinyArray, hsl.allSortedIndices, hsl.allLabels);
//					Matcher matcher = hsl.match(tinyHand);
//					if (matcher.bestshape != null) {
//						selectShape(matcher.bestshape.name);
//						selectExemplar(matcher.bestexamp);
//						renderTinyArray(hsl.getShape(selectedShape).getImagesArray());						
//						double[] scores = hsl.getScores();
//						String s = "scores: ";
//						for (int i = 0; i < hsl.numShapes(); i++) {
//							s += (scores[i]) + "\t\t";
//						}
//						//out(s);
//					}
				}

				// show most recent captured tinyhand
				for (int i = 0; i < tinyHand.length; i++) {
					int v = tinyHand[i] & 255;
					tinyHand[i] = 0xff000000 | v << 16 | v << 8 | v;
				}
 
				tinyview.copy(tinyHand);
				tinyview.update();
		
				Graphics2D viewg = kinectView.getGraphics2D();
				
				viewg.setColor(Color.red);
				int x1 = hsl.handScreenBounds[hand][0];
				int x2 = hsl.handScreenBounds[hand][1];
				int y1 = hsl.handScreenBounds[hand][2];
				int y2 = hsl.handScreenBounds[hand][3];
				viewg.drawRect(x1, y1, x2 - x1, y2 - y1);
				kinectView.update();
				Thread.sleep(30);

			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	public void selectShape(String name) {
		if (!(selectedShape.equals(name))) {
			HandShape query = hsl.getShape(name);
			if (query != null) {
				selectedShape = name;
				out("Selected shape " + name);
			} else {
				out("Shape named " + name + " doesn't exist in database.");
			}
		}
	}
	
	public void renderTinyArrayFromIndices(int[][] tinyArray, int[] indices, int[] labels) {
		// show tinyarray as tiled image
		Arrays.fill(train.pix, 0xff000000);
		int left = 0;
		int right = MAX_IMAGES_PER_PAGE;
		int selectedShapeIndex = hsl.lookupIndexFromLetter(selectedShape);
		int inTopK = 0;
		int topK = 1;
		for (int i = left; i < topK; i++) {
			if (labels[indices[i]] == selectedShapeIndex) inTopK++;
		}
//		if (inTop10 < topK)
//			out(hsl.allScores[indices[0]] + "\t MISCLASSIFIED");
//		else 
//			out(hsl.allScores[indices[0]] + "");
		//inTop10 = labels[indices[0]] == selectedShapeIndex ? 1 : 0;
		for (int i = left, t = 0; i <= right ; i++, t++) {
			int idx = i; // indices[i];
			int y = (t / tinyarray_rows) * tiny_image_unit;
			int x = (t % tinyarray_rows) * tiny_image_unit;
//			pasteRectIntoAnother(tinyArray[idx], tiny_image_unit, train.pix, dictionary_window_width, x, y);
			if (labels[idx] != selectedShapeIndex) {
				if (inTopK < topK) {
					copyAndColorizeRect(tinyArray[idx], tiny_image_unit, train.pix, dictionary_window_width, x, y, 1.0, 1.0, 0.0, 0.0);
				} else {
					copyAndColorizeRect(tinyArray[idx], tiny_image_unit, train.pix, dictionary_window_width, x, y, 1.0, 1.0, 1.0, 0.0);					
				}
			} else {
				copyAndColorizeRect(tinyArray[idx], tiny_image_unit, train.pix, dictionary_window_width, x, y, 1.0, 1.0, 1.0, 1.0);
			}
//			copyAndColorizeRect(tinyArray[idx], tiny_image_unit, train.pix, dictionary_window_width, x, y, 1.0, 1.0, 1.0, 1.0);
		}
		//byte2ARGB(train.pix);
		train.flush();
		train.update();		
	}
	
	public void renderTinyArray(int[][] tinyArray) {
		// show tinyarray as tiled image
		int w = dictionary_window_width;
		int h = dictionary_window_height;
		Arrays.fill(train.pix, 0xff000000);
		tileTinyArrayPage(tinyArray, page, train.pix);
		train.flush();
		Graphics2D traing = train.getGraphics2D();
		
		int col = (selectedExemplar - (page * MAX_IMAGES_PER_PAGE)) % tinyarray_rows;
		int row = (selectedExemplar - (page * MAX_IMAGES_PER_PAGE)) / tinyarray_rows;
		int x = col * tiny_image_unit;
		int y = row * tiny_image_unit;
		traing.setColor(Color.green);
		traing.drawRect(x,y, tiny_image_unit, tiny_image_unit);
		traing.drawRect(x+2,y+2, tiny_image_unit-4, tiny_image_unit-4);
		traing.drawLine(0, y + tiny_image_unit / 2, x, y + tiny_image_unit / 2);
		traing.drawLine(x + tiny_image_unit, y + tiny_image_unit / 2, w, y + tiny_image_unit / 2);
		traing.drawLine(x + tiny_image_unit / 2, 0, x + tiny_image_unit / 2, y);
		traing.drawLine(x + tiny_image_unit / 2, y + tiny_image_unit, x + tiny_image_unit / 2, h);
		traing.setColor(Color.yellow);
		
		traing.setColor(Color.orange);
		traing.drawString("" + page, w - 75, 75);
		train.update();		
	}

	public void tileTinyArrayPage(int[][] tinyarray, int page, int[] dst) {
		int first_index = page * MAX_IMAGES_PER_PAGE;
		for (int i = first_index, t = 0; i < first_index + MAX_IMAGES_PER_PAGE; i++, t++) {
			int y = (t / tinyarray_rows) * tiny_image_unit;
			int x = (t % tinyarray_rows) * tiny_image_unit;
			pasteRectIntoAnother(tinyarray[i], tiny_image_unit, dst, dictionary_window_width, x, y);
		}
		byte2ARGB(dst);
	}
	
	public void knockoutPage() {
		int firstIndex = page * MAX_IMAGES_PER_PAGE;
		HandShape s = hsl.getShape(selectedShape);
		long[][] tinyImages = s.getImagesArray();
		int[][] tinyImagesInt = s.getImagesArrayInt();
		for (int i = firstIndex; i < firstIndex + MAX_IMAGES_PER_PAGE; i++) {
			Arrays.fill(tinyImages[i], 0);
			Arrays.fill(tinyImagesInt[i], 0);
		}
		hsl.needToCollectImages = true;
	}
		
	public void tileTinyArrayFromIndices(int[][] tinyarray, int[] indices, int left, int right, int[] dst) {
		for (int i = left, t = 0; i <= right ; i++, t++) {
			int idx = indices[i];
			int y = (t / tinyarray_rows) * tiny_image_unit;
			int x = (t % tinyarray_rows) * tiny_image_unit;
			pasteRectIntoAnother(tinyarray[idx], tiny_image_unit, dst, dictionary_window_width, x, y);
		}
		byte2ARGB(dst);
	}

	
	public void pasteRectIntoAnother(int[] im, int im_stride, int[] dst, int dst_stride, int cornerX, int cornerY) {		
		int im_width = im_stride;
		int im_height = im.length / im_width;
		int dst_width = dst_stride;
		int dst_height = dst.length / dst_width;
		for (int y = 0; y < im_height; y++) {
			for (int x = 0; x < im_width; x++) {
				int dx = cornerX + x;
				int dy = cornerY + y;
				if (dx >= 0 && dx <= dst_width && dy >= 0 && dy < dst_height) {
					int i = x + y * im_stride;
					int di = dx + dy * dst_stride;
					dst[di] = im[i];
				}
			}
		}
	}
	
	public void copyAndColorizeRect(int[] im, int im_stride, int[] dst, int dst_stride, int cornerX, int cornerY, double alpha, double red, double green, double blue) {
		int im_width = im_stride;
		int im_height = im.length / im_width;
		int dst_width = dst_stride;
		int dst_height = dst.length / dst_width;
		double a = Math.max(0, Math.min(255, alpha));
		double r = Math.max(0, Math.min(1.0, red));
		double g = Math.max(0, Math.min(1.0, green));
		double b = Math.max(0, Math.min(1.0, blue));
		for (int y = 0; y < im_height; y++) {
			for (int x = 0; x < im_width; x++) {
				int dx = cornerX + x;
				int dy = cornerY + y;
				if (dx >= 0 && dx <= dst_width && dy >= 0 && dy < dst_height) {
					int i = x + y * im_stride;
					int di = dx + dy * dst_stride;
					int val = (Math.min(255, Math.max(0, im[i]))) > 128 ? 255 : 0;
					
					dst[di] = 0xff000000 	| (((int)(r * val)) & 255) << 16
											| (((int)(g * val)) & 255) << 8
											| (((int)(b * val)) & 255);
				}
			}
		}
	}
	
	public void byte2ARGB(int[] im, int stride, int x, int y, int width, int height) {
		width = Math.min(stride, width);
		height = Math.min(im.length / stride, height);
		for (int i = Math.max(0, x); i < x + width; i++) {
			for (int j = Math.max(0, y); j < y + height; j++) {
				int idx = i + j * stride;
//				int v = im[idx] & 255;
//				im[idx] = 0xff000000 | v << 16 | v << 8 | v;
				im[idx] = im[idx] > 128 ? 0xffffffff : 0xff000000;
			}
		}
	}
	
	public void byte2ARGB(int[] im) {
		for (int i = 0; i < im.length; i++) {
//			int v = im[i] & 255;
//			im[i] = 0xff000000 | v << 16 | v << 8 | v;
			im[i] = im[i] > 128 ? 0xffffffff : 0xff000000;
		}
	}	
	
	public void pageUp() {
		int next = MAX_IMAGES_PER_PAGE * (page - 1);
		if (next < 0) next = MAX_IMAGES_PER_PAGE * (MAX_PAGES - 1);
		selectExemplar(next);
	}

	public void pageDown() {
		int next = MAX_IMAGES_PER_PAGE * (page + 1);
		if (next > MAX_IMAGES_PER_PAGE * (MAX_PAGES - 1)) next = 0;
		selectExemplar(next);
	}

	public void selectExemplar(int n) {
		//out("trying to select: " + n);
		if (n < MAX_IMAGES_PER_HANDSHAPE) {
			page = n / MAX_IMAGES_PER_PAGE;
			selectedExemplar = n;
		} else {
			//out("but couldn't");
		}
	}
	
	
	void resetSelectedExemplar() {
		selectExemplar(0);
	}



	public void out(String s) {
		System.out.println(s);

	}

	public static void main(String[] args) {
		GestureTracker qtt = new GestureTracker();
		qtt.run();    
	}
	/********************************************
	 HAND TRACKER EVENTS
	 *********************************************/
	float[] xyz = new float[3];
	public void handleHandTrackerEvent(HandTrackerEvent e) {		
		e.getTracker().copyFingerPosition(HandTracker.LH, xyz);
		out("xyz: " + xyz[0] + " " + xyz[1] + " " + xyz[2]);
	}


	/********************************************
	 AWT EVENTS
	 *********************************************/

	public void windowClosing(WindowEvent e) {
	}

	public void windowDeactivated(WindowEvent e) {}

	public void windowActivated(WindowEvent e) {}

	public void windowIconified(WindowEvent e) {}

	public void windowDeiconified(WindowEvent e) {}

	public void windowOpened(WindowEvent e) {}

	public void windowClosed(WindowEvent e) {}

	public void mouseDragged(MouseEvent arg0) {
		mouseDown = true;
		mouseX = arg0.getX();
		mouseY = arg0.getY();		
	}

	public void mouseMoved(MouseEvent arg0) {
		mouseX = arg0.getX();
		mouseY = arg0.getY();
	}

	public void mouseClicked(MouseEvent arg0) {}

	public void mouseEntered(MouseEvent arg0) {}

	public void mouseExited(MouseEvent arg0) {}

	public void mousePressed(MouseEvent arg0) {
		mouseDown = true;
	}

	public void mouseReleased(MouseEvent arg0) {
		mouseDown = false;
	}

	public void keyPressed(KeyEvent arg0) {
	    int ctrlmask = KeyEvent.CTRL_DOWN_MASK;
	    if ((arg0.getModifiersEx() & (ctrlmask)) == ctrlmask) {
			switch (arg0.getKeyCode()) {
			case KeyEvent.VK_D: 
				showdepth = !showdepth;    	  
				break;
			case KeyEvent.VK_V:
				showvideo = !showvideo;
			case KeyEvent.VK_M:
				showmask = !showmask;
				break;
			case KeyEvent.VK_C:
				if (mode == Mode.EDIT) {
					if (captureMode == CaptureMode.CAPTURE_CONTINUOUSLY)
						captureMode = CaptureMode.CAPTURE_OFF;
					else
						captureMode = CaptureMode.CAPTURE_CONTINUOUSLY;
				}
				break;
			case KeyEvent.VK_Y:
				if (mode == Mode.EDIT) {
					if (captureMode == CaptureMode.CAPTURE_CAUTIOUSLY)
						captureMode = CaptureMode.CAPTURE_OFF;
					else
						captureMode = CaptureMode.CAPTURE_CAUTIOUSLY;
				}
				break;
			case KeyEvent.VK_T:
				if (mode == Mode.EDIT) {
					if (captureMode == CaptureMode.CAPTURE_ONCE)
						captureMode = CaptureMode.CAPTURE_OFF;
					else
						captureMode = CaptureMode.CAPTURE_ONCE;
				}				
				break;
			case KeyEvent.VK_S:
				saveshape = true;
				break;
			case KeyEvent.VK_E:
				mode = Mode.EDIT;
				out("Changed to mode: EDIT");
				break;
			case KeyEvent.VK_R:
				mode = Mode.RECOGNIZE;
				hsl.needToCollectImages = true;
				out("Changed to mode: RECOGNIZE");
				break;
			case KeyEvent.VK_L:
				toggleCaptureHand = true;
				break;
			case KeyEvent.VK_N:
				String newName = hsl.createNewShape();
				if (!newName.equals(null)) {
					selectShape(newName);
				}
				break;
			case KeyEvent.VK_K:
				if (mode == Mode.EDIT) {
					knockoutPage();
				}
				break;			
			case KeyEvent.VK_B:
				HandShape s = hsl.getShape(selectedShape);
				hsl.cleanupBlacks(s);
				break;
			case KeyEvent.VK_X:
				hsl.removeDuplicates(selectedShape);
				break;
			default: break;
			}
	    } else {
	    	//out("keycode: " + arg0.getKeyCode());
	    	int keycode = arg0.getKeyCode();
	    	if (keycode == KeyEvent.VK_BACK_SPACE || keycode == KeyEvent.VK_DELETE) {
	    		hsl.deleteTiny(selectedShape, selectedExemplar);
	    	}
	    	if (keycode == 38) {
				pageUp();
	    	}
	    	if (keycode == 40) {
	    		pageDown();
	    	}
	    	if (keycode >= 65 && keycode <= 90) {
	    		String c = String.valueOf(arg0.getKeyChar());
				selectShape(c);
	    	}
	    }
	}            

	public void keyReleased(KeyEvent arg0) {}

	public void keyTyped(KeyEvent arg0) {}

}

