package canvasframe;

import java.awt.*;
import javax.swing.JFrame;
import javax.swing.JLabel;
import java.awt.image.*;
import java.awt.event.*;
import java.util.Arrays;

public class CanvasFrame extends JFrame implements MouseListener, MouseMotionListener {
	
	public int w = 640;
	public int h = 480;
	public int N = w * h;
	
	MemoryImageSource mmi;
	Image im;
	public int[] pix;
	
	BufferStrategy bs;
	Graphics2D g;
	public GraphicsDevice myGraphicsDevice;
	Insets myInsets;

	public boolean display_fps = false;
	int frameCount;
	long referenceTime;
	double fps;
	
	static public enum IMAGE_COLOR_TYPE {RGBA, RGB, GRAY};
	static public enum IMAGE_COLOR_POST_PROCESSING {NONE, RESCALE};
	
	public IMAGE_COLOR_TYPE image_color_type = IMAGE_COLOR_TYPE.RGBA;
	public IMAGE_COLOR_POST_PROCESSING image_color_post_processing = IMAGE_COLOR_POST_PROCESSING.NONE;
	
	public CanvasFrame() {
		this(null);		
	}
	
	public CanvasFrame(Object caller) {
		this(caller, 0, 640, 480, false);		
	}

	public CanvasFrame(Object caller, int screen, int width, int height) {
		this(caller, screen, width, height, false);
	}
	
	
	public CanvasFrame(Object caller, int screen, int width, int height, boolean hideTitlebar) {
		
		//out("Creating new Canvas Frame on screen " + screen);
		
		w = width;
		h = height;
		
		setUndecorated(hideTitlebar);
		GraphicsEnvironment ge = GraphicsEnvironment.getLocalGraphicsEnvironment();
		GraphicsDevice[] gds = ge.getScreenDevices();
		
		if (screen >= gds.length) {
			out("Warning.  Screen number " + screen + " not available.  Placing window on screen 0 instead.");
			screen = 0;
		}
		
		
		myGraphicsDevice = gds[screen];
		GraphicsConfiguration gc = myGraphicsDevice.getDefaultConfiguration();
		
		setBounds(gc.getBounds());
        //setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        JLabel emptyLabel = new JLabel("");
        emptyLabel.setPreferredSize(new Dimension(w, h));
        getContentPane().add(emptyLabel, BorderLayout.NORTH);
		pack();
		setVisible(true);
		createBufferStrategy(2);
		bs = getBufferStrategy();
		pix = new int[w * h];
		mmi = new MemoryImageSource(w, h, pix, 0, w);
		mmi.setAnimated(true);
		im = createImage(mmi);
		myInsets = this.getInsets();
		
		referenceTime = System.nanoTime();
		try {			
			addWindowListener((WindowListener)caller);
			addKeyListener((KeyListener)caller);
			addMouseListener((MouseListener)caller);
			addMouseMotionListener((MouseMotionListener)caller);
			//addMouseListener((MouseListener)this);
			//addMouseMotionListener((MouseMotionListener)this);

		} finally {
		}
	}

	public void copy(Image im) {
		Object lock = new Object();
		synchronized (lock) {
			if (g == null) {
				g = getGraphics2D();
			}
			g.drawImage(im, 0, 0, null);
			//g.drawImage(im, myInsets.left, myInsets.top, null);
		}
	}
	
	public void show(Image im) {
		copy(im);
		updateFPS();
		bs.show();
	}
	
	public void copy(int[] src) {				
		Object lock = new Object();
		synchronized (lock) {
			System.arraycopy(src, 0, pix, 0, w * h);
		}

		int copylength = Math.min(src.length, pix.length);
		for (int i = 0; i < copylength; i++)
			pix[i] = 0xff000000 | src[i];

		mmi.newPixels(0, 0, w, h);
		copy(im);
	}

	public void show(int[] src) {				
		copy(src);
		updateFPS();
		bs.show();
	}
	
	public void update() { 
		updateFPS(); 
		bs.show(); 
	}

	public void flush() { 
		mmi.newPixels(0, 0, w, h);
		copy(im);
	}	
	
	public void clear() {
		Arrays.fill(pix, 0xFF000000);
		mmi.newPixels(0, 0, w, h);
		bs.show();
	}
	
	public Graphics2D getGraphics2D() { 
		if (g == null) {
			g = (Graphics2D) bs.getDrawGraphics();
			g.translate(myInsets.left, myInsets.top);
		}
		return g;
	}
	
	public void invalidate() {
			//mmi.newPixels(0, 0, w, h);
	}
	
	public int[] getBuffer() {
		return pix;
	}
	
	public void updateFPS() {
		frameCount++;
		if (frameCount == 30) {
			fps = ((double) System.nanoTime() - referenceTime) / 1000000000;
			if (display_fps) {
				System.out.format("CanvasView: Got %d frames in %4.2fs%n", frameCount, fps);
			}
			frameCount = 0;
			referenceTime = System.nanoTime();
		}
	}
		
	public void out(String message) { System.out.println(message); }	

	public void setDisplayFPS(boolean show_display) {
		display_fps = show_display;
	}
		
	/********************************************
	 MOUSE EVENTS
	 ********************************************/
	public void mouseClicked(MouseEvent e) {
		int x = e.getX() - myInsets.left;
		int y = e.getY() - myInsets.top;
		
		out("Clicked at " + x + ", " + y);
		int idx = y * w + x;
		if (idx >= 0 && idx < N) {
			Color c = new Color(pix[idx]);
			out("Value at pixel is " + "r=" + c.getRed() + " g=" + c.getGreen() + " b=" + c.getBlue());
		}
		
	}
	public void mousePressed(MouseEvent e) {}
	public void mouseReleased(MouseEvent e) {}
	public void mouseEntered(MouseEvent e) {}
	public void mouseExited(MouseEvent e) {}
	public void mouseMoved(MouseEvent e) {}
	
	public void mouseDragged(MouseEvent e) {}
	
	@Override
	public void setVisible(final boolean visible) {
		// make sure that frame is marked as not disposed if it is asked to be visible
		//if (visible) {
		//	setDisposed(false);
		//}
		// let's handle visibility...
		if (!visible || !isVisible()) { // have to check this condition simply because super.setVisible(true) invokes toFront if frame was already visible
			super.setVisible(visible);
		}
		// ...and bring frame to the front.. in a strange and weird way
		if (visible) {
			int state = super.getExtendedState();
			state &= ~JFrame.ICONIFIED;
			super.setExtendedState(state);
			super.setAlwaysOnTop(true);
			super.toFront();
			super.requestFocus();
			super.setAlwaysOnTop(false);
		}
	}
	
	@Override
	public void toFront() {
		super.setVisible(true);
		int state = super.getExtendedState();
		state &= ~JFrame.ICONIFIED;
		super.setExtendedState(state);
		super.setAlwaysOnTop(true);
		super.toFront();
		super.requestFocus();
		super.setAlwaysOnTop(false);
	}
}


