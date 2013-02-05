package app;

import java.awt.event.*;
import oniclient.*;
import canvasframe.*;

public class OpenNIClientTest 
implements  Runnable, 
WindowListener, 
KeyListener, 
MouseListener, 
MouseMotionListener,
HandTrackerListener
{
    
  int[] pix = new int[640 * 480];
  CanvasFrame view;    
  KinectClient zmq;
  boolean showdepth = false;
  boolean showvideo = true;
  
  public void run() {
    int width = 640;
    int height = 480;
    view = new CanvasFrame(this, 0, width, height);
    zmq = new KinectClient();
    Thread t = new Thread(zmq);
    t.start();
    zmq.addHandTrackerListener(this);
    while (true) {
      try {
    	if (showdepth) {
    		zmq.copyRGB(pix);
    	} else {
    		zmq.copyDepth(pix);    		
    	}
        view.copy(pix);
        view.update();
        Thread.sleep(30);
        
      } catch (InterruptedException e) {
    	  e.printStackTrace();
      }
    }
  }
    
  public void out(String s) {
    System.out.println(s);
    
  }
  
  public static void main(String[] args) {
    OpenNIClientTest qtt = new OpenNIClientTest();
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
  
	public void mouseDragged(MouseEvent arg0) {}
  
	public void mouseMoved(MouseEvent arg0) {}
  
	public void mouseClicked(MouseEvent arg0) {}
  
	public void mouseEntered(MouseEvent arg0) {}
  
	public void mouseExited(MouseEvent arg0) {}
  
	public void mousePressed(MouseEvent arg0) {}
  
	public void mouseReleased(MouseEvent arg0) {}
  
	public void keyPressed(KeyEvent arg0) {
    switch (arg0.getKeyCode()) {
      case KeyEvent.VK_D: 
    	  showdepth = !showdepth;    	  
        break;
      case KeyEvent.VK_V:
    	  showvideo = !showvideo;
        break;
        
      default: break;
    }
  }            
  
	public void keyReleased(KeyEvent arg0) {}
  
	public void keyTyped(KeyEvent arg0) {}
	
}