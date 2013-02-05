// <pre>

package render;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.ImageProducer;

public abstract class RenderApplet extends java.applet.Applet implements Renderable
{
  public Image getIm() { return render.getIm(); }

  // MEMBERS -------------------------------------
  protected RenderablePanel render;

  // CONSTRUCTORS --------------------------------
  public RenderApplet() {}

  // METHODS -------------------------------------
  public void init() {
  	//System.out.println("RenderApplet.init() glEnabled: " + glEnabled());
  	
  	if (super.getWidth() != 0) {
  		//System.out.println("RenderApplet.init(): with size already Set: " + super.getWidth() );
			
			readyToInitialize();								
			resized(super.getWidth(), super.getHeight());
  	}
		
  	addComponentListener(new ComponentAdapter() {
			public void componentResized(ComponentEvent e) {
				int width = getWidth();
				int height = getHeight();

				//System.out.println(".componentResized() width: " + width);
				
				if (width == 0)
					return;
				
				if (!alreadyInitialized) {
					readyToInitialize();								
					resized(width, height);
	
					if (!initialStartCalled)
						start();
				} else
					resized(width, height);
			}
		});
  }
  
  private boolean alreadyInitialized = false;
  private boolean initialStartCalled = false;
  
  public void resized(int width, int height) {
  	//System.out.println("RenderApplet.resized() width: " + width);
		
  }
  
  private void readyToInitialize() {
  	//System.out.println("RenderApplet.readyToInitialize() alreadyInitialized: " + alreadyInitialized + "      glEnabled: " + glEnabled());
		
    render = new RenderPanel(this);
    render.init();
    add((Panel) render);
    addInputListeners();
    
    alreadyInitialized = true;
  }
  
  public void start()  {
  	//System.out.println("RenderApplet.start(): " + alreadyInitialized);
		
  	if (alreadyInitialized) {
  		//System.out.println("RenderApplet.start(): Calling render.start()");
			
  		render.start();	
  	}
  	
  	initialStartCalled = true;
  }
  
  public void stop() {						
  		//System.out.println("RenderApplet.stop(): doesn't do the right thing.  Animate Thread still running...");		// FixMe: This needs fixing...  (GS)
  }


  /** To be overridden in subclasses */
  public void initialize() {}

  /** To be overridden in subclasses */
  public void drawOverlay(Graphics g) {}

  /** To be overridden in subclasses */
  public void animate(double time) {}

  /**
   * Must be overridden to use GL versus software-renderer(default)
   * 
   * @return false (to use GL, override to return true)
   */
  public boolean glEnabled()
  {
    return false;
  }

  public void damage()
  {
    render.refresh();
  }

  public boolean keyUp(Event evt, int key)
  {
    return render.processCommand(key);
  }

  public Geometry queryCursor(double[] point) {
    return render.queryCursor(point);
  }

  public boolean mouseDown(Event evt, int x, int y)
  {
    render.mousePressed(evt, x, y);
    return false;
  }

  public boolean mouseDrag(Event evt, int x, int y)
  {
    render.mouseDragged(evt, x, y);
    return false;
  }

  public boolean mouseMove(Event evt, int x, int y)
  {
    render.mouseMoved(evt, x, y);
    return false;
  }

  public boolean mouseUp(Event evt, int x, int y)
  {
    render.mouseReleased(evt, x, y);
    return false;
  }

  public void addKeyListener(KeyListener arg0)
  {
    if (render != null)
      render.addKeyListener(arg0);
  }

  public void addLight(double x, double y, double z, double r, double g,
      double b)
  {
    render.addLight(x, y, z, r, g, b);
  }

  public Widget addMenu(String label, int x, int y)
  {
    return render.addMenu(label, x, y);
  }

  public void addMenu(Widget menu)
  {
    render.addMenu(menu);
  }

  public void addMouseListener(MouseListener arg0)
  {
    render.addMouseListener(arg0);
  }

  public void addMouseMotionListener(MouseMotionListener arg0)
  {
    render.addMouseMotionListener(arg0);
  }

  public void addMouseWheelListener(MouseWheelListener arg0)
  {
    render.addMouseWheelListener(arg0);
  }

  public Image createImage(ImageProducer arg0)
  {
    return render.createImage(arg0);
  }

  public Image createImage(int arg0, int arg1)
  {
    return render.createImage(arg0, arg1);
  }

  public boolean equals(Object arg0)
  {
    return render.equals(arg0);
  }

  public double getCurrentTime()
  {
    return render.getCurrentTime();
  }

  public double getCx()
  {
    return render.getCx();
  }

  public double getCy()
  {
    return render.getCy();
  }

  public double getFL()
  {
    return render.getFL();
  }

  public double getFOV()
  {
    return render.getFOV();
  }
/*
  public Geometry getGeometry(int x, int y)
  {
    return render.getGeometry(x, y);
  }
*/
/*
  public boolean getGeometryBuffer()
  {
    return render.getGeometryBuffer();
  }
*/
  public int getHeight()
  {
    return render.getHeight();
  }

  public int getLod()
  {
    return render.getLod();
  }

  public Matrix[] getMatrix()
  {
    return render.getMatrix();
  }

  public boolean getOutline()
  {
    return render.getOutline();
  }

  public int[] getPix()
  {
    return render.getPix();
  }

  public boolean getPoint(int x, int y, double[] xyz)
  {
    return render.getPoint(x, y, xyz);
  }

  public Renderable getRenderable()
  {
    return render.getRenderable();
  }

  public Renderer getRenderer()
  {
    return render.getRenderer();
  }

  public boolean getTableMode()
  {
    return render.getTableMode();
  }

  public int getWidth()
  {
    return render.getWidth();
  }

  public Geometry getWorld()
  {
    return render.getWorld();
  }

  public int hashCode()
  {
    return render.hashCode();
  }

  public void identity()
  {
    render.identity();
  }

  public boolean isDragging()
  {
    return render.isDragging();
  }

  public Matrix m()
  {
    return render.m();
  }

  public Widget menu(int i)
  {
    return render.menu(i);
  }

  public void mouseClicked(Event e, int x, int y)
  {
    render.mouseClicked(e, x, y);
  }

  public void mouseDragged(Event e, int x, int y)
  {
    render.mouseDragged(e, x, y);
  }

  public void mouseEntered(Event e, int x, int y)
  {
    render.mouseEntered(e, x, y);
  }

  public void mouseExited(Event e, int x, int y)
  {
    render.mouseExited(e, x, y);
  }

  public void mouseMoved(Event e, int x, int y)
  {
    render.mouseMoved(e, x, y);
  }

  public void mousePressed(Event e, int x, int y)
  {
    render.mousePressed(e, x, y);
  }

  public void mouseReleased(Event e, int x, int y)
  {
    render.mouseReleased(e, x, y);
  }

  public void mouseWheelMoved(MouseWheelEvent e, int rotation)
  {
    render.mouseWheelMoved(e, rotation);
  }

  public void pause()
  {
    render.pause();
  }

  public void pop()
  {
    render.pop();
  }

  public boolean processCommand(int key)
  {
    return render.processCommand(key);
  }

  public int pull(Geometry s, double x0, double x1, double x2, double y0,
      double y1, double y2, double z0, double z1, double z2)
  {
    return render.pull(s, x0, x1, x2, y0, y1, y2, z0, z1, z2);
  }

  public void push()
  {
    render.push();
  }

  public void recalculateSize(int width, int height)
  {
    render.recalculateSize(width, height);
  }

  public void refresh()
  {
    render.refresh();
  }

  public void removeKeyListener(KeyListener arg0)
  {
    render.removeKeyListener(arg0);
  }

  public void removeMenu(Widget menu)
  {
    render.removeMenu(menu);
  }

  public void removeMouseListener(MouseListener arg0)
  {
    render.removeMouseListener(arg0);
  }

  public void removeMouseMotionListener(MouseMotionListener arg0)
  {
    render.removeMouseMotionListener(arg0);
  }

  public void repaint()
  {
    render.repaint();
  }

  public void rotateView(double theta, double phi)
  {
    render.rotateView(theta, phi);
  }

  public void rotateX(double t)
  {
    render.rotateX(t);
  }

  public void rotateY(double t)
  {
    render.rotateY(t);
  }

  public void rotateZ(double t)
  {
    render.rotateZ(t);
  }

  public void scale(double x, double y, double z)
  {
    render.scale(x, y, z);
  }

  public void setBgColor(double r, double g, double b)
  {
    render.setBgColor(r, g, b);
  }

  public void setDragging(boolean value)
  {
    render.setDragging(value);
  }

  public void setCx(double value)
  {
    render.setCx(value);
  }

  public void setCy(double value)
  {
    render.setCy(value);
  }

  public void setFL(double value)
  {
    render.setFL(value);
  }

  public void setFOV(double value)
  {
    render.setFOV(value);
  }

  public void setGeometryBuffer(boolean value)
  {
    render.setGeometryBuffer(value);
  }

  public void setLod(int value)
  {
    render.setLod(value);
  }

  public void setOutline(boolean value)
  {
    render.setOutline(value);
  }

  public void setRenderable(Renderable renderable)
  {
    render.setRenderable(renderable);
  }

  public void setTableMode(boolean value)
  {
    render.setTableMode(value);
  }

  public void showMesh(boolean value)
  {
    render.showMesh(value);
  }

  public String toString()
  {
    return render.toString();
  }

  public void transform(Geometry s)
  {
    render.transform(s);
  }

  public void translate(double x, double y, double z)
  {
    render.translate(x, y, z);
  }

  public void translate(double[] v)
  {
    render.translate(v);
  }

  public Widget widgetAt(int x, int y)
  {
    return render.widgetAt(x, y);
  }

  private void addInputListeners()
  {
    addKeyListener(new KeyAdapter()
    {
      public void keyPressed(KeyEvent e)
      {
        keyDown(newEvent(e), e.getKeyCode());
      }

      public void keyReleased(KeyEvent e)
      {
        keyUp(newEvent(e), e.getKeyCode());
      }
    });
    addMouseListener(new MouseAdapter()
    {
      public void mousePressed(MouseEvent e)
      {
        if (mouseDown(newEvent(e), e.getX(), e.getY()))
          setDragging(false);
      }

      public void mouseReleased(MouseEvent e)
      {
        mouseUp(newEvent(e), e.getX(), e.getY());
      }
    });
    addMouseMotionListener(new MouseMotionListener()
    {
      public void mouseDragged(MouseEvent e)
      {
        mouseDrag(newEvent(e), e.getX(), e.getY());
      }

      public void mouseMoved(MouseEvent e)
      {
        mouseMove(newEvent(e), e.getX(), e.getY());
      }
    });
  }

  Event newEvent(MouseEvent e)
  {
    return new Event(null, 0, 0, e.getX(), e.getY(), 0, e.getModifiers());
  }

  Event newEvent(KeyEvent e)
  {
    return new Event(null, 0, 0, 0, 0, e.getKeyCode(), e.getModifiers());
  }

}// end
