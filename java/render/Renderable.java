/*
 * Created on Apr 5, 2004
 *
 */
package render;

import java.awt.Graphics;
import java.awt.Rectangle;

/**
 * Main interface that for objects to be rendered.
 */
public interface Renderable
{

  /**
   * initialize all your geometries, materials and lights here
   */
  public void initialize();

  /**
   * this is the main animation routine, called per frame
   * 
   * @param time
   *          system time in seconds
   */
  public void animate(double time);

  /**
   * allows for drawing an overlay layer on top of the rendered graphics
   * 
   * @param g
   *          graphics onto which the overlay will be rendered
   */
  public void drawOverlay(Graphics g);

  /**
   * returns the bounds Rectangle for this Renderable
   * 
   * @return bounds for this Renderable
   */
  public Rectangle getBounds();
  
  /**
   * 
   * @param x
   * @param y
   * @param w
   * @param h
   */
  public void setBounds(int x, int y, int w, int h);
  
  /**
   * 
   * @param r
   */
  public void setBounds(Rectangle r);
}