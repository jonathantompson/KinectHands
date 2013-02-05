// <pre>
// Copyright 2001 Ken Perlin

package render;

//----- SIMPLE CLASS TO HANDLE BASIC 3D MATRIX OPERATIONS -----

/**
   Provides functionality for 4x4 3D matrix manipulations.
   It's thread-safe.
   @author Ken Perlin 2001
*/

public class Matrix {

   private String notice = "Copyright 2001 Ken Perlin. All rights reserved.";

   private static double identity[] = new double[16];
   static {
      for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++)
         identity[i << 2 | j] = (i == j ? 1 : 0);
   }
   private double mMatrix[] = new double[16];
   private double tmp[] = new double[16];
   private double tmp2[] = new double[16];

   /**
      Default constructor.
   */
   public Matrix() {
      identity();
   }

   /**
      Constructor takes an array of 16 elements to populate the 4x4 matrix.
      @param a 4x4 quaternion values
   */
   public Matrix(double a[]) {
      if (a.length == 4) { // quaternion
         double Nq = a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3];
         double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
         double xs = a[0] * s, ys = a[1] * s, zs = a[2] * s;
         double wx = a[3] * xs, wy = a[3] * ys, wz = a[3] * zs;
         double xx = a[0] * xs, xy = a[0] * ys, xz = a[0] * zs;
         double yy = a[1] * ys, yz = a[1] * zs, zz = a[2] * zs;

         mMatrix[0 << 2 | 0] = 1.0 - (yy + zz);
         mMatrix[1 << 2 | 0] = xy + wz;
         mMatrix[2 << 2 | 0] = xz - wy;

         mMatrix[0 << 2 | 1] = xy - wz;
         mMatrix[1 << 2 | 1] = 1.0 - (xx + zz);
         mMatrix[2 << 2 | 1] = yz + wx;

         mMatrix[0 << 2 | 2] = xz + wy;
         mMatrix[1 << 2 | 2] = yz - wx;
         mMatrix[2 << 2 | 2] = 1.0 - (xx + yy);

         mMatrix[0 << 2 | 3] = mMatrix[1 << 2 | 3] = mMatrix[2 << 2 | 3] =
         mMatrix[3 << 2 | 0] = mMatrix[3 << 2 | 1] = mMatrix[3 << 2 | 2] = 0.0;
         mMatrix[3 << 2 | 3] = 1.0;
      } else {
         System.arraycopy(a, 0, mMatrix, 0, 16);
      }
   }

   /**
      Returns matrix value at m[i, j].
      @param i row index
      @param j column index
      @return value at specified location
   */
   public final double get(int i, int j) {
      return mMatrix[i << 2 | j];
   }

   /**
      Sets matrix value at m[i,j] to d.
      @param i row index
      @param j column index
      @param d the new value
   */
   public final Matrix set(int i, int j, double d) {
      mMatrix[i << 2 | j] = d;
      return this;
   }

   /** 
       Returns the actual array containing the matrix (not thread-safe).
       @return the actual matrix array of 16 elements
   */
   public final double[] getData() {
      return mMatrix;
   }

   /**
      Returns a copy of matrix (thread-safe)/
      @return a copy of the matrix array (16 elements).
   */
   public void getData(double[] m) {
      System.arraycopy(mMatrix, 0, m, 0, 16);
   }

   /**
      Sets the desired matrix to the identity matrix.
      @param m the matrix to be modified
   */
   public static final void identity(Matrix m) {
      System.arraycopy(identity, 0, m.getData(), 0, 16);
   }

   /**
      Sets the object matrix to the identity matrix.
   */
   public final Matrix identity() {
      System.arraycopy(identity, 0, mMatrix, 0, 16);
      return this;
   }

   /**
      Sets the desired matrix array to the identity matrix.
      @param m matrix array
   */
   private static void identity(double[] m) {
      System.arraycopy(identity, 0, m, 0, 16);
   }

   /**
      Copies contents from matrix src to the object matrix.
      @param src original matrix to be copied
   */
   public final Matrix copy(Matrix src) {
      System.arraycopy(src.getData(), 0, mMatrix, 0, 16);
      return this;
   }

   private Matrix preMultiply(double src[]) {
      double dst[] = getData();
      System.arraycopy(mMatrix, 0, tmp, 0, 16);
      for (int i = 0 ; i < 4 ; i++)
      for (int j = 0 ; j < 4 ; j++)
         dst[i << 2 | j] = tmp[i << 2 | 0] * src[0 << 2 | j] +
                           tmp[i << 2 | 1] * src[1 << 2 | j] +
                           tmp[i << 2 | 2] * src[2 << 2 | j] +
                           tmp[i << 2 | 3] * src[3 << 2 | j] ;
      return this;
   }

   /**
      Premultiplies the object matrix by mb and stores the result in the object;
      As a result, the translation, scaling and rotation operations 
      contained in mb are effectively performed before those in the object . 
      @param mb the multiplier matrix
   */

   public final Matrix preMultiply(Matrix mb) {
      return preMultiply(mb.getData());
   }

   private Matrix postMultiply(double src[]) {
      double dst[] = getData();
      System.arraycopy(mMatrix, 0, tmp, 0, 16);
      for (int i = 0; i < 4; i++)
      for (int j = 0; j < 4; j++)
         dst[i << 2 | j] = src[i << 2 | 0] * tmp[0 << 2 | j] +
                           src[i << 2 | 1] * tmp[1 << 2 | j] +
                           src[i << 2 | 2] * tmp[2 << 2 | j] +
                           src[i << 2 | 3] * tmp[3 << 2 | j] ;
      return this;
   }

   /** 
        Postmultiplies the object matrix by mb and stores the result in the object matrix;
        As a result, the translation, scaling and rotation operations
        contained in mb are effectively performed after those in the object matrix. 
        @param mb the multiplier matrix
   */

   public final Matrix postMultiply(Matrix mb) {
      return postMultiply(mb.getData());
   }

   //----- ROUTINES TO ROTATE AND TRANSLATE MATRICES -----

   /**
        Applies a translation by x, y, z to the obeject matrix. The shape or 
        orientation of the object are not affected. 
        @param x amount of translation along the x axis
        @param y amount of translation along the y axis
        @param z amount of translation along the z axis
     */
   public final Matrix translate(double x, double y, double z) {
      makeTranslationMatrix(tmp2, x, y, z);
      return preMultiply(tmp2);
   }

   /** 
   	Modifies the object matrix to rotate about the X axis by angle theta.
         @param theta angle of rotation in radians
     */
   public final Matrix rotateX(double theta) {
      makeRotationMatrix(tmp2, 1, 2, theta);
      return preMultiply(tmp2);
   }

   /** 
   	Modifies the object matrix to rotate about the Y axis by angle theta.
         @param theta angle of rotation in radians
     */
   public final Matrix rotateY(double theta) {
      makeRotationMatrix(tmp2, 2, 0, theta);
      return preMultiply(tmp2);
   }

   /** 
   	Modifies the object matrix to rotate about the Z axis by angle theta.
         @param theta angle of rotation in radians
     */
   public final Matrix rotateZ(double theta) {
      makeRotationMatrix(tmp2, 0, 1, theta);
      return preMultiply(tmp2);
   }

   /**
   	  Modifies the object matrix to rotate by angle theta about axis x,y,z.
   		@param theta angle of rotation in radians
   		@param x 1st coord of rotation axis
   		@param y 2nd coord of rotation axis
   		@param z 3rd coord of rotation axis
     */
   public final Matrix rotate(double theta, double x, double y, double z) {
      double unY = Math.atan2(y, x);
      double unX = Math.atan2(Math.sqrt(x * x + y * y), z);
      rotateZ(unY);
      rotateY(unX);
      rotateZ(theta);
      rotateY(-unX);
      rotateZ(-unY);
      return this;
   }

   /** 
   	Scales the transformation matrix uniformly.
          @param s scale factor along all three axes
      */
   public final Matrix scale(double s) {
      return scale(s, s, s);
   }

   /** 
   	Scales the transformation matrix by x,y,z in the respective 
   	directions. 
          @param x scale factor along the x axis
   	@param y scale factor along the y axis
   	@param z scale factor along the z axis
      */
   public final Matrix scale(double x, double y, double z) {
      makeScaleMatrix(tmp2, x, y, z);
      return preMultiply(tmp2);
   }

   public final Matrix perspectiveX(double s) { return perspective(0, s); }
   public final Matrix perspectiveY(double s) { return perspective(1, s); }
   public final Matrix perspectiveZ(double s) { return perspective(2, s); }

   public final void getEuler(double e[]) {
      e[1] = Math.asin(get(0, 2));
      double C = Math.cos(e[1]);
      if (Math.abs(C) > 0.005) {
         e[0] = -Math.atan2( get(1, 2) / C, get(2, 2) / C);
         e[2] =  Math.atan2(-get(0, 1) / C, get(0, 0) / C);
      }
      else {
         e[0] = 0;
         e[2] = Math.atan2( get(1, 0) / C, get(1, 1) / C);
      }
   }

   public double getDeterminant() {
      double a = get(0,0), b = get(0,1), c = get(0,2),
             d = get(1,0), e = get(1,1), f = get(1,2),
             g = get(2,0), h = get(2,1), i = get(2,2);
      return a*e*i + b*f*g + c*d*h - c*e*g - b*d*i - a*f*h;
   }

   /** 
   	Skews x as a linear function of y.
        @param s the skew factor
   */
   public final Matrix skewXY(double s) { return skew(0,1, s); }
   /** 
   	Skews x as a linear function of x.
        @param s the skew factor
   */
   public final Matrix skewXZ(double s) { return skew(0,2, s); }
   /** 
   	Skews y as a linear function of x.
        @param s the skew factor
   */
   public final Matrix skewYX(double s) { return skew(1,0, s); }
   /** 
   	Skews y as a linear function of z.
        @param s the skew factor
   */
   public final Matrix skewYZ(double s) { return skew(1,2, s); }
   /** 
   	Skews z as a linear function of x.
        @param s the skew factor
   */
   public final Matrix skewZX(double s) { return skew(2,0, s); }
   /** 
   	Skews z as a linear function of y.
        @param s the skew factor
   */
   public final Matrix skewZY(double s) { return skew(2,1, s); }

   private Matrix skew(int i, int j, double s) {
      makeSkewMatrix(tmp2, i, j, s);
      return preMultiply(tmp2);
   }

   private Matrix perspective(int i, double s) {
      makePerspectiveMatrix(tmp2, i, s);
      return preMultiply(tmp2);
   }

   /**
      Transforms (x,y,z) and stores the result in dst[].
      @param x local x
      @param y local y
      @param z local z
      @param dst resulting global vector
   */

   public void transform(double x, double y, double z, double dst[]) {
      for (int i = 0 ; i < 3 ; i++)
         dst[i] = get(i,0) * x + get(i,1) * y + get(i,2) * z + get(i,3);

      double w = get(3,0) * x + get(3,1) * y + get(3,2) * z + get(3,3);
      for (int i = 0 ; i < 3 ; i++)
         dst[i] /= w;
   }

   //----- INVERTING A 4x4 THAT WAS CREATED BY TRANSLATIONS+ROTATIONS+SCALES

   /**
      Inverts the 4x4 matrix and stores the result in the object
      matrix.  
      @param msrc original matrix to be inverted
   */

   double a[][] = new double[4][4];
   double b[][] = new double[4][4];

   public final Matrix invert(Matrix msrc) {
      double src[] = msrc.getData();
      double dst[] = mMatrix;

      for (int i = 0 ; i < 4 ; i++)
      for (int j = 0 ; j < 4 ; j++)
         a[i][j] = src[i << 2 | j];

      Invert.invert(a, b);

      for (int i = 0 ; i < 4 ; i++)
      for (int j = 0 ; j < 4 ; j++)
         dst[i << 2 | j] = b[i][j];

      return this;
   }

   //----- FOR DEBUGGING -----
   /**
         Converts the transformation matrix to a String.
         @param m matrix to be translated to text
         @return a textual representation of the matrix
      */
   public final String toString(Matrix mm) {
      double m[] = mm.getData();
      String s = "{";
      for (int i = 0; i < 4; i++) {
         s += "{";
         for (int j = 0; j < 4; j++) {
            int n = (int) (100 * m[i << 2 | j]);
            s += (n / 100.) + (j == 3 ? "" : ",");
         }
         s += "}" + (i == 3 ? "" : ",");
      }
      return s + "}";
   }

   //----- ROUTINES TO GENERATE TRANSFORMATION MATRICES -----

   private static void makeTranslationMatrix(double m[], double x, double y, double z) {
      identity(m);
      m[0 << 2 | 3] = x;
      m[1 << 2 | 3] = y;
      m[2 << 2 | 3] = z;
   }
   private static void makeRotationMatrix(double m[], int i, int j, double theta) {
      identity(m);
      m[i << 2 | i] = m[j << 2 | j] = Math.cos(theta);
      m[i << 2 | j] = -Math.sin(theta);
      m[j << 2 | i] = -m[i << 2 | j];
   }
   private static void makeScaleMatrix(double m[], double x, double y, double z) {
      identity(m);
      m[0 << 2 | 0] *= x;
      m[1 << 2 | 1] *= y;
      m[2 << 2 | 2] *= z;
   }
   private static void makeSkewMatrix(double m[], int i, int j, double s) {
      System.arraycopy(identity, 0, m, 0, 16);
      m[i << 2 | j] = s;
   }
   private static void makePerspectiveMatrix(double m[], int i, double s) {
      System.arraycopy(identity, 0, m, 0, 16);
      m[3 << 2 | i] = s;
   }



   public String toString() {
      int k = 0;
      String s = new String("[ ");
      for (int i = 0; i < 4; i++) {
         s += "\t[ ";
         for (int j = 0; j < 4; j++)
            s += String.valueOf(mMatrix[k++]) + " ";
         s += "]\n";
      }
      s += "]";
      return s;
   }
   
   
   public void getRowOrderData(double[] r) {
      for (int i = 0; i < 4; i++)
         for (int j = 0; j < 4; j++)
	    r[i | j<<2] = mMatrix[i<<2 | j];
  }
   
   
  private static double[] temp_location_getAim = new double[3];
  private static double[] temp_x_prime = new double[3];
  private static double[] temp_y_prime = new double[3];
  private static double[] temp_regular_z = {0,0,1};
    
  /**
   * turns m into the matrix that rotates positive z to location
   *
   */
  
  // synchronized to protect above temp variables.
    
  public static synchronized void getAim(double[] location, Matrix m){
        
    //
    // Trying to make orthonormal basis matrix:
    // 
    // | x' y' z' 0|
    // | 0  0  0  1|
    // 
    // Where z' = normalized location
    //
        
    Vec.copy(location,temp_location_getAim);
        
    Vec.normalize(temp_location_getAim);
        
    double[] z_prime = temp_location_getAim;
        
    m.identity();
        
    // if there is no real rotation going on.
        
    if(Vec.equals(z_prime,temp_regular_z)){
            
      return;
            
    }
        
    // since result of cross product would be perpendicular to z_prime
        
    double[] x_prime = temp_x_prime;// give it a better name
    Vec.cross(temp_regular_z, z_prime, x_prime);
    Vec.normalize(x_prime);
        
    double[] y_prime = temp_y_prime;
    Vec.cross(z_prime,x_prime,y_prime);
    Vec.normalize(y_prime);
        
    // have all we need for the rotation matrix
    // now assign those values to it
        
    // x_prime
    for(int i = 0; i < 3; i++){
         
      m.set(i,0, x_prime[i]);
            
    }
            
    // y_prime
    for(int i = 0; i < 3; i++){
         
      m.set(i,1, y_prime[i]);
            
    }
        
    // z_prime
    for(int i = 0; i < 3; i++){
         
      m.set(i,2, z_prime[i]);
            
    }
        
  }

   public void aimX(double d[]) {
      for (int i = 0 ; i < 3 ; i++) {
         x[i] = d[i];
         y[i] = i == 1 ? 1 : 0;
      }
      Vec.cross(x, y, z);
      Vec.cross(z, x, y);
      setOrientation(x, y, z);
   }

   public void aimY(double d[]) {
      for (int i = 0 ; i < 3 ; i++) {
         y[i] = d[i];
         z[i] = i == 2 ? 1 : 0;
      }
      Vec.cross(y, z, x);
      Vec.cross(x, y, z);
      setOrientation(x, y, z);
   }

   public void aimZ(double d[]) {
      for (int i = 0 ; i < 3 ; i++) {
         z[i] = d[i];
         x[i] = i == 0 ? 1 : 0;
      }
      Vec.cross(z, x, y);
      Vec.cross(y, z, x);
      setOrientation(x, y, z);
   }

   public void setOrientation(double x[], double y[], double z[]) {
      Vec.normalize(x);
      Vec.normalize(y);
      Vec.normalize(z);
      for (int i = 0 ; i < 3 ; i++) {
         set(i,0, x[i]);
         set(i,1, y[i]);
         set(i,2, z[i]);
      }
   }

   private double x[] = new double[3];
   private double y[] = new double[3];
   private double z[] = new double[3];
}

