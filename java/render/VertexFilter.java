
package render;

public class VertexFilter
{
    public void init(Geometry part) {
       this.part = part;
       ref = new Geometry();
       createRef(part, ref);
    }

    public Geometry getPart() { return part; }

    public void update() {
       update(part, ref);
    }

    void update(Geometry part, Geometry ref) {
       if (part.vertices != null) {
          for (int n = 0 ; n < ref.vertices.length ; n++) {
             for (int i = 0 ; i < 3 ; i++)
                part.vertices[n][i] = ref.vertices[n][i];
             filterVertex(part.vertices[n]);
          }
          part.computeSurfaceNormals();
       }
       for (int n = 0 ; part.child(n) != null ; n++)
          update(part.child(n), ref.child(n));
    }

    public void filterVertex(double v[]) {
    }

    void createRef(Geometry part, Geometry ref) {
       if (part.vertices != null) {
          ref.vertices = new double[part.vertices.length][3];
          for (int n = 0 ; n < ref.vertices.length ; n++)
             for (int i = 0 ; i < 3 ; i++)
                ref.vertices[n][i] = part.vertices[n][i];
       }
       for (int n = 0 ; part.child(n) != null ; n++)
          createRef(part.child(n), ref.add());
    }

    Geometry part, ref;
}

