// By Molin Deng 2024-02-03
import java.util.*;
// ================ Helper Classes ================
static class Helper {
  static PVector cwiseproduct(PVector a, PVector b) { return new PVector(a.x * b.x, a.y * b.y, a.z * b.z); }
  static PVector vZero() { return new PVector(0, 0, 0);}
  static PVector cwisemin(PVector a, PVector b) { return new PVector(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)); }
  static PVector cwisemax(PVector a, PVector b) { return new PVector(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)); }
  static float max3(float a, float b, float c) { return max(a, max(b, c)); }
  static float min3(float a, float b, float c) { return min(a, min(b, c)); }
  static float getAxisValue(PVector p, int axis) { if (axis == 0) return p.x; if (axis == 1) return p.y; return p.z;}
}
// ================ Math Classes ================
class Matrix {
  protected final int M;             // number of rows
  protected final int N;             // number of columns
  protected final float[][] data;   // M-by-N array
  // create M-by-N matrix of 0's
  public Matrix(int M, int N) { this.M = M; this.N = N; data = new float[M][N]; }
  // create matrix based on 2d array
  public Matrix(float[][] data) {
    M = data.length;
    N = data[0].length;
    this.data = new float[M][N];
    for (int i = 0; i < M; i++)
      for (int j = 0; j < N; j++)
        this.data[i][j] = data[i][j];
  }
  // copy matrix
  public Matrix(Matrix A) { this(A.data); }
  // transpose
  public Matrix transpose() {
      Matrix A = new Matrix(N, M);
      for (int i = 0; i < M; i++)
          for (int j = 0; j < N; j++)
              A.data[j][i] = this.data[i][j];
      return A;
  }
  // return C = A * B
  public Matrix times(Matrix B) {
    Matrix A = this;
    if (A.N != B.M) throw new RuntimeException("Illegal matrix dimensions.");
    Matrix C = new Matrix(A.M, B.N);
    for (int i = 0; i < C.M; i++)
      for (int j = 0; j < C.N; j++)
        for (int k = 0; k < A.N; k++)
          C.data[i][j] += (A.data[i][k] * B.data[k][j]);
    return C;
  }
  // return M * Vector (Point)
  public PVector times(PVector p) {
    if (this.N != 4) throw new RuntimeException("Illegal matrix dimensions.");
    float[] a = new float[]{p.x, p.y, p.z, 1}; // Point add 4th coord, set 1
    float[] b = new float[4];
    for (int i = 0; i < N; i++) 
      for (int j = 0; j < N; j++)
        b[i] += (data[i][j] * a[j]);
    return new PVector(b[0], b[1], b[2]);
  }
  public PVector timesVec(PVector p) {
    if (this.N != 4) throw new RuntimeException("Illegal matrix dimensions.");
    float[] a = new float[]{p.x, p.y, p.z, 0}; // Point add 4th coord, set 0
    float[] b = new float[4];
    for (int i = 0; i < N; i++) 
      for (int j = 0; j < N; j++)
        b[i] += (data[i][j] * a[j]);
    return new PVector(b[0], b[1], b[2]);
  }
  // invert
  public Matrix invert() {
    if (this.N != 4) throw new RuntimeException("Illegal matrix dimensions.");
    PMatrix3D pmat = new PMatrix3D(data[0][0],data[0][1],data[0][2],data[0][3],data[1][0],data[1][1],data[1][2],data[1][3],data[2][0],data[2][1],data[2][2],data[2][3],data[3][0],data[3][1],data[3][2],data[3][3]);
    pmat.invert();
    float[] t = pmat.get(null);
    Matrix In = new Matrix(4, 4);
    for (int i = 0; i < 4; ++i) 
      for (int j = 0; j < 4; ++j)
        In.data[i][j] = t[i*4+j];
    return In;
  }
  public void show() {
    for (int i = 0; i < M; i++) {
      for (int j = 0; j < N; j++) {
        print(data[i][j]); print(" ");
      }
      println();
    }
  }
}
// Identity Matrix Class
class Identity extends Matrix {
  public Identity(int N) {
    super(N, N);
    for (int i = 0; i < N; i++) data[i][i] = 1;
  }
}
// ================ Scene Classes ================
class Light {
  public PVector pos, intensity;
  public Light(PVector p, PVector i) { pos = p; intensity = i; }
}

class Ray {
  public PVector o, d, invd;
  public Ray(PVector _o, PVector _d) { o = _o; d = _d; invd = new PVector(1.0/d.x, 1.0/d.y, 1.0/d.z); }
  public PVector getPoint(float t) { return PVector.add(o, PVector.mult(d, t)); }
  public Ray transform(Matrix mat) { return new Ray(mat.times(o), mat.timesVec(d)); }
}

class HitInfo {
  public PVector normal;
  public float time;
  public HitInfo(float t, PVector n) { 
    this.time = t;
    normal = n;
  }
  public void correctNormal(Ray ray) {
    if (PVector.dot(ray.d, normal) > 0) normal.mult(-1);
  }
}

interface HitTest {
  HitInfo hitTest(Ray ray);
}
// Ray-traced Object
class RTObject implements HitTest {
  class AABB {
    public PVector centroid;
    public PVector[] bounds;
    AABB() {
      bounds = new PVector[2];
      bounds[0] = new PVector(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY); 
      bounds[1] = new PVector(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY); 
      centroid = Helper.vZero(); 
    }
    void update(PVector p) { 
      bounds[0] = Helper.cwisemin(bounds[0], p);
      bounds[1] = Helper.cwisemax(bounds[1], p); 
      centroid = PVector.add(bounds[0], bounds[1]).mult(0.5);
    }
    void update(AABB that) { 
      update(that.bounds[0]); 
      update(that.bounds[1]); 
    }
    // Invalid for rotation
    void transform(Matrix t) { 
      bounds[0] = t.times(bounds[0]);
      bounds[1] = t.times(bounds[1]);
      centroid = t.times(centroid);
    }
  }
  public AABB aabb;
  public RTObject() { aabb = new AABB(); }
  public void applyTransform(Matrix t) {
    aabb.transform(t); 
  }
  @Override public HitInfo hitTest(Ray ray) { 
    float tmin, tmax, tymin, tymax, tzmin, tzmax;
    int[] sign = {ray.d.x < 0 ? 1 : 0, ray.d.y < 0 ? 1 : 0, ray.d.z < 0 ? 1 : 0};
    int axis = 0;
    tmin = (aabb.bounds[sign[0]].x - ray.o.x) * ray.invd.x;
    tmax = (aabb.bounds[1-sign[0]].x - ray.o.x) * ray.invd.x;
    tymin = (aabb.bounds[sign[1]].y - ray.o.y) * ray.invd.y;
    tymax = (aabb.bounds[1-sign[1]].y - ray.o.y) * ray.invd.y;
    if ((tmin > tymax) || (tymin > tmax)) return null;
    if (tymin > tmin) { tmin = tymin; axis = 1;} 
    if (tymax < tmax) tmax = tymax;
    tzmin = (aabb.bounds[sign[2]].z - ray.o.z) * ray.invd.z;
    tzmax = (aabb.bounds[1-sign[2]].z - ray.o.z) * ray.invd.z;
    if ((tmin > tzmax) || (tzmin > tmax)) return null;
    if (tzmin > tmin) { tmin = tzmin; axis = 2; }
    if (tzmax < tmax) tmax = tzmax;
    // ! TODO: maybe wrong for shadow ray test (not sure)
    if (tmin < 0 || tmin > tmax || tmax < 0)
      return null;
    // ! TODO: maybe wrong for shadow ray test (not sure)
    return new HitInfo(tmin, new PVector(axis == 0 ? 1 : 0, axis == 1 ? 1 : 0, axis == 2 ? 1 : 0));
  }
}

class Box extends RTObject {
  public Box(PVector p0, PVector p1) { super(); aabb.update(p1); aabb.update(p0); }
  @Override public HitInfo hitTest(Ray ray) { 
    return super.hitTest(ray);
  }
}

class Tri extends RTObject {
  private int i;
  public PVector[] points;
  public Tri() { 
    super();
    i = 0; points = new PVector[3];
  }
  public void addVertex(PVector p) { 
    aabb.update(p); // update AABB
    points[i++] = p; 
  }
  public PVector normal() { return PVector.sub(points[1], points[0]).cross(PVector.sub(points[2], points[0])).normalize();  }
  public boolean isInTriangle(PVector p) {
    PVector n = normal();
    boolean ccw1 = PVector.dot(PVector.sub(points[1], points[0]).cross(PVector.sub(p, points[0])), n) >= 0;
    boolean ccw2 = PVector.dot(PVector.sub(points[2], points[1]).cross(PVector.sub(p, points[1])), n) >= 0;
    boolean ccw3 = PVector.dot(PVector.sub(points[0], points[2]).cross(PVector.sub(p, points[2])), n) >= 0;
    return ccw1 == ccw2 && ccw2 == ccw3;
  }
  @Override public void applyTransform(Matrix t) { 
    for (int i = 0; i < 3; i++) points[i] = t.times(points[i]);
    super.applyTransform(t);
  }

  @Override public HitInfo hitTest(Ray ray) { 
    PVector n = normal();
    float t = PVector.dot(PVector.sub(points[0], ray.o), n) / PVector.dot(ray.d, n);
    if (t >= 0 && isInTriangle(ray.getPoint(t)))
      return new HitInfo(t, n);
    return null;
  }
}

class BVH extends RTObject {
  public PVector[] centroidBounds;
  public ArrayList<RTObject> rtObjects;
  public BVH[] children;
  public BVH() {
    super();
    rtObjects = new ArrayList<>();
    children = new BVH[2];
    centroidBounds = new PVector[2];
    centroidBounds[0] = new PVector(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY); 
    centroidBounds[1] = new PVector(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
  }
  public BVH(ArrayList<RTObject> rtObjects) {
    super();
    this.rtObjects = rtObjects;
    children = new BVH[2];
    centroidBounds = new PVector[2];
    centroidBounds[0] = new PVector(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY); 
    centroidBounds[1] = new PVector(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY);
    for (RTObject rtObj: rtObjects) {
      aabb.update(rtObj.aabb);
      centroidBounds[0] = Helper.cwisemin(centroidBounds[0], rtObj.aabb.centroid);
      centroidBounds[1] = Helper.cwisemax(centroidBounds[1], rtObj.aabb.centroid);
    }
  }
  public void addRTObj(RTObject rtObj) {
    // update centroid bound
    centroidBounds[0] = Helper.cwisemin(centroidBounds[0], rtObj.aabb.centroid);
    centroidBounds[1] = Helper.cwisemax(centroidBounds[1], rtObj.aabb.centroid);
    rtObjects.add(rtObj);
    // update merged AABB
    aabb.update(rtObj.aabb);
  }
  // split BVH into smaller ones
  public void split() {
    if (rtObjects.size() <= 5) return;
    // find the axis with largest bound
    int longestAxis = 0;
    float maxBound = centroidBounds[1].x - centroidBounds[0].x, boundY = centroidBounds[1].y - centroidBounds[0].y;
    if (boundY > maxBound) {
      maxBound = boundY;
      longestAxis = 1;
    }
    if (centroidBounds[1].z - centroidBounds[0].z > maxBound) 
      longestAxis = 2;
    // quick select the middle rtObject along the target axis
    int lo = 0, hi = rtObjects.size() - 1, k = rtObjects.size() / 2;
    while (lo <= hi) {
      int j = partition(lo, hi, longestAxis);
      if (j == k) break;
      if (j < k) lo = j+1;
      else hi = j-1;
    }
    // divide rtObjects into two groups
    children[0] = new BVH(new ArrayList<>(rtObjects.subList(0, k)));
    children[1] = new BVH(new ArrayList<>(rtObjects.subList(k, rtObjects.size())));
    children[0].split();
    children[1].split();
  }
  int partition(int lo, int hi, int axis) {
    int i = lo, j = hi+1;
    float pivot = Helper.getAxisValue(rtObjects.get(lo).aabb.centroid, axis);
    while (true) {
      while (i < hi && Helper.getAxisValue(rtObjects.get(++i).aabb.centroid, axis) > pivot);
      while (j > lo && Helper.getAxisValue(rtObjects.get(--j).aabb.centroid, axis) < pivot);
      if (i >= j) break;
      Collections.swap(rtObjects, i, j);
    }
    Collections.swap(rtObjects, lo, j);
    return j;
  }
  @Override public void applyTransform(Matrix t) {}
  @Override public HitInfo hitTest(Ray ray) { 
    HitInfo hitAABB = super.hitTest(ray);
    if (hitAABB == null) 
      return null;
    if (children[0] == null && children[1] == null) {
      HitInfo closetHit = null;
      for (RTObject obj: rtObjects) {
        HitInfo hit = obj.hitTest(ray);
        if (hit != null && (closetHit == null || hit.time < closetHit.time))
          closetHit = hit;
      }
      return closetHit;
    } else {
      HitInfo hit0 = children[0].hitTest(ray);
      HitInfo hit1 = children[1].hitTest(ray);
      if (hit0 == null) return hit1;
      if (hit1 == null) return hit0;
      return hit0.time < hit1.time ? hit0 : hit1;
    }
  }
}

class Renderable implements HitTest {
  public Matrix tMat, invMat;
  public boolean instanced, isBvh;
  public int surfId;
  public RTObject rtObj;
  public Renderable(int surfId, Matrix tMat, RTObject rtObj, boolean instanced, boolean isBvh) { 
    this.tMat = tMat;
    invMat = tMat.invert();
    this.surfId = surfId; 
    this.instanced = instanced;
    this.isBvh = isBvh;
    this.rtObj = rtObj;
  }
  public int getSurfId() { return surfId; }
  public void applyTransformation() { 
    if (instanced || isBvh) return;
    rtObj.applyTransform(tMat); 
  }
  @Override public HitInfo hitTest(Ray ray) {
    if (instanced) {
      HitInfo info = rtObj.hitTest(ray.transform(invMat));
      // we should apply transform tMat to normal, to get right shading 
      // TODO: is this a right formula to transform normal
      //if (info != null) info.normal = invMat.transpose().timesVec(info.normal).normalize();
      if (info != null) info.normal = tMat.timesVec(info.normal).normalize();
      return info;
    }
    return rtObj.hitTest(ray);
  }
}

class Scene {
  public ArrayList<Light> lights;
  public ArrayList<PVector> surfs;
  public ArrayList<Renderable> renderables;
  public HashMap<String, Renderable> InstanceSrc;
  public float tanTheta;
  public float aspectRatio = 1;
  public PVector bgColor;
  public Stack<Matrix> mStack;
  public Scene() {
    lights = new ArrayList<>();
    surfs = new ArrayList<>();
    renderables = new ArrayList<>();
    InstanceSrc = new HashMap<>();
    tanTheta = 0; // tan(FOV/2)
    aspectRatio = 1;
    bgColor = Helper.vZero();
    mStack = new Stack<>();
    mStack.push(new Identity(4)); // always push I
  }
  public void reset() {
    lights.clear();
    surfs.clear();
    renderables.clear();
    InstanceSrc.clear();
    tanTheta = 0; // tan(FOV/2)
    aspectRatio = 1;
    bgColor = Helper.vZero();
    mStack.clear();
    mStack.push(new Identity(4)); // always push I
  }
  public void addLight(Light li) { lights.add(li); }
  public void addSurf(PVector kd) { surfs.add(kd); }
  public void addSceneObj(RTObject rtObj, boolean isBvh) { 
    renderables.add(new Renderable(surfs.size() - 1, new Matrix(mStack.peek()), rtObj, false, isBvh));
  }
  public void sourceNamedObj(String name) {
    // remove last SceneObject
    Renderable rd = renderables.remove(renderables.size() - 1);
    InstanceSrc.put(name, rd);
  }
  public void addNamedObj(String name) {
    Renderable src = InstanceSrc.get(name);
    renderables.add(new Renderable(src.surfId, new Matrix(mStack.peek()), src.rtObj, true, src.isBvh));
  }
  // shadow ray test
  public boolean shadowRayTest(Ray ray) {
    for (Renderable rd: renderables) {
      HitInfo hit = rd.hitTest(ray);
      if (hit != null && hit.time > 0.000001 && hit.time < 1.0)
        return true;
    }
    return false;
  }
  public void render() {
    // apply transformation
    for (Renderable rd: renderables) rd.applyTransformation();
    // Ray Tracing
    for(int y = 0; y < height; y++) {
      for(int x = 0; x < width; x++) {
        Ray ray = new Ray(Helper.vZero(), new PVector((x+0.5 - width / 2) * 2 / width * tanTheta * aspectRatio, -1 * (y+0.5 - height / 2) * 2 / height * tanTheta, -1));
        HitInfo closetHit = null;
        Renderable hitObj = null;
        for (Renderable rd: renderables) {
          HitInfo hit = rd.hitTest(ray);
          if (hit != null && (closetHit == null || hit.time < closetHit.time)) {
            closetHit = hit;
            hitObj = rd;
          }
        }
        color c = color(round(bgColor.x * 255), round(bgColor.y * 255), round(bgColor.z * 255));
        if (closetHit != null) {
          closetHit.correctNormal(ray);
          c = shading(closetHit, ray, hitObj);
        }
        set (x, y, c);
      }
    }
  }
  // shading function
  color shading(HitInfo hit, Ray ray, Renderable obj) {
    PVector diffuse = Helper.vZero();
    PVector kd = surfs.get(obj.getSurfId());
    PVector p = ray.getPoint(hit.time);
    PVector n = hit.normal;
    for (Light light: lights) {
      PVector lightDir = PVector.sub(light.pos, p);
      // test shadow ray
      if (shadowRayTest(new Ray(p, lightDir))) continue;
      lightDir.normalize();
      PVector I = light.intensity;
      float NdotL = max(0, PVector.dot(lightDir, n));
      diffuse.add(Helper.cwiseproduct(kd, I).mult(NdotL));
    }
    return color(round(min(1, diffuse.x) * 255), round(min(1, diffuse.y) * 255), round(min(1, diffuse.z) * 255));
  }
}
// ================ Gloabl Vars ================
Scene myScene = new Scene();
char lastPressedKey = 0;
BVH tmpBVH = null;
int timer;
// ================ Procedure ==================
void setup() {
  size (300, 300);  
  noStroke();
  background (0, 0, 0);
}
void keyPressed() {
  if (lastPressedKey == key) return;
  myScene.reset();
  switch(key) {
    case '1': interpreter("s01.cli"); break;
    case '2': interpreter("s02.cli"); break;
    case '3': interpreter("s03.cli"); break;
    case '4': interpreter("s04.cli"); break;
    case '5': interpreter("s05.cli"); break;
    case '6': interpreter("s06.cli"); break;
    case '7': interpreter("s07.cli"); break;
    case '8': interpreter("s08.cli"); break;
    case '9': interpreter("s09.cli"); break;
    case '0': interpreter("s10.cli"); break;
    case 'a': interpreter("s11.cli"); break;
  }
  lastPressedKey = key;
}
// this routine parses the text in a scene description file
void interpreter(String file) {
  println("Parsing '" + file + "'");
  String str[] = loadStrings (file);
  if (str == null) println ("Error! Failed to read the file.");
  
  Tri tmpTri = null;

  for (int i = 0; i < str.length; i++) {
    String[] token = splitTokens (str[i], " ");   // get a line and separate the tokens
    if (token.length == 0) continue;              // skip blank lines
    
    if (token[0].equals("fov")) {
      myScene.tanTheta = tan(radians(float(token[1])/2));
    }
    else if (token[0].equals("background")) {
      myScene.bgColor = new PVector(float(token[1]), float(token[2]), float(token[3]));
    }
    else if (token[0].equals("light")) {
      myScene.addLight(new Light(new PVector(float(token[1]), float(token[2]), float(token[3])), new PVector(float(token[4]), float(token[5]), float(token[6]))));
    }
    else if (token[0].equals("surface")) {
      myScene.addSurf(new PVector(float(token[1]), float(token[2]), float(token[3])));
    }    
    else if (token[0].equals("begin")) {
      tmpTri = new Tri();
    }
    else if (token[0].equals("vertex")) {
      tmpTri.addVertex(new PVector(float(token[1]), float(token[2]), float(token[3])));
    }
    else if (token[0].equals("end")) {
      if (tmpBVH != null) {
        tmpTri.applyTransform(myScene.mStack.peek());
        tmpBVH.addRTObj(tmpTri);
      }
      else 
        myScene.addSceneObj(tmpTri, false);
      tmpTri = null;
    }
    else if (token[0].equals("box")) {
      if (tmpBVH != null) {
        Box b = new Box(new PVector(float(token[1]), float(token[2]), float(token[3])), new PVector(float(token[4]), float(token[5]), float(token[6])));
        b.applyTransform(myScene.mStack.peek());
        tmpBVH.addRTObj(b);
      }
      else 
        myScene.addSceneObj(new Box(new PVector(float(token[1]), float(token[2]), float(token[3])), new PVector(float(token[4]), float(token[5]), float(token[6]))), false);
    }
    else if (token[0].equals("named_object")) {
      myScene.sourceNamedObj(token[1]);
    }
    else if (token[0].equals("instance")) {
      myScene.addNamedObj(token[1]);
    }
    else if (token[0].equals("begin_accel")) {
      tmpBVH = new BVH();
    }
    else if (token[0].equals("end_accel")) {
      tmpBVH.split();
      myScene.addSceneObj(tmpBVH, true);
      tmpBVH = null;
    }
    else if (token[0].equals("render")) {
      reset_timer();
      myScene.render();   // this is where you should perform the scene rendering
      print_timer();
    }
    else if (token[0].equals("read")) {
      interpreter(token[1]);
    }
    else if (token[0].equals("translate")) {
      Matrix T = new Matrix(new float[][] {{1, 0, 0, float(token[1])}, 
                                           {0, 1, 0, float(token[2])}, 
                                           {0, 0, 1, float(token[3])}, 
                                           {0, 0, 0, 1}});
      myScene.mStack.push(myScene.mStack.pop().times(T));
    }
    else if (token[0].equals("scale")) {
      Matrix S = new Matrix(new float[][] {{float(token[1]), 0, 0, 0}, 
                                           {0, float(token[2]), 0, 0}, 
                                           {0, 0, float(token[3]), 0}, 
                                           {0, 0, 0, 1}});
      myScene.mStack.push(myScene.mStack.pop().times(S));
    }
    else if (token[0].equals("rotate")) {
      float rad = radians(float(token[1]));
      Matrix R = null;
      if (int(token[2]) == 1) 
        R = new Matrix(new float[][] {{1, 0,        0,        0}, 
                                     {0, cos(rad), -sin(rad), 0}, 
                                     {0, sin(rad), cos(rad),  0}, 
                                     {0, 0,        0,         1}});
      else if (int(token[3]) == 1)
        R = new Matrix(new float[][] {{cos(rad), 0, sin(rad), 0},  // here is positive
                                      {0,        1, 0,         0}, 
                                      {-sin(rad), 0, cos(rad),  0}, // here is negative
                                      {0,        0, 0,         1}});
      else if (int(token[4]) == 1)      
        R = new Matrix(new float[][] {{cos(rad), -sin(rad), 0, 0}, 
                                      {sin(rad), cos(rad),  0, 0}, 
                                      {0,        0,         1, 0}, 
                                      {0,        0,         0, 1}});
      myScene.mStack.push(myScene.mStack.pop().times(R));
    }
    else if (token[0].equals("push")) {
      myScene.mStack.push(new Matrix(myScene.mStack.peek()));
    }
    else if (token[0].equals("pop")) {
      if (myScene.mStack.empty())
        println ("pop failed");
      else myScene.mStack.pop();
    }
    else if (token[0].equals("#")) {
      // comment (ignore)
    }
    else {
      println ("unknown command: " + token[0]);
    }
  }
}
// prints mouse location clicks, for help in debugging
void mousePressed() { println ("You pressed the mouse at " + mouseX + " " + mouseY); }
// you don't need to add anything in the "draw" function for this project
void draw() {}
void reset_timer() { timer = millis(); }
void print_timer() {
  int new_timer = millis();
  int diff = new_timer - timer;
  float seconds = diff / 1000.0;
  println ("timer = " + seconds);
}
