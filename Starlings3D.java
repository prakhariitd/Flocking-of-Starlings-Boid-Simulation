import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Starlings3D extends PApplet {


  Flock flock;
  PImage bg;
  int num = 200;
  float depth = 1500.0f;
  float camdepth = 0.0f;

  public void setup() 
  {  
    flock = new Flock();
    // Add an initial set of boids into the system
    for (int i = 0; i < num; i++)
    {
      float w,h,d;
      w = random(400.0f);
      h = random(200.0f);
      d = random(400.0f);
      flock.addBoid(new Boid(w+672.0f,h+362.0f,d+850.0f));
    }
    bg = loadImage("sky10.jpg");
  }

  public void draw() 
  {
    background(bg);
    camera(mouseX, mouseY, camdepth, width/2, height/2, depth/2, 0, 1, 0); //(height/2) / tan(PI/12)
    println(num);
    flock.run();
  }

  public void mousePressed() 
  {
    if (mousePressed && (mouseButton == LEFT)) 
    {
      float d = random(50.0f);
      flock.addBoid(new Boid(mouseX,mouseY,d));
      num++;
    }
    else if (mousePressed && (mouseButton == RIGHT)) 
    {
      float d = random(50.0f);
      flock.addPredator(new Predator(mouseX,mouseY,d));
    }
    else
    {
      float d = random(50.0f);
      flock.addObstacle(new Obstacle(mouseX,mouseY,d));
    }
  }
  
  public void mouseWheel(MouseEvent event) {
  float e = event.getCount();
  camdepth+=-50*e;
}

  public void settings() {  
    size(2144, 1124, P3D); 
  }

  // The Flock (a list of Boid objects)
  class Flock {
    ArrayList<Boid> boids;
    ArrayList<Predator> predators;
    ArrayList<Obstacle> obstacles; 

    Flock() {
      boids = new ArrayList<Boid>();
      predators = new ArrayList<Predator>();
      obstacles = new ArrayList<Obstacle>();
    }

class Threads extends Thread{
      int a;
      int b;
      ArrayList<Boid> boids2;
      ArrayList<Predator> predators2;
      ArrayList<Obstacle> obstacles2;

      Threads(int a1, int b1, ArrayList<Boid> boids1, ArrayList<Predator> predators1, ArrayList<Obstacle> obstacles1)
      {
        a=a1;
        b=b1;
        boids2=boids1;
        predators2=predators1;
        obstacles2=obstacles1;
      }

    public void run()
    {
       try
        {
            for (int i=a; i<b; i++) 
            {
              boids.get(i).run(boids2,predators2,obstacles2);
            }
        }
        catch (Exception e)
        {
          System.out.println (e);
        }
    }
    }

    public void run() {
      ArrayList<Boid> boids1 = new ArrayList<Boid>();
        for (Boid b : boids) 
        {
          boids1.add(b);
        }
        ArrayList<Predator> predators1 = new ArrayList<Predator>();
        for (Predator p : predators) 
        {
          predators1.add(p);
        }
        ArrayList<Obstacle> obstacles1 = new ArrayList<Obstacle>();
        for (Obstacle o : obstacles) 
        {
          obstacles1.add(o);
        } 
        Threads t1 = new Threads(0,num/8,boids1,predators1,obstacles1);
        Threads t2 = new Threads(num/8,num/4,boids1,predators1,obstacles1);
        Threads t3 = new Threads(num/4,3*num/8,boids1,predators1,obstacles1);
        Threads t4 = new Threads(3*num/8,num/2,boids1,predators1,obstacles1);
        Threads t5 = new Threads(num/2,5*num/8,boids,predators,obstacles);
        Threads t6 = new Threads(5*num/8,3*num/4,boids,predators,obstacles);
        Threads t7 = new Threads(3*num/4,7*num/8,boids,predators,obstacles);
        Threads t8 = new Threads(7*num/8,num,boids,predators,obstacles);
        t1.start();
        t2.start();
        t3.start();
        t4.start();
        t5.start();
        t6.start();
        t7.start();
        t8.start();
        for (Boid b : boids) 
        {
          b.render(predators1,obstacles1);
        }
    }

    public void addBoid(Boid b) {
      boids.add(b);
    }

    public void addPredator(Predator p) {
      predators.add(p);
    }

    public void addObstacle(Obstacle o) {
      obstacles.add(o);
    }

  }

    // The Boid class
  class Boid {

    PVector position;
    PVector velocity;
    PVector acceleration;
    float mass;
    float r;
    float maxacc;    // Maximum steering force
    float maxspeed;    // Maximum speed

    Boid(float x, float y, float z) 
    {
      acceleration = new PVector(0, 0, 0);
      velocity = PVector.random3D();
      position = new PVector(x, y, z);
      mass = random(0.7f,1.3f);
      r = 4.0f;
      maxspeed = 8;
      maxacc = 1.2f;
    }

    public void run(ArrayList<Boid> boids, ArrayList<Predator> predators, ArrayList<Obstacle> obstacles) {
      flock(boids,predators,obstacles);
      update();
      borders();
    }

    public void applyForce(PVector force) {
      PVector acc;
      acc=force.div(mass);
      acceleration.add(acc);
    }

    // We accumulate a new acceleration each time based on three rules
    public void flock(ArrayList<Boid> boids, ArrayList<Predator> predators, ArrayList<Obstacle> obstacles) {
      PVector sep = separate(boids);   // Separation
      PVector ali = align(boids);      // Alignment
      PVector coh = cohesion(boids);   // Cohesion
      PVector esc = escape(predators); // Escaping from Predator
      PVector avo = avoid(obstacles);  //avoid obstacles
      // Arbitrarily weight these forces
      sep.mult(2.8f);
      ali.mult(3.2f);
      coh.mult(2.8f);
      esc.mult(5.0f);
      avo.mult(5.0f);
      // Add the force vectors to acceleration
      applyForce(sep);
      applyForce(ali);
      applyForce(coh);
      applyForce(esc);
      applyForce(avo);
    }

    // Separation
    // Method checks for nearby boids and steers away
    public PVector separate (ArrayList<Boid> boids) {

      float desiredseparation = 30.0f;
      PVector steer = new PVector(0, 0, 0);
      int count = 0;
      for (Boid other : boids)
      {
        float d = PVector.dist(position, other.position);
        if ((d > 0) && (d < desiredseparation)) 
        {
          PVector diff = PVector.sub(position, other.position);
          diff.normalize();
          diff.div(d);        // Weight by distance
          steer.add(diff);
          count++;
        }
      }

      if (count > 0) 
      {
        steer.div((float)count);
      }

      if (steer.mag() > 0) 
      {
        steer.normalize();
        steer.mult(maxspeed);
        steer.sub(velocity);
        steer.limit(maxacc);
      }

      return steer;
    }

    // Alignment
    // For every nearby boid in the system, calculate the average velocity
    public PVector align (ArrayList<Boid> boids) {
      float neighbordist = 150.0f; //also add angle to local region
      PVector sum = new PVector(0, 0, 0);
      int count = 0;
      for (Boid other : boids)
      {
        float d = PVector.dist(position, other.position);
        //PVector diff = PVector.sub(position, other.position);
        //float angle = PVector.angleBetween(velocity,diff);
        if ((d > 0) && (d < neighbordist)) {
          sum.add(other.velocity);
          count++;
        }
      }

      if (count > 0) 
      {
        sum.div((float)count); //average steer
      }

      if (sum.mag() > 0) 
      {
        sum.normalize();
        sum.mult(maxspeed);
        PVector steer = PVector.sub(sum, velocity);
        steer.limit(maxacc);
        return steer;
      } 
      else 
      {
        return new PVector(0, 0, 0);
      }
    }

    // Cohesion
    // For the average position (i.e. center) of all nearby boids, calculate steering vector towards that position
    public PVector cohesion (ArrayList<Boid> boids) {
      float neighbordist = 150.0f;
      PVector sum = new PVector(0, 0, 0);
      int count = 0;
      for (Boid other : boids) {
        float d = PVector.dist(position, other.position);
        //PVector diff = PVector.sub(position, other.position);
        //float angle = PVector.angleBetween(velocity,diff);
        if ((d > 0) && (d < neighbordist)) {
          sum.add(other.position);
          count++;
        }
      }

      if (count > 0) 
      {
        sum.div(count);
      }

      if(sum.mag() > 0)
      {
        PVector desired = PVector.sub(sum, position);
        desired.normalize();
        desired.mult(maxspeed);
        PVector steer = PVector.sub(desired, velocity);
        steer.limit(maxacc);
        return steer;
      } 
      else 
      {
        return new PVector(0, 0, 0);
      }
    }

    public PVector escape (ArrayList<Predator> predators) 
    {
      float predatordist = 100;
      PVector steer = new PVector(0, 0, 0);
      int count = 0;
      for (Predator p : predators) 
      {
        float d = PVector.dist(position, p.position);

        if ((d > 0) && (d < predatordist)) 
        {
          PVector diff = PVector.sub(position, p.position);
          diff.normalize();
          diff.div(d);        // Weight by distance
          steer.add(diff);
          count++;
        }
      }

      if (count > 0) 
      {
        steer.div((float)count);
      }

      if (steer.mag() > 0) 
      {
        steer.normalize();
        steer.mult(maxspeed);
        steer.sub(velocity);
        steer.limit(maxacc);
      }

      return steer;
      
    }
    public PVector avoid (ArrayList<Obstacle> obstacles) 
    {
      float obstdist = 100;
      PVector steer = new PVector(0, 0, 0);
      int count = 0;
      for (Obstacle o : obstacles) 
      {
        float d = PVector.dist(position, o.position);

        if ((d > 0) && (d < obstdist)) {
          PVector diff = PVector.sub(position, o.position);
          diff.normalize();
          diff.div(d);        // Weight by distance
          steer.add(diff);
          count++;
        }
      }

      if (count > 0) 
      {
        steer.div((float)count);
      }

      if (steer.mag() > 0) 
      {
        steer.normalize();
        steer.mult(maxspeed);
        steer.sub(velocity);
        steer.limit(maxacc);
      }

      return steer;
    }

    // Method to update position
    public void update() {
      velocity.add(acceleration);
      velocity.limit(maxspeed);
      position.add(velocity);
      acceleration.mult(0);
    }


  public void cylinder(float bottom, float top, float h, int sides)
  {
    pushMatrix();
    
    translate(0,0,h/2);
    
    float angle;
    float[] x = new float[sides+1];
    float[] z = new float[sides+1];
    
    float[] x2 = new float[sides+1];
    float[] z2 = new float[sides+1];
   
    //get the x and z position on a circle for all the sides
    for(int i=0; i < x.length; i++){
      angle = TWO_PI / (sides) * i;
      x[i] = sin(angle) * bottom;
      z[i] = cos(angle) * bottom;
    }
    
    for(int i=0; i < x.length; i++){
      angle = TWO_PI / (sides) * i;
      x2[i] = sin(angle) * top;
      z2[i] = cos(angle) * top;
    }
   
    //draw the bottom of the cylinder
    beginShape(TRIANGLE_FAN);
   
    vertex(0,0,-h/2);
   
    for(int i=0; i < x.length; i++){
      vertex(x[i], z[i], -h/2);
    }
   
    endShape();
   
    //draw the center of the cylinder
    beginShape(QUAD_STRIP); 
   
    for(int i=0; i < x.length; i++){
      vertex(x[i], z[i], -h/2);
      vertex(x2[i],z2[i],h/2);
    }
   
    endShape();
   
    //draw the top of the cylinder
    beginShape(TRIANGLE_FAN); 
   
    vertex(0,0,h/2);
   
    for(int i=0; i < x.length; i++){
      vertex(x2[i], z2[i], h/2);
    }
   
    endShape();
    
    popMatrix();
  }

    public void render(ArrayList<Predator> predators, ArrayList<Obstacle> obstacles) {
      for (Predator p : predators) 
      {
        p.render();
      } 
      for (Obstacle o : obstacles) 
      {
        o.render();
      } 
      float a = velocity.x;
      float b = velocity.y;
      float c = velocity.z;
      boolean chk=true;
      float theta2 = (atan(-a/c));
      float theta1 = (atan(b/sqrt(a*a+c*c)));
      pushMatrix();
      translate(position.x, position.y, position.z);
      if (a>0)
      {
        if (cos(theta1)*sin(theta2)>0) chk=true;
        else chk=false;
      }
      if (a<0)
      {
        if (cos(theta1)*sin(theta2)<0) chk=true;
        else chk=false;
      }      
      if (chk)
      {
        rotateY(theta2);
        rotateX(theta1);
      }
      else
      {
        rotateY(theta2);
        rotateX(-theta1);
      }
      fill(0, 100);
      stroke(138,43,226);
      cylinder(r,0.0f,5*r,50);
      popMatrix();
    }

    // Wraparound
    public void borders() {
      if (position.x < 0.0f) position.x = width;
      if (position.y < 350.0f) position.y = 770.0f;
      if (position.x > width) position.x = 0.0f;
      if (position.y > 770.0f) position.y = 350.0f;
      if (position.z > depth) position.z = 0.0f;
      if (position.z < 0.0f) position.z = depth;
    }
  }
  
  class Predator {

    PVector position;
    PVector velocity;
    PVector acceleration;
    int count1=0;

    Predator(float x, float y, float z) 
    {
      velocity = PVector.random3D();
      velocity.div(35.0f);
      position = new PVector(x, y, z);
    }

    public void render() 
    {
      if (count1==30000)
      {
        acceleration = PVector.random3D();
        // acceleration.div(35.0f);
        acceleration.limit(0.1f);
        velocity.add(acceleration);
        velocity.limit(0.02f);
        count1=0;
      }
      else count1++;
      
      velocity.limit(0.02f);
      position.add(velocity);
      if (position.x < 400.0f) position.x = width-400.0f;
      if (position.y < 350.0f) position.y = 770.0f;
      if (position.x > width-400.0f) position.x = 400.0f;
      if (position.y > 770.0f) position.y = 350.0f;
      if (position.z > depth) position.z = 200.0f;
      if (position.z < 200.0f) position.z = depth;
      fill(255,0,0, 100);
      stroke(255,0,0);
      pushMatrix();
      translate(position.x, position.y, position.z);
      sphere(12);
      popMatrix();
    }
  }

  class Obstacle {

    PVector position;

    Obstacle(float x, float y, float z) 
    {
      position = new PVector(x, y, z);
    }

    public void render() 
    {
      fill(0, 100);
      stroke(0);
      translate(position.x, position.y, position.z);
      box(25,25,25);
    }
  }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Starlings3D" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
