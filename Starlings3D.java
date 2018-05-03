import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.*; 
import java.io.*;


public class Starlings3D extends PApplet {

  Flock flock;
  PImage bg;

  public void setup() 
  {  
    flock = new Flock();
    // Add an initial set of boids into the system
    for (int i = 0; i < 200; i++)
    {
      float w,h,d;
      w = random(2144.0f);
      h = random(1124.0f);
      d = random(100.0f);
      flock.addBoid(new Boid(w,h,d));
    }

    bg = loadImage("sky10.jpg");
  }

  public void draw() 
  {
    background(bg);
    camera(mouseX, mouseY, (height/2) / tan(PI/6), width/2, height/2, 0, 0, 1, 0);
    flock.run();
  }

  public void mousePressed() 
  {
    if (mousePressed && (mouseButton == LEFT)) 
    {
      float d = random(100.0f);
      flock.addBoid(new Boid(mouseX,mouseY,d));
    }
    else if (mousePressed && (mouseButton == RIGHT)) 
    {
      float d = random(100.0f);
      flock.addPredator(new Predator(mouseX,mouseY,d));
    }
    else
    {
      float d = random(100.0f);
      flock.addObstacle(new Obstacle(mouseX,mouseY,d));
    }
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

    public void run() {
      for (Boid b : boids) 
      {
        b.run(boids,predators,obstacles);
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
      maxspeed = 5;
      maxacc = 0.3f;
    }

    public void run(ArrayList<Boid> boids, ArrayList<Predator> predators, ArrayList<Obstacle> obstacles) {
      flock(boids,predators,obstacles);
      update();
      borders();
      render(predators,obstacles);
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
      sep.mult(1.8f);
      ali.mult(1.2f);
      coh.mult(1.2f);
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

      float desiredseparation = 20.0f;
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
      float neighbordist = 50; //also add angle to local region
      PVector sum = new PVector(0, 0, 0);
      int count = 0;
      for (Boid other : boids)
      {
        float d = PVector.dist(position, other.position);
        PVector diff = PVector.sub(position, other.position);
        float angle = PVector.angleBetween(velocity,diff);
        if ((d > 0) && (d < neighbordist) && ((angle<2.5) || (angle>2.5))) {
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
      float neighbordist = 50;
      PVector sum = new PVector(0, 0, 0);
      int count = 0;
      for (Boid other : boids) {
        float d = PVector.dist(position, other.position);
        PVector diff = PVector.sub(position, other.position);
        float angle = PVector.angleBetween(velocity,diff);
        if ((d > 0) && (d < neighbordist) && ((angle<2.5) || (angle>2.5))) 
        {
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
        PVector ab = PVector.sub(position, p.position);
        float angle = PVector.angleBetween(velocity,ab);

        if ((d > 0) && (d < predatordist) && ((angle<2.5) || (angle>2.5))) 
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
        PVector ab = PVector.sub(position, o.position);
        float angle = PVector.angleBetween(velocity,ab);

        if ((d > 0) && (d < obstdist) && ((angle<2.5) || (angle>2.5))) {
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

    public void render(ArrayList<Predator> predators, ArrayList<Obstacle> obstacles) {
      for (Predator p : predators) 
      {
        p.render();
      } 
      for (Obstacle o : obstacles) 
      {
        o.render();
      } 
      float theta1 = velocity.x/velocity.mag();
      float theta2 = velocity.y/velocity.mag();
      float theta3 = velocity.z/velocity.mag();
      fill(0, 100);
      stroke(138,43,226);
      pushMatrix();
      translate(position.x, position.y, position.z);
      rotateX(theta1);
      rotateY(theta2);
      rotateZ(theta3);
      beginShape();
      vertex(0, -r*2, 0);
      vertex(-r, r*2, 0);
      vertex(r, r*2, 0);
      endShape();
      popMatrix();
    }

    // Wraparound
    public void borders() {
      if (position.x < -r) position.x = width+r;
      if (position.y < -r) position.y = height+r;
      if (position.x > width+r) position.x = -r;
      if (position.y > height+r) position.y = -r;
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
      if (position.x < -6.0f) position.x = width+6.0f;
      if (position.y < -6.0f) position.y = height+6.0f;
      if (position.x > width+6.0f) position.x = -6.0f;
      if (position.y > height+6.0f) position.y = -6.0f;
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