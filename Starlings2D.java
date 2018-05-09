import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.*; 
import java.io.*;
import java.lang.*;


public class Starlings2D extends PApplet {

  Flock flock;
  PImage bg;
  float ke;
  PVector v_avg;
  int num = 200;

  public void setup() 
  {  
    flock = new Flock();
    for (int i = 0; i < num; i++)
    {
        float w,h;
        w = random(2144.0f);
        h = random(1124.0f);
        flock.addBoid(new Boid(w,h));
      }

      bg = loadImage("sky10.jpg");
  }

  public void draw() 
  {
      background(bg);
      flock.run();
  }

  public void mousePressed() 
  {
      if (mousePressed && (mouseButton == LEFT)) 
      {
        flock.addBoid(new Boid(mouseX,mouseY));
        num++;
      }
      else if (mousePressed && (mouseButton == RIGHT)) 
      {
        flock.addPredator(new Predator(mouseX,mouseY));
      }
      else
      {
        flock.addObstacle(new Obstacle(mouseX,mouseY));
      }
  }

  public void settings() {  
    size(2144, 1124); 
  }

  // The Flock (a list of Boid objects)
  class Flock{
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
        v_avg = new PVector(0,0);
        ke=0;
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
        Threads t5 = new Threads(num/2,5*num/8,boids1,predators1,obstacles1);
        Threads t6 = new Threads(5*num/8,3*num/4,boids1,predators1,obstacles1);
        Threads t7 = new Threads(3*num/4,7*num/8,boids1,predators1,obstacles1);
        Threads t8 = new Threads(7*num/8,num,boids1,predators1,obstacles1);
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
  class Boid{

    PVector position;
    PVector velocity;
    PVector acceleration;
    float mass;
    float r;
    float maxacc;
    float maxspeed; 

    Boid(float x, float y) 
    {
      acceleration = new PVector(0, 0);
      velocity = PVector.random2D();
      position = new PVector(x, y);
      mass = random(0.7f,1.3f);
      r = 4.0f;
      maxspeed = 3;
      maxacc = 0.4f;
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
      esc.mult(4.0f);
      avo.mult(3.5f);
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
      PVector steer = new PVector(0, 0);
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
    public PVector align (ArrayList<Boid> boids) {
      float neighbordist = 50;
      PVector sum = new PVector(0, 0);
      int count = 0;
      for (Boid other : boids)
      {
        float d = PVector.dist(position, other.position);
        PVector diff = PVector.sub(position, other.position);
        float angle = PVector.angleBetween(velocity,diff);
        if ((d > 0) && (d < neighbordist) && ((angle<2.5) || (angle>4.0))) {
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
        return new PVector(0, 0);
      }
    }

    // Cohesion
    public PVector cohesion (ArrayList<Boid> boids) {
      float neighbordist = 50;
      PVector sum = new PVector(0, 0);
      int count = 0;
      for (Boid other : boids) {
        float d = PVector.dist(position, other.position);
        PVector diff = PVector.sub(position, other.position);
        float angle = PVector.angleBetween(velocity,diff);
        if ((d > 0) && (d < neighbordist) && ((angle<2.5) || (angle>4.0))) {
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
        return new PVector(0, 0);
      }
    }

    public PVector escape (ArrayList<Predator> predators) 
    {
      float predatordist = 100;
      PVector sum = new PVector(0, 0);
      PVector steer = new PVector(0, 0);
      int count = 0;
      for (Predator p : predators) 
      {
        float d = PVector.dist(position, p.position);
        PVector ab = PVector.sub(position, p.position);
        float angle = PVector.angleBetween(velocity,ab);

        if ((d > 0) && (d < predatordist) && ((angle<2.5) || (angle>4.0))) 
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
      PVector sum = new PVector(0, 0);
      PVector steer = new PVector(0, 0);
      int count = 0;
      for (Obstacle o : obstacles) 
      {
        float d = PVector.dist(position, o.position);
        PVector ab = PVector.sub(position, o.position);
        float angle = PVector.angleBetween(velocity,ab);

        if ((d > 0) && (d < obstdist) && ((angle<2.5) || (angle>4.0))) {
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
      v_avg.add(velocity);
      ke+=0.5*mass*velocity.mag()*velocity.mag();
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
      v_avg.div((float)num);
      String v = "Average velocity : (" + v_avg.x + "," + v_avg.y + ")";
      String k = "Average Kinetic Energy = " + ke;
      text(v,20,30);
      text(k,20,60);
      System.out.println(v);
      System.out.println(k);
      float theta = velocity.heading() + radians(90);
      fill(0, 100);
      stroke(138,43,226);
      pushMatrix();
      translate(position.x, position.y);
      rotate(theta);
      beginShape(TRIANGLES);
      vertex(0, -r*2);
      vertex(-r, r*2);
      vertex(r, r*2);
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

    Predator(float x, float y) 
    {
      velocity = PVector.random2D();
      velocity.div(35.0f);
      position = new PVector(x, y);
    }

    public void render() 
    {
      if (count1==30000)
      {
        acceleration = PVector.random2D();
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
      translate(position.x, position.y);
      ellipse(0,0,12,12);
      popMatrix();
    }
  }

  class Obstacle {

    PVector position;

    Obstacle(float x, float y) 
    {
      position = new PVector(x, y);
    }

    public void render() 
    {
      fill(0, 100);
      stroke(0);
      rect(position.x, position.y,25,25);
    }
  }

  public static void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Starlings2D" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}