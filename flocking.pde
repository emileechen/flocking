Flock flock;
PFont f;
boolean separation = true;
boolean alignment = true;
boolean cohesion = true;
boolean radi = false;

float separationTime = 0;
float alignmentTime = 0;
float cohesionTime = 0;
float going = 4;


void setup() {
//  frameRate(20);
  size(800, 600);
  flock = new Flock();
  for (int i = 0; i < 20; i++) {
//    flock.addBoid(new Boid(width/2, height/2));
    flock.addBoid(new Boid(random(width/4, width/4*3), random(height/4, height/4*3)));
  }
  
  // Create font
  f = createFont("FiraMono.tff", 16);
  textFont(f);
  
}

void draw() {
  background(40);
  
  if (mousePressed) {
    flock.addBoid(new Boid(mouseX, mouseY));
  }
  
  flock.run();
  
  // Write number of boids
  fill(200);
  textAlign(LEFT, BOTTOM);
  text("NUM: " + flock.boids.size(), 10, height - 10);
  
  // Write separation status
  if (separationTime >= 40) {
    separationTime -= going;
    fill(separationTime);
    textAlign(RIGHT, BOTTOM);
    String status = separation ? "ON" : "OFF";
    text("SEPARATION " + status, width - 10, height - 10);
  }
  if (alignmentTime >= 40) {
    alignmentTime -= going;
    fill(alignmentTime);
    textAlign(RIGHT, BOTTOM);
    String status = alignment ? "ON" : "OFF";
    text("ALIGNMENT " + status, width - 10, height - 10);
  }
  if (cohesionTime >= 40) {
    cohesionTime -= going;
    fill(cohesionTime);
    textAlign(RIGHT, BOTTOM);
    String status = cohesion ? "ON" : "OFF";
    text("COHESION " + status, width - 10, height - 10);
  }
  
}

void keyPressed() {
  if (key == '1') {
    separation = !separation;
    separationTime = 255;
  }
  else if (key == '2') {
    alignment = !alignment;
    alignmentTime = 255;
  }
  else if (key == '3') {
    cohesion = !cohesion;
    cohesionTime = 255;
  }
  else if (key == '4') {
    radi = !radi;
  }
}

class Flock {
  ArrayList<Boid> boids;
  
  Flock() {
    boids = new ArrayList<Boid>();
  }
  
  void run() {
    for (Boid b : boids) {
      b.run(boids);
    }
  }
  
  void addBoid(Boid b) {
    boids.add(b);
  }
}


class Boid {
  PVector position;
  PVector velocity;
  PVector acceleration;
  
  float r = 1;
  float bodylength = 2;
  float forceMax = 3;
  float speedMax = 2;
  float speedCruise = 2;
  
  float radiusS = 2;  // Separation radius
  float blindAngleBackS = 30;
  float radiusA = 5;  // Alignment radius
  float blindAngleBackA = 30;
  float blindAngleFrontA = 30;
  float radiusC = 15;  // Cohesion radius
  float blindAngleBackC = 45;
  
  float weightS = 10;
  float weightA = 5;
  float weightC = 2;
  float relaxationTime = 0.2;
  float weightRand = 0.5;
  
  
  Boid(float x, float y) {
    position = new PVector(x, y);
    velocity = PVector.random2D();
    acceleration = new PVector(0, 0);
    
    r = 4;
    bodylength *= r;
    radiusS *= bodylength;
    radiusA *= bodylength;
    radiusC *= bodylength;
  }
  
  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();  // Wraparound
    render();
  }
  
  void applyForce(PVector force) {
    acceleration.add(force);
  }
  
  void update() {
    // Update velocity
    velocity.add(acceleration);
//    // Limit speed
//    velocity.limit(speedMax);
    // Update position
    position.add(velocity);
    // Resert acceleration
    acceleration.mult(0);
  }
  
  void render() {
    // Draw the radi
    if (radi) {
      fill(0, 0);
      if (separation) {
        stroke(200);
        ellipse(position.x, position.y, radiusS, radiusS);
      }
      if (alignment) {
        stroke(150);
        ellipse(position.x, position.y, radiusA, radiusA);
      }
      if (cohesion) {
        stroke(100);
        ellipse(position.x, position.y, radiusC, radiusC);
      }
    }

    // Draw the boid
    float theta = velocity.heading() - radians(90);
    fill(200, 100);  // grey fill
    stroke(255);  // white stroke
    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, r);
    vertex(r / 2, - r);
    vertex(- r / 2, - r);
    endShape();
    popMatrix();
  }
  
  void borders() {
    if (position.x < - r) {
      position.x = width + r;
    }
    if (position.y < - r) {
      position.y = height + r;
    }
    if (position.x > width + r) {
      position.x = - r;
    }
    if (position.y > height + r) {
      position.y = - r;
    }
  }
  
  void flock(ArrayList<Boid> boids) {
    
    int numS = 0;
    int numA = 0;
    int numC = 0;
    PVector dirS = new PVector(0, 0);
    PVector dirA = new PVector(0, 0);
    PVector dirC = new PVector(0, 0);
    PVector forceS = new PVector(0, 0);
    PVector forceA = new PVector(0, 0);
    PVector forceC = new PVector(0, 0);
    PVector forceSpeed = new PVector(0, 0);
    PVector forceRand = new PVector(0, 0);
    PVector forceNet = new PVector(0, 0);
    
    // Loop through all the boids
    for (Boid neighbour : boids) {
      PVector forward = velocity.get();
      forward.normalize();
      PVector backward = forward.get();
      backward.mult(-1.0);
      float distance = PVector.dist(position, neighbour.position);
      
      // Do not consider yourself a neighbour
      if (distance > 0.0) {
        PVector diffVector = neighbour.position.get();
        diffVector.sub(position);
        
        if (separation) {
          // Separation force
          if (distance < radiusS) {
            if (!checkWithinAngle(backward, diffVector, blindAngleBackS)) {
              numS += 1;
              PVector tmpDirS = diffVector.get();
              tmpDirS.div(sq(distance));
              dirS.add(tmpDirS);
            }
          }
        }
        if (alignment) {
          // Alignment force
          if (distance < radiusA) {
            if (!checkWithinAngle(backward, diffVector, blindAngleBackA) && !checkWithinAngle(forward, diffVector, blindAngleFrontA)) {
              numA += 1;
              PVector tmpDirA = neighbour.velocity.get();
              tmpDirA.normalize();
              dirA.add(tmpDirA);
            }
          }
        }
        if (cohesion) {
          // Cohesion force
          if (distance < radiusC) {
            if (!checkWithinAngle(backward, diffVector, blindAngleBackC)) {
              numC += 1;
              PVector tmpDirC = diffVector.get();
              tmpDirC.div(distance);
              dirC.add(tmpDirC);
            }
          }
        }
      }
    }
    
    if (numS != 0) {  // If there are fish in the separation radius
      dirS.mult(- 1.0 / numS);
      dirS.normalize();
      forceS = dirS.get();
      forceS.mult(weightS);
    }
    if (numA != 0) {  // If there are fish in the alignment radius
      dirA.mult(1.0 / numA);
      dirA.normalize();
      forceA = dirA.get();
      forceA.mult(weightA);
    }
    if (numC != 0) {  // If there are fish in the cohesion radius
      dirC.mult(1.0 / numC);
      dirC.normalize();
      forceC = dirC.get();
      forceC.mult(weightC);
    }
    
    // Cruise speed compensating force
    forceSpeed = velocity.get();
    forceSpeed.normalize();  // Forward vector
    forceSpeed.mult((1 / relaxationTime) * (speedCruise - velocity.mag()));
    
    // Random force
    forceRand.set(random(-1, 1), random(-1, 1));
    forceRand.mult(weightRand);
    
    forceNet.add(forceS);
    forceNet.add(forceA);
    forceNet.add(forceC);
    forceNet.add(forceSpeed);
    forceNet.add(forceRand);
    
    forceNet.limit(forceMax);
    applyForce(forceNet);
  }
}
    
// Returns true if within the angle limit (in the blindangle)
boolean checkWithinAngle(PVector a, PVector b, float angle) {
  PVector v = a.get();
  v.normalize();
  PVector u = b.get();
  u.normalize();
  float dot = v.dot(u);
  float rad = radians(angle);
  float limit = cos(rad);
  
  return dot > limit;
}
