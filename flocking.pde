Flock flock;
Environment env;

PFont f;

boolean separation = true;
boolean alignment = true;
boolean cohesion = true;
boolean radii = false;
boolean dRadii = true;  // Dynamic radius of perception

float separationTime = 0;
float alignmentTime = 0;
float cohesionTime = 0;
float radiiTime = 0;
float dRadiiTime = 0;
float going = 7;

float deltaTime = 0.03;


void setup() {
  frameRate(1/deltaTime);
  size(800, 600);
  flock = new Flock();
  for (int i = 0; i < 50; i++) {
    //    flock.addBoid(new Boid(width/2, height/2));
    flock.addBoid(new Boid(random(width/4, width/4*3), random(height/4, height/4*3)));
  }

  env = new Environment();
//  env.addObstacle(new Obstacle(width/2, height/2));

  // Create font
  f = createFont("FiraMono.tff", 16);
  textFont(f);
}

void draw() {
  background(40);

  flock.run();
  env.run();

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
  if (radiiTime >= 40) {
    radiiTime -= going;
    fill(radiiTime);
    textAlign(RIGHT, BOTTOM);
    String status = radii ? "ON" : "OFF";
    text("RADII " + status, width - 10, height - 10);
  }
  if (dRadiiTime >= 40) {
    dRadiiTime -= going;
    fill(dRadiiTime);
    textAlign(RIGHT, BOTTOM);
    String status = dRadii ? "ON" : "OFF";
    text("DYNAMIC RADII " + status, width - 10, height - 10);
  }
}

void mousePressed() {
  if (mouseButton == LEFT) {
    flock.addBoid(new Boid(mouseX, mouseY));
  }
  else if (mouseButton == RIGHT) {
    env.addObstacle(new Obstacle(mouseX, mouseY));
  }
}

void keyPressed() {
  if (key == '1') {
    separation = !separation;
    separationTime = 255;
  } else if (key == '2') {
    alignment = !alignment;
    alignmentTime = 255;
  } else if (key == '3') {
    cohesion = !cohesion;
    cohesionTime = 255;
  } else if (key == '4') {
    radii = !radii;
    radiiTime = 255;
  } else if (key == '5') {
    dRadii = !dRadii;
    dRadiiTime = 255;
    for (Boid b : flock.boids) {
      b.radiusA = b.radiusAmax;
      b.radiusC = b.radiusCmax;
    }
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
  float forceMax = 0.5;
  float speedCruise = 2;

  // Perception radius interpolation weights
  float smoothness = 0.5;
  float influence = 5;

  float radiusS = 2;  // Separation radius
  float blindAngleBackS = 30;
  float radiusA = 5;  // Alignment radius
  float radiusAmax = 5;
  float blindAngleBackA = 30;
  float blindAngleFrontA = 30;
  float radiusC = 15;  // Cohesion radius
  float radiusCmax = 15;
  float blindAngleBackC = 45;

  float weightS = 10;
  float weightA = 5;
  float weightC = 9;
  float relaxationTime = 0.2;
  float weightRand = 0.5;


  Boid(float x, float y) {
    position = new PVector(x, y);
//    velocity = new PVector(1, 0);
    velocity = PVector.random2D();
    acceleration = new PVector(0, 0);

    r = 4;
    bodylength *= r;
    radiusS *= bodylength;
    radiusA *= bodylength;
    radiusAmax *= bodylength;
    radiusC *= bodylength;
    radiusCmax *= bodylength;
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
    // Update position
    position.add(velocity);
    // Resert acceleration
    acceleration.mult(0);
  }

  void render() {
    float theta = velocity.heading();

    // Draw the radi
    if (radii) {
      noFill();
      pushMatrix();
      translate(position.x, position.y);
      rotate(theta + PI);
      if (separation) {
        stroke(200);
        arc(0, 0, radiusS, radiusS, radians(blindAngleBackS), TWO_PI - radians(blindAngleBackS), PIE);
        //        ellipse(position.x, position.y, radiusS, radiusS);
      }
      if (alignment) {
        stroke(150);
        arc(0, 0, radiusA, radiusA, radians(blindAngleBackA), PI - radians(blindAngleFrontA), PIE);
        arc(0, 0, radiusA, radiusA, PI + radians(blindAngleFrontA), TWO_PI - radians(blindAngleBackA), PIE);
        //        ellipse(position.x, position.y, radiusA, radiusA);
      }
      if (cohesion) {
        stroke(100);
        arc(0, 0, radiusC, radiusC, radians(blindAngleBackC), TWO_PI - radians(blindAngleBackC), PIE);
        //        ellipse(position.x, position.y, radiusC, radiusC);
      }
      popMatrix();
    }

    // Draw the boid
    fill(200, 100);  // grey fill
    stroke(255);  // white stroke
    pushMatrix();
    translate(position.x, position.y);
    rotate(theta - radians(90));
    triangle(0, r, r/2, -r, -r/2, -r);
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

  // Given number of neighbours in radius, interpolate a new radius.
  float calcPerceptionRadius(float radius, float radiusMin, float radiusMax, int num) {
    float s = smoothness * deltaTime;
    // Density dependent term
    float d = radiusMax - (influence * num);
    return max(radiusMin, ((1 - s) * radius) + (s * d));
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

    PVector forceAvoid = new PVector(0, 0);

    PVector forward = velocity.get();
    forward.normalize();
    PVector backward = forward.get();
    backward.mult(-1.0);

    PVector threatCentre = new PVector(0, 0);
    PVector threatInter = new PVector(0, 0);
    float threatDist = MAX_FLOAT;

    for (Obstacle o : env.obstacles) {
        PVector toObs = o.position.get();
        toObs.sub(position);
        PVector towardObs = toObs.get();
        towardObs.normalize();
        PVector forwardBuffer = forward.get();
        forwardBuffer.mult(o.buffer);
        
        if (forward.dot(towardObs) > 0) {
          // Obstacle is in front of the agent
          PVector front = projection(toObs, forwardBuffer);

          if (front.mag() <= o.r + o.buffer) {            
            // Obstacle is not too far ahead
            PVector intersection = lineSphereIntersection(position, forward, o.position, o.r + o.buffer);
            PVector toInter = intersection.get();
            toInter.sub(position);
            float distToInter = toInter.mag();
            
            if (distToInter < threatDist) {
              // Is closer than current, so update
              threatCentre = o.position;
              threatInter = intersection;
              threatDist = distToInter;
            }
          }
        }
    }
    if (threatDist < MAX_FLOAT) {
      forceAvoid = threatCentre.get();
      forceAvoid.sub(threatInter);
      forceAvoid.rotate(PI);
      forceAvoid.normalize();
      forceAvoid.mult(20);
      println(forceAvoid);
    }
    println(threatDist);

    // Loop through all the boids
    for (Boid neighbour : boids) {
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
          if (radiusS < distance && distance < radiusA) {
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
          if (radiusA < distance && distance < radiusC) {
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
    forceNet.add(forceAvoid);

    forceNet.limit(forceMax);
    applyForce(forceNet);

    if (dRadii) {
      // Calculate and update perception radii
      radiusA = calcPerceptionRadius(radiusA, radiusS, radiusAmax, numA);
      radiusC = calcPerceptionRadius(radiusC, radiusAmax, radiusCmax, numC);
    }
  }
}

class Environment {
  ArrayList<Obstacle> obstacles;

  Environment() {
    obstacles = new ArrayList<Obstacle>();
  }

  void run() {
    for (Obstacle o : obstacles) {
      o.run();
    }
  }

  void addObstacle(Obstacle o) {
    obstacles.add(o);
  }
}

class Obstacle {
  PVector position;
  float r;
  float buffer;

  Obstacle(float x, float y) {
    position = new PVector(x, y);
    r = 10;
    buffer = 30;
    this.render();
  }

  void run() {
    render();
  }

  void render() {
    fill(255, 255);  // grey fill
    stroke(255);  // white stroke
    ellipse(position.x, position.y, r, r);
    
    if (radii) {
      noFill();
      stroke(200);
      ellipse(position.x, position.y, r + buffer, r + buffer);
    }
    
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

// Returns the vector projection of a onto b
PVector projection(PVector a, PVector b) {
  float scalar = a.dot(b) / b.dot(b);
  PVector vec = b.get();
  vec.mult(scalar);
  return vec;
}

PVector lineSphereIntersection(PVector origin, PVector line, PVector centre, float radius) {
  PVector l = line.get();
  l.normalize();
  
  PVector oSubC = origin.get();
  oSubC.sub(centre);
  float det = sq(l.dot(oSubC)) - oSubC.magSq() + sq(radius);
  
  float d = -1 * l.dot(oSubC);
  
  // Only has one solution
  if (det == 0);
  // There are two solutions, we only care about the closer one
  else if (det > 0) {
    d -= sqrt(det);
  }
  
  PVector intersect = l.get();
  intersect.mult(d);
  intersect.add(origin);
  return intersect;
}

