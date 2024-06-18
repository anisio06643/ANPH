// Math Utilities
const EPSILON = 0.000001;

function nearlyEqual(a, b, epsilon = EPSILON) {
    return Math.abs(a - b) < epsilon;
}

function degToRad(degrees) {
    return degrees * (Math.PI / 180);
}

function radToDeg(radians) {
    return radians * (180 / Math.PI);
}

// Vector2 Class
class Vector2 {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }

    add(v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }

    subtract(v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }

    scale(scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }

    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    normalize() {
        let len = this.length();
        if (len > 0) {
            this.x /= len;
            this.y /= len;
        }
        return this;
    }

    static add(v1, v2) {
        return new Vector2(v1.x + v2.x, v1.y + v2.y);
    }

    static subtract(v1, v2) {
        return new Vector2(v1.x - v2.x, v1.y - v2.y);
    }
}

// Vector3 Class
class Vector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    add(v) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }

    subtract(v) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }

    scale(scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
        return this;
    }

    length() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    normalize() {
        let len = this.length();
        if (len > 0) {
            this.x /= len;
            this.y /= len;
            this.z /= len;
        }
        return this;
    }

    static add(v1, v2) {
        return new Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
    }

    static subtract(v1, v2) {
        return new Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    }
}

// Matrix4 Class
class Matrix4 {
    constructor() {
        this.elements = new Float32Array(16);
        this.identity();
    }

    identity() {
        this.elements.set([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ]);
        return this;
    }

    multiply(m) {
        const ae = this.elements;
        const be = m.elements;

        const te = new Float32Array(16);

        const a11 = ae[0], a12 = ae[4], a13 = ae[8], a14 = ae[12];
        const a21 = ae[1], a22 = ae[5], a23 = ae[9], a24 = ae[13];
        const a31 = ae[2], a32 = ae[6], a33 = ae[10], a34 = ae[14];
        const a41 = ae[3], a42 = ae[7], a43 = ae[11], a44 = ae[15];

        const b11 = be[0], b12 = be[4], b13 = be[8], b14 = be[12];
        const b21 = be[1], b22 = be[5], b23 = be[9], b24 = be[13];
        const b31 = be[2], b32 = be[6], b33 = be[10], b34 = be[14];
        const b41 = be[3], b42 = be[7], b43 = be[11], b44 = be[15];

        te[0] = a11 * b11 + a12 * b21 + a13 * b31 + a14 * b41;
        te[4] = a11 * b12 + a12 * b22 + a13 * b32 + a14 * b42;
        te[8] = a11 * b13 + a12 * b23 + a13 * b33 + a14 * b43;
        te[12] = a11 * b14 + a12 * b24 + a13 * b34 + a14 * b44;

        te[1] = a21 * b11 + a22 * b21 + a23 * b31 + a24 * b41;
        te[5] = a21 * b12 + a22 * b22 + a23 * b32 + a24 * b42;
        te[9] = a21 * b13 + a22 * b23 + a23 * b33 + a24 * b43;
        te[13] = a21 * b14 + a22 * b24 + a23 * b34 + a24 * b44;

        te[2] = a31 * b11 + a32 * b21 + a33 * b31 + a34 * b41;
        te[6] = a31 * b12 + a32 * b22 + a33 * b32 + a34 * b42;
        te[10] = a31 * b13 + a32 * b23 + a33 * b33 + a34 * b43;
        te[14] = a31 * b14 + a32 * b24 + a33 * b34 + a34 * b44;

        te[3] = a41 * b11 + a42 * b21 + a43 * b31 + a44 * b41;
        te[7] = a41 * b12 + a42 * b22 + a43 * b32 + a44 * b42;
        te[11] = a41 * b13 + a42 * b23 + a43 * b33 + a44 * b43;
        te[15] = a41 * b14 + a42 * b24 + a43 * b34 + a44 * b44;

        this.elements = te;

        return this;
    }
}

// 2D Physics Engine
class RigidBody2D {
    constructor(position = new Vector2(), velocity = new Vector2(), mass = 1) {
        this.position = position;
        this.prevPosition = new Vector2(position.x, position.y);
        this.velocity = velocity;
        this.mass = mass;
        this.force = new Vector2();
    }

    applyForce(force) {
        this.force.add(force);
    }

    integrate(dt) {
        let acceleration = this.force.scale(1 / this.mass);
        this.velocity.add(acceleration.scale(dt));
        this.prevPosition = new Vector2(this.position.x, this.position.y); // Store previous position
        this.position.add(this.velocity.scale(dt));
        this.force = new Vector2();  // Reset force
    }
}

class Collider2D {
    constructor() {}

    checkCollision(rigidBody1, rigidBody2) {
        // Implement collision detection logic
        return false;
    }
}

class World2D {
    constructor() {
        this.bodies = [];
        this.constraints = []; // Array to hold distance constraints
        this.collider = new Collider2D();
    }

    addBody(body) {
        this.bodies.push(body);
    }

    addConstraint(bodyA, bodyB, length) {
        // Implement distance constraint logic
        // Example: this.constraints.push(new DistanceConstraint(bodyA, bodyB, length));
    }

    step(dt) {
        // Solve constraints first
        for (let constraint of this.constraints) {
            constraint.solve();
        }

        // Integrate bodies
        for (let body of this.bodies) {
            body.integrate(dt);
        }

        // Check for collisions
        for (let i = 0; i < this.bodies.length; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                let bodyA = this.bodies[i];
                let bodyB = this.bodies[j];
                if (this.collider.checkCollision(bodyA, bodyB)) {
                    // Resolve collision
                }
            }
        }
    }
}

// 3D Physics Engine
class RigidBody3D {
    constructor(position = new Vector3(), velocity = new Vector3(), mass = 1) {
        this.position = position;
        this.prevPosition = new Vector3(position.x, position.y, position.z);
        this.velocity = velocity;
        this.mass = mass;
        this.force = new Vector3();
    }

    applyForce(force) {
        this.force.add(force);
    }

    integrate(dt) {
        let acceleration = this.force.scale(1 / this.mass);
        this.velocity.add(acceleration.scale(dt));
        this.prevPosition = new Vector3(this.position.x, this.position.y, this.position.z); // Store previous position
        this.position.add(this.velocity.scale(dt));
        this.force = new Vector3();  // Reset force
    }
}

class Collider3D {
    constructor() {}

    checkCollision(rigidBody1, rigidBody2) {
        // Implement collision detection logic
        return false;
    }
}

class World3D {
    constructor() {
        this.bodies = [];
        this.constraints = []; // Array to hold distance constraints
        this.collider = new Collider3D();
    }

    addBody(body) {
        this.bodies.push(body);
    }

    addConstraint(bodyA, bodyB, length) {
        // Implement distance constraint logic
        // Example: this.constraints.push(new DistanceConstraint(bodyA, bodyB, length));
    }

    step(dt) {
        // Solve constraints first
        for (let constraint of this.constraints) {
            constraint.solve();
        }

        // Integrate bodies
        for (let body of this.bodies) {
            body.integrate(dt);
        }

        // Check for collisions
        for (let i = 0; i < this.bodies.length; i++) {
            for (let j = i + 1; j < this.bodies.length; j++) {
                let bodyA = this.bodies[i];
                let bodyB = this.bodies[j];
                if (this.collider.checkCollision(bodyA, bodyB)) {
                    // Resolve collision
                }
            }
        }
    }
}

// Integration with Rendering Libraries
const canvas = document.getElementById('canvas');
const context = canvas.getContext('2d');

const world2D = new World2D();
const world3D = new World3D();

const body2D = new RigidBody2D(new Vector2(50, 50), new Vector2(1, 0), 1);
world2D.addBody(body2D);

const body3D = new RigidBody3D(new Vector3(0, 0, 0), new Vector3(1, 0, 0), 1);
world3D.addBody(body3D);

function animate() {
    context.clearRect(0, 0, canvas.width, canvas.height);

    world2D.step(0.016);
    // Draw 2D bodies
    for (let body of world2D.bodies) {
        context.beginPath();
        context.arc(body.position.x, body.position.y, 10, 0, Math.PI * 2);
        context.fill();
    }

    // Simulate 3D world
    world3D.step(0.016);
    // Draw 3D bodies using Three.js or another 3D rendering library

    requestAnimationFrame(animate);
}

animate();
