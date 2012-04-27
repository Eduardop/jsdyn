// jsdyn.js

/*
Copyright 2012, Eduardo Poyart.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the
distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


function getSkewSymmMatrix(v)
{
    /*
    var m = new Matrix3();
    m.a00 = 0;             m.a01 = -this.z;       m.a02 = this.y;
    m.a10 = this.z;        m.a11 = 0;             m.a12 = -this.x;     
    m.a20 = -this.y;       m.a21 = this.x;        m.a22 = 0;        
    return m;
    */
    return mat3.create([        // column-major
            0,   v[2],  -v[1],  // column 0
        -v[2],      0,   v[0],  // column 1
         v[1],  -v[0],      0   // column 2
    ]);
}


function SpringEndPoint(body, x)
{
    // -----------------------------------------------------
    // this.x;         float3    Position
    // this.body       0 or RigidBody  0 if it's fixed in
    //                                 world coordinates
    // -----------------------------------------------------

  this.x = x;
  this.body = body;
}

function Spring(body1, x1, body2, x2, restLength, ks, kd)
{
  this.enabled = true;
  this.endPoints = [];
  this.endPoints[0] = new SpringEndPoint(body1, x1);
  this.endPoints[1] = new SpringEndPoint(body2, x2);
  this.restLength = restLength;
  this.ks = ks;
  this.kd = kd;
}

function RigidBody()
{
    // -----------------------------------------------------
    //     Constant:
    // this.l;         float3    Dimension (box)
    // this.mass;      float
    // this.Ibody;     matrix3
    // this.Ibodyinv;  matrix3
    // -----------------------------------------------------
    //     State:
    // this.x;         float3    Position
    // this.R;         matrix3   Rotation
    // this.P;         float3    Linear momentum = m v
    // this.L;         float3    Angular momentum = I omega
    // -----------------------------------------------------
    //     Derived:
    // this.Iinv;      matrix3
    // this.v;         float3    Linear velocity
    // this.omega;     float3    Angular velocity
    // -----------------------------------------------------
    //     Computed:
    // this.force;     float3
    // this.torque;    float3
    // -----------------------------------------------------

    this.force = vec3.create([0,0,0]);
    this.torque = vec3.create([0,0,0]);
}

RigidBody.prototype.computeAux = function()
{
    // v = P / m
    vec3.scale(this.P, 1/this.mass, this.v);
    // Iinv = R * Ibodyinv * Rt
    var Rt = mat3.create();
    mat3.transpose(this.R, Rt);
    mat3.multiply(this.R, this.Ibodyinv, this.Iinv);
    mat3.multiply(this.Iinv, Rt);
    // omega = Iinv * L
    mat3.multiplyVec3(this.Iinv, this.L, this.omega);
}

RigidBody.prototype.integrateEuler = function(dt)
{
    // x += v dt
    var vdt = vec3.create();
    vec3.scale(this.v, dt, vdt);
    vec3.add(this.x, vdt);
    
    // R += (sksym(omega) * R) * dt
    var m = getSkewSymmMatrix(this.omega);
    mat3.multiply(m, this.R);
    var dt3 = vec3.create([dt, dt, dt]);
    mat3.scale(m, dt3);
    mat3.add(this.R, m);
    
    // P += f dt
    var fdt = vec3.create();
    vec3.scale(this.force, dt, fdt);
    vec3.add(this.P, fdt);
    
    // L = t dt
    var tdt = vec3.create();
    vec3.scale(this.torque, dt, tdt);
    vec3.add(this.L, tdt);
    
    var kdf = vec3.create();
    vec3.scale(this.v, g_world.kdl * dt, kdf);
    vec3.negate(kdf);
    vec3.add(this.P, kdf);

    var kdt = vec3.create();
    vec3.scale(this.omega, g_world.kdw * dt, kdt);
    vec3.negate(kdt);
    vec3.add(this.L, kdt);
}

RigidBody.prototype.renormalizeR = function()
{
    var v0 = vec3.create([this.R[0], this.R[1], this.R[2]]);
    vec3.normalize(v0);
    var v1 = vec3.create([this.R[3], this.R[4], this.R[5]]);
    vec3.normalize(v1);
    var v2 = vec3.create();
    vec3.cross(v0, v1, v2);
    vec3.cross(v1, v2, v0);
    this.R[0] = v0[0];
    this.R[1] = v0[1];
    this.R[2] = v0[2];
    this.R[3] = v1[0];
    this.R[4] = v1[1];
    this.R[5] = v1[2];
    this.R[6] = v2[0];
    this.R[7] = v2[1];
    this.R[8] = v2[2];
}

RigidBody.prototype.setBox = function(lengths, m, x, R)
{
    this.l = lengths;
    this.mass = m;
    this.x = x;
    this.R = R;
    this.P = vec3.create([0,0,0]);
    this.L = vec3.create([0,0,0]);
    this.Ibody = mat3.create();
    mat3.identity(this.Ibody);
    // m=-1 for bodies fixed to the world
    if (m < 0)
      return;
    var m0 = this.mass / 12;
    var lx = lengths[0];
    var ly = lengths[1];
    var lz = lengths[2];
    this.Ibody[0] = m0 * (ly * ly + lz * lz);
    this.Ibody[4] = m0 * (lx * lx + lz * lz);
    this.Ibody[8] = m0 * (lx * lx + ly * ly);
    this.Ibodyinv = mat3.create();
    this.Ibodyinv[0] = 1 / this.Ibody[0];
    this.Ibodyinv[4] = 1 / this.Ibody[4];
    this.Ibodyinv[8] = 1 / this.Ibody[8];
    this.Iinv = mat3.create();
    this.v = vec3.create();
    this.omega = vec3.create();
    this.computeAux();
}

function World()
{
    this.gravity = vec3.create([0, -9.8, 0]);
    this.rigidBodies = [];
    this.springs = [];
    this.numSteps = 10;
    this.kdl = 0.1; // linear damping
    this.kdw = 0.1; // angular damping
}

var g_world = new World();

World.prototype.computeSpringForces = function()
{
    for (springi in this.springs)
    {
        spr = this.springs[springi];
        if (spr.enabled)
        {
            var x  = []; // Points of force application in object space
            var xw = []; // Points of force application in world space
            var v  = []; // Velocities of the points of force application
            for (var i = 0; i < 2; i++)
            {
                v[i] = vec3.create();
                var b = spr.endPoints[i].body;
                if (b == 0)
                {
                    xw[i] = vec3.create(spr.endPoints[i].x);
                }
                else
                {
                    x[i] = vec3.create(spr.endPoints[i].x);   // point of force application in object
                    mat3.multiplyVec3(b.R, x[i]); // rotated by the object rotation
                    xw[i] = vec3.create();
                    vec3.add(x[i], b.x, xw[i]);   // x[i] in world space
                    vec3.cross(b.omega, x[i], v[i]);// velocity of point of force application
                    vec3.add(v[i], b.v);          // add to velocity of center of mass
                }
            }
            var sv = vec3.create();
            vec3.subtract(xw[1], xw[0], sv);      // spring extension
            var sl = vec3.length(sv);
            var ext = sl - spr.restLength;
            var f = vec3.create();
            vec3.scale(sv, spr.ks * ext / sl, f);
            var extdot = vec3.create();
            vec3.subtract(v[1], v[0], extdot);
            vec3.scale(extdot, spr.kd);
            vec3.add(f, extdot);
            
            for (var i = 0; i < 2; i++)
            {
                var b = spr.endPoints[i].body;
                if (b != 0)
                {
                    if (i == 1)
                    {
                        vec3.negate(f);
                    }
                    var t = vec3.create();
                    vec3.cross(x[i], f, t);
                    vec3.add(b.force, f);
                    vec3.add(b.torque, t);
                }
            }
        }
    }
}

World.prototype.step = function(dt)
{
    this.computeSpringForces();
    for (var i in this.rigidBodies)
    {
        var b = this.rigidBodies[i];
        var mg = vec3.create();
        vec3.scale(this.gravity, b.mass, mg);
        vec3.add(b.force, mg);
    }
    var dt2 = dt / this.numSteps;
    for (var n = 0; n < this.numSteps; n++)
    {
        for (var i in this.rigidBodies)
        {
            var body = this.rigidBodies[i];
            if (body.mass >= 0)
            {
                body.integrateEuler(dt2);
                body.renormalizeR();
                body.computeAux();
            }
        }
    }
    for (var i in this.rigidBodies)
    {
        var b = this.rigidBodies[i];
        vec3.set([0,0,0], b.force);
        vec3.set([0,0,0], b.torque);
    }
}

