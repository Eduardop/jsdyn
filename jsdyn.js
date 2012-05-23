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

// This work is based on the Siggraph course "Physically Based
// Modeling" by David Baraff:
// http://graphics.cs.cmu.edu/courses/15-869-F08/lec/14/notesg.pdf

function Float3(x, y, z)
{
    this.x = x;
    this.y = y;
    this.z = z;
}

Float3.prototype.copyFrom = function (a)
{
    return new Float3(a.x, a.y, a.z);
}

Float3.prototype.add = function (v)
{
    return new Float3(this.x + v.x, this.y + v.y, this.z + v.z);
}

Float3.prototype.sub = function (v)
{
    return new Float3(this.x - v.x, this.y - v.y, this.z - v.z);
}

Float3.prototype.negate = function ()
{
    this.x = -this.x;
    this.y = -this.y;
    this.z = -this.z;
}

Float3.prototype.accum = function (v)
{
    this.x += v.x;
    this.y += v.y;
    this.z += v.z;
}

Float3.prototype.accumNeg = function (v)
{
    this.x -= v.x;
    this.y -= v.y;
    this.z -= v.z;
}

Float3.prototype.mulScalar = function (a)
{
    return new Float3(this.x * a, this.y * a, this.z * a);
}

Float3.prototype.divScalar = function (a)
{
    return new Float3(this.x / a, this.y / a, this.z / a);
}

Float3.prototype.cross = function (v)
{
    return new Float3(this.y * v.z - this.z * v.y, 
                      this.z * v.x - this.x * v.z,
                      this.x * v.y - this.y * v.x);
}

Float3.prototype.dot = function (v)
{
    return this.x * v.x + this.y * v.y + this.z * v.z;
}

Float3.prototype.len = function ()
{
    return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
}

Float3.prototype.normalize = function ()
{
    var l = Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    this.x /= l;
    this.y /= l;
    this.z /= l;
}

Float3.prototype.mulVector = function (v)
{
    return new Float3(this.x * v.x, this.y * v.y, this.z * v.z);
}

Float3.prototype.getSkewSymmMatrix = function ()
{
    var m = new Matrix3();
    m.a00 = 0;
    m.a01 = -this.z;
    m.a02 = this.y;
    m.a10 = this.z;
    m.a11 = 0;
    m.a12 = -this.x;
    m.a20 = -this.y;
    m.a21 = this.x;
    m.a22 = 0;
    return m;
}

var S_Float3 = new Float3(0,0,0);

function Matrix3()
{}

Matrix3.prototype.copyFrom = function (m)
{
    var r = new Matrix3();
    r.a00 = m.a00;
    r.a01 = m.a01;
    r.a02 = m.a02;
    r.a10 = m.a10;
    r.a11 = m.a11;
    r.a12 = m.a12;
    r.a20 = m.a20;
    r.a21 = m.a21;
    r.a22 = m.a22;
    return r;
}

Matrix3.prototype.setIdentity = function ()
{
    this.a00 = 1;
    this.a01 = 0;
    this.a02 = 0;
    this.a10 = 0;
    this.a11 = 1;
    this.a12 = 0;
    this.a20 = 0;
    this.a21 = 0;
    this.a22 = 1;
}

Matrix3.prototype.transpose = function ()
{
    var m = new Matrix3();
    m.a00 = this.a00;
    m.a01 = this.a10;
    m.a02 = this.a20;
    m.a10 = this.a01;
    m.a11 = this.a11;
    m.a12 = this.a21;
    m.a20 = this.a02;
    m.a21 = this.a12;
    m.a22 = this.a22;
    return m;
}

Matrix3.prototype.add = function (n)
{
    var m = new Matrix3();
    m.a00 = this.a00 + n.a00;
    m.a01 = this.a01 + n.a01;
    m.a02 = this.a02 + n.a02;
    m.a10 = this.a10 + n.a10;
    m.a11 = this.a11 + n.a11;
    m.a12 = this.a12 + n.a12;
    m.a20 = this.a20 + n.a20;
    m.a21 = this.a21 + n.a21;
    m.a22 = this.a22 + n.a22;
    return m;
}

Matrix3.prototype.accum = function (n)
{
    this.a00 += n.a00;
    this.a01 += n.a01;
    this.a02 += n.a02;
    this.a10 += n.a10;
    this.a11 += n.a11;
    this.a12 += n.a12;
    this.a20 += n.a20;
    this.a21 += n.a21;
    this.a22 += n.a22;
}

Matrix3.prototype.mul = function (n)
{
    var m = new Matrix3();
    m.a00 = this.a00 * n.a00 + this.a01 * n.a10 + this.a02 * n.a20;
    m.a01 = this.a00 * n.a01 + this.a01 * n.a11 + this.a02 * n.a21;
    m.a02 = this.a00 * n.a02 + this.a01 * n.a12 + this.a02 * n.a22;

    m.a10 = this.a10 * n.a00 + this.a11 * n.a10 + this.a12 * n.a20;
    m.a11 = this.a10 * n.a01 + this.a11 * n.a11 + this.a12 * n.a21;
    m.a12 = this.a10 * n.a02 + this.a11 * n.a12 + this.a12 * n.a22;

    m.a20 = this.a20 * n.a00 + this.a21 * n.a10 + this.a22 * n.a20;
    m.a21 = this.a20 * n.a01 + this.a21 * n.a11 + this.a22 * n.a21;
    m.a22 = this.a20 * n.a02 + this.a21 * n.a12 + this.a22 * n.a22;

    return m;
}

Matrix3.prototype.mulVector = function (v)
{
    return new Float3(this.a00 * v.x + this.a01 * v.y + this.a02 * v.z,
                      this.a10 * v.x + this.a11 * v.y + this.a12 * v.z,
                      this.a20 * v.x + this.a21 * v.y + this.a22 * v.z);
}

Matrix3.prototype.mulScalar = function (a)
{
    var m = new Matrix3();
    m.a00 = this.a00 * a;
    m.a01 = this.a01 * a;
    m.a02 = this.a02 * a;
    m.a10 = this.a10 * a;
    m.a11 = this.a11 * a;
    m.a12 = this.a12 * a;
    m.a20 = this.a20 * a;
    m.a21 = this.a21 * a;
    m.a22 = this.a22 * a;
    return m;
}

Matrix3.prototype.getAsArray4 = function ()
{
    // Column-major, OpenGL style
    return [this.a00, this.a10, this.a20, 0, 
            this.a01, this.a11, this.a21, 0, 
            this.a02, this.a12, this.a22, 0,
            0, 0, 0, 1];
}

Matrix3.prototype.getAsO3DMatrix4 = function ()
{
    // Column-major
    return [[this.a00, this.a10, this.a20, 0], 
            [this.a01, this.a11, this.a21, 0], 
            [this.a02, this.a12, this.a22, 0], 
            [0, 0, 0, 1]];
}

var S_Matrix3 = new Matrix3();

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
    // this.v;         float3
    // this.omega;     float3
    // -----------------------------------------------------
    //     Computed:
    // this.force;     float3
    // this.torque;    float3
    // -----------------------------------------------------
    this.force = new Float3(0, 0, 0);
    this.torque = new Float3(0, 0, 0);
}

RigidBody.prototype.setFrom = function (b)
{
    this.l        = S_Float3.copyFrom(b.l);
    this.mass     = b.mass;
    this.Ibody    = S_Matrix3.copyFrom(b.Ibody);
    this.Ibodyinv = S_Matrix3.copyFrom(b.Ibodyinv);
    this.x        = S_Float3.copyFrom(b.x);
    this.R        = S_Matrix3.copyFrom(b.R);
    this.P        = S_Float3.copyFrom(b.P);
    this.L        = S_Float3.copyFrom(b.L);
    this.Iinv     = S_Matrix3.copyFrom(b.Iinv);
    this.v        = S_Float3.copyFrom(b.v);
    this.omega    = S_Float3.copyFrom(b.omega);
    this.force    = S_Float3.copyFrom(b.force);
    this.torque   = S_Float3.copyFrom(b.torque);
}

RigidBody.prototype.computeAux = function ()
{
    this.v = this.P.divScalar(this.mass);
    this.Iinv = this.R.mul(this.Ibodyinv).mul(this.R.transpose());
    this.omega = this.Iinv.mulVector(this.L);
}

RigidBody.prototype.integrateEuler = function (dt)
{
    this.x.accum(this.v.mulScalar(dt));
    var m = this.omega.getSkewSymmMatrix().mul(this.R);
    this.R.accum(m.mulScalar(dt));

    this.P.accum(this.force.mulScalar(dt));
    this.L.accum(this.torque.mulScalar(dt));

    var kdf = this.v.mulScalar(g_world.kdl * dt);
    kdf.negate();
    this.P.accum(kdf);
    var kdt = this.omega.mulScalar(g_world.kdw * dt);
    kdt.negate();
    this.L.accum(kdt);
}

RigidBody.prototype.renormalizeR = function ()
{
    var v0 = new Float3(this.R.a00, this.R.a10, this.R.a20);
    v0.normalize();
    var v1 = new Float3(this.R.a01, this.R.a11, this.R.a21);
    v1.normalize();
    var v2 = v0.cross(v1);
    v1 = v2.cross(v0);
    this.R.a00 = v0.x;
    this.R.a01 = v1.x;
    this.R.a02 = v2.x;
    this.R.a10 = v0.y;
    this.R.a11 = v1.y;
    this.R.a12 = v2.y;
    this.R.a20 = v0.z;
    this.R.a21 = v1.z;
    this.R.a22 = v2.z;
}

RigidBody.prototype.setBox = function (lx, ly, lz, m, x, R)
{
    this.l = new Float3(lx, ly, lz);
    this.mass = m;
    this.x = x;
    this.R = R;
    this.P = new Float3(0, 0, 0);
    this.L = new Float3(0, 0, 0);
    // TODO: optimize. Ibody doesn't need to be stored.
    this.Ibody = new Matrix3();
    this.Ibody.setIdentity();
    // m=-1 for bodies fixed to the world
    //if (m < 0) 
    //    return;
    var m0 = this.mass / 12;
    this.Ibody.a00 = m0 * (ly * ly + lz * lz);
    this.Ibody.a11 = m0 * (lx * lx + lz * lz);
    this.Ibody.a22 = m0 * (lx * lx + ly * ly);
    this.Ibodyinv = new Matrix3();
    this.Ibodyinv.setIdentity();
    this.Ibodyinv.a00 = 1 / this.Ibody.a00;
    this.Ibodyinv.a11 = 1 / this.Ibody.a11;
    this.Ibodyinv.a22 = 1 / this.Ibody.a22;
    this.computeAux();
}

function World()
{
    this.gravity = new Float3(0, -9.8, 0);
    this.rigidBodies = [];
    this.rigidBodiesPrevious = [];
    this.springs = [];
    this.numSteps = 10;
    this.kdl = 0.1; // linear damping
    this.kdw = 0.1; // angular damping
}

var g_world = new World();

World.prototype.ptVelocity = function (b, p)
{
    return b.v.add(b.omega.cross(p.sub(b.x))); 
}

World.prototype.computeSpringForces = function ()
{
    for (springi in this.springs)
    {
        spr = this.springs[springi];
        if (spr.enabled)
        {
            var x = [];      // Points of force application in object space
            var xw = [];     // Points of force application in world space
            var v = [];      // Velocities of the points of force application
            for (var i = 0; i < 2; i++)
            {
                if (spr.endPoints[i].body == 0)
                {
                    xw[i] = spr.endPoints[i].x;
                    v[i] = new Float3(0, 0, 0);
                }
                else
                {
                    var b = spr.endPoints[i].body;
                    x[i] = spr.endPoints[i].x;     // point of force application in object
                    x[i] = b.R.mulVector(x[i]);    // rotated by the object rotation
                    xw[i] = x[i].add(b.x);         // x[i] in world space
                    v[i] = b.omega.cross(x[i]);    // velocity of point of force application
                    v[i].accum(b.v);               // add to velocity of center of mass
                }
            }
            var sv = xw[1].sub(xw[0]);             // spring extension
            var sl = sv.len();
            var ext = sl - spr.restLength;
            var f = sv.mulScalar(spr.ks * ext / sl);
            var extdot = v[1].sub(v[0]);
            f.accum(extdot.mulScalar(spr.kd));

            for (var i = 0; i < 2; i++)
            {
                var b = spr.endPoints[i].body;
                if (b != 0)
                {
                    if (i == 1)
                    {
                        f.negate();
                    }
                    var t = x[i].cross(f);
                    b.force.accum(f);
                    b.torque.accum(t);
                }
            }
        }
    }
}

World.prototype.integrateBodies = function (dt)
{
    for (var i = 0; i < this.rigidBodies.length; i++)
    {
        var body = this.rigidBodies[i];
        if (body.mass >= 0)
        {
            body.integrateEuler(dt);
            body.renormalizeR();
            body.computeAux();
        }
    }
}

function Contact()
{
    this.depth = 0;
}

var cubePoints  = [ new Float3(-0.5, 0, 0), new Float3(0.5, 0, 0),
                    new Float3(0, -0.5, 0), new Float3(0, 0.5, 0),
                    new Float3(0, 0, -0.5), new Float3(0, 0, 0.5)];
var cubeNormals = [ new Float3(-1, 0, 0),   new Float3(1, 0, 0),
                    new Float3(0, -1, 0),   new Float3(0, 1, 0),
                    new Float3(0, 0, -1),   new Float3(0, 0, 1)];

World.prototype.linePlaneIntersection = function (p, n, pa, pb)
{
    var den = ((pb.sub(pa)).dot(n));
    if (den == 0)
        return 1.0e30;
    return ((p.sub(pa)).dot(n))/den;
}

World.prototype.detectCollisionBodyBody = function (bodyb, bodya0, bodya1)
{
    var contacts = [];
    var invRb = bodyb.R.transpose();
    var invlb = new Float3(1/bodyb.l.x, 1/bodyb.l.y, 1/bodyb.l.z);
    var reldist0 = bodya0.x.sub(bodyb.x);
    var reldist1 = bodya1.x.sub(bodyb.x);
    for (var x = -0.5; x <= 0.5; x += 1)
        for (var y = -0.5; y <= 0.5; y += 1)
            for (var z = -0.5; z <= 0.5; z += 1)
            {
                var pb = new Float3(x, y, z);
                pb = pb.mulVector(bodya1.l);               // vertex of box     
                var rpb = bodya1.R.mulVector(pb);          // rotated by the object rotation
                var wrpb = rpb.add(reldist1);              // in bodyb space
                var qb = invRb.mulVector(wrpb);            // rotated by inverse bodyb rotation
                qb = qb.mulVector(invlb);                  // normalized to unit cube

                if ((qb.x < 0.5 && qb.x > -0.5) &&
                    (qb.y < 0.5 && qb.y > -0.5) &&
                    (qb.z < 0.5 && qb.z > -0.5))
                {
                    var con = new Contact();
                    //console.log("Collided");

                    var pa = new Float3(x, y, z);
                    pa = pa.mulVector(bodya0.l);           // vertex of box     
                    var rpa = bodya0.R.mulVector(pa);      // rotated by the object rotation
                    var wrpa = rpa.add(reldist0);          // in bodyb space
                    var qa = invRb.mulVector(wrpa);        // rotated by inverse bodyb rotation
                    qa = qa.mulVector(invlb);              // normalized to unit cube
                    
                    /*
                    if ((qa.x < 0.5 && qa.x > -0.5) &&
                        (qa.y < 0.5 && qa.y > -0.5) &&
                        (qa.z < 0.5 && qa.z > -0.5))
                    {
                        console.log("Warning: penetration not resolved on previous frame");
                    }
                    */
                    
                    var max_d = -1.0e30;
                    var max_f = -1;
                    for (var f = 0; f < 6; f++)
                    {
                        var d = this.linePlaneIntersection(cubePoints[f], cubeNormals[f], qa, qb);
                        //console.log("d=" + d);
                        if (d <= 1)
                        {
                            if (d > max_d)
                            {
                                max_d = d;
                                max_f = f;
                            }
                        }
                    }
                    var worldPos = qa.add((qb.sub(qa)).mulScalar(max_d));  // in unit bodyb space
                    worldPos = worldPos.mulVector(bodyb.l);                // scaled by bodyb dimensions
                    worldPos = bodyb.R.mulVector(worldPos);                // rotated by bodyb rotation
                    con.pos = worldPos.add(bodyb.x);                       // in world space
                    con.normal = cubeNormals[max_f];
                    con.depth = -((qb.sub(cubePoints[max_f])).dot(con.normal));
                    con.bodyb = bodyb;
                    con.bodya = bodya1;
                    contacts.push(con);
                }
            }
    return contacts;
}

World.prototype.detectContacts = function ()
{
    var contacts = [];
    for (var i = 0; i < this.rigidBodies.length; i++)
    {
        var bodyb = this.rigidBodies[i];
        for (var j = 0; j < this.rigidBodies.length; j++)
        {
            if (i != j)
            {
                var bodya0 = this.rigidBodiesPrevious[j];
                var bodya1 = this.rigidBodies[j];
                var c = this.detectCollisionBodyBody(bodyb, bodya0, bodya1);
                contacts.push.apply(contacts, c);
            }
        }
    }
    return contacts;
}

World.prototype.handleCollision = function (con)
{
    var padot = this.ptVelocity(con.bodya, con.pos);
    var pbdot = this.ptVelocity(con.bodyb, con.pos);
    var ra = con.pos.sub(con.bodya.x);
    var rb = con.pos.sub(con.bodyb.x);
    var vrel = con.normal.dot(padot.sub(pbdot));
    var epsilon = 0.5;
    var num = -(1 + epsilon) * vrel;
    var term1 = 0;
    var term2 = 0;
    var term3 = 0;
    var term4 = 0;
    if (con.bodya.mass > 0)
    {
        term1 = 1 / con.bodya.mass;
        term3 = con.normal.dot(con.bodya.Iinv.mulVector(ra.cross(con.normal)).cross(ra));
    }
    if (con.bodyb.mass > 0)
    {
        term2 = 1 / con.bodyb.mass;
        term4 = con.normal.dot(con.bodyb.Iinv.mulVector(rb.cross(con.normal)).cross(rb));
    }
    var j = num / (term1 + term2 + term3 + term4);
    var impulse = con.normal.mulScalar(j);
    if (con.bodya.mass > 0)
    {
        con.bodya.P.accum(impulse);
        con.bodya.L.accum(ra.cross(impulse));
        con.bodya.computeAux();
    }
    if (con.bodyb.mass > 0)
    {
        con.bodyb.P.accumNeg(impulse);
        con.bodyb.L.accumNeg(rb.cross(impulse));
        con.bodyb.computeAux();
    }
}

World.prototype.copyBodies = function (ba0, ba1, reuse) // copy ba1 to ba0
{
    if (reuse)
    {
        for (var i = 0; i < ba1.length; i++)
        {
            ba0[i].setFrom(ba1[i]);
        }
    }
    else
    {
        ba0.length = 0;
        for (var i = 0; i < ba1.length; i++)
        {
            var b0 = new RigidBody();
            b0.setFrom(ba1[i]);
            ba0.push(b0);
        }
    }
}

World.prototype.isColliding = function(con)
{
    var padot = this.ptVelocity(con.bodya, con.pos);
    var pbdot = this.ptVelocity(con.bodyb, con.pos);
    var vrel = con.normal.dot(padot.sub(pbdot));
    return (vrel < 0);
}

World.prototype.isPenetrating = function(con)
{
    return (con.depth > 0.1);
}

World.prototype.step = function (dt)
{
    //console.log("Step");
    
    var dt2 = dt / this.numSteps;
    for (var j = 0; j < this.numSteps; j++)
    {
        this.computeSpringForces();
        for (var i = 0; i < this.rigidBodies.length; i++)
        {
            this.rigidBodies[i].force.accum(this.gravity.mulScalar(this.rigidBodies[i].mass));
        }
        this.copyBodies(this.rigidBodiesPrevious, this.rigidBodies, false);
        this.integrateBodies(dt2);
        var contacts = this.detectContacts();
        
        //if (contacts.length > 0)
        //    console.log("collisions: " + contacts.length);
        
        var collided;
        do
        {
            collided = false;
            for (var i = 0; i < contacts.length; i++)
            {
                var con = contacts[i];
                if (this.isColliding(con))
                {
                    //collided = true;
                    this.handleCollision(con);
                }
            }
        }
        while (collided);
        for (var i = 0; i < this.rigidBodies.length; i++)
        {
            this.rigidBodies[i].force = new Float3(0, 0, 0);
            this.rigidBodies[i].torque = new Float3(0, 0, 0);
        }
    }

}
