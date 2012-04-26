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

function Float3(x, y, z)
{
	this.x = x;
	this.y = y;
	this.z = z;
}

Float3.prototype.add = function(v)
{
	return new Float3(this.x + v.x, this.y + v.y, this.z + v.z);
}

Float3.prototype.sub = function(v)
{
	return new Float3(this.x - v.x, this.y - v.y, this.z - v.z);
}

Float3.prototype.negate = function()
{
  this.x = -this.x; this.y = -this.y; this.z = -this.z;
}

Float3.prototype.accum = function(v)
{
	this.x += v.x; this.y += v.y; this.z += v.z;
}

Float3.prototype.accumNeg = function(v)
{
	this.x -= v.x; this.y -= v.y; this.z -= v.z;
}

Float3.prototype.mulScalar = function(a)
{
	return new Float3(this.x * a, this.y * a, this.z * a);
}

Float3.prototype.divScalar = function(a)
{
	return new Float3(this.x / a, this.y / a, this.z / a);
}

Float3.prototype.cross = function(v)
{
	return new Float3(this.y*v.z - this.z*v.y, this.z*v.x - this.x*v.z, this.x*v.y - this.y*v.x); 
}

Float3.prototype.dot = function(v)
{
	return this.x*v.x + this.y*v.y + this.z*v.z;
}

Float3.prototype.len = function()
{
	return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
}

Float3.prototype.normalize = function()
{
	var l = Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
	this.x /= l; this.y /= l; this.z /= l;
}

function Matrix3()
{
}

Matrix3.prototype.setIdentity = function()
{
	this.a00 = 1;  this.a01 = 0;  this.a02 = 0;
	this.a10 = 0;  this.a11 = 1;  this.a12 = 0;
	this.a20 = 0;  this.a21 = 0;  this.a22 = 1;
}

Matrix3.prototype.transpose = function()
{
	var m = new Matrix3();
	m.a00 = this.a00; m.a01 = this.a10; m.a02 = this.a20;
	m.a10 = this.a01; m.a11 = this.a11; m.a12 = this.a21;
	m.a20 = this.a02; m.a21 = this.a12; m.a22 = this.a22;
	return m;
}

Matrix3.prototype.add = function(n)
{
	var m = new Matrix3();
	m.a00 = this.a00 + n.a00; m.a01 = this.a01 + n.a01; m.a02 = this.a02 + n.a02;
	m.a10 = this.a10 + n.a10; m.a11 = this.a11 + n.a11; m.a12 = this.a12 + n.a12;
	m.a20 = this.a20 + n.a20; m.a21 = this.a21 + n.a21; m.a22 = this.a22 + n.a22;
	return m;	
}

Matrix3.prototype.accum = function(n)
{
	this.a00 += n.a00; this.a01 += n.a01; this.a02 += n.a02;
	this.a10 += n.a10; this.a11 += n.a11; this.a12 += n.a12;
	this.a20 += n.a20; this.a21 += n.a21; this.a22 += n.a22;
}

Matrix3.prototype.mul = function(n)
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

Matrix3.prototype.mulVector = function(v)
{
	return new Float3(
		this.a00 * v.x + this.a01 * v.y + this.a02 * v.z,
		this.a10 * v.x + this.a11 * v.y + this.a12 * v.z,
		this.a20 * v.x + this.a21 * v.y + this.a22 * v.z);
}

Matrix3.prototype.mulScalar = function(a)
{
	var m = new Matrix3();
	m.a00 = this.a00 * a;  m.a01 = this.a01 * a;  m.a02 = this.a02 * a;
	m.a10 = this.a10 * a;  m.a11 = this.a11 * a;  m.a12 = this.a12 * a;
	m.a20 = this.a20 * a;  m.a21 = this.a21 * a;  m.a22 = this.a22 * a;
	return m;
}

Matrix3.prototype.getAsArray4 = function()
{
  // Column-major, OpenGL style
	return [ this.a00, this.a10, this.a20, 0,
	         this.a01, this.a11, this.a21, 0,
	         this.a02, this.a12, this.a22, 0,
	         0,        0,        0,        1 ]; 
}

Matrix3.prototype.getAsO3DMatrix4 = function()
{
  // Column-major
  return [ [ this.a00, this.a10, this.a20, 0 ],
           [ this.a01, this.a11, this.a21, 0 ],
           [ this.a02, this.a12, this.a22, 0 ],
           [ 0,        0,        0,        1 ] ];
}

Float3.prototype.getSkewSymmMatrix = function()
{
	var m = new Matrix3();
	m.a00 = 0;             m.a01 = -this.z;       m.a02 = this.y;
	m.a10 = this.z;        m.a11 = 0;             m.a12 = -this.x;     
	m.a20 = -this.y;       m.a21 = this.x;        m.a22 = 0;        
	return m;	
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
	// this.v;         float3
	// this.omega;     float3
	// -----------------------------------------------------
	//     Computed:
	// this.force;     float3
	// this.torque;    float3
	// -----------------------------------------------------

	this.force = new Float3(0,0,0);
	this.torque = new Float3(0,0,0);
}

RigidBody.prototype.computeAux = function()
{
	this.v = this.P.divScalar(this.mass);
	this.Iinv = this.R.mul(this.Ibodyinv).mul(this.R.transpose());
	this.omega = this.Iinv.mulVector(this.L);
}

RigidBody.prototype.integrateEuler = function(dt)
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

RigidBody.prototype.renormalizeR = function()
{
	var v0 = new Float3(this.R.a00, this.R.a10, this.R.a20); v0.normalize();
	var v1 = new Float3(this.R.a01, this.R.a11, this.R.a21); v1.normalize();
	var v2 = v0.cross(v1);
	v1 = v2.cross(v0);
	this.R.a00 = v0.x; this.R.a01 = v1.x; this.R.a02 = v2.x;
	this.R.a10 = v0.y; this.R.a11 = v1.y; this.R.a12 = v2.y;
	this.R.a20 = v0.z; this.R.a21 = v1.z; this.R.a22 = v2.z;
}

RigidBody.prototype.setBox = function(lx, ly, lz, m, x, R)
{
	this.l = new Float3(lx, ly, lz);
	this.mass = m;
	this.x = x;
	this.R = R;
	this.P = new Float3(0,0,0);
	this.L = new Float3(0,0,0);
	this.Ibody = new Matrix3();
	this.Ibody.setIdentity();
    // m=-1 for bodies fixed to the world
    if (m < 0)
      return;
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
        if (spr.endPoints[i].body == 0)
        {
          xw[i] = spr.endPoints[i].x;
          v[i] = new Float3(0,0,0);
        }
        else
        {
          var b = spr.endPoints[i].body;
          x[i]  = spr.endPoints[i].x;  // point of force application in object
          x[i]  = b.R.mulVector(x[i]); // rotated by the object rotation
          xw[i] = x[i].add(b.x);       // x[i] in world space
          v[i]  = b.omega.cross(x[i]); // velocity of point of force application
          v[i].accum(b.v);             // add to velocity of center of mass
        }
      }
      var sv = xw[1].sub(xw[0]);      // spring extension
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

World.prototype.step = function(dt)
{
  this.computeSpringForces();
	for (var i = 0; i < this.rigidBodies.length; i++)
	{
		this.rigidBodies[i].force.accum(this.gravity.mulScalar(this.rigidBodies[i].mass));
	}
	var dt2 = dt / this.numSteps;
	for (var n = 0; n < this.numSteps; n++)
	{
		for (var i = 0; i < this.rigidBodies.length; i++)
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
	for (var i = 0; i < this.rigidBodies.length; i++)
	{
		this.rigidBodies[i].force = new Float3(0,0,0);
		this.rigidBodies[i].torque = new Float3(0,0,0);
	}
}

