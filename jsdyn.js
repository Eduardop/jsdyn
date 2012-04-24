// jsdyn.js

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
	this.numSteps = 10;
}

World.prototype.step = function(dt)
{
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
			body.integrateEuler(dt2);
			body.renormalizeR();
			body.computeAux();
		}
	}
	for (var i = 0; i < this.rigidBodies.length; i++)
	{
		this.rigidBodies[i].force = new Float3(0,0,0);
		this.rigidBodies[i].torque = new Float3(0,0,0);
	}
}

