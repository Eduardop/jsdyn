// jsdyntest.js

var world = new World();

function init()
{
	var body = new RigidBody();
	var pos = new Float3(0,0,0);
	var rot = new Matrix3();
	rot.setIdentity();
	body.setBox(1, 1, 1, 1, pos, rot);
	world.rigidBodies.push(body);
}

var gl;

function initGL(canvas)
{
	// Initialize
	gl = initWebGL(
		// The id of the Canvas Element
		canvas,
		// vertex and fragment shaders
		"\
		uniform mat4 u_modelViewProjMatrix;\
		uniform mat4 u_normalMatrix;\
		uniform vec3 lightDir;\
		\
		attribute vec3 vNormal;\
		attribute vec4 vColor;\
		attribute vec4 vPosition;\
		\
		varying float v_Dot;\
		varying vec4 v_Color;\
		\
		void main()\
		{\
		    gl_Position = u_modelViewProjMatrix * vPosition;\
		    v_Color = vColor;\
		    vec4 transNormal = u_normalMatrix * vec4(vNormal, 1);\
		    v_Dot = max(dot(transNormal.xyz, lightDir), 0.0);\
		}",

		"\
		precision mediump float;\
		\
		varying float v_Dot;\
		varying vec4 v_Color;\
		\
		void main()\
		{\
			gl_FragColor = vec4(v_Color.xyz * v_Dot, v_Color.a);\
		}",

		// The vertex attribute names used by the shaders.
		// The order they appear here corresponds to their index
		// used later.
		[ "vNormal", "vColor", "vPosition"],
		// The clear color and depth values
		[ 0, 0, 0, 1 ], 10000,
		// shadersInline
		true);
	if (!gl) {
	  return;
	}

	gl.console.log("Starting init...");

	// Set up a uniform variable for the shaders
	gl.uniform3f(gl.getUniformLocation(gl.program, "lightDir"), 0, 0, 1);

	// Create a box. On return 'gl' contains a 'box' property with
	// the BufferObjects containing the arrays for vertices,
	// normals, texture coords, and indices.
	gl.box = makeBox(gl);

	// Set up the array of colors for the cube's faces
	var colors = new Uint8Array(
		[  0, 0, 1, 1,   0, 0, 1, 1,   0, 0, 1, 1,   0, 0, 1, 1,     // v0-v1-v2-v3 front
		   1, 0, 0, 1,   1, 0, 0, 1,   1, 0, 0, 1,   1, 0, 0, 1,     // v0-v3-v4-v5 right
		   0, 1, 0, 1,   0, 1, 0, 1,   0, 1, 0, 1,   0, 1, 0, 1,     // v0-v5-v6-v1 top
		   1, 1, 0, 1,   1, 1, 0, 1,   1, 1, 0, 1,   1, 1, 0, 1,     // v1-v6-v7-v2 left
		   1, 0, 1, 1,   1, 0, 1, 1,   1, 0, 1, 1,   1, 0, 1, 1,     // v7-v4-v3-v2 bottom
		   0, 1, 1, 1,   0, 1, 1, 1,   0, 1, 1, 1,   0, 1, 1, 1 ]    // v4-v7-v6-v5 back
		                                    );

	// Set up the vertex buffer for the colors
	gl.box.colorObject = gl.createBuffer();
	gl.bindBuffer(gl.ARRAY_BUFFER, gl.box.colorObject);
	gl.bufferData(gl.ARRAY_BUFFER, colors, gl.STATIC_DRAW);

	// Create some matrices to use later and save their locations in the shaders
	gl.mvMatrix = new J3DIMatrix4();
	gl.u_normalMatrixLoc = gl.getUniformLocation(gl.program, "u_normalMatrix");
	gl.normalMatrix = new J3DIMatrix4();
	gl.u_modelViewProjMatrixLoc =
		    gl.getUniformLocation(gl.program, "u_modelViewProjMatrix");
	gl.mvpMatrix = new J3DIMatrix4();

	// Enable all of the vertex attribute arrays.
	gl.enableVertexAttribArray(0);
	gl.enableVertexAttribArray(1);
	gl.enableVertexAttribArray(2);

	// Set up all the vertex attributes for vertices, normals and colors
	gl.bindBuffer(gl.ARRAY_BUFFER, gl.box.vertexObject);
	gl.vertexAttribPointer(2, 3, gl.FLOAT, false, 0, 0);

	gl.bindBuffer(gl.ARRAY_BUFFER, gl.box.normalObject);
	gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);

	gl.bindBuffer(gl.ARRAY_BUFFER, gl.box.colorObject);
	gl.vertexAttribPointer(1, 4, gl.UNSIGNED_BYTE, false, 0, 0);

	// Bind the index array
	gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, gl.box.indexObject);
}

var spring1 = true;
var spring2 = true;

var width = -1;
var height = -1;
var requestId;

function reshape(gl)
{
    var canvas = document.getElementById('canvas');
    var windowWidth = window.innerWidth - 40;
    var windowHeight = window.innerHeight - 40;
    if (windowWidth == width && windowHeight == height)
        return;

    width = windowWidth;
    height = windowHeight;
    canvas.width = width;
    canvas.height = height;

    // Set the viewport and projection matrix for the scene
    gl.viewport(0, 0, width, height);
    gl.perspectiveMatrix = new J3DIMatrix4();
    gl.perspectiveMatrix.perspective(30, width/height, 1, 10000);
    gl.perspectiveMatrix.lookat(0, 0, 16, 0, 0, 0, 0, 1, 0);
}

function drawBox(gl)
{
    // Construct the normal matrix from the model-view matrix and pass it in
    gl.normalMatrix.load(gl.mvMatrix);
    gl.normalMatrix.invert();
    gl.normalMatrix.transpose();
    gl.normalMatrix.setUniform(gl, gl.u_normalMatrixLoc, false);

    // Construct the model-view * projection matrix and pass it in
    gl.mvpMatrix.load(gl.perspectiveMatrix);
    gl.mvpMatrix.multiply(gl.mvMatrix);
    gl.mvpMatrix.setUniform(gl, gl.u_modelViewProjMatrixLoc, false);

    // Draw the cube
    gl.drawElements(gl.TRIANGLES, gl.box.numIndices, gl.UNSIGNED_BYTE, 0);
}

function drawScene(gl)
{
    // Make sure the canvas is sized correctly.
    reshape(gl);

    // Clear the canvas
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);

    // Make a model/view matrix.
	gl.mvMatrix.makeIdentity();
	var b = world.rigidBodies[0];
	gl.mvMatrix.translate(b.x.x, b.x.y, b.x.z);
	gl.mvMatrix.multiply(new J3DIMatrix4(world.rigidBodies[0].R.getAsArray4()));
	gl.mvMatrix.scale(b.l.x / 2, b.l.y / 2, b.l.z / 2);

	drawBox(gl);

	if (spring1)
	{
		var b = world.rigidBodies[0];	
		var p = b.l.mulScalar(0.5);      // point of force application in object
		p.x = -p.x;
		p = b.R.mulVector(p);            // rotated by the object rotation
		var pw = p.add(b.x);             // p in world space
		var c = new Float3(-2,0,0);
		var numDots = 8;
		for (var i = 0; i < numDots; i++)
		{
			var r = i / (numDots-1);
			var x = c.mulScalar(r).add(pw.mulScalar(1.0 - r));

			gl.mvMatrix.makeIdentity();
			gl.mvMatrix.translate(x.x, x.y, x.z);
			gl.mvMatrix.scale(0.02, 0.02, 0.02);

			drawBox(gl);
		}
	}
	if (spring2)
	{
		var b = world.rigidBodies[0];	
		var p = b.l.mulScalar(0.5);      // point of force application in object
		p = b.R.mulVector(p);            // rotated by the object rotation
		var pw = p.add(b.x);             // p in world space
		var c = new Float3(2,0,0);
		var numDots = 8;
		for (var i = 0; i < numDots; i++)
		{
			var r = i / (numDots-1);
			var x = c.mulScalar(r).add(pw.mulScalar(1.0 - r));

			gl.mvMatrix.makeIdentity();
			gl.mvMatrix.translate(x.x, x.y, x.z);
			gl.mvMatrix.scale(0.02, 0.02, 0.02);

			drawBox(gl);
		}
	}
}

function handleKey(event)
{
	var k = event.keyCode;
	if (k == '1'.charCodeAt(0))
	{
		spring1 = !spring1;
	}
	if (k == '2'.charCodeAt(0))
	{
		spring2 = !spring2;
	}
}

function initCanvas()
{
    var c = document.getElementById("canvas");
    var w = Math.floor(window.innerWidth * 0.9);
    var h = Math.floor(window.innerHeight * 0.9);

    c.width = w;
    c.height = h;

	reshape(gl);

	var b = document.getElementById("body");
	b.onkeydown = handleKey;
}

function computeForces()
{
	if (spring1)
	{
		// Spring of length 0 from world -2,0,0 to object vertex -0.5,0.5,0.5
		var c = new Float3(-2,0,0);
		var b = world.rigidBodies[0];
		var ks = 5.0;
		var kd = ks / 15.0;
		var p = b.l.mulScalar(0.5);      // point of force application in object
		p.x = -p.x;
		p = b.R.mulVector(p);            // rotated by the object rotation
		var pw = p.add(b.x);             // p in world space
		var x = c.sub(pw);               // spring extension
		var f = x.mulScalar(ks);         // force in world space
		var pdot = b.omega.cross(p);     // velocity of point of force application
		pdot.accum(b.v);                 // add to velocity of center of mass
		f.accumNeg(pdot.mulScalar(kd));  // damping component
		var t = p.cross(f);              // torque
		b.force.accum(f);
		b.torque.accum(t);
	}

	if (spring2)
	{
		// Spring of length 0 from world 2,0,0 to object vertex 0.5,0.5,0.5
		var c = new Float3(2,0,0);
		var b = world.rigidBodies[0];
		var ks = 5.0;
		var kd = ks / 15.0;
		var p = b.l.mulScalar(0.5);      // point of force application in object
		p = b.R.mulVector(p);            // rotated by the object rotation
		var pw = p.add(b.x);             // p in world space
		var x = c.sub(pw);               // spring extension
		var f = x.mulScalar(ks);         // force in world space
		var pdot = b.omega.cross(p);     // velocity of point of force application
		pdot.accum(b.v);                 // add to velocity of center of mass
		f.accumNeg(pdot.mulScalar(kd));  // damping component
		var t = p.cross(f);              // torque
		b.force.accum(f);
		b.torque.accum(t);
	}
}

function step(dt)
{
	computeForces();
	world.step(dt);
}

function draw()
{
	drawScene(gl);
}

function mainloop()
{
	setTimeout(mainloop, 1000.0/30.0);
	step(1.0/30.0);
	draw();
}

function main()
{
	init();
	initGL("canvas");
	initCanvas();
	mainloop();
}

