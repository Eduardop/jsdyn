// renderer.js
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

o3djs.base.o3d = o3d;
o3djs.require('o3djs.webgl');
o3djs.require('o3djs.math');
o3djs.require('o3djs.rendergraph');
o3djs.require('o3djs.primitives');
o3djs.require('o3djs.material');

// global variables
var g_o3dElement;
var g_client;
var g_o3d;
var g_math;
var g_pack;
var g_viewInfo;
var g_thisRenderer;

// --------------------------------------------------------------------
// Rendering system initialization
Renderer = function ()
{}

/**
 * Creates the client area.
 */
Renderer.prototype.initialize = function (eyePosition, targetPosition, frameCallback, bodyColors)
{
    window.g_finished = false; // for selenium testing.
    this.eyePosition = eyePosition;
    this.targetPosition = targetPosition;
    this.frameCallback = frameCallback;
    this.springTransforms = [];
    this.bodyTransforms = [];
    this.numSpringMarkers = 8;
    this.markerScale = 0.1;
    this.bodyColors = bodyColors;
    g_thisRenderer = this;
    o3djs.webgl.makeClients(Renderer.prototype.initialize2);
}

/**
 * Initializes global variables, positions camera, draws shapes.
 * @param {Array} clientElements Array of o3d object elements.
 */
Renderer.prototype.initialize2 = function (clientElements)
{
    g_thisRenderer.initGlobals(clientElements);
    g_thisRenderer.initContext();
    g_thisRenderer.createShapes();
    g_client.setRenderCallback(this.frameCallback);
    window.g_finished = true; // for selenium testing.
}

/**
 * Initializes global variables and libraries.
 */
Renderer.prototype.initGlobals = function (clientElements)
{
    g_o3dElement = clientElements[0];
    window.g_client = g_client = g_o3dElement.client;
    g_o3d = g_o3dElement.o3d;
    g_math = o3djs.math;
    
    // Create a pack to manage the objects created.
    g_pack = g_client.createPack();
    
    // Create the render graph for a view.
    var clearColor = [.98, .98, .98, 1];
    g_viewInfo = o3djs.rendergraph.createBasicView(
        g_pack, g_client.root, g_client.renderGraphRoot, clearColor);
}

/**
 * Sets up reasonable view and projection matrices.
 */
Renderer.prototype.initContext = function ()
{
    g_viewInfo.drawContext.projection = g_math.matrix4.perspective(
        g_math.degToRad(50), g_o3dElement.clientWidth / g_o3dElement.clientHeight, 1, 5000);
    g_viewInfo.drawContext.view = g_math.matrix4.lookAt(
        this.eyePosition, this.targetPosition, [0, 1, 0]);
}

/**
 * Creates shapes based on the physics world contents.
 */
Renderer.prototype.createShapes = function ()
{
    
    var materialSpring = o3djs.material.createBasicMaterial(
        g_pack, g_viewInfo, [0.6, 0.5, 0.1, 1]);
    materialSpring.getParam('specularFactor').value = 0.0;
    var springMarker = o3djs.primitives.createSphere(
        g_pack, materialSpring, 0.5, 6, 6);
    
    for (var b in g_world.rigidBodies)
    {
        var color = (this.bodyColors) ? this.bodyColors[b] : [0.0, 0.2, 1.0, 1];
        
        var materialCube = o3djs.material.createBasicMaterial(
            g_pack, g_viewInfo, color);
        materialCube.getParam('specularFactor').value = 0.5;
        materialCube.getParam('ambient').value = [0.3, 0.3, 0.3, 1];
        
        var cube = o3djs.primitives.createCube(
            g_pack, materialCube, 1.0);
        
        var transform = g_pack.createObject('Transform');
        transform.addShape(cube);
        transform.parent = g_client.root;
        this.bodyTransforms.push(transform);
    }
    
    for (var s in g_world.springs)
    {
        var parentTransform = g_pack.createObject('Transform');
        parentTransform.parent = g_client.root;
        for (var i = 0; i < this.numSpringMarkers; i++)
        {
            var transform = g_pack.createObject('Transform');
            transform.addShape(springMarker);
            parentTransform.addChild(transform);
        }
        this.springTransforms.push(parentTransform);
    }
}

/**
 * Enable or disable rendering of spring
 */
Renderer.prototype.setSpringVisible = function (springi, visible)
{
    this.springTransforms[springi].visible = visible;
}

/**
 * Updates renderer transforms based on physics world
 */
Renderer.prototype.update = function ()
{
    for (var bi in g_world.rigidBodies)
    {
        var b = g_world.rigidBodies[bi];
        var t = this.bodyTransforms[bi];
        t.identity();
        t.translate(b.x.x, b.x.y, b.x.z);
        o3d.Transform.compose(t.localMatrix, b.R.getAsO3DMatrix4());
        t.scale(b.l.x, b.l.y, b.l.z);
    }
    
    for (var springi in g_world.springs)
    {
        var spr = g_world.springs[springi];
        if (spr.enabled)
        {
            var pw = [];
            for (var i = 0; i < 2; i++)
            {
                var ep = spr.endPoints[i];
                var b = ep.body;
                if (b == 0)
                {
                    pw[i] = ep.x; // point of spring connection in world
                }
                else
                {
                    var p = ep.x; // point of spring connection in object
                    p = b.R.mulVector(p); // rotated by the object rotation
                    pw[i] = p.add(b.x); // p in world space
                }
            }
            var transforms = this.springTransforms[springi].children;
            for (var i = 0; i < this.numSpringMarkers; i++)
            {
                var r = i / (this.numSpringMarkers - 1);
                var x = pw[0].mulScalar(r).add(pw[1].mulScalar(1.0 - r));
                
                var transform = transforms[i];
                transform.identity();
                transform.translate(x.x, x.y, x.z);
                transform.scale(this.markerScale, this.markerScale, this.markerScale);
            }
        }
    }
}