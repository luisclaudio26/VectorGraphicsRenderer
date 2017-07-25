# Vector Graphics Renderer

<img src="https://github.com/luisclaudio26/VectorGraphicsRenderer/raw/master/dancer.png" width="200">  <img src="https://github.com/luisclaudio26/VectorGraphicsRenderer/raw/master/drops.png" width="400">

Dancer and Drops illustrations, which took 57s and 276s, respectively.

<img src="https://github.com/luisclaudio26/VectorGraphicsRenderer/raw/master/hello-type1.png" width="200">    <img src="https://github.com/luisclaudio26/VectorGraphicsRenderer/raw/master/hello-ttf.png" width="200">

Type1 and TrueType font examples. The first uses cubic Bézier curves while the second use quadratic curves. Both took ~2s to render.

## Overview

This is the final product from the 2D Computer Graphics summer course I took on the Institute of Pure and Applied Mathematics (IMPA). The course covered a broad range of topics, including Bézier curves (and its implicitization), texture mapping, anti-aliasing and acceleration structures and in the end we produced a renderer which could take SVG illustrations and render it.
Obviously the renderer could not support all the features .SVG is able to describe (stroking, for example), as this was a didactic project.

This code depends on the (awesome) framework provided by prof. Diego Nehab and for such the only file available here was the one I wrote for the assignments, which contains the inside-outside tests, gradient painting, etc.

I was awarded a grade A in the end of the course and it was certainly the best academic experience I ever had.

## What is does now

The renderer handles:

- Quadratic and cubic Bézier curves, triangles, circles, straight line segments
- Alpha-blending
- Supersampling, using a precomputed blue noise sampling pattern
- Texture mapping, linear and radial gradients

Unfortunatelly the renderer has no support to acceleration structures, as this was the only assignment we did in teams and we used the code of another colleague.
