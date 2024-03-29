/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 4:35:29 AM Jul 15, 2010
 */
package org.jbox2d.callbacks;

import org.jbox2d.common.Color3f;
import org.jbox2d.common.IViewportTransform;
import org.jbox2d.common.Transform;

import pythagoras.f.Vector;

// updated to rev 100
/**
 * Implement this abstract class to allow JBox2d to
 * automatically draw your physics for debugging purposes.
 * Not intended to replace your own custom rendering
 * routines!
 * @author Daniel Murphy
 */
public abstract class DebugDraw implements fr.byob.game.box2d.callbacks.DebugDraw {

	public static final int e_shapeBit				= 0x0001; ///< draw shapes
	public static final int e_jointBit				= 0x0002; ///< draw joint connections
	public static final int e_aabbBit				= 0x0004; ///< draw core (TOI) shapes
	public static final int e_pairBit				= 0x0008; ///< draw axis aligned bounding boxes
	public static final int e_centerOfMassBit		= 0x0010; ///< draw center of mass frame
	public static final int e_dynamicTreeBit		= 0x0020; ///< draw dynamic tree.

	protected int m_drawFlags;
	protected final IViewportTransform viewportTransform;

	public DebugDraw(final IViewportTransform viewport) {
		m_drawFlags = 0;
		viewportTransform = viewport;
	}

	public void setFlags(final int flags) {
		m_drawFlags = flags;
	}

	public int getFlags() {
		return m_drawFlags;
	}

	public void appendFlags(final int flags) {
		m_drawFlags |= flags;
	}

	public void clearFlags(final int flags) {
		m_drawFlags &= ~flags;
	}

	/**
	 * Draw a closed polygon provided in CCW order.  This implementation
	 * uses {@link #drawSegment(Vector, Vector, Color3f)} to draw each side of the
	 * polygon.
	 * @param vertices
	 * @param vertexCount
	 * @param color
	 */
	public void drawPolygon(final Vector[] vertices, final int vertexCount, final Color3f color){
		if(vertexCount == 1){
			drawSegment(vertices[0], vertices[0], color);
			return;
		}

		for(int i=0; i<vertexCount-1; i+=1){
			drawSegment(vertices[i], vertices[i+1], color);
		}

		if(vertexCount > 2){
			drawSegment(vertices[vertexCount-1], vertices[0], color);
		}
	}

	public abstract void drawPoint(Vector argPoint, float argRadiusOnScreen, Color3f argColor);

	/**
	 * Draw a solid closed polygon provided in CCW order.
	 * @param vertices
	 * @param vertexCount
	 * @param color
	 */
	public abstract void drawSolidPolygon(Vector[] vertices, int vertexCount, Color3f color);

	/**
	 * Draw a circle.
	 * @param center
	 * @param radius
	 * @param color
	 */
	public abstract void drawCircle(Vector center, float radius, Color3f color);

	/**
	 * Draw a solid circle.
	 * @param center
	 * @param radius
	 * @param axis
	 * @param color
	 */
	public abstract void drawSolidCircle(Vector center, float radius, Vector axis, Color3f color);

	/**
	 * Draw a line segment.
	 * @param p1
	 * @param p2
	 * @param color
	 */
	public abstract void drawSegment(Vector p1, Vector p2, Color3f color);

	/**
	 * Draw a transform.  Choose your own length scale
	 * @param xf
	 */
	public abstract void drawTransform(Transform xf);

	/**
	 * Draw a string.
	 * @param x
	 * @param y
	 * @param s
	 * @param color
	 */
	public abstract void drawString(float x, float y, String s, Color3f color);

	public IViewportTransform getViewportTranform(){
		return viewportTransform;
	}

	/**
	 * @param x
	 * @param y
	 * @param scale
	 * @see IViewportTransform#setCamera(float, float, float)
	 */
	public void setCamera(final float x, final float y, final float scale){
		viewportTransform.setCamera(x,y,scale);
	}


	/**
	 * @param argScreen
	 * @param argWorld
	 * @see org.jbox2d.common.IViewportTransform#getScreenToWorld(org.jbox2d.common.Vector, org.jbox2d.common.Vector)
	 */
	public void getScreenToWorldToOut(final Vector argScreen, final Vector argWorld) {
		viewportTransform.getScreenToWorld(argScreen, argWorld);
	}

	/**
	 * @param argWorld
	 * @param argScreen
	 * @see org.jbox2d.common.IViewportTransform#getWorldToScreen(org.jbox2d.common.Vector, org.jbox2d.common.Vector)
	 */
	public void getWorldToScreenToOut(final Vector argWorld, final Vector argScreen) {
		viewportTransform.getWorldToScreen(argWorld, argScreen);
	}

	/**
	 * Takes the world coordinates and puts the corresponding screen
	 * coordinates in argScreen.
	 * @param worldX
	 * @param worldY
	 * @param argScreen
	 */
	public void getWorldToScreenToOut(final float worldX, final float worldY, final Vector argScreen){
		argScreen.set(worldX,worldY);
		viewportTransform.getWorldToScreen(argScreen, argScreen);
	}

	/**
	 * takes the world coordinate (argWorld) and returns
	 * the screen coordinates.
	 * @param argWorld
	 */
	public Vector getWorldToScreen(final Vector argWorld){
		final Vector screen = new Vector();
		viewportTransform.getWorldToScreen( argWorld, screen);
		return screen;
	}

	/**
	 * Takes the world coordinates and returns the screen
	 * coordinates.
	 * @param worldX
	 * @param worldY
	 */
	public Vector getWorldToScreen(final float worldX, final float worldY){
		final Vector argScreen = new Vector(worldX, worldY);
		viewportTransform.getWorldToScreen( argScreen, argScreen);
		return argScreen;
	}

	/**
	 * takes the screen coordinates and puts the corresponding
	 * world coordinates in argWorld.
	 * @param screenX
	 * @param screenY
	 * @param argWorld
	 */
	public void getScreenToWorldToOut(final float screenX, final float screenY, final Vector argWorld){
		argWorld.set(screenX,screenY);
		viewportTransform.getScreenToWorld(argWorld, argWorld);
	}

	/**
	 * takes the screen coordinates (argScreen) and returns
	 * the world coordinates
	 * @param argScreen
	 */
	public Vector getScreenToWorld(final Vector argScreen){
		final Vector world = new Vector();
		viewportTransform.getScreenToWorld(argScreen, world);
		return world;
	}

	/**
	 * takes the screen coordinates and returns the
	 * world coordinates.
	 * @param screenX
	 * @param screenY
	 */
	public Vector getScreenToWorld(final float screenX, final float screenY){
		final Vector screen = new Vector(screenX, screenY);
		viewportTransform.getScreenToWorld( screen, screen);
		return screen;
	}

	/**
	 * Clears the screen so we can draw the scene from scratch.
	 */
	public abstract void clear();
}
