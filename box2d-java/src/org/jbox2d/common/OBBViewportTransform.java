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
package org.jbox2d.common;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;

/**
 * Orientated bounding box viewport transform
 * 
 * @author Daniel Murphy
 */
public class OBBViewportTransform implements IViewportTransform {

	public static class OBB {
		public final Mat22 R = new Mat22();
		public final Vector center = new Vector();
		public final Vector extents = new Vector();
	}

	protected final OBB box = new OBB();
	private boolean yFlip = false;
	private final Mat22 yFlipMat = new Mat22(1, 0, 0, -1);
	private final Mat22 yFlipMatInv = yFlipMat.invert();

	public OBBViewportTransform() {
		box.R.setIdentity();
	}

	public void set(final OBBViewportTransform vpt) {
		box.center.set(vpt.box.center);
		box.extents.set(vpt.box.extents);
		box.R.set(vpt.box.R);
		yFlip = vpt.yFlip;
	}

	/**
	 * @see IViewportTransform#setCamera(float, float, float)
	 */

	public void setCamera(final float x, final float y, final float scale) {
		box.center.set(x, y);
		Mat22.createScaleTransform(scale, box.R);
	}

	/**
	 * @see IViewportTransform#getExtents()
	 */

	public Vector getExtents() {
		return box.extents;
	}

	/**
	 * @see IViewportTransform#setExtents(Vector)
	 */

	public void setExtents(final Vector argExtents) {
		box.extents.set(argExtents);
	}

	/**
	 * @see IViewportTransform#setExtents(float, float)
	 */

	public void setExtents(final float argHalfWidth, final float argHalfHeight) {
		box.extents.set(argHalfWidth, argHalfHeight);
	}

	/**
	 * @see IViewportTransform#getCenter()
	 */

	public Vector getCenter() {
		return box.center;
	}

	/**
	 * @see IViewportTransform#setCenter(Vector)
	 */

	public void setCenter(final Vector argPos) {
		box.center.set(argPos);
	}

	/**
	 * @see IViewportTransform#setCenter(float, float)
	 */

	public void setCenter(final float x, final float y) {
		box.center.set(x, y);
	}

	/**
	 * gets the transform of the viewport, transforms around the center. Not a
	 * copy.
	 * 
	 * @return
	 */
	public Mat22 getTransform() {
		return box.R;
	}

	/**
	 * Sets the transform of the viewport. Transforms about the center.
	 * 
	 * @param transform
	 */
	public void setTransform(final Mat22 transform) {
		box.R.set(transform);
	}

	/**
	 * Multiplies the obb transform by the given transform
	 * 
	 * @param argTransform
	 */
	public void mulByTransform(final Mat22 argTransform) {
		box.R.mulLocal(argTransform);
	}

	/**
	 * @see IViewportTransform#isYFlip()
	 */

	public boolean isYFlip() {
		return yFlip;
	}

	/**
	 * @see IViewportTransform#setYFlip(boolean)
	 */

	public void setYFlip(final boolean yFlip) {
		this.yFlip = yFlip;
	}

	// djm pooling
	private final Mat22 inv = new Mat22();

	/**
	 * @see IViewportTransform#getScreenVectorToWorld(Vector, Vector)
	 */
	public void getScreenVectorToWorld(final Vector argScreen, final Vector argWorld) {
		inv.set(box.R);
		inv.invertLocal();
		inv.mulToOut(argScreen, argWorld);
		if (yFlip) {
			yFlipMatInv.mulToOut(argWorld, argWorld);
		}
	}

	/**
	 * @see IViewportTransform#getWorldVectorToScreen(Vector, Vector)
	 */
	public void getWorldVectorToScreen(final Vector argWorld, final Vector argScreen) {
		box.R.mulToOut(argWorld, argScreen);
		if (yFlip) {
			yFlipMatInv.mulToOut(argScreen, argScreen);
		}
	}

	/**
	 * @see IViewportTransform#getWorldToScreen(Vector, Vector)
	 */
	public void getWorldToScreen(final Vector argWorld, final Vector argScreen) {
		argScreen.set(argWorld);
		argScreen.subtractLocal(box.center);
		box.R.mulToOut(argScreen, argScreen);
		if (yFlip) {
			yFlipMat.mulToOut(argScreen, argScreen);
		}
		argScreen.addLocal(box.extents);
	}

	private final Mat22 inv2 = new Mat22();

	/**
	 * @see IViewportTransform#getScreenToWorld(Vector, Vector)
	 */
	public void getScreenToWorld(final Vector argScreen, final Vector argWorld) {
		argWorld.set(argScreen);
		argWorld.subtractLocal(box.extents);
		box.R.invertToOut(inv2);
		inv2.mulToOut(argWorld, argWorld);
		if (yFlip) {
			yFlipMatInv.mulToOut(argWorld, argWorld);
		}
		argWorld.addLocal(box.center);
	}
}
