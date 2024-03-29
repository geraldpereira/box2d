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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.common;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;

// updated to rev 100

/**
 * A transform contains translation and rotation. It is used to represent
 * the position and orientation of rigid frames.
 */
public class Transform implements fr.byob.game.box2d.common.Transform {

	/** The translation caused by the transform */
	public final Vector position;

	/** A matrix representing a rotation */
	public final Mat22 R;

	// The identity transform
	public static Transform identity;

	static{
		Transform.identity = new Transform();
		Transform.identity.setIdentity();
	}

	/** The default constructor. */
	public Transform() {
		position = new Vector();
		R = new Mat22();
	}

	/** Initialize as a copy of another transform. */
	public Transform(final Transform xf) {
		position = xf.position.clone();
		R = xf.R.clone();
	}

	/** Initialize using a position vector and a rotation matrix. */
	public Transform(final Vector _position, final Mat22 _R) {
		position = _position.clone();
		R = _R.clone();
	}

	/** Set this to equal another transform. */
	public final Transform set(final Transform xf) {
		position.set(xf.position);
		R.set(xf.R);
		return this;
	}

	/**
	 * Set this based on the position and angle.
	 * 
	 * @param p
	 * @param angle
	 */
	public final void set(final Vector p, final float angle) {
		position.set(p);
		R.set(angle);
	}

	/**
	 * Calculate the angle that the rotation matrix represents.
	 */
	public final float getAngle() {
		return MathUtils.atan2(R.m12, R.m11);
	}

	/** Set this to the identity transform. */
	public final void setIdentity() {
		position.set(0,0);
		R.setIdentity();
	}

	public final static Vector mul(final Transform T, final Vector v) {
		return new Vector(T.position.x + T.R.m11 * v.x + T.R.m21 * v.y, T.position.y + T.R.m12 * v.x
				+ T.R.m22 * v.y);
	}

	/* djm added */
	public final static void mulToOut(final Transform T, final Vector v, final Vector out) {
		final float tempy = T.position.y + T.R.m12 * v.x + T.R.m22 * v.y;
		out.x = T.position.x + T.R.m11 * v.x + T.R.m21 * v.y;
		out.y = tempy;
	}

	public final static Vector mulTrans(final Transform T, final Vector v) {
		final float v1x = v.x - T.position.x;
		final float v1y = v.y - T.position.y;
		return new Vector((v1x * T.R.m11 + v1y * T.R.m12), (v1x * T.R.m21 + v1y * T.R.m22));
		// return T.R.mulT(v.sub(T.position));
	}

	public final static void mulTransToOut(final Transform T, final Vector v, final Vector out) {
		final float v1x = v.x - T.position.x;
		final float v1y = v.y - T.position.y;
		final float tempy = v1x * T.R.m21 + v1y * T.R.m22;
		out.x = v1x * T.R.m11 + v1y * T.R.m12;
		out.y = tempy;
	}

	@Override
	public final String toString() {
		String s = "XForm:\n";
		s += "Position: " + position + "\n";
		s += "R: \n" + R + "\n";
		return s;
	}

	public Vector getPosition() {
		return position;
	}
}
