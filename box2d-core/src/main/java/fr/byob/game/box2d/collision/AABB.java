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
package fr.byob.game.box2d.collision;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.MathUtils;


// updated to rev 100
/** An axis-aligned bounding box. */
public class AABB{
	/** Bottom left vertex of bounding box. */
	public final Vector lowerBound;
	/** Top right vertex of bounding box. */
	public final Vector upperBound;

	/**
	 * Creates the default object, with vertices at 0,0 and 0,0.
	 */
	public AABB() {
		lowerBound = new Vector();
		upperBound = new Vector();
	}

	/**
	 * Copies from the given object
	 * 
	 * @param copy
	 *            the object to copy from
	 */
	public AABB(final AABB copy) {
		this(copy.lowerBound, copy.upperBound);
	}

	/**
	 * Creates an AABB object using the given bounding
	 * vertices.
	 * 
	 * @param lowerVertex
	 *            the bottom left vertex of the bounding box
	 * @param upperVertex
	 *            the top right vertex of the bounding box
	 */
	public AABB(final Vector lowerVertex, final Vector upperVertex) {
		this.lowerBound = lowerVertex.clone(); // clone to be safe
		this.upperBound = upperVertex.clone();
	}

	/**
	 * Sets this object from the given object
	 * 
	 * @param aabb
	 *            the object to copy from
	 */
	public final void set(final AABB aabb) {
		lowerBound.set(aabb.lowerBound);
		upperBound.set(aabb.upperBound);
	}

	/** Verify that the bounds are sorted */
	public final boolean isValid() {
		final float dx = upperBound.x - lowerBound.x;
		if (dx < 0f) {
			return false;
		}
		final float dy = upperBound.y - lowerBound.y;
		if (dy < 0) {
			return false;
		}
		return MathUtils.isValid(lowerBound) && MathUtils.isValid(upperBound);
	}

	/**
	 * Get the center of the AABB
	 * 
	 * @return
	 */
	public final Vector getCenter() {
		final Vector center = new Vector(lowerBound);
		center.addLocal(upperBound);
		center.scaleLocal(.5f);
		return center;
	}

	public final void getCenterToOut(final Vector out) {
		out.x = (lowerBound.x + upperBound.x) * .5f;
		out.y = (lowerBound.y + upperBound.y) * .5f;
	}

	/**
	 * Get the extents of the AABB (half-widths).
	 * 
	 * @return
	 */
	public final Vector getExtents() {
		final Vector center = new Vector(upperBound);
		center.subtractLocal(lowerBound);
		center.scaleLocal(.5f);
		return center;
	}

	public final void getExtentsToOut(final Vector out) {
		out.x = (upperBound.x - lowerBound.x) * .5f;
		out.y = (upperBound.y - lowerBound.y) * .5f; // thanks FDN1
	}

	public final void getVertices(final Vector[] argRay) {
		argRay[0].set(lowerBound);
		argRay[1].set(lowerBound);
		argRay[1].x += upperBound.x - lowerBound.x;
		argRay[2].set(upperBound);
		argRay[3].set(upperBound);
		argRay[3].x -= upperBound.x - lowerBound.x;
	}

	/**
	 * Combine two AABBs into this one.
	 * 
	 * @param aabb1
	 * @param aab
	 */
	public final void combine(final AABB aabb1, final AABB aab) {
		lowerBound.x = aabb1.lowerBound.x < aab.lowerBound.x ? aabb1.lowerBound.x : aab.lowerBound.x;
		lowerBound.y = aabb1.lowerBound.y < aab.lowerBound.y ? aabb1.lowerBound.y : aab.lowerBound.y;
		upperBound.x = aabb1.upperBound.x > aab.upperBound.x ? aabb1.upperBound.x : aab.upperBound.x;
		upperBound.y = aabb1.upperBound.y > aab.upperBound.y ? aabb1.upperBound.y : aab.upperBound.y;
	}

	/**
	 * Does this aabb contain the provided AABB.
	 * 
	 * @return
	 */
	public final boolean contains(final AABB aabb) {
		/*
		 * boolean result = true;
		 * result = result && lowerBound.x <= aabb.lowerBound.x;
		 * result = result && lowerBound.y <= aabb.lowerBound.y;
		 * result = result && aabb.upperBound.x <= upperBound.x;
		 * result = result && aabb.upperBound.y <= upperBound.y;
		 * return result;
		 */
		// djm: faster putting all of them together, as if one is false we leave the logic
		// early
		return lowerBound.x > aabb.lowerBound.x && lowerBound.y > aabb.lowerBound.y && aabb.upperBound.x > upperBound.x
				&& aabb.upperBound.y > upperBound.y;
	}

	public static final boolean testOverlap(final AABB a, final AABB b) {
		if (b.lowerBound.x - a.upperBound.x > 0.0f || b.lowerBound.y - a.upperBound.y > 0.0f) {
			return false;
		}

		if (a.lowerBound.x - b.upperBound.x > 0.0f || a.lowerBound.y - b.upperBound.y > 0.0f) {
			return false;
		}

		return true;
	}

	@Override
	public final String toString() {
		final String s = "AABB[" + lowerBound + " . " + upperBound + "]";
		return s;
	}
}
