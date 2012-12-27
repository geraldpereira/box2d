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

package org.jbox2d.collision.shapes;

import org.jbox2d.common.Transform;

import pythagoras.f.Vector;
import fr.byob.game.box2d.collision.AABB;
import fr.byob.game.box2d.collision.RayCastInput;
import fr.byob.game.box2d.collision.RayCastOutput;
import fr.byob.game.box2d.collision.shapes.BoxShape;
import fr.byob.game.box2d.collision.shapes.EdgeShape;
import fr.byob.game.box2d.collision.shapes.MassData;
import fr.byob.game.box2d.collision.shapes.ShapeType;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;

//Updated to rev 100

/** A convex polygon shape. Create using Body.createShape(ShapeDef), not the ructor here. */
public class PolygonShape extends Shape implements fr.byob.game.box2d.collision.shapes.PolygonShape, BoxShape, EdgeShape {
	/** Dump lots of debug information. */
	private static boolean m_debug = false;

	/**
	 * Local position of the shape centroid in parent body frame.
	 */
	public final Vector m_centroid = new Vector();

	/**
	 * The vertices of the shape. Note: use getVertexCount(), not m_vertices.length, to
	 * get number of active vertices.
	 */
	public final Vector m_vertices[];

	/**
	 * The normals of the shape. Note: use getVertexCount(), not m_normals.length, to get
	 * number of active normals.
	 */
	public final Vector m_normals[];

	/**
	 * Number of active vertices in the shape.
	 */
	public int m_vertexCount;

	// pooling
	private final Vector pool1 = new Vector();
	private final Vector pool2 = new Vector();
	private final Vector pool3 = new Vector();
	private final Vector pool4 = new Vector();
	private final Vector pool5 = new Vector();
	private final Vector pool6 = new Vector();
	private Transform poolt1 = new Transform();

	public PolygonShape() {
		m_type = ShapeType.POLYGON;

		m_vertexCount = 0;
		m_vertices = new Vector[Settings.maxPolygonVertices];
		for (int i = 0; i < m_vertices.length; i++) {
			m_vertices[i] = new Vector();
		}
		m_normals = new Vector[Settings.maxPolygonVertices];
		for (int i = 0; i < m_normals.length; i++) {
			m_normals[i] = new Vector();
		}
		m_radius = Settings.polygonRadius;
		m_centroid.set(0,0);
	}

	@Override
	public final Shape clone() {
		final PolygonShape shape = new PolygonShape();
		shape.m_centroid.set(this.m_centroid);
		for (int i = 0; i < shape.m_normals.length; i++) {
			shape.m_normals[i].set(m_normals[i]);
			shape.m_vertices[i].set(m_vertices[i]);
		}
		shape.m_radius = this.m_radius;
		shape.m_vertexCount = this.m_vertexCount;
		return shape;
	}

	/**
	 * Get the supporting vertex index in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final int getSupport(final Vector d) {
		int bestIndex = 0;
		float bestValue = MathUtils.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; i++) {
			final float value = MathUtils.dot(m_vertices[i], d);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return bestIndex;
	}

	/**
	 * Get the supporting vertex in the given direction.
	 * 
	 * @param d
	 * @return
	 */
	public final Vector getSupportVertex(final Vector d) {
		int bestIndex = 0;
		float bestValue = MathUtils.dot(m_vertices[0], d);
		for (int i = 1; i < m_vertexCount; i++) {
			final float value = MathUtils.dot(m_vertices[i], d);
			if (value > bestValue) {
				bestIndex = i;
				bestValue = value;
			}
		}
		return m_vertices[bestIndex];
	}

	/**
	 * Copy vertices. This assumes the vertices define a convex polygon.
	 * It is assumed that the exterior is the the right of each edge.
	 */
	public final void set(final Vector[] vertices, final int count) {
		assert (2 <= count && count <= Settings.maxPolygonVertices);
		m_vertexCount = count;

		// Copy vertices.
		for (int i = 0; i < m_vertexCount; ++i) {
			if(m_vertices[i] == null){
				m_vertices[i] = new Vector();
			}
			m_vertices[i].set(vertices[i]);
		}

		final Vector edge = pool1;

		// Compute normals. Ensure the edges have non-zero length.
		for (int i = 0; i < m_vertexCount; ++i) {
			final int i1 = i;
			final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
			edge.set(m_vertices[i2]).subtractLocal(m_vertices[i1]);

			assert (edge.lengthSq() > Settings.EPSILON * Settings.EPSILON);
			MathUtils.crossToOut(edge, 1f, m_normals[i]);
			MathUtils.normalize(m_normals[i]);
		}

		if (m_debug) {

			final Vector r = pool2;

			// Ensure the polygon is convex and the interior
			// is to the left of each edge.
			for (int i = 0; i < m_vertexCount; ++i) {
				final int i1 = i;
				final int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
				edge.set(m_vertices[i2]).subtractLocal(m_vertices[i1]);

				for (int j = 0; j < m_vertexCount; ++j) {
					// Don't check vertices on the current edge.
					if (j == i1 || j == i2) {
						continue;
					}

					r.set(m_vertices[j]).subtractLocal(m_vertices[i1]);

					// Your polygon is non-convex (it has an indentation) or
					// has colinear edges.
					final float s = MathUtils.cross(edge, r);
					assert (s > 0.0f);
				}
			}
		}

		// Compute the polygon centroid.
		computeCentroidToOut(m_vertices, m_vertexCount, m_centroid);
	}

	/**
	 * Build vertices to represent an axis-aligned box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 */
	public final void setAsBox(final float hx, final float hy) {
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set(hx, -hy);
		m_vertices[2].set(hx, hy);
		m_vertices[3].set(-hx, hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.set(0,0);
	}

	/**
	 * Build vertices to represent an oriented box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 * @param center
	 *            the center of the box in local coordinates.
	 * @param angle
	 *            the rotation of the box in local coordinates.
	 */
	public final void setAsBox(final float hx, final float hy, final Vector center, final float angle) {
		m_vertexCount = 4;
		m_vertices[0].set(-hx, -hy);
		m_vertices[1].set(hx, -hy);
		m_vertices[2].set(hx, hy);
		m_vertices[3].set(-hx, hy);
		m_normals[0].set(0.0f, -1.0f);
		m_normals[1].set(1.0f, 0.0f);
		m_normals[2].set(0.0f, 1.0f);
		m_normals[3].set(-1.0f, 0.0f);
		m_centroid.set(center);

		final Transform xf = poolt1;
		xf.position.set(center);
		xf.R.set(angle);

		// Transform vertices and normals.
		for (int i = 0; i < m_vertexCount; ++i) {
			Transform.mulToOut(xf, m_vertices[i], m_vertices[i]);
			Mat22.mulToOut(xf.R, m_normals[i], m_normals[i]);
		}
	}

	/**
	 * Set this as a single edge.
	 * 
	 * @param v1
	 * @param v2
	 */
	public final void setAsEdge(final Vector v1, final Vector v2) {
		m_vertexCount = 2;
		m_vertices[0].set(v1);
		m_vertices[1].set(v2);
		m_centroid.set(v1).addLocal(v2).scaleLocal(0.5f);
		// = 0.5f * (v1 + v2);
		m_normals[0].set(v2).subtractLocal(v1);
		MathUtils.crossToOut(m_normals[0], 1f, m_normals[0]);
		// m_normals[0] = Cross(v2 - v1, 1.0f);
		MathUtils.normalize(m_normals[0]);
		m_normals[1].set(m_normals[0]).negateLocal();
	}

	/**
	 * @see Shape#testPoint(Transform, Vector)
	 */
	@Override
	public final boolean testPoint(final Transform xf, final Vector p) {

		final Vector pLocal = pool1;

		pLocal.set(p).subtractLocal(xf.position);
		Mat22.mulTransToOut(xf.R, pLocal, pLocal);

		if (m_debug) {
			System.out.println("--testPoint debug--");
			System.out.println("Vertices: ");
			for (int i = 0; i < m_vertexCount; ++i) {
				System.out.println(m_vertices[i]);
			}
			System.out.println("pLocal: " + pLocal);
		}

		final Vector temp = pool2;

		for (int i = 0; i < m_vertexCount; ++i) {
			temp.set(pLocal).subtractLocal(m_vertices[i]);
			final float dot = MathUtils.dot(m_normals[i], temp);
			if (dot > 0.0f) {
				return false;
			}
		}

		return true;
	}

	/**
	 * @see Shape#computeAABB(AABB, Transform)
	 */
	@Override
	public final void computeAABB(final AABB argAabb, final Transform argXf) {

		final Vector lower = pool1;
		final Vector upper = pool2;
		final Vector v = pool3;

		Transform.mulToOut(argXf, m_vertices[0], lower);
		upper.set(lower);

		for (int i = 1; i < m_vertexCount; ++i) {
			Transform.mulToOut(argXf, m_vertices[i], v);
			// Vec2 v = Mul(xf, m_vertices[i]);
			MathUtils.minToOut(lower, v, lower);
			MathUtils.maxToOut(upper, v, upper);
		}

		// Vec2 r(m_radius, m_radius);
		// aabb->lowerBound = lower - r;
		// aabb->upperBound = upper + r;

		argAabb.lowerBound.x = lower.x - m_radius;
		argAabb.lowerBound.y = lower.y - m_radius;
		argAabb.upperBound.x = upper.x + m_radius;
		argAabb.upperBound.y = upper.y + m_radius;
	}

	// djm pooling, and from above
	/*
	 * private static final TLVec2 tlNormalL = new TLVec2();
	 * private static final TLMassData tlMd = new TLMassData();
	 * private static final FloatArray tldepths = new FloatArray();
	 * private static final TLVec2 tlIntoVec = new TLVec2();
	 * private static final TLVec2 tlOutoVec = new TLVec2();
	 * private static final TLVec2 tlP2b = new TLVec2();
	 * private static final TLVec2 tlP3 = new TLVec2();
	 * private static final TLVec2 tlcenter = new TLVec2();
	 * /*
	 * @see Shape#computeSubmergedArea(Vec2, float, XForm, Vec2)
	 * public float computeSubmergedArea(final Vec2 normal, float offset, Transform xf,
	 * Vec2 c) {
	 * final Vec2 normalL = tlNormalL.get();
	 * final MassData md = tlMd.get();
	 * //Transform plane into shape co-ordinates
	 * Mat22.mulTransToOut(xf.R,normal, normalL);
	 * float offsetL = offset - Vec2.dot(normal,xf.position);
	 * final Float[] depths = tldepths.get(Settings.maxPolygonVertices);
	 * int diveCount = 0;
	 * int intoIndex = -1;
	 * int outoIndex = -1;
	 * boolean lastSubmerged = false;
	 * int i = 0;
	 * for (i = 0; i < m_vertexCount; ++i){
	 * depths[i] = Vec2.dot(normalL,m_vertices[i]) - offsetL;
	 * boolean isSubmerged = depths[i]<-Settings.EPSILON;
	 * if (i > 0){
	 * if (isSubmerged){
	 * if (!lastSubmerged){
	 * intoIndex = i-1;
	 * diveCount++;
	 * }
	 * }
	 * else{
	 * if (lastSubmerged){
	 * outoIndex = i-1;
	 * diveCount++;
	 * }
	 * }
	 * }
	 * lastSubmerged = isSubmerged;
	 * }
	 * switch(diveCount){
	 * case 0:
	 * if (lastSubmerged){
	 * //Completely submerged
	 * computeMass(md, 1.0f);
	 * Transform.mulToOut(xf,md.center, c);
	 * return md.mass;
	 * }
	 * else{
	 * return 0;
	 * }
	 * case 1:
	 * if(intoIndex==-1){
	 * intoIndex = m_vertexCount-1;
	 * }
	 * else{
	 * outoIndex = m_vertexCount-1;
	 * }
	 * break;
	 * }
	 * final Vec2 intoVec = tlIntoVec.get();
	 * final Vec2 outoVec = tlOutoVec.get();
	 * final Vec2 e1 = tle1.get();
	 * final Vec2 e2 = tle2.get();
	 * int intoIndex2 = (intoIndex+1) % m_vertexCount;
	 * int outoIndex2 = (outoIndex+1) % m_vertexCount;
	 * float intoLambda = (0 - depths[intoIndex]) / (depths[intoIndex2] -
	 * depths[intoIndex]);
	 * float outoLambda = (0 - depths[outoIndex]) / (depths[outoIndex2] -
	 * depths[outoIndex]);
	 * intoVec.set(m_vertices[intoIndex].x*(1-intoLambda)+m_vertices[intoIndex2].x*intoLambda
	 * ,
	 * m_vertices[intoIndex].y*(1-intoLambda)+m_vertices[intoIndex2].y*intoLambda);
	 * outoVec.set(m_vertices[outoIndex].x*(1-outoLambda)+m_vertices[outoIndex2].x*outoLambda
	 * ,
	 * m_vertices[outoIndex].y*(1-outoLambda)+m_vertices[outoIndex2].y*outoLambda);
	 * // Initialize accumulator
	 * float area = 0;
	 * final Vec2 center = tlcenter.get();
	 * center.set(0,0);
	 * final Vec2 p2b = tlP2b.get().set(m_vertices[intoIndex2]);
	 * final Vec2 p3 = tlP3.get();
	 * p3.set(0,0);
	 * float k_inv3 = 1.0f / 3.0f;
	 * // An awkward loop from intoIndex2+1 to outIndex2
	 * i = intoIndex2;
	 * while (i != outoIndex2){
	 * i = (i+1) % m_vertexCount;
	 * if (i == outoIndex2){
	 * p3.set(outoVec);
	 * }
	 * else{
	 * p3.set(m_vertices[i]);
	 * }
	 * // Add the triangle formed by intoVec,p2,p3
	 * {
	 * e1.set(p2b).subLocal(intoVec);
	 * e2.set(p3).subLocal(intoVec);
	 * float D = Vec2.cross(e1, e2);
	 * float triangleArea = 0.5f * D;
	 * area += triangleArea;
	 * // Area weighted centroid
	 * center.x += triangleArea * k_inv3 * (intoVec.x + p2b.x + p3.x);
	 * center.y += triangleArea * k_inv3 * (intoVec.y + p2b.y + p3.y);
	 * }
	 * //
	 * p2b.set(p3);
	 * }
	 * // Normalize and transform centroid
	 * center.x *= 1.0f / area;
	 * center.y *= 1.0f / area;
	 * Transform.mulToOut(xf, center, c);
	 * return area;
	 * }
	 */

	/*
	 * Get the supporting vertex index in the given direction.
	 * @param d
	 * @return
	 * public final int getSupport( final Vec2 d){
	 * int bestIndex = 0;
	 * float bestValue = Vec2.dot(m_vertices[0], d);
	 * for (int i = 1; i < m_vertexCount; ++i){
	 * final float value = Vec2.dot(m_vertices[i], d);
	 * if (value > bestValue){
	 * bestIndex = i;
	 * bestValue = value;
	 * }
	 * }
	 * return bestIndex;
	 * }
	 * /**
	 * Get the supporting vertex in the given direction.
	 * @param d
	 * @return
	 * public final Vec2 getSupportVertex( final Vec2 d){
	 * int bestIndex = 0;
	 * float bestValue = Vec2.dot(m_vertices[0], d);
	 * for (int i = 1; i < m_vertexCount; ++i){
	 * final float value = Vec2.dot(m_vertices[i], d);
	 * if (value > bestValue){
	 * bestIndex = i;
	 * bestValue = value;
	 * }
	 * }
	 * return m_vertices[bestIndex];
	 * }
	 */

	/**
	 * Get the vertex count.
	 * 
	 * @return
	 */
	public final int getVertexCount() {
		return m_vertexCount;
	}

	/**
	 * Get a vertex by index.
	 * 
	 * @param index
	 * @return
	 */
	public final Vector getVertex(final int index) {
		assert (0 <= index && index < m_vertexCount);
		return m_vertices[index];
	}


	/**
	 * @see org.jbox2d.collision.shapes.Shape#raycast(RayCastOutput, RayCastInput, Transform)
	 */
	@Override
	public final boolean raycast(final RayCastOutput argOutput, final RayCastInput argInput, final Transform argXf) {
		final Vector p1 = pool1;
		final Vector p2 = pool2;
		final Vector d = pool3;
		final Vector temp = pool4;

		p1.set(argInput.p1).subtractLocal(argXf.position);
		Mat22.mulTransToOut(argXf.R, p1, p1);
		p2.set(argInput.p2).subtractLocal(argXf.position);
		Mat22.mulTransToOut(argXf.R, p2, p2);
		d.set(p2).subtractLocal(p1);

		if (m_vertexCount == 2) {
			final Vector v1 = m_vertices[0];
			final Vector v2 = m_vertices[1];
			final Vector normal = m_normals[0];

			// q = p1 + t * d
			// dot(normal, q - v1) = 0
			// dot(normal, p1 - v1) + t * dot(normal, d) = 0
			temp.set(v1).subtractLocal(p1);
			final float numerator = MathUtils.dot(normal, temp);
			final float denominator = MathUtils.dot(normal, d);

			if (denominator == 0.0f) {
				return false;
			}

			final float t = numerator / denominator;
			if (t < 0.0f || 1.0f < t) {
				return false;
			}

			final Vector q = pool5;
			final Vector r = pool6;

			// Vec2 q = p1 + t * d;
			temp.set(d).scaleLocal(t);
			q.set(p1).addLocal(temp);

			// q = v1 + s * r
			// s = dot(q - v1, r) / dot(r, r)
			// Vec2 r = v2 - v1;
			r.set(v2).subtractLocal(v1);

			final float rr = MathUtils.dot(r, r);
			if (rr == 0.0f) {
				return false;
			}

			temp.set(q).subtractLocal(v1);
			final float s = MathUtils.dot(temp, r) / rr;
			if (s < 0.0f || 1.0f < s) {
				return false;
			}

			argOutput.fraction = t;
			if (numerator > 0.0f) {
				// argOutput.normal = -normal;
				argOutput.normal.set(normal).scaleLocal(-1);
			}
			else {
				// output.normal = normal;
				argOutput.normal.set(normal);
			}
			return true;
		}
		else {

			float lower = 0, upper = argInput.maxFraction;

			int index = -1;

			for (int i = 0; i < m_vertexCount; ++i) {
				// p = p1 + a * d
				// dot(normal, p - v) = 0
				// dot(normal, p1 - v) + a * dot(normal, d) = 0
				temp.set(m_vertices[i]).subtractLocal(p1);
				final float numerator = MathUtils.dot(m_normals[i], temp);
				final float denominator = MathUtils.dot(m_normals[i], d);

				if (denominator == 0.0f) {
					if (numerator < 0.0f) {
						return false;
					}
				}
				else {
					// Note: we want this predicate without division:
					// lower < numerator / denominator, where denominator < 0
					// Since denominator < 0, we have to flip the inequality:
					// lower < numerator / denominator <==> denominator * lower >
					// numerator.
					if (denominator < 0.0f && numerator < lower * denominator) {
						// Increase lower.
						// The segment enters this half-space.
						lower = numerator / denominator;
						index = i;
					}
					else if (denominator > 0.0f && numerator < upper * denominator) {
						// Decrease upper.
						// The segment exits this half-space.
						upper = numerator / denominator;
					}
				}

				if (upper < lower) {
					return false;
				}
			}

			assert (0.0f <= lower && lower <= argInput.maxFraction);

			if (index >= 0) {
				argOutput.fraction = lower;
				Mat22.mulToOut(argXf.R, m_normals[index], argOutput.normal);
				// normal = Mul(xf.R, m_normals[index]);
				return true;
			}
		}
		return false;
	}


	public final void computeCentroidToOut(final Vector[] vs, final int count, final Vector out) {
		assert (count >= 3);

		out.set(0.0f, 0.0f);
		float area = 0.0f;

		if (count == 2) {
			out.set(vs[0]).addLocal(vs[1]).scaleLocal(.5f);
			return;
		}

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vector pRef = pool1;
		pRef.set(0,0);

		final Vector e1 = pool2;
		final Vector e2 = pool3;

		final float inv3 = 1.0f / 3.0f;

		for (int i = 0; i < count; ++i) {
			// Triangle vertices.
			final Vector p1 = pRef;
			final Vector p2 = vs[i];
			final Vector p3 = i + 1 < count ? vs[i + 1] : vs[0];

			e1.set(p2).subtractLocal(p1);
			e2.set(p3).subtractLocal(p1);

			final float D = MathUtils.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			out.addLocal(p1).addLocal(p2).addLocal(p3).scaleLocal(triangleArea * inv3);
		}

		// Centroid
		assert (area > Settings.EPSILON);
		out.scaleLocal(1.0f / area);
	}

	/**
	 * @see Shape#computeMass(MassData, float)
	 */
	@Override
	public void computeMass(final MassData massData, final float density) {
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		assert (m_vertexCount >= 2);

		// A line segment has zero mass.
		if (m_vertexCount == 2) {
			// massData.center = 0.5f * (m_vertices[0] + m_vertices[1]);
			massData.center.set(m_vertices[0]).addLocal(m_vertices[1]).scaleLocal(0.5f);
			massData.mass = 0.0f;
			massData.I = 0.0f;
			return;
		}

		final Vector center = pool1;
		center.set(0,0);
		float area = 0.0f;
		float I = 0.0f;

		// pRef is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		final Vector pRef = pool2;
		pRef.set(0,0);

		final float k_inv3 = 1.0f / 3.0f;

		final Vector e1 = pool3;
		final Vector e2 = pool4;

		for (int i = 0; i < m_vertexCount; ++i) {
			// Triangle vertices.
			final Vector p1 = pRef;
			final Vector p2 = m_vertices[i];
			final Vector p3 = i + 1 < m_vertexCount ? m_vertices[i + 1] : m_vertices[0];

			e1.set(p2);
			e1.subtractLocal(p1);

			e2.set(p3);
			e2.subtractLocal(p1);

			final float D = MathUtils.cross(e1, e2);

			final float triangleArea = 0.5f * D;
			area += triangleArea;

			// Area weighted centroid
			center.x += triangleArea * k_inv3 * (p1.x + p2.x + p3.x);
			center.y += triangleArea * k_inv3 * (p1.y + p2.y + p3.y);

			final float px = p1.x, py = p1.y;
			final float ex1 = e1.x, ey1 = e1.y;
			final float ex2 = e2.x, ey2 = e2.y;

			final float intx2 = k_inv3 * (0.25f * (ex1 * ex1 + ex2 * ex1 + ex2 * ex2) + (px * ex1 + px * ex2)) + 0.5f
					* px * px;
			final float inty2 = k_inv3 * (0.25f * (ey1 * ey1 + ey2 * ey1 + ey2 * ey2) + (py * ey1 + py * ey2)) + 0.5f
					* py * py;

			I += D * (intx2 + inty2);
		}

		// Total mass
		massData.mass = density * area;

		// Center of mass
		assert (area > Settings.EPSILON);
		center.scaleLocal(1.0f / area);
		massData.center.set(center);

		// Inertia tensor relative to the local origin.
		massData.I = I * density;
	}

	/*
	 * Get the local centroid relative to the parent body. /
	 * public Vec2 getCentroid() {
	 * return m_centroid.clone();
	 * }
	 */

	/** Get the vertices in local coordinates. */
	public Vector[] getVertices() {
		return m_vertices;
	}

	/** Get the edge normal vectors. There is one for each vertex. */
	public Vector[] getNormals() {
		return m_normals;
	}

	/** Get the centroid and apply the supplied transform. */
	public Vector centroid(final Transform xf) {
		return Transform.mul(xf, m_centroid);
	}

	/** Get the centroid and apply the supplied transform. */
	public Vector centroidToOut(final Transform xf, final Vector out) {
		Transform.mulToOut(xf, m_centroid, out);
		return out;
	}

	public void getVertex(final int index, final Vector vertex) {
		vertex.set(getVertex(index));
	}
}
