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
package org.jbox2d.collision;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.collision.shapes.Shape;
import org.jbox2d.common.Transform;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;

// updated to rev 100
/**
 * This is non-static for faster pooling. To get an instance,
 * use the {@link SingletonPool}, don't construct a distance
 * object.
 * 
 * @author Daniel Murphy
 */
public class Distance {

	public static int GJK_CALLS = 0;
	public static int GJK_ITERS = 0;
	public static int GJK_MAX_ITERS = 20;

	/**
	 * GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
	 */
	private class SimplexVertex {
		public final Vector wA = new Vector(); // support point in shapeA
		public final Vector wB = new Vector(); // support point in shapeB
		public final Vector w = new Vector(); // wB - wA
		public float a; // barycentric coordinate for closest point
		public int indexA; // wA index
		public int indexB; // wB index

		public void set(final SimplexVertex sv) {
			wA.set(sv.wA);
			wB.set(sv.wB);
			w.set(sv.w);
			a = sv.a;
			indexA = sv.indexA;
			indexB = sv.indexB;
		}
	}

	/**
	 *  Used to warm start b2Distance.
	 *  Set count to zero on first call.
	 * @author daniel
	 */
	public static class SimplexCache {
		/** length or area */
		public float metric;
		public int count;
		/** vertices on shape A */
		public final int indexA[] = new int[3];
		/** vertices on shape B */
		public final int indexB[] = new int[3];

		public SimplexCache(){
			metric = 0;
			count = 0;
			indexA[0] = Integer.MAX_VALUE;
			indexA[1] = Integer.MAX_VALUE;
			indexA[2] = Integer.MAX_VALUE;
			indexB[0] = Integer.MAX_VALUE;
			indexB[1] = Integer.MAX_VALUE;
			indexB[2] = Integer.MAX_VALUE;
		}

		public void set(final SimplexCache sc){
			System.arraycopy(sc.indexA, 0, indexA, 0, indexA.length);
			System.arraycopy(sc.indexB, 0, indexB, 0, indexB.length);
			metric = sc.metric;
			count = sc.count;
		}
	}

	private class Simplex {
		public final SimplexVertex m_v1 = new SimplexVertex();
		public final SimplexVertex m_v2 = new SimplexVertex();
		public final SimplexVertex m_v3 = new SimplexVertex();
		public final SimplexVertex vertices[] = {m_v1, m_v2, m_v3};
		public int m_count;

		public void readCache(final SimplexCache cache, final DistanceProxy proxyA, final Transform transformA, final DistanceProxy proxyB,
				final Transform transformB) {
			assert (cache.count <= 3);

			// Copy data from cache.
			m_count = cache.count;

			for (int i = 0; i < m_count; ++i) {
				final SimplexVertex v = vertices[i];
				v.indexA = cache.indexA[i];
				v.indexB = cache.indexB[i];
				final Vector wALocal = proxyA.getVertex(v.indexA);
				final Vector wBLocal = proxyB.getVertex(v.indexB);
				Transform.mulToOut(transformA, wALocal, v.wA);
				Transform.mulToOut(transformB, wBLocal, v.wB);
				v.w.set(v.wB).subtractLocal(v.wA);
				v.a = 0.0f;
			}

			// Compute the new simplex metric, if it is substantially different than
			// old metric then flush the simplex.
			if (m_count > 1) {
				final float metric1 = cache.metric;
				final float metric2 = getMetric();
				if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2 || metric2 < Settings.EPSILON) {
					// Reset the simplex.
					m_count = 0;
				}
			}

			// If the cache is empty or invalid ...
			if (m_count == 0) {
				final SimplexVertex v = vertices[0];
				v.indexA = 0;
				v.indexB = 0;
				final Vector wALocal = proxyA.getVertex(0);
				final Vector wBLocal = proxyB.getVertex(0);
				Transform.mulToOut(transformA, wALocal, v.wA);
				Transform.mulToOut(transformB, wBLocal, v.wB);
				v.w.set(v.wB).subtractLocal(v.wA);
				m_count = 1;
			}
		}

		public void writeCache(final SimplexCache cache) {
			cache.metric = getMetric();
			cache.count = m_count;

			for (int i = 0; i < m_count; ++i) {
				cache.indexA[i] = (vertices[i].indexA);
				cache.indexB[i] = (vertices[i].indexB);
			}
		}

		private final Vector e12 = new Vector();

		public final void getSearchDirection(final Vector out) {
			switch (m_count) {
			case 1 :
				out.set(m_v1.w).negateLocal();
				return;
			case 2 :
				e12.set(m_v2.w).subtractLocal(m_v1.w);
				// use out for a temp variable real quick
				out.set(m_v1.w).negateLocal();
				final float sgn = MathUtils.cross(e12, out);

				if (sgn > 0f) {
					// Origin is left of e12.
					MathUtils.crossToOut(1f, e12, out);
					return;
				}
				else {
					// Origin is right of e12.
					MathUtils.crossToOut(e12, 1f, out);
					return;
				}
			default :
				assert (false);
				out.set(0,0);
				return;
			}
		}

		// djm pooled
		private final Vector case2 = new Vector();
		private final Vector case22 = new Vector();

		/**
		 * this returns pooled objects. don't keep or modify them
		 */
		public void getClosestPoint(final Vector out) {
			switch (m_count) {
			case 0 :
				assert (false);
				out.set(0,0);
				return;
			case 1 :
				out.set(m_v1.w);
				return;
			case 2 :
				case22.set(m_v2.w).scaleLocal(m_v2.a);
				case2.set(m_v1.w).scaleLocal(m_v1.a).addLocal(case22);
				out.set(case2);
				return;
			case 3 :
				out.set(0,0);
				return;
			default :
				assert (false);
				out.set(0,0);
				return;
			}
		}

		// djm pooled, and from above
		private final Vector case3 = new Vector();
		private final Vector case33 = new Vector();

		public void getWitnessPoints(final Vector pA, final Vector pB) {
			switch (m_count) {
			case 0 :
				assert (false);
				break;

			case 1 :
				pA.set(m_v1.wA);
				pB.set(m_v1.wB);
				break;

			case 2 :
				case2.set(m_v1.wA).scaleLocal(m_v1.a);
				pA.set(m_v2.wA).scaleLocal(m_v2.a).addLocal(case2);
				// m_v1.a * m_v1.wA + m_v2.a * m_v2.wA;
				// *pB = m_v1.a * m_v1.wB + m_v2.a * m_v2.wB;
				case2.set(m_v1.wB).scaleLocal(m_v1.a);
				pB.set(m_v2.wB).scaleLocal(m_v2.a).addLocal(case2);

				break;

			case 3 :
				pA.set(m_v1.wA).scaleLocal(m_v1.a);
				case3.set(m_v2.wA).scaleLocal(m_v2.a);
				case33.set(m_v3.wA).scaleLocal(m_v3.a);
				pA.addLocal(case3).addLocal(case33);
				pB.set(pA);
				// *pA = m_v1.a * m_v1.wA + m_v2.a * m_v2.wA + m_v3.a * m_v3.wA;
				// *pB = *pA;
				break;

			default :
				assert (false);
				break;
			}
		}

		// djm pooled, from above
		public float getMetric() {
			switch (m_count) {
			case 0 :
				assert (false);
				return 0.0f;

			case 1 :
				return 0.0f;

			case 2 :
				return MathUtils.distance(m_v1.w, m_v2.w);

			case 3 :
				case3.set(m_v2.w).subtractLocal(m_v1.w);
				case33.set(m_v3.w).subtractLocal(m_v1.w);
				// return Vec2.cross(m_v2.w - m_v1.w, m_v3.w - m_v1.w);
				return MathUtils.cross(case3, case33);

			default :
				assert (false);
				return 0.0f;
			}
		}

		// djm pooled from above
		/**
		 * Solve a line segment using barycentric coordinates.
		 */
		public void solve2() {
			// Solve a line segment using barycentric coordinates.
			//
			// p = a1 * w1 + a2 * w2
			// a1 + a2 = 1
			//
			// The vector from the origin to the closest point on the line is
			// perpendicular to the line.
			// e12 = w2 - w1
			// dot(p, e) = 0
			// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
			//
			// 2-by-2 linear system
			// [1 1 ][a1] = [1]
			// [w1.e12 w2.e12][a2] = [0]
			//
			// Define
			// d12_1 = dot(w2, e12)
			// d12_2 = -dot(w1, e12)
			// d12 = d12_1 + d12_2
			//
			// Solution
			// a1 = d12_1 / d12
			// a2 = d12_2 / d12
			final Vector w1 = m_v1.w;
			final Vector w2 = m_v2.w;
			e12.set(w2).subtractLocal(w1);

			// w1 region
			final float d12_2 = -MathUtils.dot(w1, e12);
			if (d12_2 <= 0.0f) {
				// a2 <= 0, so we clamp it to 0
				m_v1.a = 1.0f;
				m_count = 1;
				return;
			}

			// w2 region
			final float d12_1 = MathUtils.dot(w2, e12);
			if (d12_1 <= 0.0f) {
				// a1 <= 0, so we clamp it to 0
				m_v2.a = 1.0f;
				m_count = 1;
				m_v1.set(m_v2);
				return;
			}

			// Must be in e12 region.
			final float inv_d12 = 1.0f / (d12_1 + d12_2);
			m_v1.a = d12_1 * inv_d12;
			m_v2.a = d12_2 * inv_d12;
			m_count = 2;
		}

		// djm pooled, and from above
		private final Vector e13 = new Vector();
		private final Vector e23 = new Vector();
		private final Vector w1 = new Vector();
		private final Vector w2 = new Vector();
		private final Vector w3 = new Vector();

		/**
		 * Solve a line segment using barycentric coordinates.<br/>
		 * Possible regions:<br/>
		 * - points[2]<br/>
		 * - edge points[0]-points[2]<br/>
		 * - edge points[1]-points[2]<br/>
		 * - inside the triangle
		 */
		public void solve3() {
			w1.set(m_v1.w);
			w2.set(m_v2.w);
			w3.set(m_v3.w);

			// Edge12
			// [1 1 ][a1] = [1]
			// [w1.e12 w2.e12][a2] = [0]
			// a3 = 0
			e12.set(w2).subtractLocal(w1);
			final float w1e12 = MathUtils.dot(w1, e12);
			final float w2e12 = MathUtils.dot(w2, e12);
			final float d12_1 = w2e12;
			final float d12_2 = -w1e12;

			// Edge13
			// [1 1 ][a1] = [1]
			// [w1.e13 w3.e13][a3] = [0]
			// a2 = 0
			e13.set(w3).subtractLocal(w1);
			final float w1e13 = MathUtils.dot(w1, e13);
			final float w3e13 = MathUtils.dot(w3, e13);
			final float d13_1 = w3e13;
			final float d13_2 = -w1e13;

			// Edge23
			// [1 1 ][a2] = [1]
			// [w2.e23 w3.e23][a3] = [0]
			// a1 = 0
			e23.set(w3).subtractLocal(w2);
			final float w2e23 = MathUtils.dot(w2, e23);
			final float w3e23 = MathUtils.dot(w3, e23);
			final float d23_1 = w3e23;
			final float d23_2 = -w2e23;

			// Triangle123
			final float n123 = MathUtils.cross(e12, e13);

			final float d123_1 = n123 * MathUtils.cross(w2, w3);
			final float d123_2 = n123 * MathUtils.cross(w3, w1);
			final float d123_3 = n123 * MathUtils.cross(w1, w2);

			// w1 region
			if (d12_2 <= 0.0f && d13_2 <= 0.0f) {
				m_v1.a = 1.0f;
				m_count = 1;
				return;
			}

			// e12
			if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f) {
				final float inv_d12 = 1.0f / (d12_1 + d12_2);
				m_v1.a = d12_1 * inv_d12;
				m_v2.a = d12_2 * inv_d12;
				m_count = 2;
				return;
			}

			// e13
			if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f) {
				final float inv_d13 = 1.0f / (d13_1 + d13_2);
				m_v1.a = d13_1 * inv_d13;
				m_v3.a = d13_2 * inv_d13;
				m_count = 2;
				m_v2.set(m_v3);
				return;
			}

			// w2 region
			if (d12_1 <= 0.0f && d23_2 <= 0.0f) {
				m_v2.a = 1.0f;
				m_count = 1;
				m_v1.set(m_v2);
				return;
			}

			// w3 region
			if (d13_1 <= 0.0f && d23_1 <= 0.0f) {
				m_v3.a = 1.0f;
				m_count = 1;
				m_v1.set(m_v3);
				return;
			}

			// e23
			if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f) {
				final float inv_d23 = 1.0f / (d23_1 + d23_2);
				m_v2.a = d23_1 * inv_d23;
				m_v3.a = d23_2 * inv_d23;
				m_count = 2;
				m_v1.set(m_v3);
				return;
			}

			// Must be in triangle123
			final float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
			m_v1.a = d123_1 * inv_d123;
			m_v2.a = d123_2 * inv_d123;
			m_v3.a = d123_3 * inv_d123;
			m_count = 3;
		}
	}

	/**
	 * A distance proxy is used by the GJK algorithm.
	 * It encapsulates any shape.
	 *
	 * @author daniel
	 */
	public static class DistanceProxy {
		public final Vector[] m_vertices;
		public int m_count;
		public float m_radius;

		public DistanceProxy(){
			m_vertices = new Vector[Settings.maxPolygonVertices];
			for(int i=0; i<m_vertices.length; i++){
				m_vertices[i] = new Vector();
			}
			m_count = 0;
			m_radius = 0f;
		}

		/**
		 * Initialize the proxy using the given shape. The shape
		 * must remain in scope while the proxy is in use.
		 */
		public final void set(final Shape shape){
			switch(shape.getType()){
			case CIRCLE:
				final CircleShape circle = (CircleShape) shape;
				m_vertices[0].set(circle.m_p);
				m_count = 1;
				m_radius = circle.m_radius;

				break;
			case POLYGON:
				final PolygonShape poly = (PolygonShape) shape;
				m_count = poly.m_vertexCount;
				m_radius = poly.m_radius;
				for(int i=0; i<m_count; i++){
					m_vertices[i].set(poly.m_vertices[i]);
				}
				break;
			default:
				assert(false);
			}
		}

		/**
		 * Get the supporting vertex index in the given direction.
		 * @param d
		 * @return
		 */
		public final int getSupport(final Vector d){
			int bestIndex = 0;
			float bestValue = MathUtils.dot(m_vertices[0], d);
			for( int i=1; i<m_count; i++){
				final float value = MathUtils.dot(m_vertices[i], d);
				if(value > bestValue){
					bestIndex = i;
					bestValue = value;
				}
			}

			return bestIndex;
		}

		/**
		 * Get the supporting vertex in the given direction.
		 * @param d
		 * @return
		 */
		public final Vector getSupportVertex(final Vector d){
			int bestIndex = 0;
			float bestValue = MathUtils.dot(m_vertices[0], d);
			for( int i=1; i<m_count; i++){
				final float value = MathUtils.dot(m_vertices[i], d);
				if(value > bestValue){
					bestIndex = i;
					bestValue = value;
				}
			}

			return m_vertices[bestIndex];
		}

		/**
		 * Get the vertex count.
		 * @return
		 */
		public final int getVertexCount(){
			return m_count;
		}

		/**
		 * Get a vertex by index. Used by b2Distance.
		 * @param index
		 * @return
		 */
		public final Vector getVertex(final int index){
			assert(0 <= index && index < m_count);
			return m_vertices[index];
		}
	}

	private Simplex simplex = new Simplex();
	private int[] saveA = new int[3];
	private int[] saveB = new int[3];
	private Vector closestPoint = new Vector();
	private Vector d = new Vector();
	private Vector temp = new Vector();
	private Vector normal = new Vector();

	/**
	 * Compute the closest points between two shapes. Supports any combination of:
	 * CircleShape and PolygonShape. The simplex cache is input/output.
	 * On the first call set SimplexCache.count to zero.
	 * 
	 * @param output
	 * @param cache
	 * @param input
	 */
	public final void distance(final DistanceOutput output, final SimplexCache cache, final DistanceInput input) {
		GJK_CALLS++;

		final DistanceProxy proxyA = input.proxyA;
		final DistanceProxy proxyB = input.proxyB;

		final Transform transformA = input.transformA;
		final Transform transformB = input.transformB;

		// Initialize the simplex.
		simplex.readCache(cache, proxyA, transformA, proxyB, transformB);

		// Get simplex vertices as an array.
		final SimplexVertex[] vertices = simplex.vertices;

		// These store the vertices of the last simplex so that we
		// can check for duplicates and prevent cycling.
		// (pooled above)
		int saveCount = 0;

		simplex.getClosestPoint(closestPoint);
		float distanceSqr1 = closestPoint.lengthSq();
		float distanceSqr2 = distanceSqr1;

		// Main iteration loop
		int iter = 0;
		while (iter < GJK_MAX_ITERS) {

			// Copy simplex so we can identify duplicates.
			saveCount = simplex.m_count;
			for (int i = 0; i < saveCount; i++) {
				saveA[i] = vertices[i].indexA;
				saveB[i] = vertices[i].indexB;
			}

			switch (simplex.m_count) {
			case 1 :
				break;
			case 2 :
				simplex.solve2();
				break;
			case 3 :
				simplex.solve3();
				break;
			default :
				assert (false);
			}

			// If we have 3 points, then the origin is in the corresponding triangle.
			if (simplex.m_count == 3) {
				break;
			}

			// Compute closest point.
			simplex.getClosestPoint(closestPoint);
			distanceSqr2 = closestPoint.lengthSq();

			// ensure progress
			if (distanceSqr2 >= distanceSqr1) {
				// break;
			}
			distanceSqr1 = distanceSqr2;

			// get search direction;
			simplex.getSearchDirection(d);

			// Ensure the search direction is numerically fit.
			if (d.lengthSq() < Settings.EPSILON * Settings.EPSILON) {
				// The origin is probably contained by a line segment
				// or triangle. Thus the shapes are overlapped.

				// We can't return zero here even though there may be overlap.
				// In case the simplex is a point, segment, or triangle it is difficult
				// to determine if the origin is contained in the CSO or very close to it.
				break;
			}
			/*
			 * b2SimplexVertex* vertex = vertices + simplex.m_count;
			 * vertex.indexA = proxyA.GetSupport(b2MulT(transformA.R, -d));
			 * vertex.wA = b2Mul(transformA, proxyA.GetVertex(vertex.indexA));
			 * b2Vec2 wBLocal;
			 * vertex.indexB = proxyB.GetSupport(b2MulT(transformB.R, d));
			 * vertex.wB = b2Mul(transformB, proxyB.GetVertex(vertex.indexB));
			 * vertex.w = vertex.wB - vertex.wA;
			 */

			// Compute a tentative new simplex vertex using support points.
			final SimplexVertex vertex = vertices[simplex.m_count];

			Mat22.mulTransToOut(transformA.R, d.negateLocal(), temp);
			vertex.indexA = proxyA.getSupport(temp);
			Transform.mulToOut(transformA, proxyA.getVertex(vertex.indexA), vertex.wA);
			// Vec2 wBLocal;
			Mat22.mulTransToOut(transformB.R, d.negateLocal(), temp);
			vertex.indexB = proxyB.getSupport(temp);
			Transform.mulToOut(transformB, proxyB.getVertex(vertex.indexB), vertex.wB);
			vertex.w.set(vertex.wB).subtractLocal(vertex.wA);

			// Iteration count is equated to the number of support point calls.
			++iter;
			++GJK_ITERS;

			// Check for duplicate support points. This is the main termination criteria.
			boolean duplicate = false;
			for (int i = 0; i < saveCount; ++i) {
				if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i]) {
					duplicate = true;
					break;
				}
			}

			// If we found a duplicate support point we must exit to avoid cycling.
			if (duplicate) {
				break;
			}

			// New vertex is ok and needed.
			++simplex.m_count;
		}

		GJK_MAX_ITERS = MathUtils.max(GJK_MAX_ITERS, iter);

		// Prepare output.
		simplex.getWitnessPoints(output.pointA, output.pointB);
		output.distance = MathUtils.distance(output.pointA, output.pointB);
		output.iterations = iter;

		// Cache the simplex.
		simplex.writeCache(cache);

		// Apply radii if requested.
		if (input.useRadii) {
			final float rA = proxyA.m_radius;
			final float rB = proxyB.m_radius;

			if (output.distance > rA + rB && output.distance > Settings.EPSILON) {
				// Shapes are still no overlapped.
				// Move the witness points to the outer surface.
				output.distance -= rA + rB;
				normal.set(output.pointB).subtractLocal(output.pointA);
				MathUtils.normalize(normal);
				temp.set(normal).scaleLocal(rA);
				output.pointA.addLocal(temp);
				temp.set(normal).scaleLocal(rB);
				output.pointB.subtractLocal(temp);
			}
			else {
				// Shapes are overlapped when radii are considered.
				// Move the witness points to the middle.
				// b2Vec2 p = 0.5f * (output.pointA + output.pointB);
				output.pointA.addLocal(output.pointB).scaleLocal(.5f);
				output.pointB.set(output.pointA);
				output.distance = 0.0f;
			}
		}
	}
}
