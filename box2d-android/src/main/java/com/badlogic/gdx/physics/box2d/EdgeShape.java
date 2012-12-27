/*******************************************************************************
 * Copyright 2011 See AUTHORS file.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package com.badlogic.gdx.physics.box2d;

import pythagoras.f.Vector;
import fr.byob.game.box2d.collision.shapes.ShapeType;


/** A line segment (edge) shape. These can be connected in chains or loops to other edge shapes. The connectivity information is
 * used to ensure correct contact normals. */
public class EdgeShape extends Shape implements fr.byob.game.box2d.collision.shapes.EdgeShape {
	// @off
	/*JNI
#include <Box2D/Box2D.h>
	 */

	public EdgeShape () {
		addr = newEdgeShape();
	}

	private native long newEdgeShape (); /*
		return (jlong)(new b2EdgeShape());
	 */

	EdgeShape (final long addr) {
		this.addr = addr;
	}

	/** Set this as an isolated edge. */
	public void set (final Vector v1, final Vector v2) {
		set(v1.x, v1.y, v2.x, v2.y);
	}

	/** Set this as an isolated edge. */
	public void set (final float v1X, final float v1Y, final float v2X, final float v2Y) {
		jniSet(addr, v1X, v1Y, v2X, v2Y);
	}

	private native void jniSet (long addr, float v1x, float v1y, float v2x, float v2y); /*
		b2EdgeShape* edge = (b2EdgeShape*)addr;
		edge->Set(b2Vec2(v1x, v1y), b2Vec2(v2x, v2y));
	 */

	static final float[] vertex = new float[2];

	public void getVertex1 (final Vector vec) {
		jniGetVertex1(addr, vertex);
		vec.x = vertex[0];
		vec.y = vertex[1];
	}

	private native void jniGetVertex1 (long addr, float[] vertex); /*
		b2EdgeShape* edge = (b2EdgeShape*)addr;
		vertex[0] = edge->m_vertex1.x;
		vertex[1] = edge->m_vertex1.y;
	 */

	public void getVertex2 (final Vector vec) {
		jniGetVertex2(addr, vertex);
		vec.x = vertex[0];
		vec.y = vertex[1];
	}

	private native void jniGetVertex2 (long addr, float[] vertex); /*
		b2EdgeShape* edge = (b2EdgeShape*)addr;
		vertex[0] = edge->m_vertex2.x;
		vertex[1] = edge->m_vertex2.y;
	 */

	// /// @see b2Shape::TestPoint
	// bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;
	//
	// /// Implement b2Shape.
	// bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
	// const b2Transform& transform, int32 childIndex) const;
	//
	// /// @see b2Shape::ComputeAABB
	// void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const;
	//
	// /// @see b2Shape::ComputeMass
	// void ComputeMass(b2MassData* massData, float32 density) const;
	//
	// /// These are the edge vertices
	// b2Vec2 m_vertex1, m_vertex2;
	//
	// /// Optional adjacent vertices. These are used for smooth collision.
	// b2Vec2 m_vertex0, m_vertex3;
	// bool m_hasVertex0, m_hasVertex3;

	public ShapeType getType() {
		return ShapeType.EDGE;
	}

	public void set(final Vector[] vertices, final int count) {
		set(vertices[1].x, vertices[1].y, vertices[2].x, vertices[2].y);
	}

	public int getVertexCount() {
		return 2;
	}

	public void getVertex(final int index, final Vector vertex) {
		if (index == 0) {
			getVertex1(vertex);
		} else if (index == 1) {
			getVertex2(vertex);
		}
	}

	public void setAsEdge(final Vector v1, final Vector v2) {
		set(v1, v2);
	}

}