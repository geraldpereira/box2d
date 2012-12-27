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


public class ChainShape extends Shape {
	// @off
	/*JNI
#include <Box2D/Box2D.h>
	 */

	public ChainShape () {
		addr = newChainShape();
	}

	private native long newChainShape (); /*
		return (jlong)(new b2ChainShape());
	 */

	ChainShape (final long addr) {
		this.addr = addr;
	}

	public ShapeType getType() {
		return ShapeType.CHAIN;
	}

	/** Create a loop. This automatically adjusts connectivity.
	 * @param vertices an array of vertices, these are copied */
	public void createLoop (final Vector[] vertices) {
		final float[] verts = new float[vertices.length * 2];
		for (int i = 0, j = 0; i < vertices.length * 2; i += 2, j++) {
			verts[i] = vertices[j].x;
			verts[i + 1] = vertices[j].y;
		}
		jniCreateLoop(addr, verts, verts.length / 2);
	}

	private native void jniCreateLoop (long addr, float[] verts, int numVertices); /*
		b2ChainShape* chain = (b2ChainShape*)addr;
		b2Vec2* verticesOut = new b2Vec2[numVertices];
		for( int i = 0; i < numVertices; i++ )
			verticesOut[i] = b2Vec2(verts[i<<1], verts[(i<<1)+1]);
		chain->CreateLoop( verticesOut, numVertices );
		delete verticesOut;
	 */

	/** Create a chain with isolated end vertices.
	 * @param vertices an array of vertices, these are copied */
	public void createChain (final Vector[] vertices) {
		final float[] verts = new float[vertices.length * 2];
		for (int i = 0, j = 0; i < vertices.length * 2; i += 2, j++) {
			verts[i] = vertices[j].x;
			verts[i + 1] = vertices[j].y;
		}
		jniCreateChain(addr, verts, verts.length / 2);
	}

	private native void jniCreateChain (long addr, float[] verts, int numVertices); /*
		b2ChainShape* chain = (b2ChainShape*)addr;
		b2Vec2* verticesOut = new b2Vec2[numVertices];
		for( int i = 0; i < numVertices; i++ )
			verticesOut[i] = b2Vec2(verts[i<<1], verts[(i<<1)+1]);
		chain->CreateChain( verticesOut, numVertices );
		delete verticesOut;
	 */

	/** Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops. */
	public void setPrevVertex (final Vector prevVertex) {
		setPrevVertex(prevVertex.x, prevVertex.y);
	}

	/** Establish connectivity to a vertex that precedes the first vertex. Don't call this for loops. */
	public void setPrevVertex (final float prevVertexX, final float prevVertexY) {
		jniSetPrevVertex(addr, prevVertexX, prevVertexY);
	}

	private native void jniSetPrevVertex (long addr, float x, float y); /*
		b2ChainShape* chain = (b2ChainShape*)addr;
		chain->SetPrevVertex(b2Vec2(x, y));
	 */

	/** Establish connectivity to a vertex that follows the last vertex. Don't call this for loops. */
	public void setNextVertex (final Vector nextVertex) {
		setNextVertex(nextVertex.x, nextVertex.y);
	}

	/** Establish connectivity to a vertex that follows the last vertex. Don't call this for loops. */
	public void setNextVertex (final float nextVertexX, final float nextVertexY) {
		jniSetNextVertex(addr, nextVertexX, nextVertexY);
	}

	private native void jniSetNextVertex (long addr, float x, float y); /*
		b2ChainShape* chain = (b2ChainShape*)addr;
		chain->SetNextVertex(b2Vec2(x, y));
	 */

	/** @return the number of vertices */
	public int getVertexCount () {
		return jniGetVertexCount(addr);
	}

	private native int jniGetVertexCount (long addr); /*
		b2ChainShape* chain = (b2ChainShape*)addr;
		return chain->GetVertexCount();
	 */

	private static float[] verts = new float[2];

	/** Returns the vertex at the given position.
	 * @param index the index of the vertex 0 <= index < getVertexCount( )
	 * @param vertex vertex */
	public void getVertex (final int index, final Vector vertex) {
		jniGetVertex(addr, index, verts);
		vertex.x = verts[0];
		vertex.y = verts[1];
	}

	private native void jniGetVertex (long addr, int index, float[] verts); /*
		b2ChainShape* chain = (b2ChainShape*)addr;
		const b2Vec2 v = chain->GetVertex( index );
		verts[0] = v.x;
		verts[1] = v.y;
	 */

	// /// Implement b2Shape. Vertices are cloned using b2Alloc.
	// b2Shape* Clone(b2BlockAllocator* allocator) const;
	//
	// /// @see b2Shape::GetChildCount
	// int32 GetChildCount() const;
	//
	// /// Get a child edge.
	// void GetChildEdge(b2EdgeShape* edge, int32 index) const;
	//
	// /// This always return false.
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
	// /// Chains have zero mass.
	// /// @see b2Shape::ComputeMass
	// void ComputeMass(b2MassData* massData, float32 density) const;
	//
}