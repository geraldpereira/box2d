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
import fr.byob.game.box2d.collision.ManifoldType;


public class Manifold implements fr.byob.game.box2d.collision.Manifold {
	// @off
	/*JNI
#include <Box2D/Box2D.h>
	 */

	long addr;
	final ManifoldPoint[] points = new ManifoldPoint[] {new ManifoldPoint(), new ManifoldPoint()};
	final Vector localNormal = new Vector();
	final Vector localPoint = new Vector();

	final int[] tmpInt = new int[2];
	final float[] tmpFloat = new float[4];

	protected Manifold (final long addr) {
		this.addr = addr;
	}

	public ManifoldType getType () {
		final int type = jniGetType(addr);
		if (type == 0) {
			return ManifoldType.CIRLCE;
		}
		if (type == 1) {
			return ManifoldType.FACE_A;
		}
		if (type == 2) {
			return ManifoldType.FACE_B;
		}
		return ManifoldType.CIRLCE;
	}

	private native int jniGetType (long addr); /*
		b2Manifold* manifold = (b2Manifold*)addr;
		return manifold->type;
	 */

	public int getPointCount () {
		return jniGetPointCount(addr);
	}

	private native int jniGetPointCount (long addr); /*
	  	b2Manifold* manifold = (b2Manifold*)addr;
		return manifold->pointCount;
	 */

	public Vector getLocalNormal () {
		jniGetLocalNormal(addr, tmpFloat);
		localNormal.set(tmpFloat[0], tmpFloat[1]);
		return localNormal;
	}

	private native void jniGetLocalNormal (long addr, float[] values); /*
		b2Manifold* manifold = (b2Manifold*)addr;
		values[0] = manifold->localNormal.x;
		values[1] = manifold->localNormal.y;
	 */

	public Vector getLocalPoint () {
		jniGetLocalPoint(addr, tmpFloat);
		localPoint.set(tmpFloat[0], tmpFloat[1]);
		return localPoint;
	}

	private native void jniGetLocalPoint (long addr, float[] values); /*
		b2Manifold* manifold = (b2Manifold*)addr;
		values[0] = manifold->localPoint.x;
		values[1] = manifold->localPoint.y;
	 */

	public ManifoldPoint[] getPoints () {
		final int count = jniGetPointCount(addr);

		for (int i = 0; i < count; i++) {
			final int contactID = jniGetPoint(addr, tmpFloat, i);
			final ManifoldPoint point = points[i];
			point.contactID = contactID;
			point.localPoint.set(tmpFloat[0], tmpFloat[1]);
			point.normalImpulse = tmpFloat[2];
			point.tangentImpulse = tmpFloat[3];
		}

		return points;
	}

	private native int jniGetPoint (long addr, float[] values, int idx); /*
		b2Manifold* manifold = (b2Manifold*)addr;

		values[0] = manifold->points[idx].localPoint.x;
		values[1] = manifold->points[idx].localPoint.y;
		values[2] = manifold->points[idx].normalImpulse;
		values[3] = manifold->points[idx].tangentImpulse;

		return (jint)manifold->points[idx].id.key;
	 */

	public class ManifoldPoint extends fr.byob.game.box2d.collision.ManifoldPoint {
		public int contactID = 0;

		@Override
		public String toString() {
			return "id: " + contactID + ", " + localPoint + ", " + normalImpulse + ", " + tangentImpulse;
		}
	}
}
