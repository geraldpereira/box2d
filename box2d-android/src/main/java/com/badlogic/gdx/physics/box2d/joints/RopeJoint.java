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

package com.badlogic.gdx.physics.box2d.joints;

import com.badlogic.gdx.physics.box2d.Joint;
import com.badlogic.gdx.physics.box2d.World;

/** A rope joint enforces a maximum distance between two points on two bodies. It has no other effect. Warning: if you attempt to
 * change the maximum length during the simulation you will get some non-physical behavior. A model that would allow you to
 * dynamically modify the length would have some sponginess, so I chose not to implement it that way. See b2DistanceJoint if you
 * want to dynamically control length. */
public class RopeJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.RopeJoint {
	// @off
	/*JNI
#include <Box2D/Box2D.h>
	 */

	public RopeJoint (final World world, final long addr) {
		super(world, addr);
	}

	/** Get the maximum length of the rope. */
	public float getMaxLength () {
		return jniGetMaxLength(addr);
	}

	private native float jniGetMaxLength (long addr); /*
		b2RopeJoint* rope = (b2RopeJoint*)addr;
		return rope->GetMaxLength();
	 */
}