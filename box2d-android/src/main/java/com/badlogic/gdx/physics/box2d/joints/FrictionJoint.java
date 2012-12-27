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

/** Friction joint. This is used for top-down friction. It provides 2D translational friction and angular friction. */
public class FrictionJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.FrictionJoint {
	// @off
	/*JNI
#include <Box2D/Box2D.h>
	 */

	public FrictionJoint (final World world, final long addr) {
		super(world, addr);
	}

	/** Set the maximum friction force in N. */
	public void setMaxForce (final float force) {
		jniSetMaxForce(addr, force);
	}

	private native void jniSetMaxForce (long addr, float force); /*
		b2FrictionJoint* joint = (b2FrictionJoint*)addr;
		joint->SetMaxForce( force );
	 */

	/** Get the maximum friction force in N. */
	public float getMaxForce () {
		return jniGetMaxForce(addr);
	}

	private native float jniGetMaxForce (long addr); /*
		b2FrictionJoint* joint = (b2FrictionJoint*)addr;
		return joint->GetMaxForce();
	 */

	/** Set the maximum friction torque in N*m. */
	public void setMaxTorque (final float torque) {
		jniSetMaxTorque(addr, torque);
	}

	private native void jniSetMaxTorque (long addr, float torque); /*
		b2FrictionJoint* joint = (b2FrictionJoint*)addr;
		joint->SetMaxTorque( torque );
	 */

	/** Get the maximum friction torque in N*m. */
	public float getMaxTorque () {
		return jniGetMaxTorque(addr);
	}

	private native float jniGetMaxTorque (long addr); /*
		b2FrictionJoint* joint = (b2FrictionJoint*)addr;
		return joint->GetMaxTorque();
	 */
}
