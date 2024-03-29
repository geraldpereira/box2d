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

package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;


/** Rope joint definition. This requires two body anchor points and a maximum lengths. Note: by default the connected objects will
 * not collide. see collideConnected in b2JointDef.
 * @author mzechner */
public class RopeJointDef extends JointDef {
	public RopeJointDef () {
		type = JointType.ROPE;
	}

	/** The local anchor point relative to bodyA's origin. **/
	public final Vector localAnchorA = new Vector(-1, 0);

	/** The local anchor point relative to bodyB's origin. **/
	public final Vector localAnchorB = new Vector(1, 0);

	/** The maximum length of the rope. Warning: this must be larger than b2_linearSlop or the joint will have no effect. */
	public float maxLength = 0;
}
