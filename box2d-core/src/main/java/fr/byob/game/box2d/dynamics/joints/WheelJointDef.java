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
import fr.byob.game.box2d.dynamics.Body;

/** Wheel joint definition. This requires defining a line of motion using an axis and an anchor point. The definition uses local
 * anchor points and a local axis so that the initial configuration can violate the constraint slightly. The joint translation is
 * zero when the local anchor points coincide in world space. Using local anchors and a local axis helps when saving and loading a
 * game. */
public class WheelJointDef extends JointDef {
	public WheelJointDef () {
		type = JointType.WHEEL;
	}

	public void initialize (final Body bodyA, final Body bodyB, final Vector anchor, final Vector axis) {
		this.bodyA = bodyA;
		this.bodyB = bodyB;
		localAnchorA.set(bodyA.getLocalPoint(anchor));
		localAnchorB.set(bodyB.getLocalPoint(anchor));
		localAxisA.set(bodyA.getLocalVector(axis));
	}

	/** The local anchor point relative to body1's origin. **/
	public final Vector localAnchorA = new Vector();

	/** The local anchor point relative to body2's origin. **/
	public final Vector localAnchorB = new Vector();

	/** The local translation axis in body1. **/
	public final Vector localAxisA = new Vector(1, 0);

	/** Enable/disable the joint motor. **/
	public boolean enableMotor = false;

	/** The maximum motor torque, usually in N-m. */
	public float maxMotorTorque = 0;

	/** The desired motor speed in radians per second. */
	public float motorSpeed = 0;

	/** Suspension frequency, zero indicates no suspension */
	public float frequencyHz = 2;

	/** Suspension damping ratio, one indicates critical damping */
	public float dampingRatio = 0.7f;
}
