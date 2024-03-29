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
/**
 * Created at 7:27:31 AM Jan 21, 2011
 */
package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;
import fr.byob.game.box2d.dynamics.Body;

/**
 * @author Daniel Murphy
 */
public class LineJointDef extends JointDef {

	/**
	 * The local anchor point relative to body1's origin.
	 */
	public final Vector localAnchorA = new Vector();

	/**
	 * The local anchor point relative to body2's origin.
	 */
	public final Vector localAnchorB = new Vector();

	/**
	 * The local translation axis in body1.
	 */
	public final Vector localAxisA = new Vector();

	/**
	 * Enable/disable the joint limit.
	 */
	public boolean enableLimit;

	/**
	 * The lower translation limit, usually in meters.
	 */
	public float lowerTranslation;

	/**
	 * The upper translation limit, usually in meters.
	 */
	public float upperTranslation;

	/**
	 * Enable/disable the joint motor.
	 */
	public boolean enableMotor;

	/**
	 * The maximum motor torque, usually in N-m.
	 */
	public float maxMotorForce;

	/**
	 * The desired motor speed in radians per second.
	 */
	public float motorSpeed;

	public LineJointDef() {
		type = JointType.LINE;
		localAxisA.set(1, 0);
		enableLimit = false;
		lowerTranslation = 0;
		upperTranslation = 0;
		enableMotor = false;
		maxMotorForce = 0f;
		motorSpeed = 0f;
	}

	public void initialize(final Body b1, final Body b2, final Vector anchor, final Vector axis) {
		bodyA = b1;
		bodyB = b2;
		b1.getLocalPointToOut(anchor, localAnchorA);
		b2.getLocalPointToOut(anchor, localAnchorB);
		bodyA.getLocalVectorToOut(axis, localAxisA);
	}
}
