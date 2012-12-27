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
 * Created at 12:11:41 PM Jan 23, 2011
 */
package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.Body;

/**
 * Pulley joint definition. This requires two ground anchors,
 * two dynamic body anchor points, max lengths for each side,
 * and a pulley ratio.
 * 
 * @author Daniel Murphy
 */
public class PulleyJointDef extends JointDef {

	/**
	 * The first ground anchor in world coordinates. This point never moves.
	 */
	public Vector groundAnchorA;

	/**
	 * The second ground anchor in world coordinates. This point never moves.
	 */
	public Vector groundAnchorB;

	/**
	 * The local anchor point relative to bodyA's origin.
	 */
	public Vector localAnchorA;

	/**
	 * The local anchor point relative to bodyB's origin.
	 */
	public Vector localAnchorB;

	/**
	 * The a reference length for the segment attached to bodyA.
	 */
	public float lengthA;

	/**
	 * The maximum length of the segment attached to bodyA.
	 */
	public float maxLengthA;

	/**
	 * The a reference length for the segment attached to bodyB.
	 */
	public float lengthB;

	/**
	 * The maximum length of the segment attached to bodyB.
	 */
	public float maxLengthB;

	/**
	 * The pulley ratio, used to simulate a block-and-tackle.
	 */
	public float ratio;

	public PulleyJointDef() {
		type = JointType.PULLEY;
		groundAnchorA = new Vector(-1.0f, 1.0f);
		groundAnchorB = new Vector(1.0f, 1.0f);
		localAnchorA = new Vector(-1.0f, 0.0f);
		localAnchorB = new Vector(1.0f, 0.0f);
		lengthA = 0.0f;
		maxLengthA = 0.0f;
		lengthB = 0.0f;
		maxLengthB = 0.0f;
		ratio = 1.0f;
		collideConnected = true;
	}

	/**
	 * Initialize the bodies, anchors, lengths, max lengths, and ratio using the world
	 * anchors.
	 */
	public void initialize(final Body b1, final Body b2, final Vector ga1, final Vector ga2, final Vector anchor1, final Vector anchor2, final float r) {
		bodyA = b1;
		bodyB = b2;
		groundAnchorA = ga1;
		groundAnchorB = ga2;
		localAnchorA = bodyA.getLocalPoint(anchor1);
		localAnchorB = bodyB.getLocalPoint(anchor2);
		lengthA = MathUtils.tmp(anchor1).subtractLocal(ga1).length();
		lengthB = MathUtils.tmp(anchor2).subtractLocal(ga2).length();
		ratio = r;
		assert (ratio > Settings.EPSILON);
		final float C = lengthA + ratio * lengthB;
		maxLengthA = C - ratio * PulleyJoint.MIN_PULLEY_LENGTH;
		maxLengthB = (C - PulleyJoint.MIN_PULLEY_LENGTH) / ratio;
	}
}
