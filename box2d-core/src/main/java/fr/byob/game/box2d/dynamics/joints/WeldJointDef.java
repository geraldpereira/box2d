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
package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;
import fr.byob.game.box2d.dynamics.Body;

/**
 * Created at 3:38:52 AM Jan 15, 2011
 */

/**
 * @author Daniel Murphy
 */
public class WeldJointDef extends JointDef {
	/**
	 * The local anchor point relative to body1's origin.
	 */
	public final Vector localAnchorA;

	/**
	 * The local anchor point relative to body2's origin.
	 */
	public final Vector  localAnchorB;

	/**
	 * The body2 angle minus body1 angle in the reference state (radians).
	 */
	public float referenceAngle;

	public WeldJointDef(){
		type = JointType.WELD;
		localAnchorA = new Vector();
		localAnchorB = new Vector();
		referenceAngle = 0.0f;
	}

	/**
	 * Initialize the bodies, anchors, and reference angle using a world
	 * anchor point.
	 * @param bA
	 * @param bB
	 * @param anchor
	 */
	public void initialize(final Body bA, final Body bB, final Vector anchor){
		bodyA = bA;
		bodyB = bB;
		bodyA.getLocalPointToOut(anchor, localAnchorA);
		bodyB.getLocalPointToOut(anchor, localAnchorB);
		referenceAngle = bodyB.getAngle() - bodyA.getAngle();
	}
}
