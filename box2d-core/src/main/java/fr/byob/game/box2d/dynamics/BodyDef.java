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
package fr.byob.game.box2d.dynamics;

import pythagoras.f.Vector;


// updated to rev 100
/**
 * A body definition holds all the data needed to construct a rigid body.
 * You can safely re-use body definitions. Shapes are added to a body
 * after construction.
 * 
 * @author daniel
 */
public class BodyDef {

	/**
	 * The body type: static, kinematic, or dynamic.
	 * Note: if a dynamic body would have zero mass, the mass is set to one.
	 */
	public BodyType type;

	/**
	 * Use this to store application specific body data.
	 */
	public Object userData;

	/**
	 * The world position of the body. Avoid creating bodies at the origin
	 * since this can lead to many overlapping shapes.
	 */
	public Vector position;

	/**
	 * The world angle of the body in radians.
	 */
	public float angle;

	/**
	 * The linear velocity of the body in world co-ordinates.
	 */
	public Vector linearVelocity;

	/**
	 * The angular velocity of the body.
	 */
	public float angularVelocity;

	/**
	 * Linear damping is use to reduce the linear velocity. The damping parameter
	 * can be larger than 1.0f but the damping effect becomes sensitive to the
	 * time step when the damping parameter is large.
	 */
	public float linearDamping;

	/**
	 * Angular damping is use to reduce the angular velocity. The damping parameter
	 * can be larger than 1.0f but the damping effect becomes sensitive to the
	 * time step when the damping parameter is large.
	 */
	public float angularDamping;

	/**
	 * Set this flag to false if this body should never fall asleep. Note that
	 * this increases CPU usage.
	 */
	public boolean allowSleep;

	/**
	 * Is this body initially sleeping?
	 */
	public boolean awake;

	/**
	 * Should this body be prevented from rotating? Useful for characters.
	 */
	public boolean fixedRotation;

	/**
	 * Is this a fast moving body that should be prevented from tunneling through
	 * other moving bodies? Note that all bodies are prevented from tunneling through
	 * kinematic and static bodies. This setting is only considered on dynamic bodies.
	 * 
	 * @warning You should use this flag sparingly since it increases processing time.
	 */
	public boolean bullet;

	/**
	 * Does this body start out active?
	 */
	public boolean active;

	/**
	 * Experimental: scales the inertia tensor.
	 */
	public float inertiaScale;

	/** Scale the gravity applied to this body. **/
	public float gravityScale = 1;

	public BodyDef() {
		userData = null;
		position = new Vector();
		angle = 0f;
		linearVelocity = new Vector();
		angularVelocity = 0f;
		linearDamping = 0f;
		angularDamping = 0f;
		allowSleep = true;
		awake = true;
		fixedRotation = false;
		bullet = false;
		type = BodyType.STATIC;
		active = true;
		inertiaScale = 1.0f;
	}
}
