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
package org.jbox2d.dynamics.joints;

import java.util.Iterator;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import fr.byob.game.box2d.B2DIterator;
import fr.byob.game.box2d.B2DLinkedList;
import fr.byob.game.box2d.dynamics.joints.ConstantVolumeJointDef;
import fr.byob.game.box2d.dynamics.joints.DistanceJointDef;
import fr.byob.game.box2d.dynamics.joints.FrictionJointDef;
import fr.byob.game.box2d.dynamics.joints.GearJointDef;
import fr.byob.game.box2d.dynamics.joints.JointDef;
import fr.byob.game.box2d.dynamics.joints.JointType;
import fr.byob.game.box2d.dynamics.joints.LineJointDef;
import fr.byob.game.box2d.dynamics.joints.MouseJointDef;
import fr.byob.game.box2d.dynamics.joints.PrismaticJointDef;
import fr.byob.game.box2d.dynamics.joints.PulleyJointDef;
import fr.byob.game.box2d.dynamics.joints.RevoluteJointDef;
import fr.byob.game.box2d.dynamics.joints.WeldJointDef;

// updated to rev 100
/**
 * The base joint class. Joints are used to raint two bodies together in
 * various fashions. Some joints also feature limits and motors.
 * 
 * @author Daniel Murphy
 */
public abstract class Joint implements B2DLinkedList<Joint>, Iterable<Joint>, fr.byob.game.box2d.dynamics.joints.Joint {

	public static Joint create(final World argWorld, final JointDef def) {
		//Joint joint = null;
		switch(def.type){
		case MOUSE:
			return new MouseJoint(argWorld.getPool(), (MouseJointDef) def);
		case DISTANCE:
			return new DistanceJoint(argWorld.getPool(), (DistanceJointDef) def);
		case PRISMATIC:
			return new PrismaticJoint(argWorld.getPool(), (PrismaticJointDef) def);
		case REVOLUTE:
			return new RevoluteJoint(argWorld.getPool(), (RevoluteJointDef) def);
		case WELD:
			return new WeldJoint(argWorld.getPool(), (WeldJointDef) def);
		case FRICTION:
			return new FrictionJoint(argWorld.getPool(), (FrictionJointDef) def);
		case LINE:
			return new LineJoint(argWorld.getPool(), (LineJointDef) def);
		case GEAR:
			return new GearJoint(argWorld.getPool(), (GearJointDef) def);
		case PULLEY:
			return new PulleyJoint(argWorld.getPool(), (PulleyJointDef) def);
		case CONSTANT_VOLUME:
			return new ConstantVolumeJoint(argWorld, (ConstantVolumeJointDef) def);
		}
		return null;
	}

	public static void destroy(final Joint joint) {
		joint.destructor();
	}

	public JointType m_type;
	public Joint m_prev;
	public Joint m_next;
	public JointEdge m_edgeA;
	public JointEdge m_edgeB;
	public Body m_bodyA;
	public Body m_bodyB;

	public boolean m_islandFlag;
	public boolean m_collideConnected;

	public Object m_userData;

	protected IWorldPool pool;

	// Cache here per time step to reduce cache misses.
	public final Vector m_localCenterA, m_localCenterB;
	float m_invMassA, m_invIA;
	float m_invMassB, m_invIB;

	protected Joint(final IWorldPool argWorldPool, final JointDef def) {
		assert (def.bodyA != def.bodyB);

		pool = argWorldPool;
		m_type = def.type;
		m_prev = null;
		m_next = null;
		m_bodyA = (Body) def.bodyA;
		m_bodyB = (Body) def.bodyB;
		m_collideConnected = def.collideConnected;
		m_islandFlag = false;
		m_userData = def.userData;

		m_edgeA = new JointEdge();
		m_edgeA.joint = null;
		m_edgeA.other = null;
		m_edgeA.prev = null;
		m_edgeA.next = null;

		m_edgeB = new JointEdge();
		m_edgeB.joint = null;
		m_edgeB.other = null;
		m_edgeB.prev = null;
		m_edgeB.next = null;

		m_localCenterA = new Vector();
		m_localCenterB = new Vector();
	}

	/**
	 * get the type of the concrete joint.
	 * 
	 * @return
	 */
	public JointType getType() {
		return m_type;
	}

	/**
	 * get the first body attached to this joint.
	 */
	public Body getBodyA() {
		return m_bodyA;
	}

	/**
	 * get the second body attached to this joint.
	 * 
	 * @return
	 */
	public Body getBodyB() {
		return m_bodyB;
	}

	/**
	 * get the next joint the world joint list.
	 */
	public Joint getNext() {
		return m_next;
	}

	/**
	 * get the user data pointer.
	 */
	public Object getUserData() {
		return m_userData;
	}

	/**
	 * Set the user data pointer.
	 */
	public void setUserData(final Object data) {
		m_userData = data;
	}

	/**
	 * Short-cut function to determine if either body is inactive.
	 * 
	 * @return
	 */
	public boolean isActive() {
		return m_bodyA.isActive() && m_bodyB.isActive();
	}

	public abstract void initVelocityConstraints(TimeStep step);

	public abstract void solveVelocityConstraints(TimeStep step);

	/**
	 * This returns true if the position errors are within tolerance.
	 * 
	 * @param baumgarte
	 * @return
	 */
	public abstract boolean solvePositionConstraints(float baumgarte);

	/**
	 * Override to handle destruction of joint
	 */
	public void destructor() { }

	public Vector getReactionForce(final float inv_dt) {
		final Vector vec2 = new Vector();
		getReactionForce(inv_dt, vec2);
		return vec2;
	}

	/**
	 * get the anchor point on bodyA in world coordinates.
	 */
	public abstract void getAnchorA(Vector argOut);

	/**
	 * get the anchor point on bodyB in world coordinates.
	 */
	public abstract void getAnchorB(Vector argOut);

	public Iterator<Joint> iterator() {
		return new B2DIterator<Joint>(this);
	}

}
