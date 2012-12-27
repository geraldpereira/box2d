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
 * Created at 11:34:45 AM Jan 23, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.BodyType;
import fr.byob.game.box2d.dynamics.joints.GearJointDef;
import fr.byob.game.box2d.dynamics.joints.JointType;

//Gear Joint:
//C0 = (coordinate1 + ratio * coordinate2)_initial
//C = C0 - (cordinate1 + ratio * coordinate2) = 0
//Cdot = -(Cdot1 + ratio * Cdot2)
//J = -[J1 ratio * J2]
//K = J * invM * JT
//= J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
//Revolute:
//coordinate = rotation
//Cdot = angularVelocity
//J = [0 0 1]
//K = J * invM * JT = invI
//
//Prismatic:
//coordinate = dot(p - pg, ug)
//Cdot = dot(v + cross(w, r), ug)
//J = [ug cross(r, ug)]
//K = J * invM * JT = invMass + invI * cross(r, ug)^2

/**
 * A gear joint is used to connect two joints together. Either joint
 * can be a revolute or prismatic joint. You specify a gear ratio
 * to bind the motions together:
 * coordinate1 + ratio * coordinate2 = constant
 * The ratio can be negative or positive. If one joint is a revolute joint
 * and the other joint is a prismatic joint, then the ratio will have units
 * of length or units of 1/length.
 * 
 * @warning The revolute and prismatic joints must be attached to
 *          fixed bodies (which must be body1 on those joints).
 * @author Daniel Murphy
 */
public class GearJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.GearJoint {

	private Body m_ground1;
	private Body m_ground2;

	// One of these is null.
	private RevoluteJoint m_revolute1;
	private PrismaticJoint m_prismatic1;

	// One of these is null.
	private RevoluteJoint m_revolute2;
	private PrismaticJoint m_prismatic2;

	private final Vector m_groundAnchor1 = new Vector();
	private final Vector m_groundAnchor2 = new Vector();

	private final Vector m_localAnchor1 = new Vector();
	private final Vector m_localAnchor2 = new Vector();

	private final Jacobian m_J;

	private float m_constant;
	private float m_ratio;

	// Effective mass
	private float m_mass;

	// Impulse for accumulation/warm starting.
	private float m_impulse;

	/**
	 * @param argWorldPool
	 * @param def
	 */
	public GearJoint(final IWorldPool argWorldPool, final GearJointDef def) {
		super(argWorldPool, def);

		final JointType type1 = def.joint1.getType();
		final JointType type2 = def.joint2.getType();

		assert (type1 == JointType.REVOLUTE || type1 == JointType.PRISMATIC);
		assert (type2 == JointType.REVOLUTE || type2 == JointType.PRISMATIC);
		assert (def.joint1.getBodyA().getType() == BodyType.STATIC);
		assert (def.joint2.getBodyA().getType() == BodyType.STATIC);

		m_revolute1 = null;
		m_prismatic1 = null;
		m_revolute2 = null;
		m_prismatic2 = null;

		m_J = new Jacobian();

		float coordinate1, coordinate2;

		m_ground1 = (Body) def.joint1.getBodyA();
		m_bodyA = (Body) def.joint1.getBodyB();
		if (type1 == JointType.REVOLUTE) {
			m_revolute1 = (RevoluteJoint) def.joint1;
			m_groundAnchor1.set(m_revolute1.m_localAnchor1);
			m_localAnchor1.set(m_revolute1.m_localAnchor2);
			coordinate1 = m_revolute1.getJointAngle();
		}
		else {
			m_prismatic1 = (PrismaticJoint) def.joint1;
			m_groundAnchor1.set(m_prismatic1.m_localAnchor1);
			m_localAnchor1.set(m_prismatic1.m_localAnchor2);
			coordinate1 = m_prismatic1.getJointTranslation();
		}

		m_ground2 = (Body) def.joint2.getBodyA();
		m_bodyB = (Body) def.joint2.getBodyB();
		if (type2 == JointType.REVOLUTE) {
			m_revolute2 = (RevoluteJoint) def.joint2;
			m_groundAnchor2.set(m_revolute2.m_localAnchor1);
			m_localAnchor2.set(m_revolute2.m_localAnchor2);
			coordinate2 = m_revolute2.getJointAngle();
		}
		else {
			m_prismatic2 = (PrismaticJoint) def.joint2;
			m_groundAnchor2.set(m_prismatic2.m_localAnchor1);
			m_localAnchor2.set(m_prismatic2.m_localAnchor2);
			coordinate2 = m_prismatic2.getJointTranslation();
		}

		m_ratio = def.ratio;

		m_constant = coordinate1 + m_ratio * coordinate2;

		m_impulse = 0.0f;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorA(org.jbox2d.common.Vector)
	 */
	@Override
	public void getAnchorA(final Vector argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorB(org.jbox2d.common.Vector)
	 */
	@Override
	public void getAnchorB(final Vector argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float,
	 *      org.jbox2d.common.Vector)
	 */
	public void getReactionForce(final float inv_dt, final Vector argOut) {
		// TODO_ERIN not tested
		argOut.set(m_J.linearB).scaleLocal(m_impulse);
		argOut.scaleLocal(inv_dt);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	public float getReactionTorque(final float inv_dt) {

		final Vector r = pool.popVector();
		final Vector p = pool.popVector();

		r.set(m_localAnchor2).subtractLocal(m_bodyB.getLocalCenter());
		Mat22.mulToOut(m_bodyB.getTransform().R, r, r);
		p.set(m_J.linearB).scaleLocal(m_impulse);
		final float L = m_impulse * m_J.angularB - MathUtils.cross(r, p);

		pool.pushVector(2);
		return inv_dt * L;
	}

	public void setRatio(final float argRatio) {
		m_ratio = argRatio;
	}

	public float getRatio() {
		return m_ratio;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body g1 = m_ground1;
		final Body g2 = m_ground2;
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		float K = 0.0f;
		m_J.setZero();

		if (m_revolute1 != null) {
			m_J.angularA = -1.0f;
			K += b1.m_invI;
		}
		else {
			final Vector ug = pool.popVector();
			final Vector r = pool.popVector();
			Mat22.mulToOut(g1.getTransform().R, m_prismatic1.m_localXAxis1, ug);

			r.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
			Mat22.mulToOut(b1.getTransform().R, r, r);
			final float crug = MathUtils.cross(r, ug);
			m_J.linearA.set(ug).negateLocal();
			m_J.angularA = -crug;
			K += b1.m_invMass + b1.m_invI * crug * crug;
			pool.pushVector(2);
		}

		if (m_revolute2 != null) {
			m_J.angularB = -m_ratio;
			K += m_ratio * m_ratio * b2.m_invI;
		}
		else {
			final Vector ug = pool.popVector();
			final Vector r = pool.popVector();

			Mat22.mulToOut(g2.getTransform().R, m_prismatic2.m_localXAxis1, ug);

			r.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());
			Mat22.mulToOut(b2.getTransform().R, r, r);
			final float crug = MathUtils.cross(r, ug);
			m_J.linearB.set(ug).scaleLocal(-m_ratio);
			m_J.angularB = -m_ratio * crug;
			K += m_ratio * m_ratio * (b2.m_invMass + b2.m_invI * crug * crug);

			pool.pushVector(2);
		}

		// Compute effective mass.
		m_mass = K > 0.0f ? 1.0f / K : 0.0f;

		if (step.warmStarting) {
			final Vector temp = pool.popVector();
			// Warm starting.
			temp.set(m_J.linearA).scaleLocal(b1.m_invMass).scaleLocal(m_impulse);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * m_impulse * m_J.angularA;

			temp.set(m_J.linearB).scaleLocal(b2.m_invMass).scaleLocal(m_impulse);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * m_impulse * m_J.angularB;

			pool.pushVector(1);
		}
		else {
			m_impulse = 0.0f;
		}
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final float Cdot = m_J.compute(b1.m_linearVelocity, b1.m_angularVelocity, b2.m_linearVelocity, b2.m_angularVelocity);

		final float impulse = m_mass * (-Cdot);
		m_impulse += impulse;

		final Vector temp = pool.popVector();
		temp.set(m_J.linearA).scaleLocal(b1.m_invMass).scaleLocal(impulse);
		b1.m_linearVelocity.addLocal(temp);
		b1.m_angularVelocity += b1.m_invI * impulse * m_J.angularA;

		temp.set(m_J.linearB).scaleLocal(b2.m_invMass).scaleLocal(impulse);
		b2.m_linearVelocity.addLocal(temp);
		b2.m_angularVelocity += b2.m_invI * impulse * m_J.angularB;

		pool.pushVector(1);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		final float linearError = 0.0f;

		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		float coordinate1, coordinate2;
		if (m_revolute1 != null) {
			coordinate1 = m_revolute1.getJointAngle();
		}
		else {
			coordinate1 = m_prismatic1.getJointTranslation();
		}

		if (m_revolute2 != null) {
			coordinate2 = m_revolute2.getJointAngle();
		}
		else {
			coordinate2 = m_prismatic2.getJointTranslation();
		}

		final float C = m_constant - (coordinate1 + m_ratio * coordinate2);

		final float impulse = m_mass * (-C);

		final Vector temp = pool.popVector();
		temp.set(m_J.linearA).scaleLocal(b1.m_invMass).scaleLocal(impulse);
		b1.m_sweep.c.addLocal(temp);
		b1.m_sweep.a += b1.m_invI * impulse * m_J.angularA;

		temp.set(m_J.linearB).scaleLocal(b2.m_invMass).scaleLocal(impulse);
		b2.m_sweep.c.addLocal(temp);
		b2.m_sweep.a += b2.m_invI * impulse * m_J.angularB;

		b1.synchronizeTransform();
		b2.synchronizeTransform();

		pool.pushVector(1);
		// TODO_ERIN not implemented
		return linearError < Settings.linearSlop;
	}
}
