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
 * Created at 12:12:02 PM Jan 23, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.joints.PulleyJointDef;

/**
 * @author Daniel Murphy
 */
public class PulleyJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.PulleyJoint {

	public static final float MIN_PULLEY_LENGTH = 2.0f;

	private final Vector m_groundAnchor1 = new Vector();
	private final Vector m_groundAnchor2 = new Vector();
	private final Vector m_localAnchor1 = new Vector();
	private final Vector m_localAnchor2 = new Vector();

	private final Vector m_u1 = new Vector();
	private final Vector m_u2 = new Vector();

	private float m_constant;
	private float m_ratio;

	private float m_maxLength1;
	private float m_maxLength2;

	// Effective masses
	private float m_pulleyMass;
	private float m_limitMass1;
	private float m_limitMass2;

	// Impulses for accumulation/warm starting.
	private float m_impulse;
	private float m_limitImpulse1;
	private float m_limitImpulse2;

	private LimitState m_state;
	private LimitState m_limitState1;
	private LimitState m_limitState2;

	/**
	 * @param argWorldPool
	 * @param def
	 */
	public PulleyJoint(final IWorldPool argWorldPool, final PulleyJointDef def) {
		super(argWorldPool, def);
		m_groundAnchor1.set(def.groundAnchorA);
		m_groundAnchor2.set(def.groundAnchorB);
		m_localAnchor1.set(def.localAnchorA);
		m_localAnchor2.set(def.localAnchorB);

		assert (def.ratio != 0.0f);
		m_ratio = def.ratio;

		m_constant = def.lengthA + m_ratio * def.lengthB;

		m_maxLength1 = MathUtils.min(def.maxLengthA, m_constant - m_ratio * MIN_PULLEY_LENGTH);
		m_maxLength2 = MathUtils.min(def.maxLengthB, (m_constant - MIN_PULLEY_LENGTH) / m_ratio);

		m_impulse = 0.0f;
		m_limitImpulse1 = 0.0f;
		m_limitImpulse2 = 0.0f;
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
		argOut.set(m_u2).scaleLocal(m_impulse).scaleLocal(inv_dt);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	public float getReactionTorque(final float inv_dt) {
		return 0f;
	}

	public Vector getGroundAnchorA() {
		return m_groundAnchor1;
	}

	public Vector getGroundAnchorB() {
		return m_groundAnchor2;
	}

	public float getLength1() {
		final Vector p = pool.popVector();
		m_bodyA.getWorldPointToOut(m_localAnchor1, p);
		p.subtractLocal(m_groundAnchor1);

		final float len = p.length();
		pool.pushVector(1);
		return len;
	}

	public float getLength2() {
		final Vector p = pool.popVector();
		m_bodyB.getWorldPointToOut(m_localAnchor2, p);
		p.subtractLocal(m_groundAnchor2);

		final float len = p.length();
		pool.pushVector(1);
		return len;
	}

	public float getRatio() {
		return m_ratio;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector r1 = pool.popVector();
		final Vector r2 = pool.popVector();
		final Vector p1 = pool.popVector();
		final Vector p2 = pool.popVector();
		final Vector s1 = pool.popVector();
		final Vector s2 = pool.popVector();

		r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());

		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);

		p1.set(b1.m_sweep.c).addLocal(r1);
		p2.set(b2.m_sweep.c).addLocal(r2);

		s1.set(m_groundAnchor1);
		s2.set(m_groundAnchor2);

		// Get the pulley axes.
		m_u1.set(p1).subtractLocal(s1);
		m_u2.set(p2).subtractLocal(s2);

		final float length1 = m_u1.length();
		final float length2 = m_u2.length();

		if (length1 > Settings.linearSlop) {
			m_u1.scaleLocal(1.0f / length1);
		}
		else {
			m_u1.set(0,0);
		}

		if (length2 > Settings.linearSlop) {
			m_u2.scaleLocal(1.0f / length2);
		}
		else {
			m_u2.set(0,0);
		}

		final float C = m_constant - length1 - m_ratio * length2;
		if (C > 0.0f) {
			m_state = LimitState.INACTIVE;
			m_impulse = 0.0f;
		}
		else {
			m_state = LimitState.AT_UPPER;
		}

		if (length1 < m_maxLength1) {
			m_limitState1 = LimitState.INACTIVE;
			m_limitImpulse1 = 0.0f;
		}
		else {
			m_limitState1 = LimitState.AT_UPPER;
		}

		if (length2 < m_maxLength2) {
			m_limitState2 = LimitState.INACTIVE;
			m_limitImpulse2 = 0.0f;
		}
		else {
			m_limitState2 = LimitState.AT_UPPER;
		}

		// Compute effective mass.
		final float cr1u1 = MathUtils.cross(r1, m_u1);
		final float cr2u2 = MathUtils.cross(r2, m_u2);

		m_limitMass1 = b1.m_invMass + b1.m_invI * cr1u1 * cr1u1;
		m_limitMass2 = b2.m_invMass + b2.m_invI * cr2u2 * cr2u2;
		m_pulleyMass = m_limitMass1 + m_ratio * m_ratio * m_limitMass2;
		assert (m_limitMass1 > Settings.EPSILON);
		assert (m_limitMass2 > Settings.EPSILON);
		assert (m_pulleyMass > Settings.EPSILON);
		m_limitMass1 = 1.0f / m_limitMass1;
		m_limitMass2 = 1.0f / m_limitMass2;
		m_pulleyMass = 1.0f / m_pulleyMass;

		if (step.warmStarting) {
			// Scale impulses to support variable time steps.
			m_impulse *= step.dtRatio;
			m_limitImpulse1 *= step.dtRatio;
			m_limitImpulse2 *= step.dtRatio;

			final Vector P1 = pool.popVector();
			final Vector P2 = pool.popVector();
			final Vector temp = pool.popVector();

			// Warm starting.
			P1.set(m_u1).scaleLocal(-(m_impulse + m_limitImpulse1));
			P2.set(m_u2).scaleLocal(-m_ratio * m_impulse - m_limitImpulse2);

			temp.set(P1).scaleLocal(b1.m_invMass);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * MathUtils.cross(r1, P1);

			temp.set(P2).scaleLocal(b2.m_invMass);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * MathUtils.cross(r2, P2);

			pool.pushVector(3);
		}
		else {
			m_impulse = 0.0f;
			m_limitImpulse1 = 0.0f;
			m_limitImpulse2 = 0.0f;
		}

		pool.pushVector(6);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector r1 = pool.popVector();
		final Vector r2 = pool.popVector();

		r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());

		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);

		if (m_state == LimitState.AT_UPPER) {
			final Vector v1 = pool.popVector();
			final Vector v2 = pool.popVector();

			MathUtils.crossToOut(b1.m_angularVelocity, r1, v1);
			MathUtils.crossToOut(b2.m_angularVelocity, r2, v2);

			v1.addLocal(b1.m_linearVelocity);
			v2.addLocal(b2.m_linearVelocity);

			final float Cdot = -MathUtils.dot(m_u1, v1) - m_ratio * MathUtils.dot(m_u2, v2);
			float impulse = m_pulleyMass * (-Cdot);
			final float oldImpulse = m_impulse;
			m_impulse = MathUtils.max(0.0f, m_impulse + impulse);
			impulse = m_impulse - oldImpulse;

			final Vector P1 = pool.popVector();
			final Vector P2 = pool.popVector();
			final Vector temp = pool.popVector();

			P1.set(m_u1).scaleLocal(-impulse);
			P2.set(m_u2).scaleLocal(-m_ratio * impulse);

			temp.set(P1).scaleLocal(b1.m_invMass);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * MathUtils.cross(r1, P1);

			temp.set(P2).scaleLocal(b2.m_invMass);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * MathUtils.cross(r2, P2);

			pool.pushVector(5);
		}

		if (m_limitState1 == LimitState.AT_UPPER) {
			final Vector v1 = pool.popVector();

			MathUtils.crossToOut(b1.m_angularVelocity, r1, v1);
			v1.addLocal(b1.m_linearVelocity);

			final float Cdot = -MathUtils.dot(m_u1, v1);
			float impulse = -m_limitMass1 * Cdot;
			final float oldImpulse = m_limitImpulse1;
			m_limitImpulse1 = MathUtils.max(0.0f, m_limitImpulse1 + impulse);
			impulse = m_limitImpulse1 - oldImpulse;

			final Vector P1 = pool.popVector();
			final Vector temp = pool.popVector();

			P1.set(m_u1).scaleLocal(-impulse);

			temp.set(P1).scaleLocal(b1.m_invMass);
			b1.m_linearVelocity.addLocal(temp);
			b1.m_angularVelocity += b1.m_invI * MathUtils.cross(r1, P1);

			pool.pushVector(3);
		}

		if (m_limitState2 == LimitState.AT_UPPER) {

			final Vector v2 = pool.popVector();
			MathUtils.crossToOut(b2.m_angularVelocity, r2, v2);
			v2.addLocal(b2.m_linearVelocity);

			final float Cdot = -MathUtils.dot(m_u2, v2);
			float impulse = -m_limitMass2 * Cdot;
			final float oldImpulse = m_limitImpulse2;
			m_limitImpulse2 = MathUtils.max(0.0f, m_limitImpulse2 + impulse);
			impulse = m_limitImpulse2 - oldImpulse;

			final Vector P2 = pool.popVector();
			final Vector temp = pool.popVector();

			P2.set(m_u2).scaleLocal(-impulse);

			temp.set(P2).scaleLocal(b2.m_invMass);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += b2.m_invI * MathUtils.cross(r2, P2);

			pool.pushVector(3);
		}

		pool.pushVector(2);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector s1 = pool.popVector();
		final Vector s2 = pool.popVector();

		s1.set(m_groundAnchor1);
		s2.set(m_groundAnchor2);

		float linearError = 0.0f;

		if (m_state == LimitState.AT_UPPER) {
			final Vector r1 = pool.popVector();
			final Vector r2 = pool.popVector();
			final Vector p1 = pool.popVector();
			final Vector p2 = pool.popVector();

			r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
			r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());

			Mat22.mulToOut(b1.getTransform().R, r1, r1);
			Mat22.mulToOut(b2.getTransform().R, r2, r2);

			p1.set(b1.m_sweep.c).addLocal(r1);
			p2.set(b2.m_sweep.c).addLocal(r2);

			// Get the pulley axes.
			m_u1.set(p1).subtractLocal(s1);
			m_u2.set(p2).subtractLocal(s2);

			final float length1 = m_u1.length();
			final float length2 = m_u2.length();

			if (length1 > Settings.linearSlop) {
				m_u1.scaleLocal(1.0f / length1);
			}
			else {
				m_u1.set(0,0);
			}

			if (length2 > Settings.linearSlop) {
				m_u2.scaleLocal(1.0f / length2);
			}
			else {
				m_u2.set(0,0);
			}

			float C = m_constant - length1 - m_ratio * length2;
			linearError = MathUtils.max(linearError, -C);

			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			final float impulse = -m_pulleyMass * C;

			final Vector P1 = pool.popVector();
			final Vector P2 = pool.popVector();
			final Vector temp = pool.popVector();

			P1.set(m_u1).scaleLocal(-impulse);
			P2.set(m_u2).scaleLocal(-m_ratio * impulse);

			temp.set(P1).scaleLocal(b1.m_invMass);
			b1.m_sweep.c.addLocal(temp);
			b1.m_sweep.a += b1.m_invI * MathUtils.cross(r1, P1);

			temp.set(P2).scaleLocal(b2.m_invMass);
			b2.m_sweep.c.addLocal(temp);
			b2.m_sweep.a += b2.m_invI * MathUtils.cross(r2, P2);

			b1.synchronizeTransform();
			b2.synchronizeTransform();

			pool.pushVector(7);
		}

		if (m_limitState1 == LimitState.AT_UPPER) {
			final Vector r1 = pool.popVector();
			final Vector p1 = pool.popVector();

			r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());

			Mat22.mulToOut(b1.getTransform().R, r1, r1);

			p1.set(b1.m_sweep.c).addLocal(r1);

			m_u1.set(p1).subtractLocal(s1);

			final float length1 = m_u1.length();

			if (length1 > Settings.linearSlop) {
				m_u1.scaleLocal(1.0f / length1);
			}
			else {
				m_u1.set(0,0);
			}

			float C = m_maxLength1 - length1;
			linearError = MathUtils.max(linearError, -C);
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			final float impulse = -m_limitMass1 * C;

			final Vector P1 = pool.popVector();
			final Vector temp = pool.popVector();

			P1.set(m_u1).scaleLocal(-impulse);

			temp.set(P1).scaleLocal(b1.m_invMass);
			b1.m_sweep.c.addLocal(temp);
			b1.m_sweep.a += b1.m_invI * MathUtils.cross(r1, P1);

			b1.synchronizeTransform();

			pool.pushVector(4);
		}

		if (m_limitState2 == LimitState.AT_UPPER) {
			final Vector r2 = pool.popVector();
			final Vector p2 = pool.popVector();

			r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());

			Mat22.mulToOut(b2.getTransform().R, r2, r2);

			p2.set(b2.m_sweep.c).addLocal(r2);

			// Get the pulley axes.
			m_u2.set(p2).subtractLocal(s2);

			final float length2 = m_u2.length();

			if (length2 > Settings.linearSlop) {
				m_u2.scaleLocal(1.0f / length2);
			}
			else {
				m_u2.set(0,0);
			}

			float C = m_maxLength2 - length2;
			linearError = MathUtils.max(linearError, -C);
			C = MathUtils.clamp(C + Settings.linearSlop, -Settings.maxLinearCorrection, 0.0f);
			final float impulse = -m_limitMass2 * C;

			final Vector P2 = pool.popVector();
			final Vector temp = pool.popVector();

			P2.set(m_u2).scaleLocal(-impulse);

			temp.set(P2).scaleLocal(b2.m_invMass);
			b2.m_sweep.c.addLocal(temp);
			b2.m_sweep.a += b2.m_invI * MathUtils.cross(r2, P2);

			b2.synchronizeTransform();

			pool.pushVector(4);
		}
		pool.pushVector(2);

		return linearError < Settings.linearSlop;
	}
}
