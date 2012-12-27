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
 * Created at 9:06:02 PM Jan 21, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.common.Transform;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.joints.LineJointDef;

/**
 * @author Daniel Murphy
 */
public class LineJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.LineJoint {

	private final Vector m_localAnchor1 = new Vector();
	private final Vector m_localAnchor2 = new Vector();
	private final Vector m_localXAxis1 = new Vector();
	private final Vector m_localYAxis1 = new Vector();

	private final Vector m_axis = new Vector();
	private final Vector m_perp = new Vector();
	private float m_s1, m_s2;
	private float m_a1, m_a2;

	private final Mat22 m_K = new Mat22();
	private final Vector m_impulse = new Vector();

	private float m_motorMass; // effective mass for motor/limit translational
	// constraint.
	private float m_motorImpulse;

	private float m_lowerTranslation;
	private float m_upperTranslation;
	private float m_maxMotorForce;
	private float m_motorSpeed;

	private boolean m_enableLimit;
	private boolean m_enableMotor;
	private LimitState m_limitState;

	public LineJoint(final IWorldPool argPool, final LineJointDef def) {
		super(argPool, def);
		m_localAnchor1.set(def.localAnchorA);
		m_localAnchor2.set(def.localAnchorB);
		m_localXAxis1.set(def.localAxisA);
		MathUtils.crossToOut(m_localXAxis1, 1.0f, m_localYAxis1);

		m_impulse.set(0,0);
		m_motorMass = 0.0f;
		m_motorImpulse = 0.0f;

		m_lowerTranslation = def.lowerTranslation;
		m_upperTranslation = def.upperTranslation;
		m_maxMotorForce = def.maxMotorForce;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
		m_limitState = LimitState.INACTIVE;

		m_axis.set(0,0);
		m_perp.set(0,0);
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
		final Vector temp = pool.popVector();
		temp.set(m_perp).scaleLocal(m_impulse.x);
		argOut.set(m_axis).scaleLocal(m_motorImpulse + m_impulse.y).addLocal(temp).scaleLocal(inv_dt);
		pool.pushVector(1);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	public float getReactionTorque(final float inv_dt) {
		return 0.0f;
	}

	public float getJointTranslation() {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector p1 = pool.popVector();
		final Vector p2 = pool.popVector();
		final Vector axis = pool.popVector();
		b1.getWorldPointToOut(m_localAnchor1, p1);
		b2.getWorldPointToOut(m_localAnchor1, p2);
		p2.subtractLocal(p1);
		b1.getWorldVectorToOut(m_localXAxis1, axis);

		final float translation = MathUtils.dot(p2, axis);
		pool.pushVector(3);
		return translation;
	}

	public float getJointSpeed() {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

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
		p2.subtractLocal(p1);

		final Vector axis = pool.popVector();
		b1.getWorldPointToOut(m_localXAxis1, axis);

		final Vector v1 = b1.m_linearVelocity;
		final Vector v2 = b2.m_linearVelocity;
		final float w1 = b1.m_angularVelocity;
		final float w2 = b2.m_angularVelocity;

		final Vector temp1 = pool.popVector();
		final Vector temp2 = pool.popVector();

		MathUtils.crossToOut(w1, r1, temp1);
		MathUtils.crossToOut(w2, r2, temp2);
		temp2.addLocal(v2).subtractLocal(v1).subtractLocal(temp1);
		final float s2 = MathUtils.dot(axis, temp2);

		MathUtils.crossToOut(w1, axis, temp1);
		final float speed = MathUtils.dot(p2, temp1) + s2;

		pool.pushVector(7);
		return speed;
	}

	public boolean isLimitEnabled() {
		return m_enableLimit;
	}

	public void EnableLimit(final boolean flag) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableLimit = flag;
	}

	public float getLowerLimit() {
		return m_lowerTranslation;
	}

	public float getUpperLimit() {
		return m_upperTranslation;
	}

	public void setLimits(final float lower, final float upper) {
		assert (lower <= upper);
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
	}

	public boolean isMotorEnabled() {
		return m_enableMotor;
	}

	public void EnableMotor(final boolean flag) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableMotor = flag;
	}

	public void setMotorSpeed(final float speed) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_motorSpeed = speed;
	}

	public void setMaxMotorForce(final float force) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_maxMotorForce = force;
	}

	public float getMotorForce() {
		return m_motorImpulse;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		m_localCenterA.set(b1.getLocalCenter());
		m_localCenterB.set(b2.getLocalCenter());

		final Transform xf1 = b1.getTransform();
		final Transform xf2 = b2.getTransform();

		// Compute the effective masses.
		final Vector r1 = pool.popVector();
		final Vector r2 = pool.popVector();
		final Vector temp = pool.popVector();

		r1.set(m_localAnchor1).subtractLocal(m_localCenterA);
		r2.set(m_localAnchor2).subtractLocal(m_localCenterB);
		Mat22.mulToOut(xf1.R, r1, r1);
		Mat22.mulToOut(xf2.R, r2, r2);

		final Vector d = pool.popVector();
		d.set(b2.m_sweep.c).addLocal(r2).subtractLocal(b1.m_sweep.c).subtractLocal(r1);

		m_invMassA = b1.m_invMass;
		m_invIA = b1.m_invI;
		m_invMassB = b2.m_invMass;
		m_invIB = b2.m_invI;

		// Compute motor Jacobian and effective mass.
		{
			Mat22.mulToOut(xf1.R, m_localXAxis1, m_axis);
			temp.set(d).addLocal(r1);
			m_a1 = MathUtils.cross(temp, m_axis);
			m_a2 = MathUtils.cross(r2, m_axis);

			m_motorMass = m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
			if (m_motorMass > Settings.EPSILON) {
				m_motorMass = 1.0f / m_motorMass;
			}
			else {
				m_motorMass = 0.0f;
			}
		}

		// Prismatic constraint.
		{
			Mat22.mulToOut(xf1.R, m_localYAxis1, m_perp);

			temp.set(d).addLocal(r1);
			m_s1 = MathUtils.cross(temp, m_perp);
			m_s2 = MathUtils.cross(r2, m_perp);

			final float m1 = m_invMassA, m2 = m_invMassB;
			final float i1 = m_invIA, i2 = m_invIB;

			final float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
			final float k12 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
			final float k22 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

			m_K.m11 = k11;
			m_K.m12 = k12;
			m_K.m21 = k12;
			m_K.m22 = k22;
		}

		// Compute motor and limit terms.
		if (m_enableLimit) {
			final float jointTranslation = MathUtils.dot(m_axis, d);
			if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
				m_limitState = LimitState.EQUAL;
			}
			else if (jointTranslation <= m_lowerTranslation) {
				if (m_limitState != LimitState.AT_LOWER) {
					m_limitState = LimitState.AT_LOWER;
					m_impulse.y = 0.0f;
				}
			}
			else if (jointTranslation >= m_upperTranslation) {
				if (m_limitState != LimitState.AT_UPPER) {
					m_limitState = LimitState.AT_UPPER;
					m_impulse.y = 0.0f;
				}
			}
			else {
				m_limitState = LimitState.INACTIVE;
				m_impulse.y = 0.0f;
			}
		}
		else {
			m_limitState = LimitState.INACTIVE;
		}

		if (m_enableMotor == false) {
			m_motorImpulse = 0.0f;
		}

		if (step.warmStarting) {
			// Account for variable time step.
			m_impulse.scaleLocal(step.dtRatio);
			m_motorImpulse *= step.dtRatio;

			final Vector P = pool.popVector();
			temp.set(m_axis).scaleLocal(m_motorImpulse + m_impulse.y);
			P.set(m_perp).scaleLocal(m_impulse.x).addLocal(temp);

			final float L1 = m_impulse.x * m_s1 + (m_motorImpulse + m_impulse.y) * m_a1;
			final float L2 = m_impulse.x * m_s2 + (m_motorImpulse + m_impulse.y) * m_a2;

			temp.set(P).scaleLocal(m_invMassA);
			b1.m_linearVelocity.subtractLocal(temp);
			b1.m_angularVelocity -= m_invIA * L1;

			temp.set(P).scaleLocal(m_invMassB);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += m_invIB * L2;
			pool.pushVector(1);
		}
		else {
			m_impulse.set(0,0);
			m_motorImpulse = 0.0f;
		}
		pool.pushVector(4);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector v1 = b1.m_linearVelocity;
		float w1 = b1.m_angularVelocity;
		final Vector v2 = b2.m_linearVelocity;
		float w2 = b2.m_angularVelocity;

		final Vector temp = pool.popVector();

		// Solve linear motor constraint.
		if (m_enableMotor && m_limitState != LimitState.EQUAL) {
			temp.set(v2).subtractLocal(v1);
			final float Cdot = MathUtils.dot(m_axis, temp) + m_a2 * w2 - m_a1 * w1;
			float impulse = m_motorMass * (m_motorSpeed - Cdot);
			final float oldImpulse = m_motorImpulse;
			final float maxImpulse = step.dt * m_maxMotorForce;
			m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_motorImpulse - oldImpulse;

			final Vector P = pool.popVector();
			P.set(m_axis).scaleLocal(impulse);
			final float L1 = impulse * m_a1;
			final float L2 = impulse * m_a2;

			temp.set(P).scaleLocal(m_invMassA);
			v1.subtractLocal(temp);
			w1 -= m_invIA * L1;

			temp.set(P).scaleLocal(m_invMassB);
			v2.addLocal(temp);
			w2 += m_invIB * L2;
			pool.pushVector(1);
		}

		temp.set(v2).subtractLocal(v1);
		final float Cdot1 = MathUtils.dot(m_perp, temp) + m_s2 * w2 - m_s1 * w1;

		if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
			// Solve prismatic and limit constraint in block form.
			temp.set(v2).subtractLocal(v1);
			final float Cdot2 = MathUtils.dot(m_axis, temp) + m_a2 * w2 - m_a1 * w1;

			final Vector Cdot = pool.popVector();
			Cdot.set(Cdot1, Cdot2);

			final Vector f1 = pool.popVector();
			f1.set(m_impulse);
			final Vector df = pool.popVector();
			m_K.solveToOut(Cdot.negateLocal(), df); // just leave negated
			m_impulse.addLocal(df);

			if (m_limitState == LimitState.AT_LOWER) {
				m_impulse.y = MathUtils.max(m_impulse.y, 0.0f);
			}
			else if (m_limitState == LimitState.AT_UPPER) {
				m_impulse.y = MathUtils.min(m_impulse.y, 0.0f);
			}

			// f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
			final float b = -Cdot1 - (m_impulse.y - f1.y) * m_K.m21;
			float f2r;
			if (m_K.m11 != 0.0f) {
				f2r = b / m_K.m11 + f1.x;
			}
			else {
				f2r = f1.x;
			}

			m_impulse.x = f2r;

			df.set(m_impulse).subtractLocal(f1);

			final Vector P = pool.popVector();
			temp.set(m_axis).scaleLocal(df.y);
			P.set(m_perp).scaleLocal(df.x).addLocal(temp);

			final float L1 = df.x * m_s1 + df.y * m_a1;
			final float L2 = df.x * m_s2 + df.y * m_a2;

			temp.set(P).scaleLocal(m_invMassA);
			v1.subtractLocal(temp);
			w1 -= m_invIA * L1;

			temp.set(P).scaleLocal(m_invMassB);
			v2.addLocal(temp);
			w2 += m_invIB * L2;
			pool.pushVector(4);
		}
		else {
			// Limit is inactive, just solve the prismatic constraint in block
			// form.
			float df;
			if (m_K.m11 != 0.0f) {
				df = -Cdot1 / m_K.m11;
			}
			else {
				df = 0.0f;
			}
			m_impulse.x += df;

			final Vector P = pool.popVector();
			P.set(m_perp).scaleLocal(df);

			final float L1 = df * m_s1;
			final float L2 = df * m_s2;

			temp.set(P).scaleLocal(m_invMassA);
			v1.subtractLocal(temp);
			w1 -= m_invIA * L1;

			temp.set(P).scaleLocal(m_invMassB);
			v2.addLocal(temp);
			w2 += m_invIB * L2;
			pool.pushVector(1);
		}

		pool.pushVector(1);

		b1.m_angularVelocity = w1;
		b2.m_angularVelocity = w2;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector c1 = b1.m_sweep.c;
		float a1 = b1.m_sweep.a;

		final Vector c2 = b2.m_sweep.c;
		float a2 = b2.m_sweep.a;

		// Solve linear limit constraint.
		float linearError = 0.0f, angularError = 0.0f;
		boolean active = false;
		float C2 = 0.0f;

		final Mat22 R1 = pool.popMat22();
		final Mat22 R2 = pool.popMat22();
		R1.set(a1);
		R2.set(a2);

		final Vector r1 = pool.popVector();
		final Vector r2 = pool.popVector();
		final Vector temp = pool.popVector();
		final Vector d = pool.popVector();

		r1.set(m_localAnchor1).subtractLocal(m_localCenterA);
		r2.set(m_localAnchor2).subtractLocal(m_localCenterB);
		Mat22.mulToOut(R1, r1, r1);
		Mat22.mulToOut(R2, r2, r2);
		d.set(c2).addLocal(r2).subtractLocal(c1).subtractLocal(r1);

		if (m_enableLimit) {
			Mat22.mulToOut(R1, m_localXAxis1, m_axis);

			temp.set(d).addLocal(r1);
			m_a1 = MathUtils.cross(temp, m_axis);
			m_a2 = MathUtils.cross(r2, m_axis);

			final float translation = MathUtils.dot(m_axis, d);
			if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
				// Prevent large angular corrections
				C2 = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
				linearError = MathUtils.abs(translation);
				active = true;
			}
			else if (translation <= m_lowerTranslation) {
				// Prevent large linear corrections and allow some slop.
				C2 = MathUtils.clamp(translation - m_lowerTranslation + Settings.linearSlop,
						-Settings.maxLinearCorrection, 0.0f);
				linearError = m_lowerTranslation - translation;
				active = true;
			}
			else if (translation >= m_upperTranslation) {
				// Prevent large linear corrections and allow some slop.
				C2 = MathUtils.clamp(translation - m_upperTranslation - Settings.linearSlop, 0.0f,
						Settings.maxLinearCorrection);
				linearError = translation - m_upperTranslation;
				active = true;
			}
		}

		Mat22.mulToOut(R1, m_localYAxis1, m_perp);

		temp.set(d).addLocal(r1);
		m_s1 = MathUtils.cross(temp, m_perp);
		m_s2 = MathUtils.cross(r2, m_perp);

		final Vector impulse = pool.popVector();
		float C1;
		C1 = MathUtils.dot(m_perp, d);

		linearError = MathUtils.max(linearError, MathUtils.abs(C1));
		angularError = 0.0f;

		if (active) {
			final float m1 = m_invMassA, m2 = m_invMassB;
			final float i1 = m_invIA, i2 = m_invIB;

			final float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
			final float k12 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
			final float k22 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;

			m_K.m11 = k11;
			m_K.m12 = k12;
			m_K.m21 = k12;
			m_K.m22 = k22;

			final Vector C = pool.popVector();
			C.x = C1;
			C.y = C2;

			m_K.solveToOut(C.negateLocal(), impulse);
			pool.pushVector(1);
		}
		else {
			final float m1 = m_invMassA, m2 = m_invMassB;
			final float i1 = m_invIA, i2 = m_invIB;

			final float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;

			float impulse1;
			if (k11 != 0.0f) {
				impulse1 = -C1 / k11;
			}
			else {
				impulse1 = 0.0f;
			}

			impulse.x = impulse1;
			impulse.y = 0.0f;
		}

		final Vector P = pool.popVector();
		temp.set(m_axis).scaleLocal(impulse.y);
		P.set(m_perp).scaleLocal(impulse.x).addLocal(temp);

		final float L1 = impulse.x * m_s1 + impulse.y * m_a1;
		final float L2 = impulse.x * m_s2 + impulse.y * m_a2;

		temp.set(P).scaleLocal(m_invMassA);
		c1.subtractLocal(temp);
		a1 -= m_invIA * L1;

		temp.set(P).scaleLocal(m_invMassB);
		c2.addLocal(temp);
		a2 += m_invIB * L2;

		// TODO_ERIN remove need for this.
		b1.m_sweep.a = a1;
		b2.m_sweep.a = a2;
		b1.synchronizeTransform();
		b2.synchronizeTransform();

		pool.pushVector(6);
		pool.pushMat22(2);

		return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop;
	}
}
