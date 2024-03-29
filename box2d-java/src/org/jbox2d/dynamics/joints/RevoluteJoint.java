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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 * 
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import pythagoras.f.Vector3;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.Matrix3;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.joints.RevoluteJointDef;

//Point-to-point constraint
//C = p2 - p1
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)

//Motor constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A revolute joint constrains two bodies to share a common point while they
 * are free to rotate about the point. The relative rotation about the shared
 * point is the joint angle. You can limit the relative rotation with
 * a joint limit that specifies a lower and upper angle. You can use a motor
 * to drive the relative rotation about the shared point. A maximum motor torque
 * is provided so that infinite forces are not generated.
 * 
 * @author Daniel Murphy
 */
public class RevoluteJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.RevoluteJoint {

	public final Vector m_localAnchor1 = new Vector(); // relative
	public final Vector m_localAnchor2 = new Vector();
	public final Vector3 m_impulse = new Vector3();
	public float m_motorImpulse;

	public final Matrix3 m_mass = new Matrix3(); // effective mass for point-to-point
	// constraint.
	public float m_motorMass; // effective mass for motor/limit angular constraint.

	public boolean m_enableMotor;
	public float m_maxMotorTorque;
	public float m_motorSpeed;

	public boolean m_enableLimit;
	public float m_referenceAngle;
	public float m_lowerAngle;
	public float m_upperAngle;
	public LimitState m_limitState;

	public RevoluteJoint(final IWorldPool argWorld, final RevoluteJointDef def) {
		super(argWorld, def);
		m_localAnchor1.set(def.localAnchorA);
		m_localAnchor2.set(def.localAnchorB);
		m_referenceAngle = def.referenceAngle;

		m_motorImpulse = 0;

		m_lowerAngle = def.lowerAngle;
		m_upperAngle = def.upperAngle;
		m_maxMotorTorque = def.maxMotorTorque;
		m_motorSpeed = def.motorSpeed;
		m_enableLimit = def.enableLimit;
		m_enableMotor = def.enableMotor;
	}

	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		if (m_enableMotor || m_enableLimit) {
			// You cannot create a rotation limit between bodies that
			// both have fixed rotation.
			assert (b1.m_invI > 0.0f || b2.m_invI > 0.0f);
		}

		final Vector r1 = pool.popVector();
		final Vector r2 = pool.popVector();

		// Compute the effective mass matrix.
		r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
		r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());
		Mat22.mulToOut(b1.getTransform().R, r1, r1);
		Mat22.mulToOut(b2.getTransform().R, r2, r2);

		// J = [-I -r1_skew I r2_skew]
		// [ 0 -1 0 1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ m1+r1y^2*i1+m2+r2y^2*i2, -r1y*i1*r1x-r2y*i2*r2x, -r1y*i1-r2y*i2]
		// [ -r1y*i1*r1x-r2y*i2*r2x, m1+r1x^2*i1+m2+r2x^2*i2, r1x*i1+r2x*i2]
		// [ -r1y*i1-r2y*i2, r1x*i1+r2x*i2, i1+i2]

		// K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 *
		// skew(r2)]
		// = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y
		// -r1.x*r1.y]
		// [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]

		final float m1 = b1.m_invMass, m2 = b2.m_invMass;
		final float i1 = b1.m_invI, i2 = b2.m_invI;

		m_mass.col1.x = m1 + m2 + r1.y * r1.y * i1 + r2.y * r2.y * i2;
		m_mass.col2.x = -r1.y * r1.x * i1 - r2.y * r2.x * i2;
		m_mass.col3.x = -r1.y * i1 - r2.y * i2;
		m_mass.col1.y = m_mass.col2.x;
		m_mass.col2.y = m1 + m2 + r1.x * r1.x * i1 + r2.x * r2.x * i2;
		m_mass.col3.y = r1.x * i1 + r2.x * i2;
		m_mass.col1.z = m_mass.col3.x;
		m_mass.col2.z = m_mass.col3.y;
		m_mass.col3.z = i1 + i2;

		m_motorMass = i1 + i2;
		if (m_motorMass > 0.0f) {
			m_motorMass = 1.0f / m_motorMass;
		}

		if (m_enableMotor == false) {
			m_motorImpulse = 0.0f;
		}

		//		System.out.printf("joint angle: %f, %f, and %f\n", b2.m_sweep.a, b1.m_sweep.a, m_referenceAngle);

		if (m_enableLimit) {
			final float jointAngle = b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			if (MathUtils.abs(m_upperAngle - m_lowerAngle) < 2.0f * Settings.angularSlop) {
				m_limitState = LimitState.EQUAL;
			}
			else if (jointAngle <= m_lowerAngle) {
				if (m_limitState != LimitState.AT_LOWER) {
					m_impulse.z = 0.0f;
				}
				m_limitState = LimitState.AT_LOWER;
			}
			else if (jointAngle >= m_upperAngle) {
				if (m_limitState != LimitState.AT_UPPER) {
					m_impulse.z = 0.0f;
				}
				m_limitState = LimitState.AT_UPPER;
			}
			else {
				m_limitState = LimitState.INACTIVE;
				m_impulse.z = 0.0f;
			}
		}
		else {
			m_limitState = LimitState.INACTIVE;
		}
		//		System.out.printf("limit state: %s\n", m_limitState.toString());

		if (step.warmStarting) {
			// Scale impulses to support a variable time step.
			m_impulse.multLocal(step.dtRatio);
			m_motorImpulse *= step.dtRatio;

			final Vector temp = pool.popVector();
			final Vector P = pool.popVector();
			P.set(m_impulse.x, m_impulse.y);

			temp.set(P).scaleLocal(m1);
			b1.m_linearVelocity.subtractLocal(temp);
			b1.m_angularVelocity -= i1 * (MathUtils.cross(r1, P) + m_motorImpulse + m_impulse.z);

			temp.set(P).scaleLocal(m2);
			b2.m_linearVelocity.addLocal(temp);
			b2.m_angularVelocity += i2 * (MathUtils.cross(r2, P) + m_motorImpulse + m_impulse.z);

			pool.pushVector(2);
		}
		else {
			m_impulse.set(0, 0, 0);
			m_motorImpulse = 0.0f;
		}
		pool.pushVector(2);
	}

	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		final Vector v1 = b1.m_linearVelocity;
		float w1 = b1.m_angularVelocity;
		final Vector v2 = b2.m_linearVelocity;
		float w2 = b2.m_angularVelocity;

		final float m1 = b1.m_invMass, m2 = b2.m_invMass;
		final float i1 = b1.m_invI, i2 = b2.m_invI;

		// Solve motor constraint.
		if (m_enableMotor && m_limitState != LimitState.EQUAL) {
			final float Cdot = w2 - w1 - m_motorSpeed;
			float impulse = m_motorMass * (-Cdot);
			final float oldImpulse = m_motorImpulse;
			final float maxImpulse = step.dt * m_maxMotorTorque;
			m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_motorImpulse - oldImpulse;

			w1 -= i1 * impulse;
			w2 += i2 * impulse;
		}
		final Vector temp = pool.popVector();
		final Vector r1 = pool.popVector();
		final Vector r2 = pool.popVector();

		// Solve limit constraint.
		if (m_enableLimit && m_limitState != LimitState.INACTIVE) {

			r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
			r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());
			Mat22.mulToOut(b1.getTransform().R, r1, r1);
			Mat22.mulToOut(b2.getTransform().R, r2, r2);
			// Vec2 r1 = b2Mul(b1.getTransform().R, m_localAnchor1 - b1.getLocalCenter());
			// Vec2 r2 = b2Mul(b2.getTransform().R, m_localAnchor2 - b2.getLocalCenter());

			final Vector Cdot1 = pool.popVector();
			final Vector3 Cdot = pool.popVector3();

			// Solve point-to-point constraint
			MathUtils.crossToOut(w1, r1, temp);
			MathUtils.crossToOut(w2, r2, Cdot1);
			Cdot1.addLocal(v2).subtractLocal(v1).subtractLocal(temp);
			final float Cdot2 = w2 - w1;
			Cdot.set(Cdot1.x, Cdot1.y, Cdot2);
			// Vec2 Cdot1 = v2 + b2Cross(w2, r2) - v1 - b2Cross(w1, r1);
			// float Cdot2 = w2 - w1;
			// b2Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);

			final Vector3 impulse = pool.popVector3();
			m_mass.solve33ToOut(Cdot.negateLocal(), impulse);
			// Cdot.negateLocal(); just leave negated, we don't use later

			if (m_limitState == LimitState.EQUAL) {
				m_impulse.addLocal(impulse);
			}
			else if (m_limitState == LimitState.AT_LOWER) {
				final float newImpulse = m_impulse.z + impulse.z;
				if (newImpulse < 0.0f) {
					m_mass.solve22ToOut(Cdot1.negateLocal(), temp);
					//Cdot1.negateLocal(); just leave negated, we don't use it again
					impulse.x = temp.x;
					impulse.y = temp.y;
					impulse.z = -m_impulse.z;
					m_impulse.x += temp.x;
					m_impulse.y += temp.y;
					m_impulse.z = 0.0f;
				}
			}
			else if (m_limitState == LimitState.AT_UPPER) {
				final float newImpulse = m_impulse.z + impulse.z;
				if (newImpulse > 0.0f) {
					m_mass.solve22ToOut(Cdot1.negateLocal(), temp);
					//Cdot1.negateLocal(); just leave negated, we don't use it again
					impulse.x = temp.x;
					impulse.y = temp.y;
					impulse.z = -m_impulse.z;
					m_impulse.x += temp.x;
					m_impulse.y += temp.y;
					m_impulse.z = 0.0f;
				}
			}
			final Vector P = pool.popVector();

			P.set(impulse.x, impulse.y);

			temp.set(P).scaleLocal(m1);
			v1.subtractLocal(temp);
			w1 -= i1 * (MathUtils.cross(r1, P) + impulse.z);

			temp.set(P).scaleLocal(m2);
			v2.addLocal(temp);
			w2 += i2 * (MathUtils.cross(r2, P) + impulse.z);

			pool.pushVector(2);
			pool.pushVector3(2);
		}
		else {
			r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
			r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());
			Mat22.mulToOut(b1.getTransform().R, r1, r1);
			Mat22.mulToOut(b2.getTransform().R, r2, r2);
			// Vec2 r1 = b2Mul(b1.getTransform().R, m_localAnchor1 - b1.getLocalCenter());
			// Vec2 r2 = b2Mul(b2.getTransform().R, m_localAnchor2 - b2.getLocalCenter());

			// Solve point-to-point constraint
			final Vector Cdot = pool.popVector();
			final Vector impulse = pool.popVector();

			MathUtils.crossToOut(w1, r1, temp);
			MathUtils.crossToOut(w2, r2, Cdot);
			Cdot.addLocal(v2).subtractLocal(v1).subtractLocal(temp);
			m_mass.solve22ToOut(Cdot.negateLocal(), impulse); // just leave negated

			m_impulse.x += impulse.x;
			m_impulse.y += impulse.y;

			temp.set(impulse).scaleLocal(m1);
			v1.subtractLocal(temp);
			w1 -= i1 * MathUtils.cross(r1, impulse);

			temp.set(impulse).scaleLocal(m2);
			v2.addLocal(temp);
			w2 += i2 * MathUtils.cross(r2, impulse);

			pool.pushVector(2);
		}

		b1.m_angularVelocity = w1;
		b2.m_angularVelocity = w2;

		pool.pushVector(3);
	}

	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;

		// TODO_ERIN block solve with limit.

		float angularError = 0.0f;
		float positionError = 0.0f;

		// Solve angular limit constraint.
		if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
			final float angle = b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
			float limitImpulse = 0.0f;

			if (m_limitState == LimitState.EQUAL) {
				// Prevent large angular corrections
				final float C = MathUtils.clamp(angle - m_lowerAngle, -Settings.maxAngularCorrection,
						Settings.maxAngularCorrection);
				limitImpulse = -m_motorMass * C;
				angularError = MathUtils.abs(C);
			}
			else if (m_limitState == LimitState.AT_LOWER) {
				float C = angle - m_lowerAngle;
				angularError = -C;

				// Prevent large angular corrections and allow some slop.
				C = MathUtils.clamp(C + Settings.angularSlop, -Settings.maxAngularCorrection, 0.0f);
				limitImpulse = -m_motorMass * C;
			}
			else if (m_limitState == LimitState.AT_UPPER) {
				float C = angle - m_upperAngle;
				angularError = C;

				// Prevent large angular corrections and allow some slop.
				C = MathUtils.clamp(C - Settings.angularSlop, 0.0f, Settings.maxAngularCorrection);
				limitImpulse = -m_motorMass * C;
			}

			b1.m_sweep.a -= b1.m_invI * limitImpulse;
			b2.m_sweep.a += b2.m_invI * limitImpulse;

			b1.synchronizeTransform();
			b2.synchronizeTransform();
		}

		// Solve point-to-point constraint.
		{
			final Vector impulse = pool.popVector();

			final Vector r1 = pool.popVector();
			final Vector r2 = pool.popVector();
			final Vector C = pool.popVector();

			r1.set(m_localAnchor1).subtractLocal(b1.getLocalCenter());
			r2.set(m_localAnchor2).subtractLocal(b2.getLocalCenter());
			Mat22.mulToOut(b1.getTransform().R, r1, r1);
			Mat22.mulToOut(b2.getTransform().R, r2, r2);

			C.set(b2.m_sweep.c).addLocal(r2).subtractLocal(b1.m_sweep.c).subtractLocal(r1);
			positionError = C.length();

			final float invMass1 = b1.m_invMass, invMass2 = b2.m_invMass;
			final float invI1 = b1.m_invI, invI2 = b2.m_invI;

			// Handle large detachment.
			final float k_allowedStretch = 10.0f * Settings.linearSlop;
			if (C.lengthSq() > k_allowedStretch * k_allowedStretch) {
				final Vector u = pool.popVector();

				// Use a particle solution (no rotation).
				// u.set(C);
				// u.normalize(); ?? we don't even use this
				float m = invMass1 + invMass2;
				if (m > 0.0f) {
					m = 1.0f / m;
				}
				impulse.set(C).negateLocal().scaleLocal(m);
				final float k_beta = 0.5f;
				// using u as temp variable
				u.set(impulse).scaleLocal(k_beta * invMass1);
				b1.m_sweep.c.subtractLocal(u);
				u.set(impulse).scaleLocal(k_beta * invMass2);
				b2.m_sweep.c.addLocal(u);

				C.set(b2.m_sweep.c).addLocal(r2).subtractLocal(b1.m_sweep.c).subtractLocal(r1);

				pool.pushVector(1);
			}

			final Mat22 K1 = pool.popMat22();
			K1.m11 = invMass1 + invMass2;
			K1.m21 = 0.0f;
			K1.m12 = 0.0f;
			K1.m22 = invMass1 + invMass2;

			final Mat22 K2 = pool.popMat22();
			K2.m11 = invI1 * r1.y * r1.y;
			K2.m21 = -invI1 * r1.x * r1.y;
			K2.m12 = -invI1 * r1.x * r1.y;
			K2.m22 = invI1 * r1.x * r1.x;

			final Mat22 K3 = pool.popMat22();
			K3.m11 = invI2 * r2.y * r2.y;
			K3.m21 = -invI2 * r2.x * r2.y;
			K3.m12 = -invI2 * r2.x * r2.y;
			K3.m22 = invI2 * r2.x * r2.x;

			K1.addLocal(K2).addLocal(K3);
			K1.solveToOut(C.negateLocal(), impulse); // just leave c negated

			// using C as temp variable
			C.set(impulse).scaleLocal(b1.m_invMass);
			b1.m_sweep.c.subtractLocal(C);
			b1.m_sweep.a -= b1.m_invI * MathUtils.cross(r1, impulse);

			C.set(impulse).scaleLocal(b2.m_invMass);
			b2.m_sweep.c.addLocal(C);
			b2.m_sweep.a += b2.m_invI * MathUtils.cross(r2, impulse);

			b1.synchronizeTransform();
			b2.synchronizeTransform();

			pool.pushMat22(3);
			pool.pushVector(4);

		}

		return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
	}

	@Override
	public void getAnchorA(final Vector argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
	}

	@Override
	public void getAnchorB(final Vector argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
	}

	public void getReactionForce(final float inv_dt, final Vector argOut) {
		argOut.set(m_impulse.x, m_impulse.y).scaleLocal(inv_dt);
	}

	public float getReactionTorque(final float inv_dt) {
		return inv_dt * m_impulse.z;
	}

	public float getJointAngle() {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;
		return b2.m_sweep.a - b1.m_sweep.a - m_referenceAngle;
	}

	public float getJointSpeed() {
		final Body b1 = m_bodyA;
		final Body b2 = m_bodyB;
		return b2.m_angularVelocity - b1.m_angularVelocity;
	}

	public boolean isMotorEnabled() {
		return m_enableMotor;
	}

	public void enableMotor(final boolean flag) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableMotor = flag;
	}

	public float getMotorTorque(final float inv_dt) {
		return m_motorImpulse * inv_dt;
	}

	public void setMotorSpeed(final float speed) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_motorSpeed = speed;
	}

	public void setMaxMotorTorque(final float torque) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_maxMotorTorque = torque;
	}

	public boolean isLimitEnabled() {
		return m_enableLimit;
	}

	public void enableLimit(final boolean flag) {
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_enableLimit = flag;
	}

	public float getLowerLimit() {
		return m_lowerAngle;
	}

	public float getUpperLimit() {
		return m_upperAngle;
	}

	public void setLimits(final float lower, final float upper) {
		assert (lower <= upper);
		m_bodyA.setAwake(true);
		m_bodyB.setAwake(true);
		m_lowerAngle = lower;
		m_upperAngle = upper;
	}
}
