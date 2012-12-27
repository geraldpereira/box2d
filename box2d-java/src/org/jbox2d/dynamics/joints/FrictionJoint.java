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
 * Created at 7:27:32 AM Jan 20, 2011
 */
package org.jbox2d.dynamics.joints;

import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.dynamics.joints.FrictionJointDef;

/**
 * @author Daniel Murphy
 */
public class FrictionJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.FrictionJoint {

	private final Vector m_localAnchorA;
	private final Vector m_localAnchorB;

	private final Mat22 m_linearMass;
	private float m_angularMass;

	private final Vector m_linearImpulse;
	private float m_angularImpulse;

	private float m_maxForce;
	private float m_maxTorque;

	/**
	 * @param argWorldPool
	 * @param def
	 */
	public FrictionJoint(final IWorldPool argWorldPool, final FrictionJointDef def) {
		super(argWorldPool, def);
		m_localAnchorA = new Vector(def.localAnchorA);
		m_localAnchorB = new Vector(def.localAnchorB);

		m_linearImpulse = new Vector();
		m_angularImpulse = 0.0f;

		m_maxForce = def.maxForce;
		m_maxTorque = def.maxTorque;

		m_linearMass = new Mat22();
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorA(org.jbox2d.common.Vector)
	 */
	@Override
	public void getAnchorA(final Vector argOut) {
		m_bodyA.getWorldPointToOut(m_localAnchorA, argOut);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getAnchorB(org.jbox2d.common.Vector)
	 */
	@Override
	public void getAnchorB(final Vector argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchorB, argOut);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float,
	 *      org.jbox2d.common.Vector)
	 */
	public void getReactionForce(final float inv_dt, final Vector argOut) {
		argOut.set(m_linearImpulse).scaleLocal(inv_dt);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	public float getReactionTorque(final float inv_dt) {
		return inv_dt * m_angularImpulse;
	}

	public void setMaxForce(final float force) {
		assert (force >= 0.0f);
		m_maxForce = force;
	}

	public float getMaxForce() {
		return m_maxForce;
	}

	public void setMaxTorque(final float torque) {
		assert (torque >= 0.0f);
		m_maxTorque = torque;
	}

	public float getMaxTorque() {
		return m_maxTorque;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#initVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body bA = m_bodyA;
		final Body bB = m_bodyB;

		// Compute the effective mass matrix.
		final Vector rA = pool.popVector();
		final Vector rB = pool.popVector();

		rA.set(m_localAnchorA).subtractLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subtractLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);

		// J = [-I -r1_skew I r2_skew]
		// [ 0 -1 0 1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
		// [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
		// [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]

		final float mA = bA.m_invMass, mB = bB.m_invMass;
		final float iA = bA.m_invI, iB = bB.m_invI;

		final Mat22 K1 = pool.popMat22();
		K1.m11 = mA + mB;
		K1.m21 = 0.0f;
		K1.m12 = 0.0f;
		K1.m22 = mA + mB;

		final Mat22 K2 = pool.popMat22();
		K2.m11 = iA * rA.y * rA.y;
		K2.m21 = -iA * rA.x * rA.y;
		K2.m12 = -iA * rA.x * rA.y;
		K2.m22 = iA * rA.x * rA.x;

		final Mat22 K3 = pool.popMat22();
		K3.m11 = iB * rB.y * rB.y;
		K3.m21 = -iB * rB.x * rB.y;
		K3.m12 = -iB * rB.x * rB.y;
		K3.m22 = iB * rB.x * rB.x;

		K1.addLocal(K2).addLocal(K3);
		m_linearMass.set(K1).invertLocal();

		m_angularMass = iA + iB;
		if (m_angularMass > 0.0f) {
			m_angularMass = 1.0f / m_angularMass;
		}

		if (step.warmStarting) {
			// Scale impulses to support a variable time step.
			m_linearImpulse.scaleLocal(step.dtRatio);
			m_angularImpulse *= step.dtRatio;

			final Vector P = pool.popVector();
			P.set(m_linearImpulse.x, m_linearImpulse.y);

			final Vector temp = pool.popVector();
			temp.set(P).scaleLocal(mA);
			bA.m_linearVelocity.subtractLocal(temp);
			bA.m_angularVelocity -= iA * (MathUtils.cross(rA, P) + m_angularImpulse);

			temp.set(P).scaleLocal(mB);
			bB.m_linearVelocity.addLocal(temp);
			bB.m_angularVelocity += iB * (MathUtils.cross(rB, P) + m_angularImpulse);

			pool.pushVector(2);
		}
		else {
			m_linearImpulse.set(0,0);
			m_angularImpulse = 0.0f;
		}

		pool.pushVector(2);
		pool.pushMat22(3);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solveVelocityConstraints(org.jbox2d.dynamics.TimeStep)
	 */
	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body bA = m_bodyA;
		final Body bB = m_bodyB;

		final Vector vA = bA.m_linearVelocity;
		float wA = bA.m_angularVelocity;
		final Vector vB = bB.m_linearVelocity;
		float wB = bB.m_angularVelocity;

		final float mA = bA.m_invMass, mB = bB.m_invMass;
		final float iA = bA.m_invI, iB = bB.m_invI;

		final Vector rA = pool.popVector();
		final Vector rB = pool.popVector();

		rA.set(m_localAnchorA).subtractLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subtractLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);

		// Solve angular friction
		{
			final float Cdot = wB - wA;
			float impulse = -m_angularMass * Cdot;

			final float oldImpulse = m_angularImpulse;
			final float maxImpulse = step.dt * m_maxTorque;
			m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve linear friction
		{
			final Vector Cdot = pool.popVector();
			final Vector temp = pool.popVector();

			MathUtils.crossToOut(wA, rA, temp);
			MathUtils.crossToOut(wB, rB, Cdot);
			Cdot.addLocal(vB).subtractLocal(vA).subtractLocal(temp);

			final Vector impulse = pool.popVector();
			Mat22.mulToOut(m_linearMass, Cdot, impulse);
			impulse.negateLocal();


			final Vector oldImpulse = pool.popVector();
			oldImpulse.set(m_linearImpulse);
			m_linearImpulse.addLocal(impulse);

			final float maxImpulse = step.dt * m_maxForce;

			if (m_linearImpulse.lengthSq() > maxImpulse * maxImpulse) {
				MathUtils.normalize(m_linearImpulse);
				m_linearImpulse.scaleLocal(maxImpulse);
			}

			impulse.set(m_linearImpulse).subtractLocal(oldImpulse);

			temp.set(impulse).scaleLocal(mA);
			vA.subtractLocal(temp);
			wA -= iA * MathUtils.cross(rA, impulse);

			temp.set(impulse).scaleLocal(mB);
			vB.addLocal(temp);
			wB += iB * MathUtils.cross(rB, impulse);
		}

		pool.pushVector(6);

		bA.m_angularVelocity = wA;
		bB.m_angularVelocity = wB;
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		return true;
	}
}
