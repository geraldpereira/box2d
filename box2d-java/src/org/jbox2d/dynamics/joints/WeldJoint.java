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
 * Created at 3:38:38 AM Jan 15, 2011
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
import fr.byob.game.box2d.dynamics.joints.WeldJointDef;

/**
 * @author Daniel Murphy
 */
public class WeldJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.WeldJoint {

	private final Vector m_localAnchorA;
	private final Vector m_localAnchorB;
	private float m_referenceAngle;

	private final Vector3 m_impulse;

	private final Matrix3 m_mass;

	/**
	 * @param argWorld
	 * @param def
	 */
	protected WeldJoint(final IWorldPool argWorld, final WeldJointDef def) {
		super(argWorld, def);
		m_localAnchorA = new Vector(def.localAnchorA);
		m_localAnchorB = new Vector(def.localAnchorB);
		m_referenceAngle = def.referenceAngle;

		m_impulse = new Vector3();
		m_impulse.set(0, 0, 0);

		m_mass = new Matrix3();
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
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float, org.jbox2d.common.Vector)
	 */
	public void getReactionForce(final float inv_dt, final Vector argOut) {
		argOut.set(m_impulse.x, m_impulse.y);
		argOut.scaleLocal(inv_dt);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
	 */
	public float getReactionTorque(final float inv_dt) {
		return inv_dt * m_impulse.z;
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
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		final float mA = bA.m_invMass, mB = bB.m_invMass;
		final float iA = bA.m_invI, iB = bB.m_invI;

		m_mass.col1.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		m_mass.col2.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		m_mass.col3.x = -rA.y * iA - rB.y * iB;
		m_mass.col1.y = m_mass.col2.x;
		m_mass.col2.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		m_mass.col3.y = rA.x * iA + rB.x * iB;
		m_mass.col1.z = m_mass.col3.x;
		m_mass.col2.z = m_mass.col3.y;
		m_mass.col3.z = iA + iB;

		if (step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			m_impulse.multLocal(step.dtRatio);

			final Vector P = pool.popVector();
			final Vector temp = pool.popVector();
			P.set(m_impulse.x, m_impulse.y);

			temp.set(P).scaleLocal(mA);
			bA.m_linearVelocity.subtractLocal(temp);
			bA.m_angularVelocity -= iA * (MathUtils.cross(rA, P) + m_impulse.z);

			temp.set(P).scaleLocal(mB);
			bB.m_linearVelocity.addLocal(temp);
			bB.m_angularVelocity += iB * (MathUtils.cross(rB, P) + m_impulse.z);

			pool.pushVector(2);
		}
		else
		{
			m_impulse.set(0, 0, 0);
		}
		pool.pushVector(2);
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


		final Vector Cdot1 = pool.popVector();
		final Vector temp = pool.popVector();
		// Solve point-to-point finalraint
		MathUtils.crossToOut(wA, rA, temp);
		MathUtils.crossToOut(wB, rB, Cdot1);
		Cdot1.addLocal(vB).subtractLocal(vA).subtractLocal(temp);
		final float Cdot2 = wB - wA;

		final Vector3 Cdot = pool.popVector3();
		Cdot.set(Cdot1.x, Cdot1.y, Cdot2);

		final Vector3 impulse = pool.popVector3();
		m_mass.solve33ToOut(Cdot.negateLocal(), impulse); // just leave Cdot negated..
		m_impulse.addLocal(impulse);

		final Vector P = pool.popVector();
		P.set(impulse.x, impulse.y);

		temp.set(P).scaleLocal(mA);
		vA.subtractLocal(temp);
		wA -= iA * (MathUtils.cross(rA, P) + impulse.z);

		temp.set(P).scaleLocal(mB);
		vB.addLocal(temp);
		wB += iB * (MathUtils.cross(rB, P) + impulse.z);

		bA.m_linearVelocity.set(vA);
		bA.m_angularVelocity = wA;
		bB.m_linearVelocity.set(vB);
		bB.m_angularVelocity = wB;

		pool.pushVector(5);
		pool.pushVector3(2);
	}

	/**
	 * @see org.jbox2d.dynamics.joints.Joint#solvePositionConstraints(float)
	 */
	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		final Body bA = m_bodyA;
		final Body bB = m_bodyB;

		final float mA = bA.m_invMass, mB = bB.m_invMass;
		float iA = bA.m_invI, iB = bB.m_invI;

		final Vector rA = pool.popVector();
		final Vector rB = pool.popVector();
		rA.set(m_localAnchorA).subtractLocal(bA.getLocalCenter());
		rB.set(m_localAnchorB).subtractLocal(bB.getLocalCenter());
		Mat22.mulToOut(bA.getTransform().R, rA, rA);
		Mat22.mulToOut(bB.getTransform().R, rB, rB);

		final Vector C1 = pool.popVector();
		C1.set(bB.m_sweep.c).addLocal(rB).subtractLocal(bA.m_sweep.c).subtractLocal(rA);
		final float C2 = bB.m_sweep.a - bA.m_sweep.a - m_referenceAngle;

		// Handle large detachment.
		final float k_allowedStretch = 10.0f * Settings.linearSlop;
		final float positionError = C1.length();
		final float angularError = MathUtils.abs(C2);
		if (positionError > k_allowedStretch){
			iA *= 1.0f;
			iB *= 1.0f;
		}

		m_mass.col1.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		m_mass.col2.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		m_mass.col3.x = -rA.y * iA - rB.y * iB;
		m_mass.col1.y = m_mass.col2.x;
		m_mass.col2.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		m_mass.col3.y = rA.x * iA + rB.x * iB;
		m_mass.col1.z = m_mass.col3.x;
		m_mass.col2.z = m_mass.col3.y;
		m_mass.col3.z = iA + iB;

		final Vector3 C = pool.popVector3();
		final Vector3 impulse = pool.popVector3();
		C.set(C1.x, C1.y, C2);

		m_mass.solve33ToOut(C.negateLocal(), impulse); // just leave c negated..

		final Vector P = pool.popVector();
		final Vector temp = pool.popVector();
		P.set(impulse.x, impulse.y);

		temp.set(P).scaleLocal(mA);
		bA.m_sweep.c.subtractLocal(temp);
		bA.m_sweep.a -= iA * (MathUtils.cross(rA, P) + impulse.z);

		temp.set(P).scaleLocal(mB);
		bB.m_sweep.c.addLocal(temp);
		bB.m_sweep.a += iB * (MathUtils.cross(rB, P) + impulse.z);

		bA.synchronizeTransform();
		bB.synchronizeTransform();

		pool.pushVector(5);
		pool.pushVector3(2);

		return positionError <= Settings.linearSlop && angularError <= Settings.angularSlop;
	}

}
