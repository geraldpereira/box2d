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

import org.jbox2d.common.Transform;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.pooling.IWorldPool;

import pythagoras.f.Vector;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.common.MathUtils;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.joints.MouseJointDef;

public class MouseJoint extends Joint implements fr.byob.game.box2d.dynamics.joints.MouseJoint {

	private final Vector m_localAnchor = new Vector();
	private final Vector m_target = new Vector();
	private final Vector m_impulse = new Vector();

	private final Mat22 m_mass = new Mat22();	// effective mass for point-to-point constraint.
	private final Vector m_C = new Vector();		// position error
	private float m_maxForce;
	private float m_frequencyHz;
	private float m_dampingRatio;
	private float m_beta;
	private float m_gamma;


	protected MouseJoint(final IWorldPool argWorld, final MouseJointDef def) {
		super(argWorld, def);
		assert (MathUtils.isValid(def.target));
		assert(def.maxForce >= 0);
		assert(def.frequencyHz >= 0);
		assert(def.dampingRatio >= 0);

		m_target.set(def.target);
		Transform.mulTransToOut(m_bodyB.getTransform(), m_target, m_localAnchor);

		m_maxForce = def.maxForce;
		m_impulse.set(0,0);

		m_frequencyHz = def.frequencyHz;
		m_dampingRatio = def.dampingRatio;

		m_beta = 0;
		m_gamma = 0;
	}

	@Override
	public void getAnchorA(final Vector argOut) {
		argOut.set(m_target);
	}

	@Override
	public void getAnchorB(final Vector argOut) {
		m_bodyB.getWorldPointToOut(m_localAnchor, argOut);
	}

	public void getReactionForce(final float invDt, final Vector argOut) {
		argOut.set(m_impulse).scaleLocal(invDt);
	}

	public float getReactionTorque(final float invDt) {
		return invDt * 0.0f;
	}


	public void setTarget( final Vector target){
		if(m_bodyB.isAwake() == false){
			m_bodyB.setAwake(true);
		}
		m_target.set(target);
	}
	public Vector getTarget(){
		return m_target;
	}

	/// set/get the maximum force in Newtons.
	public void setMaxForce(final float force){
		m_maxForce = force;
	}
	public float getMaxForce(){
		return m_maxForce;
	}

	/// set/get the frequency in Hertz.
	public void setFrequency(final float hz){
		m_frequencyHz = hz;
	}
	public float getFrequency(){
		return m_frequencyHz;
	}

	/// set/get the damping ratio (dimensionless).
	public void setDampingRatio(final float ratio){
		m_dampingRatio = ratio;
	}
	public float getDampingRatio(){
		return m_dampingRatio;
	}

	@Override
	public void initVelocityConstraints(final TimeStep step) {
		final Body b = m_bodyB;

		final float mass = b.getMass();

		// Frequency
		final float omega = 2.0f * MathUtils.PI * m_frequencyHz;

		// Damping coefficient
		final float d = 2.0f * mass * m_dampingRatio * omega;

		// Spring stiffness
		final float k = mass * (omega * omega);

		// magic formulas
		// gamma has units of inverse mass.
		// beta has units of inverse time.
		assert(d + step.dt * k > Settings.EPSILON);
		m_gamma = step.dt * (d + step.dt * k);
		if (m_gamma != 0.0f){
			m_gamma = 1.0f / m_gamma;
		}
		m_beta = step.dt * k * m_gamma;

		final Vector r = pool.popVector();

		// Compute the effective mass matrix.
		//Vec2 r = Mul(b.getTransform().R, m_localAnchor - b.getLocalCenter());
		r.set(m_localAnchor).subtractLocal(b.getLocalCenter());
		Mat22.mulToOut(b.getTransform().R, r, r);

		// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
		//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
		//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
		final float invMass = b.m_invMass;
		final float invI = b.m_invI;

		final Mat22 K1 = pool.popMat22();
		K1.m11 = invMass; K1.m21 = 0.0f;
		K1.m12 = 0.0f;    K1.m22 = invMass;

		final Mat22 K2 = pool.popMat22();
		K2.m11 =  invI * r.y * r.y; K2.m21 = -invI * r.x * r.y;
		K2.m12 = -invI * r.x * r.y; K2.m22 =  invI * r.x * r.x;

		final Mat22 K = pool.popMat22();
		K.set(K1).addLocal(K2);
		K.m11 += m_gamma;
		K.m22 += m_gamma;

		K.invertToOut(m_mass);

		m_C.set(b.m_sweep.c).addLocal(r).subtractLocal(m_target);

		// Cheat with some damping
		b.m_angularVelocity *= 0.98f;

		// Warm starting.
		m_impulse.scaleLocal(step.dtRatio);
		// pool
		final Vector temp = pool.popVector();
		temp.set(m_impulse).scaleLocal(invMass);
		b.m_linearVelocity.addLocal(temp);
		b.m_angularVelocity += invI * MathUtils.cross(r, m_impulse);

		pool.pushVector(2);
		pool.pushMat22(3);
	}

	@Override
	public boolean solvePositionConstraints(final float baumgarte) {
		return true;
	}

	@Override
	public void solveVelocityConstraints(final TimeStep step) {
		final Body b = m_bodyB;

		final Vector r = pool.popVector();

		r.set(m_localAnchor).subtractLocal(b.getLocalCenter());
		Mat22.mulToOut(b.getTransform().R, r, r);

		// Cdot = v + cross(w, r)
		final Vector Cdot = pool.popVector();
		MathUtils.crossToOut(b.m_angularVelocity, r, Cdot);
		Cdot.addLocal(b.m_linearVelocity);

		final Vector impulse = pool.popVector();
		final Vector temp = pool.popVector();

		//Mul(m_mass, -(Cdot + m_beta * m_C + m_gamma * m_impulse));
		impulse.set(m_C).scaleLocal(m_beta);
		temp.set(m_impulse).scaleLocal(m_gamma);
		temp.addLocal(impulse).addLocal(Cdot).scaleLocal(-1);
		Mat22.mulToOut(m_mass, temp, impulse);

		final Vector oldImpulse = temp;
		oldImpulse.set(m_impulse);
		m_impulse.addLocal(impulse);
		final float maxImpulse = step.dt * m_maxForce;
		if (m_impulse.lengthSq() > maxImpulse * maxImpulse){
			m_impulse.scaleLocal(maxImpulse / m_impulse.length());
		}
		impulse.set(m_impulse).subtractLocal(oldImpulse);

		// pooling
		oldImpulse.set(impulse).scaleLocal(b.m_invMass);
		b.m_linearVelocity.addLocal(oldImpulse);
		b.m_angularVelocity += b.m_invI * MathUtils.cross(r, impulse);

		pool.pushVector(4);
	}

}
