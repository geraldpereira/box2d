/*******************************************************************************
 * Copyright 2011 See AUTHORS file.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *   http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package com.badlogic.gdx.physics.box2d;

import java.util.ArrayList;

import pythagoras.f.Vector;
import fr.byob.game.box2d.collision.shapes.MassData;
import fr.byob.game.box2d.dynamics.BodyType;
import fr.byob.game.box2d.dynamics.FixtureDef;
import fr.byob.game.box2d.dynamics.joints.JointEdge;

/** A rigid body. These are created via World.CreateBody.
 * @author mzechner */
public class Body implements fr.byob.game.box2d.dynamics.Body {
	// @off
	/*JNI
#include <Box2D/Box2D.h>
	 */

	/** the address of the body **/
	protected long addr;

	private final ContactEdge contactEdge;

	/** temporary float array **/
	private final float[] tmp = new float[4];

	/** World **/
	private final World world;

	/** Fixtures of this body **/
	private ArrayList<Fixture> fixtures = new ArrayList<Fixture>(2);

	/** Joints of this body **/
	protected ArrayList<JointEdge> joints = new ArrayList<JointEdge>(2);

	/** user data **/
	private Object userData;

	/** Constructs a new body with the given address
	 * @param world the world
	 * @param addr the address */
	protected Body (final World world, final long addr) {
		this.world = world;
		this.addr = addr;
		this.contactEdge = new ContactEdge(world);
	}

	/** Resets this body after fetching it from the {@link World#freeBodies} Pool. */
	protected void reset (final long addr) {
		this.addr = addr;
		this.userData = null;
		for (int i = 0; i < fixtures.size(); i++) {
			this.world.freeFixtures.free(fixtures.get(i));
		}
		fixtures.clear();
		this.joints.clear();
	}

	/** Creates a fixture and attach it to this body. Use this function if you need to set some fixture parameters, like friction.
	 * Otherwise you can create the fixture directly from a shape. If the density is non-zero, this function automatically updates
	 * the mass of the body. Contacts are not created until the next time step.
	 * @param def the fixture definition.
	 * @warning This function is locked during callbacks. */
	public Fixture createFixture (final FixtureDef def) {
		final long fixtureAddr = jniCreateFixture(addr, ((Shape) def.shape).addr, def.friction, def.restitution, def.density, def.isSensor,
				def.filter.categoryBits, def.filter.maskBits, def.filter.groupIndex);
		final Fixture fixture = (Fixture) this.world.freeFixtures.obtain();
		fixture.reset(this, fixtureAddr);
		this.world.fixtures.put(fixture.addr, fixture);
		this.fixtures.add(fixture);
		return fixture;
	}

	private native long jniCreateFixture (long addr, long shapeAddr, float friction, float restitution, float density,
			boolean isSensor, short filterCategoryBits, short filterMaskBits, short filterGroupIndex); /*
	b2Body* body = (b2Body*)addr;
	b2Shape* shape = (b2Shape*)shapeAddr;
	b2FixtureDef fixtureDef;

	fixtureDef.shape = shape;
	fixtureDef.friction = friction;
	fixtureDef.restitution = restitution;
	fixtureDef.density = density;
	fixtureDef.isSensor = isSensor;
	fixtureDef.filter.maskBits = filterMaskBits;
	fixtureDef.filter.categoryBits = filterCategoryBits;
	fixtureDef.filter.groupIndex = filterGroupIndex;

	return (jlong)body->CreateFixture( &fixtureDef );
			 */

	/** Creates a fixture from a shape and attach it to this body. This is a convenience function. Use b2FixtureDef if you need to
	 * set parameters like friction, restitution, user data, or filtering. If the density is non-zero, this function automatically
	 * updates the mass of the body.
	 * @param shape the shape to be cloned.
	 * @param density the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks. */
	public Fixture createFixture(final fr.byob.game.box2d.collision.shapes.Shape shape, final float density) {
		final long fixtureAddr = jniCreateFixture(addr, ((Shape) shape).addr, density);
		final Fixture fixture = (Fixture) this.world.freeFixtures.obtain();
		fixture.reset(this, fixtureAddr);
		this.world.fixtures.put(fixture.addr, fixture);
		this.fixtures.add(fixture);
		return fixture;
	}

	private native long jniCreateFixture (long addr, long shapeAddr, float density); /*
		b2Body* body = (b2Body*)addr;
		b2Shape* shape = (b2Shape*)shapeAddr;
		return (jlong)body->CreateFixture( shape, density );
	 */

	/** Destroy a fixture. This removes the fixture from the broad-phase and destroys all contacts associated with this fixture.
	 * This will automatically adjust the mass of the body if the body is dynamic and the fixture has positive density. All
	 * fixtures attached to a body are implicitly destroyed when the body is destroyed.
	 * @param fixture the fixture to be removed.
	 * @warning This function is locked during callbacks. */
	public void destroyFixture(final fr.byob.game.box2d.dynamics.Fixture fixtureArg) {
		final Fixture fixture = (Fixture) fixtureArg;
		jniDestroyFixture(addr, fixture.addr);
		this.world.fixtures.remove(fixture.addr);
		this.fixtures.remove(fixture);
		this.world.freeFixtures.free(fixture);
	}

	private native void jniDestroyFixture (long addr, long fixtureAddr); /*
		b2Body* body = (b2Body*)addr;
		b2Fixture* fixture = (b2Fixture*)fixtureAddr;
		body->DestroyFixture(fixture);
	 */

	/** Set the position of the body's origin and rotation. This breaks any contacts and wakes the other bodies. Manipulating a
	 * body's transform may cause non-physical behavior.
	 * @param position the world position of the body's local origin.
	 * @param angle the world rotation in radians. */

	public void setTransform (final Vector position, final float angle) {
		jniSetTransform(addr, position.x, position.y, angle);
	}

	/** Set the position of the body's origin and rotation. This breaks any contacts and wakes the other bodies. Manipulating a
	 * body's transform may cause non-physical behavior.
	 * @param x the world position on the x-axis
	 * @param y the world position on the y-axis
	 * @param angle the world rotation in radians. */
	public void setTransform (final float x, final float y, final float angle) {
		jniSetTransform(addr, x, y, angle);
	}

	/** Set the position of the body's origin and rotation. This breaks any contacts and wakes the other bodies. Manipulating a
	 * body's transform may cause non-physical behavior.
	 * @param x the world position on the x-axis
	 * @param y the world position on the y-axis
	 * @param angle the world rotation in radians.
	 * @param updateContacts Box2D SetTransform internally calls contactManager.FindNewContacts() method, sometimes multiple
	 * bodies are updated and it is undesirable to trigger Box2D to find new contacts each time, updateContacts should be
	 * false in those cases, true otherwise, more information at <a href="http://box2d.org/forum/viewtopic.php?f=3&t=8757">Box2d forums</a>. */
	public void setTransform (final float x, final float y, final float angle, final boolean updateContacts) {
		jniSetTransform(addr, x, y, angle, updateContacts);
	}

	private native void jniSetTransform (long addr, float positionX, float positionY, float angle); /*
		b2Body* body = (b2Body*)addr;
		body->SetTransform(b2Vec2(positionX, positionY), angle);
	 */

	private native void jniSetTransform (long addr, float positionX, float positionY, float angle, boolean updateContacts); /*
		b2Body* body = (b2Body*)addr;
		body->SetTransform(b2Vec2(positionX, positionY), angle, updateContacts);
	 */

	/** Get the body transform for the body's origin. */
	private final Transform transform = new Transform();

	public Transform getTransform () {
		jniGetTransform(addr, transform.vals);
		return transform;
	}

	private native void jniGetTransform (long addr, float[] vals); /*
		b2Body* body = (b2Body*)addr;
		b2Transform t = body->GetTransform();
		vals[0] = t.p.x;
		vals[1] = t.p.y;
		vals[2] = t.q.c;
		vals[3] = t.q.s;
	 */

	private final Vector position = new Vector();

	/** Get the world body origin position.
	 * @return the world position of the body's origin. */

	public Vector getPosition () {
		jniGetPosition(addr, tmp);
		position.x = tmp[0];
		position.y = tmp[1];
		return position;
	}

	private native void jniGetPosition (long addr, float[] position); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 p = body->GetPosition();
		position[0] = p.x;
		position[1] = p.y;
	 */

	/** Get the angle in radians.
	 * @return the current world rotation angle in radians. */

	public float getAngle () {
		return jniGetAngle(addr);
	}

	private native float jniGetAngle (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetAngle();
	 */

	/** Get the world position of the center of mass. */
	private final Vector worldCenter = new Vector();


	public Vector getWorldCenter () {
		jniGetWorldCenter(addr, tmp);
		worldCenter.x = tmp[0];
		worldCenter.y = tmp[1];
		return worldCenter;
	}

	private native void jniGetWorldCenter (long addr, float[] worldCenter); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetWorldCenter();
		worldCenter[0] = w.x;
		worldCenter[1] = w.y;
	 */

	/** Get the local position of the center of mass. */
	private final Vector localCenter = new Vector();


	public Vector getLocalCenter () {
		jniGetLocalCenter(addr, tmp);
		localCenter.x = tmp[0];
		localCenter.y = tmp[1];
		return localCenter;
	}

	private native void jniGetLocalCenter (long addr, float[] localCenter); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetLocalCenter();
		localCenter[0] = w.x;
		localCenter[1] = w.y;
	 */

	/** Set the linear velocity of the center of mass. */

	public void setLinearVelocity (final Vector v) {
		jniSetLinearVelocity(addr, v.x, v.y);
	}

	/** Set the linear velocity of the center of mass. */
	public void setLinearVelocity (final float vX, final float vY) {
		jniSetLinearVelocity(addr, vX, vY);
	}

	private native void jniSetLinearVelocity (long addr, float x, float y); /*
		b2Body* body = (b2Body*)addr;
		body->SetLinearVelocity(b2Vec2(x, y));
	 */

	/** Get the linear velocity of the center of mass. */
	private final Vector linearVelocity = new Vector();


	public Vector getLinearVelocity () {
		jniGetLinearVelocity(addr, tmp);
		linearVelocity.x = tmp[0];
		linearVelocity.y = tmp[1];
		return linearVelocity;
	}

	private native void jniGetLinearVelocity (long addr, float[] linearVelocity); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 l = body->GetLinearVelocity();
		linearVelocity[0] = l.x;
		linearVelocity[1] = l.y;
	 */

	/** Set the angular velocity. */

	public void setAngularVelocity (final float omega) {
		jniSetAngularVelocity(addr, omega);
	}

	private native void jniSetAngularVelocity (long addr, float omega); /*
		b2Body* body = (b2Body*)addr;
		body->SetAngularVelocity(omega);
	 */

	/** Get the angular velocity. */

	public float getAngularVelocity () {
		return jniGetAngularVelocity(addr);
	}

	private native float jniGetAngularVelocity (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetAngularVelocity();
	 */

	/** Apply a force at a world point. If the force is not applied at the center of mass, it will generate a torque and affect the
	 * angular velocity. This wakes up the body.
	 * @param force the world force vector, usually in Newtons (N).
	 * @param point the world position of the point of application. */

	public void applyForce (final Vector force, final Vector point) {
		jniApplyForce(addr, force.x, force.y, point.x, point.y);
	}

	/** Apply a force at a world point. If the force is not applied at the center of mass, it will generate a torque and affect the
	 * angular velocity. This wakes up the body.
	 * @param forceX the world force vector on x, usually in Newtons (N).
	 * @param forceY the world force vector on y, usually in Newtons (N).
	 * @param pointX the world position of the point of application on x.
	 * @param pointY the world position of the point of application on y. */
	public void applyForce (final float forceX, final float forceY, final float pointX, final float pointY) {
		jniApplyForce(addr, forceX, forceY, pointX, pointY);
	}

	private native void jniApplyForce (long addr, float forceX, float forceY, float pointX, float pointY); /*
		b2Body* body = (b2Body*)addr;
		body->ApplyForce(b2Vec2(forceX, forceY), b2Vec2(pointX, pointY));
	 */

	/** Apply a force to the center of mass. This wakes up the body.
	 * @param force the world force vector, usually in Newtons (N). */
	public void applyForceToCenter (final Vector force) {
		jniApplyForceToCenter(addr, force.x, force.y);
	}

	/** Apply a force to the center of mass. This wakes up the body.
	 * @param forceX the world force vector, usually in Newtons (N).
	 * @param forceY the world force vector, usually in Newtons (N). */
	public void applyForceToCenter (final float forceX, final float forceY) {
		jniApplyForceToCenter(addr, forceX, forceY);
	}

	private native void jniApplyForceToCenter (long addr, float forceX, float forceY); /*
		b2Body* body = (b2Body*)addr;
		body->ApplyForceToCenter(b2Vec2(forceX, forceY));
	 */

	/** Apply a torque. This affects the angular velocity without affecting the linear velocity of the center of mass. This wakes up
	 * the body.
	 * @param torque about the z-axis (out of the screen), usually in N-m. */

	public void applyTorque (final float torque) {
		jniApplyTorque(addr, torque);
	}

	private native void jniApplyTorque (long addr, float torque); /*
		b2Body* body = (b2Body*)addr;
		body->ApplyTorque(torque);
	 */

	/** Apply an impulse at a point. This immediately modifies the velocity. It also modifies the angular velocity if the point of
	 * application is not at the center of mass. This wakes up the body.
	 * @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point the world position of the point of application. */

	public void applyLinearImpulse (final Vector impulse, final Vector point) {
		jniApplyLinearImpulse(addr, impulse.x, impulse.y, point.x, point.y);
	}

	/** Apply an impulse at a point. This immediately modifies the velocity. It also modifies the angular velocity if the point of
	 * application is not at the center of mass. This wakes up the body.
	 * @param impulseX the world impulse vector on the x-axis, usually in N-seconds or kg-m/s.
	 * @param impulseY the world impulse vector on the y-axis, usually in N-seconds or kg-m/s.
	 * @param pointX the world position of the point of application on the x-axis.
	 * @param pointY the world position of the point of application on the y-axis. */
	public void applyLinearImpulse (final float impulseX, final float impulseY, final float pointX, final float pointY) {
		jniApplyLinearImpulse(addr, impulseX, impulseY, pointX, pointY);
	}

	private native void jniApplyLinearImpulse (long addr, float impulseX, float impulseY, float pointX, float pointY); /*
		b2Body* body = (b2Body*)addr;
		body->ApplyLinearImpulse( b2Vec2( impulseX, impulseY ), b2Vec2( pointX, pointY ) );
	 */

	/** Apply an angular impulse.
	 * @param impulse the angular impulse in units of kg*m*m/s */

	public void applyAngularImpulse (final float impulse) {
		jniApplyAngularImpulse(addr, impulse);
	}

	private native void jniApplyAngularImpulse (long addr, float impulse); /*
		b2Body* body = (b2Body*)addr;
		body->ApplyAngularImpulse(impulse);
	 */

	/** Get the total mass of the body.
	 * @return the mass, usually in kilograms (kg). */

	public float getMass () {
		return jniGetMass(addr);
	}

	private native float jniGetMass (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetMass();
	 */

	/** Get the rotational inertia of the body about the local origin.
	 * @return the rotational inertia, usually in kg-m^2. */
	public float getInertia () {
		return jniGetInertia(addr);
	}

	private native float jniGetInertia (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetInertia();
	 */

	private final MassData massData = new MassData();

	/** Get the mass data of the body.
	 * @return a struct containing the mass, inertia and center of the body. */
	public MassData getMassData () {
		jniGetMassData(addr, tmp);
		massData.mass = tmp[0];
		massData.center.x = tmp[1];
		massData.center.y = tmp[2];
		massData.I = tmp[3];
		return massData;
	}

	private native void jniGetMassData (long addr, float[] massData); /*
		b2Body* body = (b2Body*)addr;
		b2MassData m;
		body->GetMassData(&m);
		massData[0] = m.mass;
		massData[1] = m.center.x;
		massData[2] = m.center.y;
		massData[3] = m.I;
	 */

	/** Set the mass properties to override the mass properties of the fixtures. Note that this changes the center of mass position.
	 * Note that creating or destroying fixtures can also alter the mass. This function has no effect if the body isn't dynamic.
	 * @param data the mass properties. */
	public void setMassData (final MassData data) {
		jniSetMassData(addr, data.mass, data.center.x, data.center.y, data.I);
	}

	private native void jniSetMassData (long addr, float mass, float centerX, float centerY, float I); /*
		b2Body* body = (b2Body*)addr;
		b2MassData m;
		m.mass = mass;
		m.center.x = centerX;
		m.center.y = centerY;
		m.I = I;
		body->SetMassData(&m);
	 */

	/** This resets the mass properties to the sum of the mass properties of the fixtures. This normally does not need to be called
	 * unless you called SetMassData to override the mass and you later want to reset the mass. */

	public void resetMassData () {
		jniResetMassData(addr);
	}

	private native void jniResetMassData (long addr); /*
		b2Body* body = (b2Body*)addr;
		body->ResetMassData();
	 */

	private final Vector localPoint = new Vector();

	/** Get the world coordinates of a point given the local coordinates.
	 * @param localPoint a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates. */

	public Vector getWorldPoint (final Vector localPoint) {
		jniGetWorldPoint(addr, localPoint.x, localPoint.y, tmp);
		this.localPoint.x = tmp[0];
		this.localPoint.y = tmp[1];
		return this.localPoint;
	}

	public void getWorldPointToOut(final Vector localPoint, final Vector out) {
		jniGetWorldPoint(addr, localPoint.x, localPoint.y, tmp);
		out.x = tmp[0];
		out.y = tmp[1];
	}

	private native void jniGetWorldPoint (long addr, float localPointX, float localPointY, float[] worldPoint); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetWorldPoint( b2Vec2( localPointX, localPointY ) );
		worldPoint[0] = w.x;
		worldPoint[1] = w.y;
	 */

	private final Vector worldVector = new Vector();

	/** Get the world coordinates of a vector given the local coordinates.
	 * @param localVector a vector fixed in the body.
	 * @return the same vector expressed in world coordinates. */
	public Vector getWorldVector (final Vector localVector) {
		jniGetWorldVector(addr, localVector.x, localVector.y, tmp);
		worldVector.x = tmp[0];
		worldVector.y = tmp[1];
		return worldVector;
	}

	public void getWorldVectorToOut(final Vector localVector, final Vector out) {
		jniGetWorldVector(addr, localVector.x, localVector.y, tmp);
		out.x = tmp[0];
		out.y = tmp[1];
	}

	private native void jniGetWorldVector (long addr, float localVectorX, float localVectorY, float[] worldVector); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetWorldVector( b2Vec2( localVectorX, localVectorY ) );
		worldVector[0] = w.x;
		worldVector[1] = w.y;
	 */

	public final Vector localPoint2 = new Vector();

	/** Gets a local point relative to the body's origin given a world point.
	 * @param worldPoint a point in world coordinates.
	 * @return the corresponding local point relative to the body's origin. */

	public Vector getLocalPoint (final Vector worldPoint) {
		getLocalPointToOut(worldPoint, localPoint2);
		return localPoint2;
	}


	public void getLocalPointToOut(final Vector worldPoint, final Vector out) {
		jniGetLocalPoint(addr, worldPoint.x, worldPoint.y, tmp);
		out.x = tmp[0];
		out.y = tmp[1];
	}

	private native void jniGetLocalPoint (long addr, float worldPointX, float worldPointY, float[] localPoint); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetLocalPoint( b2Vec2( worldPointX, worldPointY ) );
		localPoint[0] = w.x;
		localPoint[1] = w.y;
	 */

	public final Vector localVector = new Vector();

	/** Gets a local vector given a world vector.
	 * @param worldVector a vector in world coordinates.
	 * @return the corresponding local vector. */

	public Vector getLocalVector (final Vector worldVector) {
		getLocalVectorToOut(worldVector, localVector);
		return localVector;
	}


	public void getLocalVectorToOut(final Vector worldVector, final Vector out) {
		jniGetLocalVector(addr, worldVector.x, worldVector.y, tmp);
		out.x = tmp[0];
		out.y = tmp[1];
	}


	private native void jniGetLocalVector (long addr, float worldVectorX, float worldVectorY, float[] worldVector); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetLocalVector( b2Vec2( worldVectorX, worldVectorY ) );
		worldVector[0] = w.x;
		worldVector[1] = w.y;
	 */

	public final Vector linVelWorld = new Vector();

	/** Get the world linear velocity of a world point attached to this body.
	 * @param worldPoint a point in world coordinates.
	 * @return the world velocity of a point. */
	public Vector getLinearVelocityFromWorldPoint (final Vector worldPoint) {
		jniGetLinearVelocityFromWorldPoint(addr, worldPoint.x, worldPoint.y, tmp);
		linVelWorld.x = tmp[0];
		linVelWorld.y = tmp[1];
		return linVelWorld;
	}

	private native void jniGetLinearVelocityFromWorldPoint (long addr, float worldPointX, float worldPointY, float[] linVelWorld); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetLinearVelocityFromWorldPoint( b2Vec2( worldPointX, worldPointY ) );
		linVelWorld[0] = w.x;
		linVelWorld[1] = w.y;
	 */

	public final Vector linVelLoc = new Vector();

	/** Get the world velocity of a local point.
	 * @param localPoint a point in local coordinates.
	 * @return the world velocity of a point. */
	public Vector getLinearVelocityFromLocalPoint (final Vector localPoint) {
		jniGetLinearVelocityFromLocalPoint(addr, localPoint.x, localPoint.y, tmp);
		linVelLoc.x = tmp[0];
		linVelLoc.y = tmp[1];
		return linVelLoc;
	}

	private native void jniGetLinearVelocityFromLocalPoint (long addr, float localPointX, float localPointY, float[] linVelLoc); /*
		b2Body* body = (b2Body*)addr;
		b2Vec2 w = body->GetLinearVelocityFromLocalPoint( b2Vec2( localPointX, localPointY ) );
		linVelLoc[0] = w.x;
		linVelLoc[1] = w.y;
	 */

	/** Get the linear damping of the body. */

	public float getLinearDamping () {
		return jniGetLinearDamping(addr);
	}

	private native float jniGetLinearDamping (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetLinearDamping();
	 */

	/** Set the linear damping of the body. */

	public void setLinearDamping (final float linearDamping) {
		jniSetLinearDamping(addr, linearDamping);
	}

	private native void jniSetLinearDamping (long addr, float linearDamping); /*
		b2Body* body = (b2Body*)addr;
		body->SetLinearDamping(linearDamping);
	 */

	/** Get the angular damping of the body. */

	public float getAngularDamping () {
		return jniGetAngularDamping(addr);
	}

	private native float jniGetAngularDamping (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetAngularDamping();
	 */

	/** Set the angular damping of the body. */

	public void setAngularDamping (final float angularDamping) {
		jniSetAngularDamping(addr, angularDamping);
	}

	private native void jniSetAngularDamping (long addr, float angularDamping); /*
		b2Body* body = (b2Body*)addr;
		body->SetAngularDamping(angularDamping);
	 */

	/** Set the type of this body. This may alter the mass and velocity. */

	public void setType (final BodyType type) {
		jniSetType(addr, type.getValue());
	}

	// @off
	/*JNI
inline b2BodyType getBodyType( int type )
{
	switch( type )
	{
	case 0: return b2_staticBody;
	case 1: return b2_kinematicBody;
	case 2: return b2_dynamicBody;
	default:
		return b2_staticBody;
	}
}
	 */

	private native void jniSetType (long addr, int type); /*
		b2Body* body = (b2Body*)addr;
		body->SetType(getBodyType(type));
	 */

	/** Get the type of this body. */

	public BodyType getType () {
		final int type = jniGetType(addr);
		if (type == 0) {
			return BodyType.STATIC;
		}
		if (type == 1) {
			return BodyType.KINEMATIC;
		}
		if (type == 2) {
			return BodyType.DYNAMIC;
		}
		return BodyType.STATIC;
	}

	private native int jniGetType (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetType();
	 */

	/** Should this body be treated like a bullet for continuous collision detection? */

	public void setBullet (final boolean flag) {
		jniSetBullet(addr, flag);
	}

	private native void jniSetBullet (long addr, boolean flag); /*
		b2Body* body = (b2Body*)addr;
		body->SetBullet(flag);
	 */

	/** Is this body treated like a bullet for continuous collision detection? */

	public boolean isBullet () {
		return jniIsBullet(addr);
	}

	private native boolean jniIsBullet (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->IsBullet();
	 */

	/** You can disable sleeping on this body. If you disable sleeping, the */
	public void setSleepingAllowed (final boolean flag) {
		jniSetSleepingAllowed(addr, flag);
	}

	private native void jniSetSleepingAllowed (long addr, boolean flag); /*
		b2Body* body = (b2Body*)addr;
		body->SetSleepingAllowed(flag);
	 */

	/** Is this body allowed to sleep */
	public boolean isSleepingAllowed () {
		return jniIsSleepingAllowed(addr);
	}

	private native boolean jniIsSleepingAllowed (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->IsSleepingAllowed();
	 */

	/** Set the sleep state of the body. A sleeping body has very low CPU cost.
	 * @param flag set to true to put body to sleep, false to wake it. */

	public void setAwake (final boolean flag) {
		jniSetAwake(addr, flag);
	}

	private native void jniSetAwake (long addr, boolean flag); /*
		b2Body* body = (b2Body*)addr;
		body->SetAwake(flag);
	 */

	/** Get the sleeping state of this body.
	 * @return true if the body is sleeping. */

	public boolean isAwake () {
		return jniIsAwake(addr);
	}

	private native boolean jniIsAwake (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->IsAwake();
	 */

	/** Set the active state of the body. An inactive body is not simulated and cannot be collided with or woken up. If you pass a
	 * flag of true, all fixtures will be added to the broad-phase. If you pass a flag of false, all fixtures will be removed from
	 * the broad-phase and all contacts will be destroyed. Fixtures and joints are otherwise unaffected. You may continue to
	 * create/destroy fixtures and joints on inactive bodies. Fixtures on an inactive body are implicitly inactive and will not
	 * participate in collisions, ray-casts, or queries. Joints connected to an inactive body are implicitly inactive. An inactive
	 * body is still owned by a b2World object and remains in the body list. */

	public void setActive (final boolean flag) {
		jniSetActive(addr, flag);
	}

	private native void jniSetActive (long addr, boolean flag); /*
		b2Body* body = (b2Body*)addr;
		body->SetActive(flag);
	 */

	/** Get the active state of the body. */

	public boolean isActive () {
		return jniIsActive(addr);
	}

	private native boolean jniIsActive (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->IsActive();
	 */

	/** Set this body to have fixed rotation. This causes the mass to be reset. */

	public void setFixedRotation (final boolean flag) {
		jniSetFixedRotation(addr, flag);
	}

	private native void jniSetFixedRotation (long addr, boolean flag); /*
		b2Body* body = (b2Body*)addr;
		body->SetFixedRotation(flag);
	 */

	/** Does this body have fixed rotation? */

	public boolean isFixedRotation () {
		return jniIsFixedRotation(addr);
	}

	private native boolean jniIsFixedRotation (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->IsFixedRotation();
	 */

	/** Get the list of all fixtures attached to this body. Do not modify the list! */

	public Iterable<? extends fr.byob.game.box2d.dynamics.Fixture> getFixtureList() {
		return fixtures;
	}

	/** Get the list of all joints attached to this body. Do not modify the list! */

	public ArrayList<JointEdge> getJointList () {
		return joints;
	}

	/** Get the list of all contacts attached to this body.
	 * @warning this list changes during the time step and you may miss some collisions if you don't use b2ContactListener. Do not
	 *          modify the returned list! */
	// ArrayList<ContactEdge> getContactList()
	// {
	// return contacts;
	// }

	/** @return Get the gravity scale of the body. */
	public float getGravityScale () {
		return jniGetGravityScale(addr);
	}

	private native float jniGetGravityScale (long addr); /*
		b2Body* body = (b2Body*)addr;
		return body->GetGravityScale();
	 */

	/** Sets the gravity scale of the body */
	public void setGravityScale (final float scale) {
		jniSetGravityScale(addr, scale);
	}

	private native void jniSetGravityScale (long addr, float scale); /*
		b2Body* body = (b2Body*)addr;
		body->SetGravityScale(scale);
	 */

	public ContactEdge getContactList() {
		final long contactEdgeAddr = jniGetContactList(addr);
		contactEdge.setAddr(contactEdgeAddr);
		return contactEdge;
	}

	private native long jniGetContactList(long addr);/*
		b2Body* body = (b2Body*)addr;
		return body->m_contactList;
	 */

	/** Get the parent world of this body. */
	public World getWorld () {
		return world;
	}

	/** Get the user data */

	public Object getUserData () {
		return userData;
	}

	/** Set the user data */
	public void setUserData (final Object userData) {
		this.userData = userData;
	}


}
