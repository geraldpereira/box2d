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
import java.util.List;

import pythagoras.f.Vector;

import com.badlogic.gdx.physics.box2d.joints.DistanceJoint;
import com.badlogic.gdx.physics.box2d.joints.FrictionJoint;
import com.badlogic.gdx.physics.box2d.joints.GearJoint;
import com.badlogic.gdx.physics.box2d.joints.MouseJoint;
import com.badlogic.gdx.physics.box2d.joints.PrismaticJoint;
import com.badlogic.gdx.physics.box2d.joints.PulleyJoint;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJoint;
import com.badlogic.gdx.physics.box2d.joints.RopeJoint;
import com.badlogic.gdx.physics.box2d.joints.WeldJoint;
import com.badlogic.gdx.physics.box2d.joints.WheelJoint;
import com.badlogic.gdx.utils.Disposable;
import com.badlogic.gdx.utils.LongMap;
import com.badlogic.gdx.utils.Pool;

import fr.byob.game.box2d.callbacks.ContactFilter;
import fr.byob.game.box2d.callbacks.ContactListener;
import fr.byob.game.box2d.callbacks.DebugDraw;
import fr.byob.game.box2d.callbacks.DestructionListener;
import fr.byob.game.box2d.callbacks.QueryCallback;
import fr.byob.game.box2d.callbacks.RayCastCallback;
import fr.byob.game.box2d.collision.AABB;
import fr.byob.game.box2d.dynamics.BodyDef;
import fr.byob.game.box2d.dynamics.Filter;
import fr.byob.game.box2d.dynamics.Fixture;
import fr.byob.game.box2d.dynamics.joints.DistanceJointDef;
import fr.byob.game.box2d.dynamics.joints.FrictionJointDef;
import fr.byob.game.box2d.dynamics.joints.GearJointDef;
import fr.byob.game.box2d.dynamics.joints.JointDef;
import fr.byob.game.box2d.dynamics.joints.JointEdge;
import fr.byob.game.box2d.dynamics.joints.JointType;
import fr.byob.game.box2d.dynamics.joints.MouseJointDef;
import fr.byob.game.box2d.dynamics.joints.PrismaticJointDef;
import fr.byob.game.box2d.dynamics.joints.PulleyJointDef;
import fr.byob.game.box2d.dynamics.joints.RevoluteJointDef;
import fr.byob.game.box2d.dynamics.joints.RopeJointDef;
import fr.byob.game.box2d.dynamics.joints.WeldJointDef;
import fr.byob.game.box2d.dynamics.joints.WheelJointDef;

/** The world class manages all physics entities, dynamic simulation, and asynchronous queries. The world also contains efficient
 * memory management facilities.
 * @author mzechner */
public final class World implements Disposable, fr.byob.game.box2d.dynamics.World {
	// @off
	/*JNI
#include <Box2D/Box2D.h>

static jclass worldClass = 0;
static jmethodID shouldCollideID = 0;
static jmethodID beginContactID = 0;
static jmethodID endContactID = 0;
static jmethodID preSolveID = 0;
static jmethodID postSolveID = 0;
static jmethodID reportFixtureID = 0;
static jmethodID reportRayFixtureID = 0;

class CustomRayCastCallback: public b2RayCastCallback
{
private:
	JNIEnv* env;
	jobject obj;

public:
	CustomRayCastCallback( JNIEnv *env, jobject obj )
	{
		this->env = env;
		this->obj = obj;
	}

	virtual float32 ReportFixture( b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		return env->CallFloatMethod(obj, reportRayFixtureID, (jlong)fixture, (jfloat)point.x, (jfloat)point.y,
																(jfloat)normal.x, (jfloat)normal.y, (jfloat)fraction );
	}
};

class CustomContactFilter: public b2ContactFilter
{
private:
	JNIEnv* env;
	jobject obj;

public:
	CustomContactFilter( JNIEnv* env, jobject obj )
	{
		this->env = env;
		this->obj = obj;
	}

	virtual bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB)
	{
		if( shouldCollideID != 0 )
			return env->CallBooleanMethod( obj, shouldCollideID, (jlong)fixtureA, (jlong)fixtureB );
		else
			return true;
	}
};

class CustomContactListener: public b2ContactListener
{
private:
	JNIEnv* env;
	jobject obj;

public:
		CustomContactListener( JNIEnv* env, jobject obj )
		{
			this->env = env;
			this->obj = obj;
		}

		/// Called when two fixtures begin to touch.
		virtual void BeginContact(b2Contact* contact)
		{
			if( beginContactID != 0 )
				env->CallVoidMethod(obj, beginContactID, (jlong)contact );
		}

		/// Called when two fixtures cease to touch.
		virtual void EndContact(b2Contact* contact)
		{
			if( endContactID != 0 )
				env->CallVoidMethod(obj, endContactID, (jlong)contact);
		}

		/// This is called after a contact is updated.
		virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
		{
			if( preSolveID != 0 )
				env->CallVoidMethod(obj, preSolveID, (jlong)contact, (jlong)oldManifold);
		}

		/// This lets you inspect a contact after the solver is finished.
		virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
		{
			if( postSolveID != 0 )
				env->CallVoidMethod(obj, postSolveID, (jlong)contact, (jlong)impulse);
		}
};

class CustomQueryCallback: public b2QueryCallback
{
private:
	JNIEnv* env;
	jobject obj;

public:
	CustomQueryCallback( JNIEnv* env, jobject obj )
	{
		this->env = env;
		this->obj = obj;
	}

	virtual bool ReportFixture( b2Fixture* fixture )
	{
		return env->CallBooleanMethod(obj, reportFixtureID, (jlong)fixture );
	}
};

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

b2ContactFilter defaultFilter;
	 */

	/** pool for bodies **/
	protected final Pool<Body> freeBodies = new Pool<Body>(100, 200) {
		@Override
		protected Body newObject () {
			return new Body(World.this, 0);
		}
	};

	/** pool for fixtures **/
	protected final Pool<Fixture> freeFixtures = new Pool<Fixture>(100, 200) {
		@Override
		protected Fixture newObject () {
			return new com.badlogic.gdx.physics.box2d.Fixture(null, 0);
		}
	};

	/** the address of the world instance **/
	private final long addr;

	/** all known bodies **/
	protected final LongMap<Body> bodies = new LongMap<Body>(100);

	/** all known fixtures **/
	protected final LongMap<Fixture> fixtures = new LongMap<Fixture>(100);

	/** all known joints **/
	protected final LongMap<Joint> joints = new LongMap<Joint>(100);

	/** Contact filter **/
	protected ContactFilter contactFilter = null;

	/** Contact listener **/
	protected ContactListener contactListener = null;

	/** Construct a world object.
	 * @param gravity the world gravity vector.
	 * @param doSleep improve performance by not simulating inactive bodies. */
	public World (final Vector gravity, final boolean doSleep) {
		addr = newWorld(gravity.x, gravity.y, doSleep);

		contacts.ensureCapacity(contactAddrs.length);
		freeContacts.ensureCapacity(contactAddrs.length);

		for (final long contactAddr : contactAddrs) {
			freeContacts.add(new Contact(this, 0));
		}
	}

	private native long newWorld (float gravityX, float gravityY, boolean doSleep); /*
		// we leak one global ref.
		if(!worldClass) {
			worldClass = (jclass)env->NewGlobalRef(env->GetObjectClass(object));
			beginContactID = env->GetMethodID(worldClass, "beginContact", "(J)V" );
			endContactID = env->GetMethodID( worldClass, "endContact", "(J)V" );
			preSolveID = env->GetMethodID( worldClass, "preSolve", "(JJ)V" );
			postSolveID = env->GetMethodID( worldClass, "postSolve", "(JJ)V" );
			reportFixtureID = env->GetMethodID(worldClass, "reportFixture", "(J)Z" );
			reportRayFixtureID = env->GetMethodID(worldClass, "reportRayFixture", "(JFFFFF)F" );
			shouldCollideID = env->GetMethodID( worldClass, "contactFilter", "(JJ)Z");
		}

		b2World* world = new b2World( b2Vec2( gravityX, gravityY ), doSleep );
		return (jlong)world;
	 */

	/** Register a destruction listener. The listener is owned by you and must remain in scope. */
	public void setDestructionListener (final DestructionListener listener) {

	}

	/** Register a contact filter to provide specific control over collision. Otherwise the default filter is used
	 * (b2_defaultFilter). The listener is owned by you and must remain in scope. */
	public void setContactFilter (final ContactFilter filter) {
		this.contactFilter = filter;
		setUseDefaultContactFilter(filter == null);
	}

	/** tells the native code not to call the Java world class if use is false **/
	private native void setUseDefaultContactFilter(boolean use); /*
		// FIXME
	 */

	/** Register a contact event listener. The listener is owned by you and must remain in scope. */
	public void setContactListener (final ContactListener listener) {
		this.contactListener = listener;
	}

	/** Create a rigid body given a definition. No reference to the definition is retained.
	 * @warning This function is locked during callbacks. */
	public Body createBody (final BodyDef def) {
		final long bodyAddr = jniCreateBody(addr, def.type.getValue(), def.position.x, def.position.y, def.angle, def.linearVelocity.x,
				def.linearVelocity.y, def.angularVelocity, def.linearDamping, def.angularDamping, def.allowSleep, def.awake,
				def.fixedRotation, def.bullet, def.active, def.gravityScale);
		final Body body = freeBodies.obtain();
		body.reset(bodyAddr);
		this.bodies.put(body.addr, body);
		return body;
	}

	private native long jniCreateBody (long addr, int type, float positionX, float positionY, float angle, float linearVelocityX,
			float linearVelocityY, float angularVelocity, float linearDamping, float angularDamping, boolean allowSleep, boolean awake,
			boolean fixedRotation, boolean bullet, boolean active, float inertiaScale); /*
		b2BodyDef bodyDef;
		bodyDef.type = getBodyType(type);
		bodyDef.position.Set( positionX, positionY );
		bodyDef.angle = angle;
		bodyDef.linearVelocity.Set( linearVelocityX, linearVelocityY );
		bodyDef.angularVelocity = angularVelocity;
		bodyDef.linearDamping = linearDamping;
		bodyDef.angularDamping = angularDamping;
		bodyDef.allowSleep = allowSleep;
		bodyDef.awake = awake;
		bodyDef.fixedRotation = fixedRotation;
		bodyDef.bullet = bullet;
		bodyDef.active = active;
		bodyDef.gravityScale = inertiaScale;

		b2World* world = (b2World*)addr;
		b2Body* body = world->CreateBody( &bodyDef );
		return (jlong)body;
			 */

	/** Destroy a rigid body given a definition. No reference to the definition is retained. This function is locked during
	 * callbacks.
	 * @warning This automatically deletes all associated shapes and joints.
	 * @warning This function is locked during callbacks. */
	public void destroyBody(final fr.byob.game.box2d.dynamics.Body bodyArg) {
		final Body body = (Body) bodyArg;
		body.setUserData(null);
		this.bodies.remove(body.addr);
		final List<Fixture> fixtureList = (List<Fixture>) body.getFixtureList();
		while(!fixtureList.isEmpty()) {
			this.fixtures.remove(((com.badlogic.gdx.physics.box2d.Fixture) fixtureList.remove(0)).addr).setUserData(null);
		}
		final List<JointEdge> jointList = body.getJointList();
		while (!jointList.isEmpty()) {
			destroyJoint(body.getJointList().get(0).joint);
		}
		jniDestroyBody(addr, body.addr);
		freeBodies.free(body);
	}

	private native void jniDestroyBody (long addr, long bodyAddr); /*
		b2World* world = (b2World*)addr;
		b2Body* body = (b2Body*)bodyAddr;
		world->DestroyBody(body);
	 */

	/** Create a joint to constrain bodies together. No reference to the definition is retained. This may cause the connected bodies
	 * to cease colliding.
	 * @warning This function is locked during callbacks. */
	public Joint createJoint (final JointDef def) {
		final long jointAddr = createProperJoint(def);
		Joint joint = null;
		if (def.type == JointType.DISTANCE) {
			joint = new DistanceJoint(this, jointAddr);
		}
		if (def.type == JointType.FRICTION) {
			joint = new FrictionJoint(this, jointAddr);
		}
		if (def.type == JointType.GEAR) {
			joint = new GearJoint(this, jointAddr);
		}
		if (def.type == JointType.MOUSE) {
			joint = new MouseJoint(this, jointAddr);
		}
		if (def.type == JointType.PRISMATIC) {
			joint = new PrismaticJoint(this, jointAddr);
		}
		if (def.type == JointType.PULLEY) {
			joint = new PulleyJoint(this, jointAddr);
		}
		if (def.type == JointType.REVOLUTE) {
			joint = new RevoluteJoint(this, jointAddr);
		}
		if (def.type == JointType.WELD) {
			joint = new WeldJoint(this, jointAddr);
		}
		if (def.type == JointType.ROPE) {
			joint = new RopeJoint(this, jointAddr);
		}
		if (def.type == JointType.WHEEL) {
			joint = new WheelJoint(this, jointAddr);
		}
		if (joint != null) {
			joints.put(joint.addr, joint);
		}
		final JointEdge jointEdgeA = new JointEdge(def.bodyB, joint);
		final JointEdge jointEdgeB = new JointEdge(def.bodyA, joint);
		joint.jointEdgeA = jointEdgeA;
		joint.jointEdgeB = jointEdgeB;
		((Body) def.bodyA).joints.add(jointEdgeA);
		((Body) def.bodyB).joints.add(jointEdgeB);
		return joint;
	}

	private long createProperJoint (final JointDef def) {

		final Body bodyA = (Body) def.bodyA;
		final Body bodyB = (Body) def.bodyB;

		if (def.type == JointType.DISTANCE) {
			final DistanceJointDef d = (DistanceJointDef)def;
			return jniCreateDistanceJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.length, d.frequencyHz, d.dampingRatio);
		}
		if (def.type == JointType.FRICTION) {
			final FrictionJointDef d = (FrictionJointDef)def;
			return jniCreateFrictionJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.maxForce, d.maxTorque);
		}
		if (def.type == JointType.GEAR) {
			final GearJointDef d = (GearJointDef)def;
			return jniCreateGearJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, ((Joint) d.joint1).addr, ((Joint) d.joint2).addr, d.ratio);
		}
		if (def.type == JointType.MOUSE) {
			final MouseJointDef d = (MouseJointDef)def;
			return jniCreateMouseJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.target.x, d.target.y, d.maxForce,
					d.frequencyHz, d.dampingRatio);
		}
		if (def.type == JointType.PRISMATIC) {
			final PrismaticJointDef d = (PrismaticJointDef)def;
			return jniCreatePrismaticJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.localAxis1.x, d.localAxis1.y, d.referenceAngle, d.enableLimit,
					d.lowerTranslation, d.upperTranslation, d.enableMotor, d.maxMotorForce, d.motorSpeed);
		}
		if (def.type == JointType.PULLEY) {
			final PulleyJointDef d = (PulleyJointDef)def;
			return jniCreatePulleyJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.groundAnchorA.x, d.groundAnchorA.y,
					d.groundAnchorB.x, d.groundAnchorB.y, d.localAnchorA.x, d.localAnchorA.y, d.localAnchorB.x, d.localAnchorB.y,
					d.lengthA, d.lengthB, d.ratio);

		}
		if (def.type == JointType.REVOLUTE) {
			final RevoluteJointDef d = (RevoluteJointDef)def;
			return jniCreateRevoluteJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.referenceAngle, d.enableLimit, d.lowerAngle, d.upperAngle, d.enableMotor,
					d.motorSpeed, d.maxMotorTorque);
		}
		if (def.type == JointType.WELD) {
			final WeldJointDef d = (WeldJointDef)def;
			return jniCreateWeldJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.referenceAngle);
		}
		if (def.type == JointType.ROPE) {
			final RopeJointDef d = (RopeJointDef)def;
			return jniCreateRopeJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.maxLength);
		}
		if (def.type == JointType.WHEEL) {
			final WheelJointDef d = (WheelJointDef)def;
			return jniCreateWheelJoint(addr, bodyA.addr, bodyB.addr, d.collideConnected, d.localAnchorA.x, d.localAnchorA.y,
					d.localAnchorB.x, d.localAnchorB.y, d.localAxisA.x, d.localAxisA.y, d.enableMotor, d.maxMotorTorque, d.motorSpeed,
					d.frequencyHz, d.dampingRatio);
		}

		return 0;
	}

	private native long jniCreateWheelJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float localAxisAX, float localAxisAY, boolean enableMotor,
			float maxMotorTorque, float motorSpeed, float frequencyHz, float dampingRatio); /*
		b2World* world = (b2World*)addr;
		b2WheelJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.localAxisA = b2Vec2(localAxisAX, localAxisAY);
		def.enableMotor = enableMotor;
		def.maxMotorTorque = maxMotorTorque;
		def.motorSpeed = motorSpeed;
		def.frequencyHz = frequencyHz;
		def.dampingRatio = dampingRatio;

		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateRopeJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float maxLength); /*
		b2World* world = (b2World*)addr;
		b2RopeJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.maxLength = maxLength;

		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateDistanceJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float length, float frequencyHz, float dampingRatio); /*
		b2World* world = (b2World*)addr;
		b2DistanceJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.length = length;
		def.frequencyHz = frequencyHz;
		def.dampingRatio = dampingRatio;

		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateFrictionJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float maxForce, float maxTorque); /*
		b2World* world = (b2World*)addr;
		b2FrictionJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.maxForce = maxForce;
		def.maxTorque = maxTorque;
		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateGearJoint (long addr, long bodyA, long bodyB, boolean collideConnected, long joint1, long joint2,
			float ratio); /*
		b2World* world = (b2World*)addr;
		b2GearJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.joint1 = (b2Joint*)joint1;
		def.joint2 = (b2Joint*)joint2;
		def.ratio = ratio;
		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateMouseJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float targetX,
			float targetY, float maxForce, float frequencyHz, float dampingRatio); /*
		b2World* world = (b2World*)addr;
		b2MouseJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.target = b2Vec2( targetX, targetY );
		def.maxForce = maxForce;
		def.frequencyHz = frequencyHz;
		def.dampingRatio = dampingRatio;
		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreatePrismaticJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float localAxisAX, float localAxisAY, float referenceAngle,
			boolean enableLimit, float lowerTranslation, float upperTranslation, boolean enableMotor, float maxMotorForce,
			float motorSpeed); /*
		b2World* world = (b2World*)addr;
		b2PrismaticJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.localAxisA = b2Vec2( localAxisAX, localAxisAY );
		def.referenceAngle = referenceAngle;
		def.enableLimit = enableLimit;
		def.lowerTranslation = lowerTranslation;
		def.upperTranslation = upperTranslation;
		def.enableMotor = enableMotor;
		def.maxMotorForce = maxMotorForce;
		def.motorSpeed = motorSpeed;
		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreatePulleyJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float groundAnchorAX,
			float groundAnchorAY, float groundAnchorBX, float groundAnchorBY, float localAnchorAX, float localAnchorAY,
			float localAnchorBX, float localAnchorBY, float lengthA, float lengthB, float ratio); /*
		b2World* world = (b2World*)addr;
		b2PulleyJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.groundAnchorA = b2Vec2( groundAnchorAX, groundAnchorAY );
		def.groundAnchorB = b2Vec2( groundAnchorBX, groundAnchorBY );
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.lengthA = lengthA;
		def.lengthB = lengthB;
		def.ratio = ratio;

		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateRevoluteJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float referenceAngle, boolean enableLimit, float lowerAngle,
			float upperAngle, boolean enableMotor, float motorSpeed, float maxMotorTorque); /*
		b2World* world = (b2World*)addr;
		b2RevoluteJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.referenceAngle = referenceAngle;
		def.enableLimit = enableLimit;
		def.lowerAngle = lowerAngle;
		def.upperAngle = upperAngle;
		def.enableMotor = enableMotor;
		def.motorSpeed = motorSpeed;
		def.maxMotorTorque = maxMotorTorque;
		return (jlong)world->CreateJoint(&def);
			 */

	private native long jniCreateWeldJoint (long addr, long bodyA, long bodyB, boolean collideConnected, float localAnchorAX,
			float localAnchorAY, float localAnchorBX, float localAnchorBY, float referenceAngle); /*
		b2World* world = (b2World*)addr;
		b2WeldJointDef def;
		def.bodyA = (b2Body*)bodyA;
		def.bodyB = (b2Body*)bodyB;
		def.collideConnected = collideConnected;
		def.localAnchorA = b2Vec2(localAnchorAX, localAnchorAY);
		def.localAnchorB = b2Vec2(localAnchorBX, localAnchorBY);
		def.referenceAngle = referenceAngle;

		return (jlong)world->CreateJoint(&def);
			 */

	/** Destroy a joint. This may cause the connected bodies to begin colliding.
	 * @warning This function is locked during callbacks. */
	public void destroyJoint(final fr.byob.game.box2d.dynamics.joints.Joint jointArg) {
		final Joint joint = (Joint) jointArg;
		joints.remove(joint.addr);
		((Body) joint.jointEdgeA.other).joints.remove(joint.jointEdgeB);
		((Body) joint.jointEdgeB.other).joints.remove(joint.jointEdgeA);
		jniDestroyJoint(addr, joint.addr);
	}

	private native void jniDestroyJoint (long addr, long jointAddr); /*
		b2World* world = (b2World*)addr;
		b2Joint* joint = (b2Joint*)jointAddr;
		world->DestroyJoint( joint );
	 */

	/** Take a time step. This performs collision detection, integration, and constraint solution.
	 * @param timeStep the amount of time to simulate, this should not vary.
	 * @param velocityIterations for the velocity constraint solver.
	 * @param positionIterations for the position constraint solver. */
	public void step (final float timeStep, final int velocityIterations, final int positionIterations) {
		jniStep(addr, timeStep, velocityIterations, positionIterations);
	}

	private native void jniStep (long addr, float timeStep, int velocityIterations, int positionIterations); /*
		b2World* world = (b2World*)addr;
		CustomContactFilter contactFilter(env, object);
		CustomContactListener contactListener(env,object);
		world->SetContactFilter(&contactFilter);
		world->SetContactListener(&contactListener);
		world->Step( timeStep, velocityIterations, positionIterations );
		world->SetContactFilter(&defaultFilter);
		world->SetContactListener(0);
	 */

	/** Manually clear the force buffer on all bodies. By default, forces are cleared automatically after each call to Step. The
	 * default behavior is modified by calling SetAutoClearForces. The purpose of this function is to support sub-stepping.
	 * Sub-stepping is often used to maintain a fixed sized time step under a variable frame-rate. When you perform sub-stepping
	 * you will disable auto clearing of forces and instead call ClearForces after all sub-steps are complete in one pass of your
	 * game loop. {@link #setAutoClearForces(boolean)} */
	public void clearForces () {
		jniClearForces(addr);
	}

	private native void jniClearForces (long addr); /*
		b2World* world = (b2World*)addr;
		world->ClearForces();
	 */

	/** Enable/disable warm starting. For testing. */
	public void setWarmStarting (final boolean flag) {
		jniSetWarmStarting(addr, flag);
	}

	private native void jniSetWarmStarting (long addr, boolean flag); /*
		b2World* world = (b2World*)addr;
		world->SetWarmStarting(flag);
	 */

	/** Enable/disable continuous physics. For testing. */
	public void setContinuousPhysics (final boolean flag) {
		jniSetContiousPhysics(addr, flag);
	}

	private native void jniSetContiousPhysics (long addr, boolean flag); /*
		b2World* world = (b2World*)addr;
		world->SetContinuousPhysics(flag);
	 */

	/** Get the number of broad-phase proxies. */
	public int getProxyCount () {
		return jniGetProxyCount(addr);
	}

	private native int jniGetProxyCount (long addr); /*
		b2World* world = (b2World*)addr;
		return world->GetProxyCount();
	 */

	/** Get the number of bodies. */
	public int getBodyCount () {
		return jniGetBodyCount(addr);
	}

	private native int jniGetBodyCount (long addr); /*
		b2World* world = (b2World*)addr;
		return world->GetBodyCount();
	 */

	/** Get the number of joints. */
	public int getJointCount () {
		return jniGetJointcount(addr);
	}

	private native int jniGetJointcount (long addr); /*
		b2World* world = (b2World*)addr;
		return world->GetJointCount();
	 */

	/** Get the number of contacts (each may have 0 or more contact points). */
	public int getContactCount () {
		return jniGetContactCount(addr);
	}

	private native int jniGetContactCount (long addr); /*
		b2World* world = (b2World*)addr;
		return world->GetContactCount();
	 */

	/** Change the global gravity vector. */
	public void setGravity (final Vector gravity) {
		jniSetGravity(addr, gravity.x, gravity.y);
	}

	private native void jniSetGravity (long addr, float gravityX, float gravityY); /*
		b2World* world = (b2World*)addr;
		world->SetGravity( b2Vec2( gravityX, gravityY ) );
	 */

	/** Get the global gravity vector. */
	final float[] tmpGravity = new float[2];
	final Vector gravity = new Vector();

	public Vector getGravity () {
		jniGetGravity(addr, tmpGravity);
		gravity.x = tmpGravity[0];
		gravity.y = tmpGravity[1];
		return gravity;
	}

	private native void jniGetGravity (long addr, float[] gravity); /*
		b2World* world = (b2World*)addr;
		b2Vec2 g = world->GetGravity();
		gravity[0] = g.x;
		gravity[1] = g.y;
	 */

	/** Is the world locked (in the middle of a time step). */
	public boolean isLocked () {
		return jniIsLocked(addr);
	}

	private native boolean jniIsLocked (long addr); /*
		b2World* world = (b2World*)addr;
		return world->IsLocked();
	 */

	/** Set flag to control automatic clearing of forces after each time step. */
	public void setAutoClearForces (final boolean flag) {
		jniSetAutoClearForces(addr, flag);
	}

	private native void jniSetAutoClearForces (long addr, boolean flag); /*
		b2World* world = (b2World*)addr;
		world->SetAutoClearForces(flag);
	 */

	/** Get the flag that controls automatic clearing of forces after each time step. */
	public boolean getAutoClearForces () {
		return jniGetAutoClearForces(addr);
	}

	private native boolean jniGetAutoClearForces (long addr); /*
		b2World* world = (b2World*)addr;
		return world->GetAutoClearForces();
	 */

	/** Query the world for all fixtures that potentially overlap the provided AABB.
	 * @param callback a user implemented callback class.
	 * @param lowerX the x coordinate of the lower left corner
	 * @param lowerY the y coordinate of the lower left corner
	 * @param upperX the x coordinate of the upper right corner
	 * @param upperY the y coordinate of the upper right corner */
	public void QueryAABB (final QueryCallback callback, final float lowerX, final float lowerY, final float upperX, final float upperY) {
		queryCallback = callback;
		jniQueryAABB(addr, lowerX, lowerY, upperX, upperY);
	}

	private QueryCallback queryCallback = null;;

	private native void jniQueryAABB (long addr, float lowX, float lowY, float upX, float upY); /*
		b2World* world = (b2World*)addr;
		b2AABB aabb;
		aabb.lowerBound = b2Vec2( lowX, lowY );
		aabb.upperBound = b2Vec2( upX, upY );

		CustomQueryCallback callback( env, object );
		world->QueryAABB( &callback, aabb );
	 */

	//
	// /// Ray-cast the world for all fixtures in the path of the ray. Your callback
	// /// controls whether you get the closest point, any point, or n-points.
	// /// The ray-cast ignores shapes that contain the starting point.
	// /// @param callback a user implemented callback class.
	// /// @param point1 the ray starting point
	// /// @param point2 the ray ending point
	// void RayCast(b2RayCastCallback* callback, const b2Vec2& point1, const b2Vec2& point2) const;
	//
	// /// Get the world contact list. With the returned contact, use b2Contact::GetNext to get
	// /// the next contact in the world list. A NULL contact indicates the end of the list.
	// /// @return the head of the world contact list.
	// /// @warning contacts are
	// b2Contact* GetContactList();

	private long[] contactAddrs = new long[200];
	private final ArrayList<Contact> contacts = new ArrayList<Contact>();
	private final ArrayList<Contact> freeContacts = new ArrayList<Contact>();

	/** Returns the list of {@link Contact} instances produced by the last call to {@link #step(float, int, int)}. Note that the
	 * returned list will have O(1) access times when using indexing. contacts are created and destroyed in the middle of a time
	 * step. Use {@link ContactListener} to avoid missing contacts
	 * @return the contact list */
	public Iterable<? extends fr.byob.game.box2d.dynamics.Contact> getContactList() {
		final int numContacts = getContactCount();
		if (numContacts > contactAddrs.length) {
			final int newSize = 2 * numContacts;
			contactAddrs = new long[newSize];
			contacts.ensureCapacity(newSize);
			freeContacts.ensureCapacity(newSize);
		}
		if (numContacts > freeContacts.size()) {
			final int freeConts = freeContacts.size();
			for (int i = 0; i < numContacts - freeConts; i++) {
				freeContacts.add(new Contact(this, 0));
			}
		}
		jniGetContactList(addr, contactAddrs);

		contacts.clear();
		for (int i = 0; i < numContacts; i++) {
			final Contact contact = freeContacts.get(i);
			contact.addr = contactAddrs[i];
			contacts.add(contact);
		}

		return contacts;
	}

	private native void jniGetContactList (long addr, long[] contacts); /*
		b2World* world = (b2World*)addr;

		b2Contact* contact = world->GetContactList();
		int i = 0;
		while( contact != 0 )
		{
			contacts[i++] = (long long)contact;
			contact = contact->GetNext();
		}
	 */

	public void dispose () {
		jniDispose(addr);
	}

	private native void jniDispose (long addr); /*
		b2World* world = (b2World*)(addr);
		delete world;
	 */

	/** Internal method called from JNI in case a contact happens
	 * @param fixtureA
	 * @param fixtureB
	 * @return whether the things collided */
	private boolean contactFilter (final long fixtureA, final long fixtureB) {
		if (contactFilter != null) {
			return contactFilter.shouldCollide(fixtures.get(fixtureA), fixtures.get(fixtureB));
		} else {
			final Filter filterA = fixtures.get(fixtureA).getFilterData();
			final Filter filterB = fixtures.get(fixtureB).getFilterData();

			if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0) {
				return filterA.groupIndex > 0;
			}

			final boolean collide = (filterA.maskBits & filterB.categoryBits) != 0 && (filterA.categoryBits & filterB.maskBits) != 0;
			return collide;
		}
	}

	private final Contact contact = new Contact(this, 0);
	private final Manifold manifold = new Manifold(0);
	private final ContactImpulse impulse = new ContactImpulse(this, 0);

	private void beginContact (final long contactAddr) {
		contact.addr = contactAddr;
		if (contactListener != null) {
			contactListener.beginContact(contact);
		}
	}

	private void endContact (final long contactAddr) {
		contact.addr = contactAddr;
		if (contactListener != null) {
			contactListener.endContact(contact);
		}
	}

	private void preSolve (final long contactAddr, final long manifoldAddr) {
		contact.addr = contactAddr;
		manifold.addr = manifoldAddr;
		if (contactListener != null) {
			contactListener.preSolve(contact, manifold);
		}
	}

	private void postSolve (final long contactAddr, final long impulseAddr) {
		contact.addr = contactAddr;
		impulse.addr = impulseAddr;
		if (contactListener != null) {
			contactListener.postSolve(contact, impulse);
		}
	}

	private boolean reportFixture (final long addr) {
		if (queryCallback != null) {
			return queryCallback.reportFixture(fixtures.get(addr));
		} else {
			return false;
		}
	}

	/** Sets the box2d velocity threshold globally, for all World instances.
	 * @param threshold the threshold, default 1.0f */
	public static native void setVelocityThreshold (float threshold); /*
		b2_velocityThreshold = threshold;
	 */

	/** @return the global box2d velocity threshold. */
	public static native float getVelocityThreshold (); /*
		return b2_velocityThreshold;
	 */

	/** Ray-cast the world for all fixtures in the path of the ray. The ray-cast ignores shapes that contain the starting point.
	 * @param callback a user implemented callback class.
	 * @param point1 the ray starting point
	 * @param point2 the ray ending point */
	public void raycast(final RayCastCallback callback, final Vector point1, final Vector point2) {
		rayCastCallback = callback;
		jniRayCast(addr, point1.x, point1.y, point2.x, point2.y);
	}

	private RayCastCallback rayCastCallback = null;

	private native void jniRayCast (long addr, float aX, float aY, float bX, float bY); /*
		b2World *world = (b2World*)addr;
		CustomRayCastCallback callback( env, object );
		world->RayCast( &callback, b2Vec2(aX,aY), b2Vec2(bX,bY) );
	 */

	private Vector rayPoint = new Vector();
	private Vector rayNormal = new Vector();

	private float reportRayFixture (final long addr, final float pX, final float pY, final float nX, final float nY, final float fraction) {
		if (rayCastCallback != null) {
			rayPoint.x = pX;
			rayPoint.y = pY;
			rayNormal.x = nX;
			rayNormal.y = nY;
			return rayCastCallback.reportFixture(fixtures.get(addr), rayPoint, rayNormal, fraction);
		} else {
			return 0.0f;
		}
	}

	public void setDebugDraw(final DebugDraw debugDraw) {
	}

	public void drawDebugData() {
	}

	public Iterable<? extends fr.byob.game.box2d.dynamics.Body> getBodyList() {
		return bodies.values();
	}

	public Iterable<? extends fr.byob.game.box2d.dynamics.joints.Joint> getJointList() {
		return joints.values();
	}

	public void queryAABB(final QueryCallback callback, final AABB aabb) {
		QueryAABB(callback, aabb.lowerBound.x, aabb.lowerBound.y, aabb.upperBound.x, aabb.upperBound.y);
	}
}
