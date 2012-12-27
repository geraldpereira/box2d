package fr.byob.game.box2d.dynamics;

import pythagoras.f.Vector;
import fr.byob.game.box2d.collision.shapes.Shape;
import fr.byob.game.box2d.common.Transform;
import fr.byob.game.box2d.dynamics.contacts.ContactEdge;
import fr.byob.game.box2d.dynamics.joints.JointEdge;

public interface Body {
	/**
	 * Get the body transform for the body's origin.
	 * 
	 * @return the world transform of the body's origin.
	 */
	Transform getTransform();

	/**
	 * Set the position of the body's origin and rotation. This breaks any
	 * contacts and wakes the other bodies. Manipulating a body's transform may
	 * cause non-physical behavior.
	 * 
	 * @param position
	 *            the world position of the body's local origin.
	 * @param angle
	 *            the world rotation in radians.
	 */
	void setTransform(Vector position, float angle);

	/**
	 * Get the angle in radians.
	 * 
	 * @return the current world rotation angle in radians.
	 */
	float getAngle();

	/**
	 * Get the world body origin position.
	 * 
	 * @return the world position of the body's origin.
	 */
	Vector getPosition();

	/**
	 * Get the world position of the center of mass. Do not modify.
	 */
	Vector getWorldCenter();

	/**
	 * Get the world coordinates of a point given the local coordinates.
	 * 
	 * @param localPoint
	 *            a point on the body measured relative the the body's origin.
	 * @return the same point expressed in world coordinates.
	 */
	Vector getWorldPoint(Vector localPoint);

	void getWorldPointToOut(final Vector localPoint, final Vector out);

	/**
	 * Get the world coordinates of a vector given the local coordinates.
	 * 
	 * @param localVector
	 *            a vector fixed in the body.
	 * @return the same vector expressed in world coordinates.
	 */
	Vector getWorldVector(final Vector localVector);

	void getWorldVectorToOut(final Vector localVector, final Vector out);

	/**
	 * Does this body have fixed rotation?
	 * 
	 * @return
	 */
	boolean isFixedRotation();

	/**
	 * Set this body to have fixed rotation. This causes the mass to be reset.
	 * 
	 * @param flag
	 */
	void setFixedRotation(boolean b);

	/**
	 * Get the list of all fixtures attached to this body. Do not modify the
	 * list!
	 */
	Iterable<? extends Fixture> getFixtureList();

	/**
	 * Set the linear velocity of the center of mass.
	 * 
	 * @param v
	 *            the new linear velocity of the center of mass.
	 */
	public void setLinearVelocity(Vector vec2);

	/**
	 * Set the angular velocity.
	 * 
	 * @param w
	 *            the new angular velocity in radians/second.
	 */
	void setAngularVelocity(float f);

	/**
	 * Get the linear velocity of the center of mass. Do not modify, instead use
	 * {@link #setLinearVelocity(Vector)}.
	 * 
	 * @return the linear velocity of the center of mass.
	 */
	Vector getLinearVelocity();

	/**
	 * Get the angular velocity.
	 * 
	 * @return the angular velocity in radians/second.
	 */
	float getAngularVelocity();

	/** Get the user data pointer that was provided in the body definition. */
	Object getUserData();

	/**
	 * Set the user data. Use this to store your application specific data.
	 */
	void setUserData(Object object);

	/**
	 * Set the sleep state of the body. A sleeping body has very low CPU cost.
	 * 
	 * @param flag
	 *            set to true to put body to sleep, false to wake it.
	 */
	void setAwake(boolean flag);

	/**
	 * Get the sleeping state of this body.
	 * 
	 * @return true if the body is sleeping.
	 */
	boolean isAwake();

	/**
	 * Set the active state of the body. An inactive body is not simulated and
	 * cannot be collided with or woken up. If you pass a flag of true, all
	 * fixtures will be added to the broad-phase. If you pass a flag of false,
	 * all fixtures will be removed from the broad-phase and all contacts will
	 * be destroyed. Fixtures and joints are otherwise unaffected. You may
	 * continue to create/destroy fixtures and joints on inactive bodies.
	 * Fixtures on an inactive body are implicitly inactive and will not
	 * participate in collisions, ray-casts, or queries. Joints connected to an
	 * inactive body are implicitly inactive. An inactive body is still owned by
	 * a World object and remains in the body list.
	 * 
	 * @param flag
	 */
	void setActive(boolean flag);

	/**
	 * Get the active state of the body.
	 * 
	 * @return
	 */
	boolean isActive();

	/**
	 * Get the local position of the center of mass. Do not modify.
	 */
	Vector getLocalPoint(Vector anchor1);

	void getLocalPointToOut(Vector worldPoint, Vector out);

	/**
	 * Gets a local vector given a world vector.
	 * 
	 * @param worldVector
	 *            vector in world coordinates.
	 * @return the corresponding local vector.
	 */
	Vector getLocalVector(Vector axis);

	void getLocalVectorToOut(Vector worldVector, Vector out);

	/**
	 * Get the local position of the center of mass. Do not modify.
	 */
	Vector getLocalCenter();

	/**
	 * Apply a force at a world point. If the force is not applied at the center
	 * of mass, it will generate a torque and affect the angular velocity. This
	 * wakes up the body.
	 * 
	 * @param force
	 *            the world force vector, usually in Newtons (N).
	 * @param point
	 *            the world position of the point of application.
	 */
	void applyForce(Vector force, Vector point);

	/**
	 * Apply a torque. This affects the angular velocity without affecting the
	 * linear velocity of the center of mass. This wakes up the body.
	 * 
	 * @param torque
	 *            about the z-axis (out of the screen), usually in N-m.
	 */
	void applyTorque(float torque);

	/**
	 * Apply an impulse at a point. This immediately modifies the velocity. It
	 * also modifies the angular velocity if the point of application is not at
	 * the center of mass. This wakes up the body.
	 * 
	 * @param impulse
	 *            the world impulse vector, usually in N-seconds or kg-m/s.
	 * @param point
	 *            the world position of the point of application.
	 */
	void applyLinearImpulse(Vector impulse, Vector point);

	/**
	 * Apply an angular impulse.
	 * 
	 * @param impulse
	 *            the angular impulse in units of kg*m*m/s
	 */
	public void applyAngularImpulse(float impulse);

	/**
	 * Get the type of this body.
	 */
	BodyType getType();

	/**
	 * Set the type of this body. This may alter the mass and velocity.
	 */
	void setType(BodyType type);

	/**
	 * Get the total mass of the body.
	 * 
	 * @return the mass, usually in kilograms (kg).
	 */
	float getMass();

	/**
	 * This resets the mass properties to the sum of the mass properties of the
	 * fixtures. This normally does not need to be called unless you called
	 * SetMassData to override the mass and you later want to reset the mass.
	 */
	void resetMassData();

	/**
	 * Get the list of all joints attached to this body. Do not modify the list!
	 */
	Iterable<? extends JointEdge> getJointList();

	/**
	 * Creates a fixture and attach it to this body. Use this function if you
	 * need to set some fixture parameters, like friction. Otherwise you can
	 * create the fixture directly from a shape. If the density is non-zero,
	 * this function automatically updates the mass of the body. Contacts are
	 * not created until the next time step.
	 * 
	 * @param def
	 *            the fixture definition.
	 * @warning This function is locked during callbacks.
	 */
	Fixture createFixture(FixtureDef def);

	/**
	 * Creates a fixture from a shape and attach it to this body. This is a
	 * convenience function. Use FixtureDef if you need to set parameters like
	 * friction, restitution, user data, or filtering. If the density is
	 * non-zero, this function automatically updates the mass of the body.
	 * 
	 * @param shape
	 *            the shape to be cloned.
	 * @param density
	 *            the shape density (set to zero for static bodies).
	 * @warning This function is locked during callbacks.
	 */
	Fixture createFixture(Shape shape, float density);

	/**
	 * Destroy a fixture. This removes the fixture from the broad-phase and
	 * destroys all contacts associated with this fixture. This will
	 * automatically adjust the mass of the body if the body is dynamic and the
	 * fixture has positive density. All fixtures attached to a body are
	 * implicitly destroyed when the body is destroyed.
	 * 
	 * @param fixture
	 *            the fixture to be removed.
	 * @warning This function is locked during callbacks.
	 */
	void destroyFixture(Fixture fixture);

	/**
	 * Should this body be treated like a bullet for continuous collision
	 * detection?
	 */
	void setBullet(boolean flag);

	/**
	 * Is this body treated like a bullet for continuous collision detection?
	 */
	boolean isBullet();

	/**
	 * Get the angular damping of the body.
	 */
	float getAngularDamping();

	/**
	 * Set the angular damping of the body.
	 */
	void setAngularDamping(float angularDamping);

	/**
	 * Set the linear damping of the body.
	 */
	void setLinearDamping(float linearDamping);

	/**
	 * Get the linear damping of the body.
	 * */
	float getLinearDamping();

	ContactEdge getContactList();
}
