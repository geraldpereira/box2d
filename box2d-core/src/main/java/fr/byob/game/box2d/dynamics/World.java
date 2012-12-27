package fr.byob.game.box2d.dynamics;

import pythagoras.f.Vector;
import fr.byob.game.box2d.Disposable;
import fr.byob.game.box2d.callbacks.ContactFilter;
import fr.byob.game.box2d.callbacks.ContactListener;
import fr.byob.game.box2d.callbacks.DebugDraw;
import fr.byob.game.box2d.callbacks.DestructionListener;
import fr.byob.game.box2d.callbacks.QueryCallback;
import fr.byob.game.box2d.callbacks.RayCastCallback;
import fr.byob.game.box2d.collision.AABB;
import fr.byob.game.box2d.dynamics.joints.Joint;
import fr.byob.game.box2d.dynamics.joints.JointDef;

public interface World extends Disposable {

	/**
	 * Register a destruction listener. The listener is owned by you and must
	 * remain in scope.
	 * 
	 * @param listener
	 */
	public void setDestructionListener(DestructionListener listener);

	/**
	 * Register a contact filter to provide specific control over collision.
	 * Otherwise the default filter is used (_defaultFilter). The listener is
	 * owned by you and must remain in scope.
	 * 
	 * @param filter
	 */
	public void setContactFilter(ContactFilter filter);

	/**
	 * Register a contact event listener. The listener is owned by you and must
	 * remain in scope.
	 * 
	 * @param listener
	 */
	public void setContactListener(ContactListener listener);

	/**
	 * Register a routine for debug drawing. The debug draw functions are called
	 * inside with World.DrawDebugData method. The debug draw object is owned
	 * by you and must remain in scope.
	 * 
	 * @param debugDraw
	 */
	public void setDebugDraw(DebugDraw debugDraw);

	/**
	 * create a rigid body given a definition. No reference to the definition
	 * is retained.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param def
	 * @return
	 */
	public Body createBody(BodyDef def);

	/**
	 * destroy a rigid body given a definition. No reference to the definition
	 * is retained. This function is locked during callbacks.
	 * 
	 * @warning This automatically deletes all associated shapes and joints.
	 * @warning This function is locked during callbacks.
	 * @param body
	 */
	public void destroyBody(Body body);

	/**
	 * create a joint to constrain bodies together. No reference to the definition
	 * is retained. This may cause the connected bodies to cease colliding.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param def
	 * @return
	 */
	public Joint createJoint(JointDef def) ;

	/**
	 * destroy a joint. This may cause the connected bodies to begin colliding.
	 * 
	 * @warning This function is locked during callbacks.
	 * @param j
	 */
	public void destroyJoint(Joint j);

	/**
	 * Take a time step. This performs collision detection, integration,
	 * and constraint solution.
	 * 
	 * @param dt
	 *            the amount of time to simulate, this should not vary.
	 * @param velocityIterations
	 *            for the velocity constraint solver.
	 * @param positionIterations
	 *            for the position constraint solver.
	 */
	public void step(float dt, int velocityIterations, int positionIterations);

	/**
	 * Call this after you are done with time steps to clear the forces. You normally
	 * call this after each call to Step, unless you are performing sub-steps. By default,
	 * forces will be automatically cleared, so you don't need to call this function.
	 * 
	 * @see #clearForces()
	 */
	public void clearForces();
	/**
	 * Call this to draw shapes and other debug draw data.
	 */
	public void drawDebugData();

	/**
	 * Query the world for all fixtures that potentially overlap the
	 * provided AABB.
	 * 
	 * @param callback
	 *            a user implemented callback class.
	 * @param aabb
	 *            the query box.
	 */
	public void queryAABB(QueryCallback callback, AABB aabb) ;

	/**
	 * Ray-cast the world for all fixtures in the path of the ray. Your callback
	 * controls whether you get the closest point, any point, or n-points.
	 * The ray-cast ignores shapes that contain the starting point.
	 * 
	 * @param callback
	 *            a user implemented callback class.
	 * @param point1
	 *            the ray starting point
	 * @param point2
	 *            the ray ending point
	 */
	public void raycast(RayCastCallback callback, Vector point1, Vector point2);

	/**
	 * Get the world body list. With the returned body, use Body.getNext to get
	 * the next body in the world list. A null body indicates the end of the list.
	 * 
	 * @return the head of the world body list.
	 */
	public Iterable<? extends Body> getBodyList();

	/**
	 * Get the world joint list. With the returned joint, use Joint.getNext to get
	 * the next joint in the world list. A null joint indicates the end of the list.
	 * 
	 * @return the head of the world joint list.
	 */
	public Iterable<? extends Joint> getJointList();
	/**
	 * Get the world contact list. With the returned contact, use Contact.getNext to get
	 * the next contact in the world list. A null contact indicates the end of the list.
	 * 
	 * @return the head of the world contact list.
	 * @warning contacts are
	 */
	public Iterable<? extends Contact> getContactList();

	/**
	 * Enable/disable warm starting. For testing.
	 * 
	 * @param flag
	 */
	public void setWarmStarting(boolean flag);
	/**
	 * Enable/disable continuous physics. For testing.
	 * 
	 * @param flag
	 */
	public void setContinuousPhysics(boolean flag);

	/**
	 * Get the number of broad-phase proxies.
	 * 
	 * @return
	 */
	public int getProxyCount();

	/**
	 * Get the number of bodies.
	 * 
	 * @return
	 */
	public int getBodyCount() ;
	/**
	 * Get the number of joints.
	 * 
	 * @return
	 */
	public int getJointCount() ;

	/**
	 * Get the number of contacts (each may have 0 or more contact points).
	 * 
	 * @return
	 */
	public int getContactCount();

	/**
	 * Change the global gravity vector.
	 * 
	 * @param gravity
	 */
	public void setGravity(Vector gravity);

	/**
	 * Get the global gravity vector.
	 * 
	 * @return
	 */
	public Vector getGravity();

	/**
	 * Is the world locked (in the middle of a time step).
	 * 
	 * @return
	 */
	public boolean isLocked() ;
	/**
	 * Set flag to control automatic clearing of forces after each time step.
	 * 
	 * @param flag
	 */
	public void setAutoClearForces(boolean flag);

	/**
	 * Get the flag that controls automatic clearing of forces after each time step.
	 * 
	 * @return
	 */
	public boolean getAutoClearForces();

}
