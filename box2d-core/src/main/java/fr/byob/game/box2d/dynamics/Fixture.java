package fr.byob.game.box2d.dynamics;

import pythagoras.f.Vector;
import fr.byob.game.box2d.collision.shapes.Shape;
import fr.byob.game.box2d.collision.shapes.ShapeType;

public interface Fixture {


	/**
	 * Get the type of the child shape. You can use this to down cast to the concrete shape.
	 * @return the shape type.
	 */
	public ShapeType getType();

	/**
	 * Get the child shape. You can modify the child shape, however you should not change the
	 * number of vertices because this will crash some collision caching mechanisms.
	 * @return
	 */
	public Shape getShape();
	/**
	 * Is this fixture a sensor (non-solid)?
	 * @return the true if the shape is a sensor.
	 */
	public boolean isSensor();

	/**
	 * Set if this fixture is a sensor.
	 * @param sensor
	 */
	public void setSensor(boolean sensor);

	/**
	 * Set the contact filtering data. This is an expensive operation and should
	 * not be called frequently. This will not update contacts until the next time
	 * step when either parent body is awake.
	 * @param filter
	 */
	public void setFilterData(final Filter filter);

	/**
	 * Get the contact filtering data.
	 * @return
	 */
	public Filter getFilterData();

	/**
	 * Get the parent body of this fixture. This is NULL if the fixture is not attached.
	 * @return the parent body.
	 */
	public Body getBody();

	public void setDensity(float density);

	public float getDensity();

	/**
	 * Get the user data that was assigned in the fixture definition. Use this to
	 * store your application specific data.
	 * @return
	 */
	public Object getUserData();

	/**
	 * Set the user data. Use this to store your application specific data.
	 * @param data
	 */
	public void setUserData(Object data);

	/**
	 * Test a point for containment in this fixture. This only works for convex shapes.
	 * @param p a point in world coordinates.
	 * @return
	 */
	public boolean testPoint(final Vector p);

	/**
	 * Get the coefficient of friction.
	 * @return
	 */
	public float getFriction();
	/**
	 * Set the coefficient of friction.
	 * @param friction
	 */
	public void setFriction(float friction);

	/**
	 * Get the coefficient of restitution.
	 * @return
	 */
	public float getRestitution();

	/**
	 * Set the coefficient of restitution.
	 * @param restitution
	 */
	public void setRestitution(float restitution);


}
