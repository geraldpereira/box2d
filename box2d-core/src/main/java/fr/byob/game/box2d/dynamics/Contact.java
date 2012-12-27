package fr.byob.game.box2d.dynamics;

import fr.byob.game.box2d.collision.Manifold;
import fr.byob.game.box2d.collision.WorldManifold;

public interface Contact {

	/**
	 * Is this contact touching
	 * 
	 * @return
	 */
	public boolean isTouching();

	/**
	 * Enable/disable this contact. This can be used inside the pre-solve
	 * contact listener. The contact is only disabled for the current time step
	 * (or sub-step in continuous collisions).
	 * 
	 * @param flag
	 */
	public void setEnabled(boolean flag);

	/**
	 * Has this contact been disabled?
	 * 
	 * @return
	 */
	public boolean isEnabled();

	/**
	 * Get the first fixture in this contact.
	 * 
	 * @return
	 */
	public Fixture getFixtureA();

	/**
	 * Get the second fixture in this contact.
	 * 
	 * @return
	 */
	public Fixture getFixtureB();

	/**
	 * Get the contact manifold. Do not set the point count to zero. Instead
	 * call Disable.
	 */
	public Manifold getManifold();

	public WorldManifold getWorldManifold();

	public void getWorldManifoldToOut(final WorldManifold worldManifold);

}
