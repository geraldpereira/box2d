package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;
import fr.byob.game.box2d.dynamics.Body;

public interface Joint {

	/** Short-cut function to determine if either body is inactive. */
	boolean isActive();

	/** Get the type of the concrete joint. */
	JointType getType();

	/** Get the first body attached to this joint. */
	Body getBodyA();

	/** Get the second body attached to this joint. */
	Body getBodyB();

	Vector getReactionForce(final float inv_dt);

	/** Get the reaction torque on body2 in N*m. */
	float getReactionTorque(final float inv_dt);

	/**
	 * get the reaction force on body2 at the joint anchor in Newtons.
	 * 
	 * @param inv_dt
	 */
	void getReactionForce(float inv_dt, Vector argOut);

	/**
	 * get the user data pointer.
	 */
	Object getUserData();

	/**
	 * Set the user data pointer.
	 */
	void setUserData(final Object data);

}
