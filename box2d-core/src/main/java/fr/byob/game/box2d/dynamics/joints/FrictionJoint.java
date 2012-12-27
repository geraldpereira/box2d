package fr.byob.game.box2d.dynamics.joints;

/** Friction joint. This is used for top-down friction. It provides 2D translational friction and angular friction. */
public interface FrictionJoint extends Joint {
	public void setMaxForce(float force);
	
	public float getMaxForce();
	
	public void setMaxTorque(float torque);
	
	public float getMaxTorque();
}
