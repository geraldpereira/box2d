package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;

public interface MouseJoint extends Joint {

	public void setTarget(Vector target);

	public Vector getTarget();

	// / set/get the maximum force in Newtons.
	public void setMaxForce(float force);

	public float getMaxForce();

	// / set/get the frequency in Hertz.
	public void setFrequency(float hz);

	public float getFrequency();

	// / set/get the damping ratio (dimensionless).
	public void setDampingRatio(float ratio);

	public float getDampingRatio();
}
