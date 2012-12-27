package fr.byob.game.box2d.dynamics.joints;

public interface RevoluteJoint extends Joint {


	public float getJointAngle() ;

	public float getJointSpeed();

	public boolean isMotorEnabled();

	public void enableMotor(boolean flag);

	public float getMotorTorque(float inv_dt);

	public void setMotorSpeed(final float speed);

	public void setMaxMotorTorque(final float torque);

	public boolean isLimitEnabled() ;

	public void enableLimit(final boolean flag);

	public float getLowerLimit();

	public float getUpperLimit();

	public void setLimits(final float lower, final float upper);
}
