package fr.byob.game.box2d.dynamics.joints;

public interface PrismaticJoint extends Joint {

	// / Get the current joint translation, usually in meters.
	public float getJointTranslation() ;

	// / Get the current joint translation speed, usually in meters per second.
	public float getJointSpeed() ;

	// / Is the joint limit enabled?
	public boolean isLimitEnabled();

	// / Enable/disable the joint limit.
	public void enableLimit(boolean flag) ;

	// / Get the lower joint limit, usually in meters.
	public float getLowerLimit() ;

	// / Get the upper joint limit, usually in meters.
	public float getUpperLimit() ;

	// / Set the joint limits, usually in meters.
	public void setLimits(float lower, float upper) ;

	// / Is the joint motor enabled?
	public boolean isMotorEnabled() ;

	// / Enable/disable the joint motor.
	public void enableMotor(boolean flag) ;

	// / Set the motor speed, usually in meters per second.
	public void setMotorSpeed(float speed) ;

	// / Get the motor speed, usually in meters per second.
	public float getMotorSpeed() ;

	// / Set the maximum motor force, usually in N.
	public void setMaxMotorForce(float force) ;

	// / Get the current motor force, usually in N.
	public float getMotorForce(float inv_dt);
}
