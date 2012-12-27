package fr.byob.game.box2d.dynamics.joints;

public interface WheelJoint extends Joint{
	
	/** Get the current joint translation, usually in meters. */
	public float getJointTranslation () ;

	/** Get the current joint translation speed, usually in meters per second. */
	public float getJointSpeed ();


	/** Enable/disable the joint motor. */
	public void enableMotor (boolean flag) ;

	/** Set the motor speed, usually in radians per second. */
	public void setMotorSpeed (float speed) ;

	/** Get the motor speed, usually in radians per second. */
	public float getMotorSpeed () ;

	/** Set/Get the maximum motor force, usually in N-m. */
	public void setMaxMotorTorque (float torque) ;

	public float getMaxMotorTorque () ;

	/** Get the current motor torque given the inverse time step, usually in N-m. */
	public float getMotorTorque (float invDt) ;

	/** Set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring. */
	public void setSpringFrequencyHz (float hz) ;

	public float getSpringFrequencyHz () ;

	/** Set/Get the spring damping ratio */
	public void setSpringDampingRatio (float ratio);

	public float getSpringDampingRatio () ;
}