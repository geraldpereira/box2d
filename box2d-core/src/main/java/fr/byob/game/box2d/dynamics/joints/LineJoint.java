package fr.byob.game.box2d.dynamics.joints;

public interface LineJoint extends Joint {
	public float getJointTranslation();
	
	public float getJointSpeed();
	
	public boolean isLimitEnabled();
	
	public void EnableLimit(boolean flag);
	
	public float getLowerLimit() ;
	
	public float getUpperLimit();
	
	public void setLimits(float lower, float upper);
	
	public boolean isMotorEnabled();
	
	public void EnableMotor(boolean flag);
	
	public void setMotorSpeed(float speed);
	
	public void setMaxMotorForce(float force);
	
	public float getMotorForce() ;
	
}
