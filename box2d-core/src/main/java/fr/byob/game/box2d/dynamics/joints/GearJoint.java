package fr.byob.game.box2d.dynamics.joints;

public interface GearJoint extends Joint {
	
	public void setRatio(float argRatio) ;
	
	public float getRatio() ;
}
