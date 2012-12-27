package fr.byob.game.box2d.dynamics.joints;

import fr.byob.game.box2d.dynamics.Body;

public interface ConstantVolumeJoint extends Joint {

	public Body[] getBodies() ;

	public void inflate(float factor) ;

}
