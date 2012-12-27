package fr.byob.game.box2d.dynamics.joints;

import fr.byob.game.box2d.dynamics.Body;

public class JointEdge {

	/**
	 * Provides quick access to the other body attached
	 */
	public Body other = null;
	
	/**
	 * the joint
	 */
	public Joint joint = null;
	
	public JointEdge(){
		
	}
	
	public JointEdge (Body other, Joint joint) {
		this.other = other;
		this.joint = joint;
	}
	
}
