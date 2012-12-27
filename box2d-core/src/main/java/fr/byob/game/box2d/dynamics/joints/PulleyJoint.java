package fr.byob.game.box2d.dynamics.joints;

import pythagoras.f.Vector;

public interface PulleyJoint extends Joint {

	public static final float MIN_PULLEY_LENGTH = 2.0f;

	public Vector getGroundAnchorA();

	public Vector getGroundAnchorB();

	public float getLength1();

	public float getLength2();

	public float getRatio();
}
