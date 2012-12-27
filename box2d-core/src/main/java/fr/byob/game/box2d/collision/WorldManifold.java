package fr.byob.game.box2d.collision;

import pythagoras.f.Vector;

public abstract class WorldManifold {
	/**
	 * World vector pointing from A to B
	 */
	public Vector normal;

	/**
	 * World contact point (point of intersection)
	 */
	public Vector[] points;
}
