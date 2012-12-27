package fr.byob.game.box2d.collision.shapes;

import pythagoras.f.Vector;

public interface EdgeShape extends PolygonShape {
	/**
	 * Set this as a single edge.
	 * 
	 * @param v1
	 * @param v2
	 */
	public void setAsEdge(final Vector v1, final Vector v2);
}
