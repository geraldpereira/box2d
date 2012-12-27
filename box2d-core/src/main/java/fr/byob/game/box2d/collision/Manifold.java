
package fr.byob.game.box2d.collision;

import pythagoras.f.Vector;


public interface Manifold {

	public ManifoldPoint[] getPoints();

	public Vector getLocalPoint();

	public Vector getLocalNormal();

	public ManifoldType getType();

	public int getPointCount();
}
