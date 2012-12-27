package fr.byob.game.box2d.collision.shapes;

import pythagoras.f.Vector;

public interface BoxShape extends PolygonShape {
	/**
	 * Build vertices to represent an axis-aligned box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 */
	public void setAsBox(float hx, float hy);

	/**
	 * Build vertices to represent an oriented box.
	 * 
	 * @param hx
	 *            the half-width.
	 * @param hy
	 *            the half-height.
	 * @param center
	 *            the center of the box in local coordinates.
	 * @param angle
	 *            the rotation in radians of the box in local coordinates.
	 */
	public void setAsBox(float hx, float hy, Vector center, float angle);
}
