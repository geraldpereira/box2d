package fr.byob.game.box2d.collision.shapes;

import pythagoras.f.Vector;

public interface PolygonShape extends Shape {

	public void set(final Vector[] vertices, final int count);

	/** @return the number of vertices */
	public int getVertexCount();

	/**
	 * Returns the vertex at the given position.
	 * 
	 * @param index
	 *            the index of the vertex 0 <= index < getVertexCount( )
	 * @param vertex
	 *            vertex
	 */
	public void getVertex(int index, Vector vertex);

}
