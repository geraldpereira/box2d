package fr.byob.game.box2d.collision.shapes;

import fr.byob.game.box2d.Disposable;



public interface Shape extends Disposable {

	void dispose();

	/**
	 * Get the type of this shape. You can use this to down cast to the concrete
	 * shape.
	 * 
	 * @return the shape type.
	 */
	public ShapeType getType();

}
