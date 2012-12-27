package org.jbox2d.common;

import playn.core.CanvasImage;
import playn.core.DebugDrawBox2D;
import pythagoras.f.Vector;
import fr.byob.game.box2d.callbacks.DebugDraw;
import fr.byob.game.box2d.collision.WorldManifold;
import fr.byob.game.box2d.collision.shapes.BoxShape;
import fr.byob.game.box2d.collision.shapes.CircleShape;
import fr.byob.game.box2d.collision.shapes.EdgeShape;
import fr.byob.game.box2d.collision.shapes.PolygonShape;
import fr.byob.game.box2d.dynamics.World;


public class Box2D implements fr.byob.game.box2d.Box2D {

	public World newWorld(final Vector gravity, final boolean doSleep) {
		return new org.jbox2d.dynamics.World(gravity, doSleep);
	}

	public CircleShape newCircleShape() {
		return new org.jbox2d.collision.shapes.CircleShape();
	}

	public PolygonShape newPolygonShape() {
		return new org.jbox2d.collision.shapes.PolygonShape();
	}

	public BoxShape newBoxShape() {
		return new org.jbox2d.collision.shapes.PolygonShape();
	}

	public EdgeShape newEdgeShape() {
		return new org.jbox2d.collision.shapes.PolygonShape();
	}

	public DebugDraw newDebugDraw(final CanvasImage canvas) {
		final DebugDrawBox2D jDebugDraw = new DebugDrawBox2D();
		jDebugDraw.setCanvas(canvas);
		jDebugDraw.setFlipY(false);
		jDebugDraw.setStrokeAlpha(75);
		jDebugDraw.setFillAlpha(50);
		jDebugDraw.setStrokeWidth(2.0f);
		jDebugDraw.setFlags(DebugDrawBox2D.e_shapeBit | DebugDrawBox2D.e_jointBit | DebugDrawBox2D.e_aabbBit);

		return jDebugDraw;
	}

	public WorldManifold newWorldManifold() {
		return new org.jbox2d.collision.WorldManifold();
	}

}
