package com.badlogic.gdx.physics.box2d;

import playn.core.CanvasImage;
import pythagoras.f.Vector;
import fr.byob.game.box2d.callbacks.DebugDraw;
import fr.byob.game.box2d.collision.WorldManifold;
import fr.byob.game.box2d.collision.shapes.BoxShape;
import fr.byob.game.box2d.collision.shapes.CircleShape;
import fr.byob.game.box2d.collision.shapes.EdgeShape;
import fr.byob.game.box2d.collision.shapes.PolygonShape;
import fr.byob.game.box2d.common.Settings;
import fr.byob.game.box2d.dynamics.World;

public class Box2D implements fr.byob.game.box2d.Box2D {

	public World newWorld(final Vector gravity, final boolean doSleep) {
		return new com.badlogic.gdx.physics.box2d.World(gravity, doSleep);
	}

	public CircleShape newCircleShape() {
		return new com.badlogic.gdx.physics.box2d.CircleShape();
	}

	public PolygonShape newPolygonShape() {
		return new com.badlogic.gdx.physics.box2d.PolygonShape();
	}

	public BoxShape newBoxShape() {
		return new com.badlogic.gdx.physics.box2d.PolygonShape();
	}

	public EdgeShape newEdgeShape() {
		return new com.badlogic.gdx.physics.box2d.EdgeShape();
	}

	public DebugDraw newDebugDraw(final CanvasImage canvas) {
		throw new UnsupportedOperationException("DebugDraw not supported with native code");
	}

	public WorldManifold newWorldManifold() {
		final com.badlogic.gdx.physics.box2d.WorldManifold worldManifold = new com.badlogic.gdx.physics.box2d.WorldManifold();
		worldManifold.normal = new Vector();
		worldManifold.points = new Vector[Settings.maxManifoldPoints];
		for (int i = 0; i < worldManifold.points.length; i++) {
			worldManifold.points[i] = new Vector();
		}
		return worldManifold;
	}

}
