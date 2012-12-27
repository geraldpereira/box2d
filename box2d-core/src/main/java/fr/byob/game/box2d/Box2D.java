package fr.byob.game.box2d;

import playn.core.CanvasImage;
import pythagoras.f.Vector;
import fr.byob.game.box2d.callbacks.DebugDraw;
import fr.byob.game.box2d.collision.WorldManifold;
import fr.byob.game.box2d.collision.shapes.BoxShape;
import fr.byob.game.box2d.collision.shapes.CircleShape;
import fr.byob.game.box2d.collision.shapes.EdgeShape;
import fr.byob.game.box2d.collision.shapes.PolygonShape;
import fr.byob.game.box2d.dynamics.World;

public interface Box2D {

	World newWorld(Vector gravity, boolean doSleep);

	CircleShape newCircleShape();

	PolygonShape newPolygonShape();

	BoxShape newBoxShape();

	EdgeShape newEdgeShape();

	WorldManifold newWorldManifold();

	DebugDraw newDebugDraw(final CanvasImage canvas);
}
