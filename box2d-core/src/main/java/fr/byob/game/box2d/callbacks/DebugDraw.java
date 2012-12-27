package fr.byob.game.box2d.callbacks;

import playn.core.Canvas;

public interface DebugDraw {

	void setCamera(float x, float y, float scale);

	Canvas getCanvas();
}
