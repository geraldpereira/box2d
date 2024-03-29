package playn.core;

import static playn.core.PlayN.log;

import org.jbox2d.callbacks.DebugDraw;
import org.jbox2d.common.Color3f;
import org.jbox2d.common.OBBViewportTransform;
import org.jbox2d.common.Transform;

import pythagoras.f.Vector;

public class DebugDrawBox2D extends DebugDraw {

	private static String CANVASERROR =
			"Must set canvas (DebugDrawBox2D.setCanvas()) in DebugDrawBox2D before drawing.";

	private Canvas canvas;

	private float strokeWidth;

	private int strokeAlpha;

	private int fillAlpha;

	private float cameraX, cameraY, cameraScale = 1;

	private static float cacheFillR, cacheFillG, cacheFillB; // cached fill color

	private static float cacheStrokeR, cacheStrokeG, cacheStrokeB; // cached
	// stroke color

	private final Vector tempVec1 = new Vector();

	private final Vector tempVec2 = new Vector();

	private final Vector tempVec3 = new Vector();

	public DebugDrawBox2D() {
		super(new OBBViewportTransform());
		viewportTransform.setYFlip(true);
		strokeWidth = 1.0f;
		strokeAlpha = 255;
		fillAlpha = 150;
	}

	@Override
	public void drawCircle(final Vector center, final float radius, final Color3f color) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		setFillColor(color);
		setStrokeColor(color);
		// calculate the effective radius
		tempVec1.set(center.x + radius, center.y + radius);
		getWorldToScreenToOut(tempVec1, tempVec1);
		getWorldToScreenToOut(center, tempVec2);
		canvas.fillCircle(tempVec2.x, tempVec2.y, tempVec1.x - tempVec2.x);
		canvas.strokeCircle(tempVec2.x, tempVec2.y, tempVec1.x - tempVec2.x);
	}

	@Override
	public void drawPoint(final Vector argPoint, final float argRadiusOnScreen, final Color3f color) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		setFillColor(color);
		setStrokeColor(color);
		getWorldToScreenToOut(argPoint, tempVec1);
		canvas.fillCircle(tempVec1.x, tempVec1.y, argRadiusOnScreen);
	}

	@Override
	public void drawSegment(final Vector p1, final Vector p2, final Color3f color) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		setStrokeColor(color);
		setFillColor(color);
		getWorldToScreenToOut(p1, tempVec1);
		getWorldToScreenToOut(p2, tempVec2);
		canvas.drawLine(tempVec1.x, tempVec1.y, tempVec2.x, tempVec2.y);
	}

	@Override
	public void drawSolidCircle(final Vector center, final float radius, final Vector axis, final Color3f color) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		setFillColor(color);
		setStrokeColor(color);
		// calculate the effective radius
		tempVec1.set(center.x + radius, center.y + radius);
		getWorldToScreenToOut(tempVec1, tempVec1);
		getWorldToScreenToOut(center, tempVec2);
		getWorldToScreenToOut(axis, tempVec3);
		canvas.fillCircle(tempVec2.x, tempVec2.y, tempVec1.x - tempVec2.x);
		canvas.strokeCircle(tempVec2.x, tempVec2.y, tempVec1.x - tempVec2.x);
	}

	@Override
	public void drawSolidPolygon(final Vector[] vertices, final int vertexCount, final Color3f color) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		setFillColor(color);
		setStrokeColor(color);
		final Path path = canvas.createPath();
		for (int i = 0; i < vertexCount; i++) {
			getWorldToScreenToOut(vertices[i], tempVec1);
			if (i == 0) {
				path.moveTo(tempVec1.x, tempVec1.y);
			} else {
				path.lineTo(tempVec1.x, tempVec1.y);
			}
		}
		path.close();
		canvas.fillPath(path);
		canvas.strokePath(path);
	}

	@Override
	public void drawString(final float x, final float y, final String s, final Color3f color) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		log().info("drawString not yet implemented in DebugDrawBox2D: " + s);
	}

	@Override
	public void drawTransform(final Transform xf) {
		if (canvas == null) {
			log().error(CANVASERROR);
			return;
		}

		getWorldToScreenToOut(xf.position, tempVec1);
		tempVec2.set(0,0);
		final float k_axisScale = 0.4f;

		canvas.setStrokeColor(Color.rgb(1, 0, 0)); // note: violates
		// strokeAlpha
		tempVec2.x = xf.position.x + k_axisScale * xf.R.m11;
		tempVec2.y = xf.position.y + k_axisScale * xf.R.m12;
		getWorldToScreenToOut(tempVec2, tempVec2);
		canvas.drawLine(tempVec1.x, tempVec1.y, tempVec2.x, tempVec2.y);

		canvas.setStrokeColor(Color.rgb(0, 1, 0)); // note: violates
		// strokeAlpha
		tempVec2.x = xf.position.x + k_axisScale * xf.R.m21;
		tempVec2.y = xf.position.y + k_axisScale * xf.R.m22;
		getWorldToScreenToOut(tempVec2, tempVec2);
		canvas.drawLine(tempVec1.x, tempVec1.y, tempVec2.x, tempVec2.y);

		canvas.setStrokeColor(Color.argb(strokeAlpha, 1, 0, 0)); // restores
		// strokeAlpha
	}

	public Canvas getCanvas() {
		return canvas;
	}

	public int getFillAlpha() {
		return fillAlpha;
	}

	public int getStrokeAlpha() {
		return strokeAlpha;
	}

	public float getStrokeWidth() {
		return strokeWidth;
	}

	@Override
	public void setCamera(final float x, final float y, final float scale) {
		cameraX = x;
		cameraY = y;
		cameraScale = scale;
		updateCamera();
	}

	public void setCameraScale(final float scale) {
		cameraScale = scale;
		updateCamera();
	}

	public void setCameraX(final float x) {
		cameraX = x;
		updateCamera();
	}

	public void setCameraY(final float y) {
		cameraY = y;
		updateCamera();
	}

	public void setCanvas(final CanvasImage image) {
		this.canvas = image.canvas();
		canvas.setStrokeWidth(strokeWidth);
	}

	public void setFillAlpha(final int fillAlpha) {
		this.fillAlpha = fillAlpha;
	}

	public void setFlipY(final boolean flip) {
		viewportTransform.setYFlip(flip);
	}

	public void setStrokeAlpha(final int strokeAlpha) {
		this.strokeAlpha = strokeAlpha;
	}

	public void setStrokeWidth(final float strokeWidth) {
		this.strokeWidth = strokeWidth;
		if (canvas != null) {
			canvas.setStrokeWidth(strokeWidth);
		}
	}

	/**
	 * Sets the fill color from a Color3f
	 * 
	 * @param color color where (r,g,b) = (x,y,z)
	 */
	private void setFillColor(final Color3f color) {
		if (cacheFillR == color.x && cacheFillG == color.y && cacheFillB == color.z) {
			// no need to re-set the fill color, just use the cached values
		} else {
			cacheFillR = color.x;
			cacheFillG = color.y;
			cacheFillB = color.z;
			canvas
			.setFillColor(
					Color.argb(fillAlpha, (int) (255 * color.x), (int) (255 * color.y),
							(int) (255 * color.z)));
		}
	}

	/**
	 * Sets the stroke color from a Color3f
	 * 
	 * @param color color where (r,g,b) = (x,y,z)
	 */
	private void setStrokeColor(final Color3f color) {
		if (cacheStrokeR == color.x && cacheStrokeG == color.y && cacheStrokeB == color.z) {
			// no need to re-set the stroke color, just use the cached values
		} else {
			cacheStrokeR = color.x;
			cacheStrokeG = color.y;
			cacheStrokeB = color.z;
			canvas.setStrokeColor(
					Color.argb(strokeAlpha, (int) (255 * color.x), (int) (255 * color.y),
							(int) (255 * color.z)));
		}
	}

	private void updateCamera() {
		super.setCamera(cameraX, cameraY, cameraScale);
	}

	@Override
	public void clear() {
		canvas.clear();
	}
}
