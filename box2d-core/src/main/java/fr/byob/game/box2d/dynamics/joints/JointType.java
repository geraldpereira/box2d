package fr.byob.game.box2d.dynamics.joints;

public enum JointType {


	UNKNOWN(0), REVOLUTE(1), PRISMATIC(2), DISTANCE(3), PULLEY(
			4), MOUSE(5), GEAR(6), WHEEL(7), WELD(8), FRICTION(
					9), ROPE(10),LINE(-1),CONSTANT_VOLUME(-1);

	// LINE(-1),CONSTANT_VOLUME(-1) are not supported in libGDX

	public static JointType[] valueTypes = new JointType[] { UNKNOWN, REVOLUTE, PRISMATIC, DISTANCE, PULLEY, MOUSE, GEAR, WHEEL, WELD, FRICTION, ROPE, LINE, CONSTANT_VOLUME };

	private int value;

	JointType(final int value) {
		this.value = value;
	}

	public int getValue() {
		return value;
	}
}
