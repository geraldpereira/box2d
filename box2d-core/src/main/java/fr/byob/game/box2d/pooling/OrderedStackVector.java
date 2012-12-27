package fr.byob.game.box2d.pooling;

import pythagoras.f.Vector;

public class OrderedStackVector extends OrderedStack<Vector> {

	public OrderedStackVector(final int argStackSize) {
		super(argStackSize);
	}

	@Override
	protected Vector[] newArray(final int argSize) {
		return new Vector[argSize];
	}

	@Override
	protected Vector newObject() {
		return new Vector();
	}
}
