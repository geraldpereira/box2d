package org.jbox2d.pooling;

import pythagoras.f.Vector3;
import fr.byob.game.box2d.pooling.OrderedStack;

public class OrderedStackVector3 extends OrderedStack<Vector3> {

	public OrderedStackVector3(final int argStackSize) {
		super(argStackSize);
	}

	@Override
	protected Vector3[] newArray(final int argSize) {
		return new Vector3[argSize];
	}

	@Override
	protected Vector3 newObject() {
		return new Vector3();
	}

}
