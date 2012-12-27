package org.jbox2d.pooling;

import fr.byob.game.box2d.collision.AABB;
import fr.byob.game.box2d.pooling.OrderedStack;

public class OrderedStackAABB extends OrderedStack<AABB> {

	public OrderedStackAABB(final int argStackSize) {
		super(argStackSize);
	}

	@Override
	protected AABB[] newArray(final int argSize) {
		return new AABB[argSize];
	}

	@Override
	protected AABB newObject() {
		return new AABB();
	}

}
