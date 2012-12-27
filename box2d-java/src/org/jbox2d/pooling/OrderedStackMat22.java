package org.jbox2d.pooling;

import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.pooling.OrderedStack;

public class OrderedStackMat22 extends OrderedStack<Mat22> {

	public OrderedStackMat22(final int argStackSize) {
		super(argStackSize);
	}

	@Override
	protected Mat22[] newArray(final int argSize) {
		return new Mat22[argSize];
	}

	@Override
	protected Mat22 newObject() {
		return new Mat22();
	}

}
