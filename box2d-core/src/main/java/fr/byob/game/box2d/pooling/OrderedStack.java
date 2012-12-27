/*******************************************************************************
 * Copyright (c) 2011, Daniel Murphy
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DANIEL MURPHY BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
/**
 * Created at 12:52:04 AM Jan 20, 2011
 */
package fr.byob.game.box2d.pooling;


/**
 * @author Daniel Murphy
 */
public abstract class OrderedStack<E> implements IOrderedStack<E> {

	protected E[] pool;
	private int index;
	private final int size;

	public OrderedStack(final int argStackSize) {
		index = 0;
		size = argStackSize;
		pool = newArray(argStackSize);
		for (int i = 0; i < argStackSize; i++) {
			pool[i] = newObject();
		}
	}

	protected abstract E newObject();

	protected abstract E[] newArray(int argSize);

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.jbox2d.pooling.IPoolingStack#pop()
	 */
	public final E pop() {
		assert (index < size) : "End of stack reached, there is probably a leak somewhere";
		return pool[index++];
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.jbox2d.pooling.IPoolingStack#pop(int)
	 */
	public final E[] pop(final int argNum) {
		assert (index + argNum < size) : "End of stack reached, there is probably a leak somewhere";
		final E[] container = newArray(argNum);
		for (int i = 0; i < argNum; i++) {
			container[i] = pool[index + i];
		}
		index += argNum;
		return container;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.jbox2d.pooling.IPoolingStack#push(int)
	 */
	public final void push(final int argNum) {
		index -= argNum;
		assert (index >= 0) : "Beginning of stack reached, push/pops are unmatched";
	}
}
