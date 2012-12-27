package fr.byob.game.box2d;

import java.util.Iterator;

public class B2DIterator<T extends B2DLinkedList<T>> implements Iterator<T> {

	private final T first;

	public B2DIterator(final T first){
		this.first = first;
	}

	private T current;

	public boolean hasNext() {
		return current == null || current.getNext() != null;
	}

	public T next() {
		if (current == null) {
			current = first;
			return current;
		}
		current = current.getNext();
		return current;
	}

	public void remove() {
		throw new UnsupportedOperationException();
	}
}
