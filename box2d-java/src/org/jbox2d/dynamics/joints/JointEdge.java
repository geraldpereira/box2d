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
package org.jbox2d.dynamics.joints;

import java.util.Iterator;

import fr.byob.game.box2d.B2DIterator;
import fr.byob.game.box2d.B2DLinkedList;

//updated to rev 100 - ec
/**
 * A joint edge is used to connect bodies and joints together
 * in a joint graph where each body is a node and each joint
 * is an edge. A joint edge belongs to a doubly linked list
 * maintained in each attached body. Each joint has two joint
 * nodes, one for each attached body.
 * @author Daniel
 */
public class JointEdge extends fr.byob.game.box2d.dynamics.joints.JointEdge implements B2DLinkedList<JointEdge>, Iterable<JointEdge> {

	/**
	 * the previous joint edge in the body's joint list
	 */
	public JointEdge prev = null;

	/**
	 * the next joint edge in the body's joint list
	 */
	public JointEdge next = null;

	public JointEdge getNext() {
		return next;
	}

	public Iterator<JointEdge> iterator() {
		return new B2DIterator<JointEdge>(this);
	}
}
