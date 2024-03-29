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
package org.jbox2d.dynamics;

import org.jbox2d.callbacks.ContactFilter;
import org.jbox2d.callbacks.PairCallback;
import org.jbox2d.collision.broadphase.BroadPhase;
import org.jbox2d.collision.broadphase.DynamicTreeNode;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.contacts.ContactEdge;

import fr.byob.game.box2d.callbacks.ContactListener;

// updated to rev 100
/**
 * Delegate of World.
 * 
 * @author Daniel Murphy
 */
public class ContactManager implements PairCallback {

	public BroadPhase m_broadPhase;
	public Contact m_contactList;
	public int m_contactCount;
	public ContactFilter m_contactFilter;
	public ContactListener m_contactListener;

	private final World pool;

	public ContactManager(final World argPool) {
		m_contactList = null;
		m_contactCount = 0;
		m_contactFilter = new ContactFilter();
		m_contactListener = null;
		m_broadPhase = new BroadPhase();
		pool = argPool;
	}

	/**
	 * Broad-phase callback.
	 * 
	 * @param proxyUserDataA
	 * @param proxyUserDataB
	 */
	public void addPair(final Object proxyUserDataA, final Object proxyUserDataB) {
		Fixture fixtureA = (Fixture) proxyUserDataA;
		Fixture fixtureB = (Fixture) proxyUserDataB;

		Body bodyA = fixtureA.getBody();
		Body bodyB = fixtureB.getBody();

		// Are the fixtures on the same body?
		if (bodyA == bodyB) {
			return;
		}

		// Does a contact already exist?
		ContactEdge edge = bodyB.getContactList();
		while (edge != null) {
			if (edge.other == bodyA) {
				final Fixture fA = edge.contact.getFixtureA();
				final Fixture fB = edge.contact.getFixtureB();
				if (fA == fixtureA && fB == fixtureB) {
					// A contact already exists.
					return;
				}

				if (fA == fixtureB && fB == fixtureA) {
					// A contact already exists.
					return;
				}
			}

			edge = edge.next;
		}

		// Does a joint override collision? is at least one body dynamic?
		if (bodyB.shouldCollide(bodyA) == false) {
			return;
		}

		// Check user filtering.
		if (m_contactFilter != null && m_contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
			return;
		}

		// Call the factory.
		final Contact c = pool.popContact(fixtureA, fixtureB);

		// Contact creation may swap fixtures.
		fixtureA = c.getFixtureA();
		fixtureB = c.getFixtureB();
		bodyA = fixtureA.getBody();
		bodyB = fixtureB.getBody();

		// Insert into the world.
		c.m_prev = null;
		c.m_next = m_contactList;
		if (m_contactList != null) {
			m_contactList.m_prev = c;
		}
		m_contactList = c;

		// Connect to island graph.

		// Connect to body A
		c.m_nodeA.contact = c;
		c.m_nodeA.other = bodyB;

		c.m_nodeA.prev = null;
		c.m_nodeA.next = bodyA.m_contactList;
		if (bodyA.m_contactList != null) {
			bodyA.m_contactList.prev = c.m_nodeA;
		}
		bodyA.m_contactList = c.m_nodeA;

		// Connect to body B
		c.m_nodeB.contact = c;
		c.m_nodeB.other = bodyA;

		c.m_nodeB.prev = null;
		c.m_nodeB.next = bodyB.m_contactList;
		if (bodyB.m_contactList != null) {
			bodyB.m_contactList.prev = c.m_nodeB;
		}
		bodyB.m_contactList = c.m_nodeB;

		++m_contactCount;
	}

	public void findNewContacts() {
		m_broadPhase.updatePairs(this);
	}

	public void destroy(final Contact c) {
		final Fixture fixtureA = c.getFixtureA();
		final Fixture fixtureB = c.getFixtureB();
		final Body bodyA = fixtureA.getBody();
		final Body bodyB = fixtureB.getBody();

		if (m_contactListener != null && c.isTouching()) {
			m_contactListener.endContact(c);
		}

		// Remove from the world.
		if (c.m_prev != null) {
			c.m_prev.m_next = c.m_next;
		}

		if (c.m_next != null) {
			c.m_next.m_prev = c.m_prev;
		}

		if (c == m_contactList) {
			m_contactList = c.m_next;
		}

		// Remove from body 1
		if (c.m_nodeA.prev != null) {
			c.m_nodeA.prev.next = c.m_nodeA.next;
		}

		if (c.m_nodeA.next != null) {
			c.m_nodeA.next.prev = c.m_nodeA.prev;
		}

		if (c.m_nodeA == bodyA.m_contactList) {
			bodyA.m_contactList = c.m_nodeA.next;
		}

		// Remove from body 2
		if (c.m_nodeB.prev != null) {
			c.m_nodeB.prev.next = c.m_nodeB.next;
		}

		if (c.m_nodeB.next != null) {
			c.m_nodeB.next.prev = c.m_nodeB.prev;
		}

		if (c.m_nodeB == bodyB.m_contactList) {
			bodyB.m_contactList = c.m_nodeB.next;
		}

		// Call the factory.
		pool.pushContact(c);
		--m_contactCount;
	}

	/**
	 * This is the top level collision call for the time step. Here all the
	 * narrow phase collision is processed for the world contact list.
	 */
	public void collide() {
		// Update awake contacts.
		Contact c = m_contactList;
		while (c != null) {
			final Fixture fixtureA = c.getFixtureA();
			final Fixture fixtureB = c.getFixtureB();
			final Body bodyA = fixtureA.getBody();
			final Body bodyB = fixtureB.getBody();

			if (bodyA.isAwake() == false && bodyB.isAwake() == false) {
				c = c.getNext();
				continue;
			}

			// is this contact flagged for filtering?
			if ((c.m_flags & Contact.FILTER_FLAG) == Contact.FILTER_FLAG) {
				// Should these bodies collide?
				if (bodyB.shouldCollide(bodyA) == false) {
					final Contact cNuke = c;
					c = cNuke.getNext();
					destroy(cNuke);
					continue;
				}

				// Check user filtering.
				if (m_contactFilter != null && m_contactFilter.shouldCollide(fixtureA, fixtureB) == false) {
					final Contact cNuke = c;
					c = cNuke.getNext();
					destroy(cNuke);
					continue;
				}

				// Clear the filtering flag.
				c.m_flags &= ~Contact.FILTER_FLAG;
			}

			final DynamicTreeNode proxyIdA = fixtureA.m_proxy;
			final DynamicTreeNode proxyIdB = fixtureB.m_proxy;
			final boolean overlap = m_broadPhase.testOverlap(proxyIdA, proxyIdB);

			// Here we destroy contacts that cease to overlap in the
			// broad-phase.
			if (overlap == false) {
				final Contact cNuke = c;
				c = cNuke.getNext();
				destroy(cNuke);
				continue;
			}

			// The contact persists.
			c.update(m_contactListener);
			c = c.getNext();
		}
	}
}
