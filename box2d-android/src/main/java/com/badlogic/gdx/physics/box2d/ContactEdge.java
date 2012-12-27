package com.badlogic.gdx.physics.box2d;


//struct b2ContactEdge
//{
//	b2Body* other;			///< provides quick access to the other body attached.
//	b2Contact* contact;		///< the contact
//	b2ContactEdge* prev;	///< the previous contact edge in the body's contact list
//	b2ContactEdge* next;	///< the next contact edge in the body's contact list
//};

public class ContactEdge implements fr.byob.game.box2d.dynamics.contacts.ContactEdge {

	/** the address of the ContactEdge instance **/
	private long addr;

	private final World world;

	private final Contact contact;

	protected ContactEdge(final World world) {
		this.world = world;
		this.contact = new Contact(world, 0);
	}

	public void setAddr(final long addr) {
		this.addr = addr;
	}

	/**
	 * Warning the current contact is modified and returned
	 */
	public ContactEdge getNext() {
		addr = jniGetNext(addr);
		return this;
	}

	private native long jniGetNext (long addr); /*
		b2ContactEdge* contactEdge = (b2ContactEdge*)addr;
		return contactEdge->next;
	 */

	public fr.byob.game.box2d.dynamics.Body getOther() {
		return world.bodies.get(jniGetOther(addr));
	}


	private native long jniGetOther (long addr); /*
		b2ContactEdge* contactEdge = (b2ContactEdge*)addr;
		return contactEdge->other;
	 */

	public fr.byob.game.box2d.dynamics.Contact getContact() {
		contact.addr = jniGetContact(addr);
		return contact;
	}

	private native long jniGetContact (long addr); /*
		b2ContactEdge* contactEdge = (b2ContactEdge*)addr;
		return contactEdge->contact;
	 */
}
