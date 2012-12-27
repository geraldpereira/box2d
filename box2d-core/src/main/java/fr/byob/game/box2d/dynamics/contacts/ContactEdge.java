package fr.byob.game.box2d.dynamics.contacts;

import fr.byob.game.box2d.dynamics.Body;
import fr.byob.game.box2d.dynamics.Contact;

public interface ContactEdge {

	/**
	 * provides quick access to the other body attached.
	 */
	public Body getOther();

	/**
	 * the contact
	 */
	public Contact getContact();

	public ContactEdge getNext();

}
