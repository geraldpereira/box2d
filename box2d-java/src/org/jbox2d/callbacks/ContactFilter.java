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
 * Created at 4:25:42 AM Jul 15, 2010
 */
package org.jbox2d.callbacks;

import org.jbox2d.dynamics.Fixture;

import fr.byob.game.box2d.dynamics.Filter;


// updated to rev 100
/**
 * Implement this class to provide collision filtering. In other words, you can implement
 * this class if you want finer control over contact creation.
 * @author Daniel Murphy
 */
public class ContactFilter implements fr.byob.game.box2d.callbacks.ContactFilter {

	/**
	 * Return true if contact calculations should be performed between these two
	 * shapes.
	 * 
	 * @warning for performance reasons this is only called when the AABBs begin
	 *          to overlap.
	 * @param fixtureA
	 * @param fixtureBs
	 * @return
	 */
	public boolean shouldCollide(final Fixture fixtureA, final Fixture fixtureB){
		final Filter filterA = fixtureA.getFilterData();
		final Filter filterB = fixtureB.getFilterData();

		if (filterA.groupIndex == filterB.groupIndex && filterA.groupIndex != 0){
			return filterA.groupIndex > 0;
		}

		final boolean collide = (filterA.maskBits & filterB.categoryBits) != 0 &&
				(filterA.categoryBits & filterB.maskBits) != 0;
		return collide;
	}

	public boolean shouldCollide(final fr.byob.game.box2d.dynamics.Fixture fixtureA, final fr.byob.game.box2d.dynamics.Fixture fixtureB) {
		return shouldCollide((Fixture) fixtureA, (Fixture) fixtureB);
	}
}
