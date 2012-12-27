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
 * Created at 2:26:49 PM Jul 6, 2010
 */
package org.jbox2d.dynamics.contacts;

import org.jbox2d.dynamics.Body;

import pythagoras.f.Vector;
import fr.byob.game.box2d.collision.ManifoldType;
import fr.byob.game.box2d.common.Settings;

// updated to rev 100 - ec
/**
 * @author daniel
 */
public class TOIConstraint {
	public final Vector localPoints[] = new Vector[Settings.maxManifoldPoints];
	public final Vector localNormal = new Vector();
	public final Vector localPoint = new Vector();
	public ManifoldType type;
	public float radius;
	public int pointCount;
	public Body bodyA;
	public Body bodyB;

	public TOIConstraint(){
		for(int i=0; i<localPoints.length; i++){
			localPoints[i] = new Vector();
		}
	}

	public TOIConstraint(final TOIConstraint argToClone){
		this();
		set(argToClone);
	}

	public void set(final TOIConstraint argOther){
		for(int i=0; i<localPoints.length; i++){
			localPoints[i].set(argOther.localPoints[i]);
		}
		localNormal.set(argOther.localNormal);
		localPoint.set(argOther.localPoint);
		type = argOther.type;
		radius = argOther.radius;
		pointCount = argOther.pointCount;
		bodyA = argOther.bodyA;
		bodyB = argOther.bodyB;
	}
}
