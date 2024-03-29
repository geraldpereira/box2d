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
package org.jbox2d.pooling;

import org.jbox2d.collision.Collision;
import org.jbox2d.collision.Distance;
import org.jbox2d.collision.TimeOfImpact;
import org.jbox2d.dynamics.contacts.Contact;

import pythagoras.f.Vector;
import pythagoras.f.Vector3;
import fr.byob.game.box2d.collision.AABB;
import fr.byob.game.box2d.common.Mat22;
import fr.byob.game.box2d.pooling.IOrderedStack;

/**
 * World pool interface
 * @author Daniel
 *
 */
public interface IWorldPool {

	public IDynamicStack<Contact> getPolyContactStack();

	public IDynamicStack<Contact> getCircleContactStack();

	public IDynamicStack<Contact> getPolyCircleContactStack();

	public IOrderedStack<Vector> getVectorStack();

	public Vector popVector();

	public Vector[] popVector(int argNum);

	public void pushVector(int argNum);

	public IOrderedStack<Vector3> getVector3Stack();

	public Vector3 popVector3();

	public Vector3[] popVector3(int argNum);

	public void pushVector3(int argNum);

	public IOrderedStack<Mat22> getMat22Stack();

	public Mat22 popMat22();

	public Mat22[] popMat22(int argNum);

	public void pushMat22(int argNum);

	public IOrderedStack<AABB> getAABBStack();

	public AABB popAABB();

	public AABB[] popAABB(int argNum);

	public void pushAABB(int argNum);

	public Collision getCollision();

	public TimeOfImpact getTimeOfImpact();

	public Distance getDistance();

	public float[] getFloatArray(int argLength);

	public int[] getIntArray(int argLength);

	public Vector[] getVectorArray(int argLength);

}