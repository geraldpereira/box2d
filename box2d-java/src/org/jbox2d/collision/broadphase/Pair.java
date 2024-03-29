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
package org.jbox2d.collision.broadphase;

import java.util.Comparator;

// updated to rev 100
/**
 * Java note: at the "creation" of each node, a random key is given to that
 * node, and that's what we sort from.
 */
public class Pair {
	public DynamicTreeNode proxyA;
	public DynamicTreeNode proxyB;
	public int stableIdx;

	public int compareTo(final Pair pair2) {
		if (this.proxyA.key < pair2.proxyA.key) {
			return -1;
		}

		if (this.proxyA.key == pair2.proxyA.key) {

			if (proxyB.key < pair2.proxyB.key) {
				return -1;
			} else {
				if (proxyB.key == pair2.proxyB.key) {
					return 0;
				} else {
					return 1;
				}
			}
		}

		return 1;
	}

	/**
	 * Note: Must set pair.stableIdx before using to ensure stability.
	 */
	public static class StableComparator implements Comparator<Pair> {
		public int compare(final Pair pair1, final Pair pair2) {
			final int stdRtn = pair1.compareTo(pair2);
			if (stdRtn == 0) {
				if (pair1.stableIdx < pair2.stableIdx) {
					return -1;
				} else if (pair1.stableIdx > pair2.stableIdx) {
					return 1;
				} else {
					return 0;
				}
			} else {
				return stdRtn;
			}
		}
	}

	public static class PairComparator implements Comparator<Pair> {
		public int compare(final Pair pair1, final Pair pair2) {
			return pair1.compareTo(pair2);
		}
	}
}
