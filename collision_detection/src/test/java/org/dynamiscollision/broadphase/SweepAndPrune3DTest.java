/*
 * Copyright 2024-2026 DynamisFX Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.dynamiscollision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class SweepAndPrune3DTest {

    @Test
    void findsPairsThatOverlapOnAllAxes() {
        record Body(String id, Aabb bounds) {
        }

        Body a = new Body("a", new Aabb(0, 0, 0, 2, 2, 2));
        Body b = new Body("b", new Aabb(1, 1, 1, 3, 3, 3));
        Body c = new Body("c", new Aabb(3.1, 0, 0, 4, 1, 1));

        SweepAndPrune3D<Body> sap = new SweepAndPrune3D<>();
        Set<CollisionPair<Body>> pairs = sap.findPotentialPairs(List.of(a, b, c), Body::bounds);

        assertEquals(1, pairs.size());
        assertTrue(pairs.contains(new CollisionPair<>(a, b)));
    }

    @Test
    void touchingBoundsCountAsPotentialPair() {
        record Body(String id, Aabb bounds) {
        }

        Body a = new Body("a", new Aabb(0, 0, 0, 1, 1, 1));
        Body b = new Body("b", new Aabb(1, 0, 0, 2, 1, 1));

        SweepAndPrune3D<Body> sap = new SweepAndPrune3D<>();
        Set<CollisionPair<Body>> pairs = sap.findPotentialPairs(List.of(a, b), Body::bounds);

        assertEquals(1, pairs.size());
        assertTrue(pairs.contains(new CollisionPair<>(a, b)));
    }

    @Test
    void supportsBroadPhaseInterfaceUsage() {
        record Body(String id, Aabb bounds) {
        }

        Body a = new Body("a", new Aabb(0, 0, 0, 1, 1, 1));
        Body b = new Body("b", new Aabb(0.5, 0.5, 0.5, 1.5, 1.5, 1.5));

        BroadPhase3D<Body> broadPhase = new SweepAndPrune3D<>();
        Set<CollisionPair<Body>> pairs = broadPhase.findPotentialPairs(List.of(a, b), Body::bounds);

        assertEquals(1, pairs.size());
    }
}
