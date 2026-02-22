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
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class SpatialHash3DTest {

    @Test
    void constructorRejectsInvalidCellSize() {
        assertThrows(IllegalArgumentException.class, () -> new SpatialHash3D<String>(0));
        assertThrows(IllegalArgumentException.class, () -> new SpatialHash3D<String>(-1));
    }

    @Test
    void potentialPairsIncludeOverlappingBucketsOnly() {
        record Body(String id, Aabb aabb) {
        }

        Body a = new Body("a", new Aabb(0.0, 0.0, 0.0, 0.9, 0.9, 0.9));
        Body b = new Body("b", new Aabb(0.8, 0.0, 0.0, 1.6, 0.9, 0.9));
        Body c = new Body("c", new Aabb(5.0, 5.0, 5.0, 6.0, 6.0, 6.0));

        SpatialHash3D<Body> hash = new SpatialHash3D<>(1.0);
        Set<CollisionPair<Body>> pairs = hash.findPotentialPairs(List.of(a, b, c), Body::aabb);

        assertEquals(1, pairs.size());
        assertTrue(pairs.contains(new CollisionPair<>(a, b)));
    }

    @Test
    void pairSetIsUniqueWhenObjectsShareMultipleCells() {
        record Body(String id, Aabb aabb) {
        }

        Body a = new Body("a", new Aabb(0.0, 0.0, 0.0, 2.2, 2.2, 2.2));
        Body b = new Body("b", new Aabb(1.0, 1.0, 1.0, 3.0, 3.0, 3.0));

        SpatialHash3D<Body> hash = new SpatialHash3D<>(1.0);
        Set<CollisionPair<Body>> pairs = hash.findPotentialPairs(List.of(a, b), Body::aabb);

        assertEquals(1, pairs.size());
        assertTrue(pairs.contains(new CollisionPair<>(a, b)));
    }
}
