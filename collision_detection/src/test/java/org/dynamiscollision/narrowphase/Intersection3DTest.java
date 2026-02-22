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
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class Intersection3DTest {

    @Test
    void aabbIntersectionIncludesTouchingFaces() {
        Aabb a = new Aabb(0, 0, 0, 1, 1, 1);
        Aabb b = new Aabb(1, 0, 0, 2, 1, 1);
        Aabb c = new Aabb(1.0001, 0, 0, 2, 1, 1);

        assertTrue(Intersection3D.intersects(a, b));
        assertFalse(Intersection3D.intersects(a, c));
    }

    @Test
    void sphereChecksHandleOverlappingAndSeparated() {
        BoundingSphere a = new BoundingSphere(0, 0, 0, 1);
        BoundingSphere b = new BoundingSphere(1.5, 0, 0, 0.6);
        BoundingSphere c = new BoundingSphere(3.0, 0, 0, 1);

        assertTrue(Intersection3D.intersects(a, b));
        assertFalse(Intersection3D.intersects(a, c));
    }

    @Test
    void sphereAabbCheckUsesClosestPoint() {
        BoundingSphere s1 = new BoundingSphere(0.2, 0.2, 0.2, 0.25);
        BoundingSphere s2 = new BoundingSphere(2.0, 2.0, 2.0, 0.25);
        Aabb box = new Aabb(0, 0, 0, 1, 1, 1);

        assertTrue(Intersection3D.intersects(s1, box));
        assertFalse(Intersection3D.intersects(s2, box));
    }

    @Test
    void rayAabbReturnsNearestNonNegativeHitDistance() {
        Aabb box = new Aabb(1, -1, -1, 3, 1, 1);
        Ray3D ray = new Ray3D(0, 0, 0, 1, 0, 0);

        OptionalDouble distance = Intersection3D.rayAabbIntersectionDistance(ray, box);
        assertTrue(distance.isPresent());
        assertEquals(1.0, distance.getAsDouble(), 1e-9);
    }

    @Test
    void rayAabbHandlesRayStartingInsideBox() {
        Aabb box = new Aabb(-1, -1, -1, 1, 1, 1);
        Ray3D ray = new Ray3D(0, 0, 0, 1, 0, 0);

        OptionalDouble distance = Intersection3D.rayAabbIntersectionDistance(ray, box);
        assertTrue(distance.isPresent());
        assertEquals(0.0, distance.getAsDouble(), 1e-9);
    }

    @Test
    void rayAabbReturnsEmptyWhenMissing() {
        Aabb box = new Aabb(5, 5, 5, 6, 6, 6);
        Ray3D ray = new Ray3D(0, 0, 0, 1, 0, 0);

        assertTrue(Intersection3D.rayAabbIntersectionDistance(ray, box).isEmpty());
        assertFalse(Intersection3D.intersects(ray, box));
    }
}
