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

import org.junit.jupiter.api.Test;

class AabbAndSphereValidationTest {

    @Test
    void aabbValidatesBoundsOrder() {
        assertThrows(IllegalArgumentException.class, () -> new Aabb(2, 0, 0, 1, 1, 1));
        assertThrows(IllegalArgumentException.class, () -> new Aabb(0, 2, 0, 1, 1, 1));
        assertThrows(IllegalArgumentException.class, () -> new Aabb(0, 0, 2, 1, 1, 1));
    }

    @Test
    void sphereRequiresNonNegativeRadius() {
        assertThrows(IllegalArgumentException.class, () -> new BoundingSphere(0, 0, 0, -0.001));
    }

    @Test
    void sphereToAabbExpandsByRadius() {
        BoundingSphere sphere = new BoundingSphere(10, -2, 4, 3);
        Aabb aabb = sphere.toAabb();

        assertEquals(7.0, aabb.minX(), 1e-9);
        assertEquals(-5.0, aabb.minY(), 1e-9);
        assertEquals(1.0, aabb.minZ(), 1e-9);
        assertEquals(13.0, aabb.maxX(), 1e-9);
        assertEquals(1.0, aabb.maxY(), 1e-9);
        assertEquals(7.0, aabb.maxZ(), 1e-9);
    }

    @Test
    void rayRequiresNonZeroDirection() {
        assertThrows(IllegalArgumentException.class, () -> new Ray3D(0, 0, 0, 0, 0, 0));
    }
}
