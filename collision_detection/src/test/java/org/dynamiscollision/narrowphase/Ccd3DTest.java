/*
 * Copyright 2024-2026 DynamisCollision Contributors
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

package org.dynamiscollision.narrowphase;

import static org.junit.jupiter.api.Assertions.*;

import java.util.OptionalDouble;
import org.dynamiscollision.bounds.Aabb;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3d;

class Ccd3DTest {

    @Test
    void segmentAabbTimeOfImpactHitsAtExpectedFraction() {
        Aabb box = new Aabb(5, -1, -1, 6, 1, 1);
        Vector3d start = new Vector3d(0, 0, 0);
        Vector3d end = new Vector3d(10, 0, 0);

        OptionalDouble toi = Ccd3D.segmentAabbTimeOfImpact(start, end, box);
        assertTrue(toi.isPresent());
        assertEquals(0.5, toi.getAsDouble(), 1e-9);
    }

    @Test
    void segmentAabbTimeOfImpactReturnsEmptyOnMiss() {
        Aabb box = new Aabb(5, 5, 5, 6, 6, 6);
        Vector3d start = new Vector3d(0, 0, 0);
        Vector3d end = new Vector3d(1, 0, 0);

        assertTrue(Ccd3D.segmentAabbTimeOfImpact(start, end, box).isEmpty());
    }

    @Test
    void sweptAabbTimeOfImpactMatchesExpectedContact() {
        Aabb moving = new Aabb(0, 0, 0, 1, 1, 1);
        Aabb target = new Aabb(5, 0, 0, 6, 1, 1);
        Vector3d delta = new Vector3d(10, 0, 0);

        OptionalDouble toi = Ccd3D.sweptAabbTimeOfImpact(moving, delta, target);
        assertTrue(toi.isPresent());
        assertEquals(0.4, toi.getAsDouble(), 1e-9);
    }

    @Test
    void sweptAabbTimeOfImpactIsZeroWhenStartingOverlapped() {
        Aabb moving = new Aabb(0, 0, 0, 2, 2, 2);
        Aabb target = new Aabb(1, 1, 1, 3, 3, 3);
        Vector3d delta = new Vector3d(5, 0, 0);

        OptionalDouble toi = Ccd3D.sweptAabbTimeOfImpact(moving, delta, target);
        assertTrue(toi.isPresent());
        assertEquals(0.0, toi.getAsDouble(), 1e-9);
    }

    @Test
    void sweptConvexTimeOfImpactDetectsCollision() {
        ConvexSupport3D shapeA = Gjk3D.fromAabb(new Aabb(0, 0, 0, 1, 1, 1));
        ConvexSupport3D shapeB = Gjk3D.fromAabb(new Aabb(3, 0, 0, 4, 1, 1));
        Vector3d deltaA = new Vector3d(4, 0, 0);
        Vector3d deltaB = new Vector3d(0, 0, 0);

        OptionalDouble toi = Ccd3D.sweptConvexTimeOfImpact(shapeA, deltaA, shapeB, deltaB, 64, 24);
        assertTrue(toi.isPresent());
        assertEquals(0.5, toi.getAsDouble(), 0.02);
    }

    @Test
    void sweptConvexTimeOfImpactReturnsEmptyWhenNoHit() {
        ConvexSupport3D shapeA = Gjk3D.fromAabb(new Aabb(0, 0, 0, 1, 1, 1));
        ConvexSupport3D shapeB = Gjk3D.fromAabb(new Aabb(10, 0, 0, 11, 1, 1));

        OptionalDouble toi = Ccd3D.sweptConvexTimeOfImpact(
                shapeA, new Vector3d(1, 0, 0),
                shapeB, new Vector3d(0, 0, 0));
        assertTrue(toi.isEmpty());
    }

    @Test
    void sweptConvexTimeOfImpactIsZeroForInitialOverlap() {
        ConvexSupport3D shapeA = Gjk3D.fromAabb(new Aabb(0, 0, 0, 2, 2, 2));
        ConvexSupport3D shapeB = Gjk3D.fromAabb(new Aabb(1, 1, 1, 3, 3, 3));

        OptionalDouble toi = Ccd3D.sweptConvexTimeOfImpact(
                shapeA, new Vector3d(5, 0, 0),
                shapeB, new Vector3d(0, 0, 0));
        assertTrue(toi.isPresent());
        assertEquals(0.0, toi.getAsDouble(), 1e-9);
    }
}
