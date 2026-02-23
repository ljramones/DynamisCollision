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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector2f;

class Sat2DTest {

    @Test
    void constructorRejectsConcavePolygon() {
        List<Vector2f> concave = List.of(
                new Vector2f(0, 0),
                new Vector2f(2, 0),
                new Vector2f(1, 1),
                new Vector2f(2, 2),
                new Vector2f(0, 2));

        assertThrows(IllegalArgumentException.class, () -> new ConvexPolygon2D(concave));
    }

    @Test
    void projectionIntervalDetectsOverlap() {
        ProjectionInterval a = new ProjectionInterval(0.0, 2.0);
        ProjectionInterval b = new ProjectionInterval(2.0, 5.0);
        ProjectionInterval c = new ProjectionInterval(2.1, 3.0);

        assertTrue(a.overlaps(b));
        assertTrue(a.overlapDepth(b) >= 0.0);
        assertFalse(a.overlaps(c));
        assertTrue(a.overlapDepth(c) < 0.0);
    }

    @Test
    void satDetectsSeparatedAndOverlappingPolygons() {
        ConvexPolygon2D a = rectangle(0, 0, 2, 2, 0);
        ConvexPolygon2D b = rectangle(5, 0, 2, 2, 0);
        ConvexPolygon2D c = rectangle(1.5, 0, 2, 2, Math.toRadians(30));

        assertFalse(Sat2D.intersects(a, b));
        assertTrue(Sat2D.intersects(a, c));
    }

    @Test
    void satTreatsTouchingEdgesAsIntersection() {
        ConvexPolygon2D a = rectangle(0, 0, 2, 2, 0);
        ConvexPolygon2D b = rectangle(2, 0, 2, 2, 0);

        assertTrue(Sat2D.intersects(a, b));
        Optional<CollisionManifold2D> manifold = Sat2D.intersectsWithManifold(a, b);
        assertTrue(manifold.isPresent());
        assertEquals(0.0, manifold.get().penetrationDepth(), 1e-9);
    }

    @Test
    void manifoldUsesMinimumTranslationAxis() {
        ConvexPolygon2D a = rectangle(0, 0, 2, 2, 0);
        ConvexPolygon2D b = rectangle(1.2, 0.1, 2, 2, Math.toRadians(15));

        CollisionManifold2D manifold = Sat2D.intersectsWithManifold(a, b).orElseThrow();
        assertTrue(manifold.penetrationDepth() > 0.0);

        double len = Math.hypot(manifold.normalX(), manifold.normalY());
        assertEquals(1.0, len, 1e-6);

        double centerDeltaX = b.centroid().x() - a.centroid().x();
        double centerDeltaY = b.centroid().y() - a.centroid().y();
        double directionalDot = centerDeltaX * manifold.normalX() + centerDeltaY * manifold.normalY();
        assertTrue(directionalDot >= -1e-9);
    }

    private static ConvexPolygon2D rectangle(
            double centerX,
            double centerY,
            double width,
            double height,
            double angleRadians) {
        double hw = width * 0.5;
        double hh = height * 0.5;
        Vector2f[] local = new Vector2f[] {
                new Vector2f((float) -hw, (float) -hh),
                new Vector2f((float) hw, (float) -hh),
                new Vector2f((float) hw, (float) hh),
                new Vector2f((float) -hw, (float) hh)
        };
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        List<Vector2f> world = java.util.Arrays.stream(local)
                .map(p -> new Vector2f(
                        (float) (centerX + (p.x() * cos - p.y() * sin)),
                        (float) (centerY + (p.x() * sin + p.y() * cos))))
                .toList();
        return new ConvexPolygon2D(world);
    }
}
