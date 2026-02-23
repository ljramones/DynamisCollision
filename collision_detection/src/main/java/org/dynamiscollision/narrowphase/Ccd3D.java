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

import java.util.OptionalDouble;
import org.dynamiscollision.bounds.Aabb;
import org.vectrix.core.Vector3d;

/**
 * Continuous collision detection utilities (time-of-impact style).
 */
public final class Ccd3D {

    private Ccd3D() {
    }

    /**
     * Returns first time of impact in [0,1] for segment start->end vs AABB.
     */
    public static OptionalDouble segmentAabbTimeOfImpact(Vector3d start, Vector3d end, Aabb aabb) {
        if (start == null || end == null || aabb == null) {
            throw new IllegalArgumentException("start, end and aabb must not be null");
        }
        double dirX = end.x() - start.x();
        double dirY = end.y() - start.y();
        double dirZ = end.z() - start.z();

        double tMin = 0.0;
        double tMax = 1.0;

        AxisResult x = clipAxis(start.x(), dirX, aabb.minX(), aabb.maxX(), tMin, tMax);
        if (!x.hit()) {
            return OptionalDouble.empty();
        }
        tMin = x.tMin();
        tMax = x.tMax();

        AxisResult y = clipAxis(start.y(), dirY, aabb.minY(), aabb.maxY(), tMin, tMax);
        if (!y.hit()) {
            return OptionalDouble.empty();
        }
        tMin = y.tMin();
        tMax = y.tMax();

        AxisResult z = clipAxis(start.z(), dirZ, aabb.minZ(), aabb.maxZ(), tMin, tMax);
        if (!z.hit()) {
            return OptionalDouble.empty();
        }
        tMin = z.tMin();

        return OptionalDouble.of(tMin);
    }

    public static boolean intersectsSegmentAabb(Vector3d start, Vector3d end, Aabb aabb) {
        return segmentAabbTimeOfImpact(start, end, aabb).isPresent();
    }

    /**
     * Returns first time of impact in [0,1] for moving AABB swept by delta against a static AABB.
     */
    public static OptionalDouble sweptAabbTimeOfImpact(Aabb moving, Vector3d delta, Aabb target) {
        if (moving == null || delta == null || target == null) {
            throw new IllegalArgumentException("moving, delta and target must not be null");
        }
        double halfX = moving.sizeX() * 0.5;
        double halfY = moving.sizeY() * 0.5;
        double halfZ = moving.sizeZ() * 0.5;

        Aabb expanded = new Aabb(
                target.minX() - halfX,
                target.minY() - halfY,
                target.minZ() - halfZ,
                target.maxX() + halfX,
                target.maxY() + halfY,
                target.maxZ() + halfZ);

        Vector3d start = new Vector3d(moving.centerX(), moving.centerY(), moving.centerZ());
        Vector3d end = new Vector3d(
                moving.centerX() + delta.x(),
                moving.centerY() + delta.y(),
                moving.centerZ() + delta.z());

        return segmentAabbTimeOfImpact(start, end, expanded);
    }

    /**
     * Approximates first time of impact in [0,1] for two moving convex shapes.
     *
     * This method performs sampled bracketing followed by binary refinement over GJK intersection tests.
     * For exact physics-grade TOI, replace with a full conservative advancement implementation.
     */
    public static OptionalDouble sweptConvexTimeOfImpact(
            ConvexSupport3D shapeA,
            Vector3d deltaA,
            ConvexSupport3D shapeB,
            Vector3d deltaB) {
        return sweptConvexTimeOfImpact(shapeA, deltaA, shapeB, deltaB, 32, 24);
    }

    /**
     * Approximates first time of impact in [0,1] for two moving convex shapes.
     *
     * @param samples number of coarse steps used to bracket first collision.
     * @param refinements number of bisection iterations after bracketing.
     */
    public static OptionalDouble sweptConvexTimeOfImpact(
            ConvexSupport3D shapeA,
            Vector3d deltaA,
            ConvexSupport3D shapeB,
            Vector3d deltaB,
            int samples,
            int refinements) {
        if (shapeA == null || deltaA == null || shapeB == null || deltaB == null) {
            throw new IllegalArgumentException("shape/delta arguments must not be null");
        }
        if (samples < 2) {
            throw new IllegalArgumentException("samples must be >= 2");
        }
        if (refinements < 1) {
            throw new IllegalArgumentException("refinements must be >= 1");
        }

        if (Gjk3D.intersects(shapeA, shapeB)) {
            return OptionalDouble.of(0.0);
        }

        double previousT = 0.0;
        for (int i = 1; i <= samples; i++) {
            double t = (double) i / samples;
            if (intersectsAt(shapeA, deltaA, shapeB, deltaB, t)) {
                double low = previousT;
                double high = t;
                for (int r = 0; r < refinements; r++) {
                    double mid = (low + high) * 0.5;
                    if (intersectsAt(shapeA, deltaA, shapeB, deltaB, mid)) {
                        high = mid;
                    } else {
                        low = mid;
                    }
                }
                return OptionalDouble.of(high);
            }
            previousT = t;
        }
        return OptionalDouble.empty();
    }

    private static boolean intersectsAt(
            ConvexSupport3D shapeA,
            Vector3d deltaA,
            ConvexSupport3D shapeB,
            Vector3d deltaB,
            double t) {
        ConvexSupport3D movedA = direction -> add(shapeA.support(direction), scaled(deltaA, t));
        ConvexSupport3D movedB = direction -> add(shapeB.support(direction), scaled(deltaB, t));
        return Gjk3D.intersects(movedA, movedB);
    }

    private static Vector3d scaled(Vector3d value, double scale) {
        return new Vector3d(value.x() * scale, value.y() * scale, value.z() * scale);
    }

    private static Vector3d add(Vector3d left, Vector3d right) {
        return new Vector3d(
                left.x() + right.x(),
                left.y() + right.y(),
                left.z() + right.z());
    }

    private static AxisResult clipAxis(double origin, double direction, double min, double max, double tMin, double tMax) {
        if (direction == 0.0) {
            if (origin < min || origin > max) {
                return AxisResult.miss();
            }
            return new AxisResult(true, tMin, tMax);
        }
        double inv = 1.0 / direction;
        double t0 = (min - origin) * inv;
        double t1 = (max - origin) * inv;
        if (t0 > t1) {
            double tmp = t0;
            t0 = t1;
            t1 = tmp;
        }
        double nextMin = Math.max(tMin, t0);
        double nextMax = Math.min(tMax, t1);
        if (nextMax < nextMin) {
            return AxisResult.miss();
        }
        return new AxisResult(true, nextMin, nextMax);
    }

    private record AxisResult(boolean hit, double tMin, double tMax) {
        private static AxisResult miss() {
            return new AxisResult(false, 0.0, -1.0);
        }
    }
}
