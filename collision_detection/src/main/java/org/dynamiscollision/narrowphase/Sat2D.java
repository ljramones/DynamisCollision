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

import java.util.List;
import java.util.Optional;
import org.vectrix.core.Vector2f;

/**
 * Separating Axis Theorem checks for 2D convex polygons.
 */
public final class Sat2D {

    private static final double EPSILON = 1e-9;

    private Sat2D() {
    }

    public static boolean intersects(ConvexPolygon2D a, ConvexPolygon2D b) {
        return intersectsWithManifold(a, b).isPresent();
    }

    public static Optional<CollisionManifold2D> intersectsWithManifold(ConvexPolygon2D a, ConvexPolygon2D b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("polygons must not be null");
        }

        Vector2f centerA = a.centroid();
        Vector2f centerB = b.centroid();
        Vector2f centerDelta = new Vector2f(centerB.x() - centerA.x(), centerB.y() - centerA.y());

        double minOverlap = Double.POSITIVE_INFINITY;
        double bestAxisX = 0.0;
        double bestAxisY = 0.0;

        AxisResult fromA = evaluateAxes(a.vertices(), a, b, centerDelta, minOverlap, bestAxisX, bestAxisY);
        if (!fromA.hit()) {
            return Optional.empty();
        }
        minOverlap = fromA.minOverlap();
        bestAxisX = fromA.axisX();
        bestAxisY = fromA.axisY();

        AxisResult fromB = evaluateAxes(b.vertices(), a, b, centerDelta, minOverlap, bestAxisX, bestAxisY);
        if (!fromB.hit()) {
            return Optional.empty();
        }
        minOverlap = fromB.minOverlap();
        bestAxisX = fromB.axisX();
        bestAxisY = fromB.axisY();

        return Optional.of(new CollisionManifold2D(bestAxisX, bestAxisY, minOverlap));
    }

    public static ProjectionInterval project(ConvexPolygon2D polygon, double axisX, double axisY) {
        if (polygon == null) {
            throw new IllegalArgumentException("polygon must not be null");
        }
        if (!Double.isFinite(axisX) || !Double.isFinite(axisY)) {
            throw new IllegalArgumentException("axis components must be finite");
        }
        double axisLengthSq = axisX * axisX + axisY * axisY;
        if (axisLengthSq <= EPSILON) {
            throw new IllegalArgumentException("axis must be non-zero");
        }
        double invLen = 1.0 / Math.sqrt(axisLengthSq);
        double nx = axisX * invLen;
        double ny = axisY * invLen;

        List<Vector2f> points = polygon.vertices();
        double first = points.get(0).x() * nx + points.get(0).y() * ny;
        double min = first;
        double max = first;
        for (int i = 1; i < points.size(); i++) {
            Vector2f p = points.get(i);
            double projection = p.x() * nx + p.y() * ny;
            min = Math.min(min, projection);
            max = Math.max(max, projection);
        }
        return new ProjectionInterval(min, max);
    }

    private static AxisResult evaluateAxes(
            List<Vector2f> sourceVertices,
            ConvexPolygon2D a,
            ConvexPolygon2D b,
            Vector2f centerDelta,
            double startMinOverlap,
            double startAxisX,
            double startAxisY) {

        double minOverlap = startMinOverlap;
        double bestAxisX = startAxisX;
        double bestAxisY = startAxisY;

        for (int i = 0; i < sourceVertices.size(); i++) {
            Vector2f p0 = sourceVertices.get(i);
            Vector2f p1 = sourceVertices.get((i + 1) % sourceVertices.size());
            double edgeX = p1.x() - p0.x();
            double edgeY = p1.y() - p0.y();
            double axisX = -edgeY;
            double axisY = edgeX;
            double axisLengthSq = axisX * axisX + axisY * axisY;
            if (axisLengthSq <= EPSILON) {
                continue;
            }
            double invLen = 1.0 / Math.sqrt(axisLengthSq);
            axisX *= invLen;
            axisY *= invLen;

            ProjectionInterval projA = project(a, axisX, axisY);
            ProjectionInterval projB = project(b, axisX, axisY);
            double overlap = projA.overlapDepth(projB);
            if (overlap < 0.0) {
                return AxisResult.miss();
            }
            if (overlap < minOverlap) {
                double orientedAxisX = axisX;
                double orientedAxisY = axisY;
                if ((centerDelta.x() * axisX + centerDelta.y() * axisY) < 0.0) {
                    orientedAxisX = -axisX;
                    orientedAxisY = -axisY;
                }
                minOverlap = overlap;
                bestAxisX = orientedAxisX;
                bestAxisY = orientedAxisY;
            }
        }
        return new AxisResult(true, minOverlap, bestAxisX, bestAxisY);
    }

    private record AxisResult(boolean hit, double minOverlap, double axisX, double axisY) {
        private static AxisResult miss() {
            return new AxisResult(false, 0.0, 0.0, 0.0);
        }
    }
}
