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

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.vectrix.core.Vector2f;

/**
 * Immutable convex polygon in 2D.
 */
public final class ConvexPolygon2D {

    private static final double EPSILON = 1e-9;

    private final List<Vector2f> vertices;

    public ConvexPolygon2D(List<Vector2f> vertices) {
        if (vertices == null) {
            throw new IllegalArgumentException("vertices must not be null");
        }
        if (vertices.size() < 3) {
            throw new IllegalArgumentException("convex polygon requires at least 3 points");
        }
        List<Vector2f> copy = new ArrayList<>(vertices.size());
        for (int i = 0; i < vertices.size(); i++) {
            Vector2f p = vertices.get(i);
            if (p == null || !Float.isFinite(p.x()) || !Float.isFinite(p.y())) {
                throw new IllegalArgumentException("all vertices must be non-null finite points");
            }
            copy.add(new Vector2f(p));
        }
        validateEdges(copy);
        validateConvex(copy);
        this.vertices = List.copyOf(copy);
    }

    public static ConvexPolygon2D of(double... coordinates) {
        if (coordinates == null || coordinates.length < 6 || coordinates.length % 2 != 0) {
            throw new IllegalArgumentException("coordinates must contain at least 3 xy pairs");
        }
        List<Vector2f> points = new ArrayList<>(coordinates.length / 2);
        for (int i = 0; i < coordinates.length; i += 2) {
            points.add(new Vector2f((float) coordinates[i], (float) coordinates[i + 1]));
        }
        return new ConvexPolygon2D(points);
    }

    public List<Vector2f> vertices() {
        return Collections.unmodifiableList(vertices);
    }

    public int vertexCount() {
        return vertices.size();
    }

    public Vector2f centroid() {
        double sumX = 0.0;
        double sumY = 0.0;
        for (Vector2f p : vertices) {
            sumX += p.x();
            sumY += p.y();
        }
        return new Vector2f((float) (sumX / vertices.size()), (float) (sumY / vertices.size()));
    }

    private static void validateEdges(List<Vector2f> points) {
        for (int i = 0; i < points.size(); i++) {
            Vector2f a = points.get(i);
            Vector2f b = points.get((i + 1) % points.size());
            if (Vector2f.distance(a.x(), a.y(), b.x(), b.y()) <= EPSILON) {
                throw new IllegalArgumentException("polygon contains duplicate consecutive points");
            }
        }
    }

    private static void validateConvex(List<Vector2f> points) {
        int sign = 0;
        for (int i = 0; i < points.size(); i++) {
            Vector2f a = points.get(i);
            Vector2f b = points.get((i + 1) % points.size());
            Vector2f c = points.get((i + 2) % points.size());
            double cross = crossZ(a, b, c);
            if (Math.abs(cross) <= EPSILON) {
                continue;
            }
            int currentSign = cross > 0 ? 1 : -1;
            if (sign == 0) {
                sign = currentSign;
            } else if (sign != currentSign) {
                throw new IllegalArgumentException("polygon must be convex and ordered");
            }
        }
        if (sign == 0) {
            throw new IllegalArgumentException("polygon points are collinear");
        }
    }

    private static double crossZ(Vector2f a, Vector2f b, Vector2f c) {
        double abX = b.x() - a.x();
        double abY = b.y() - a.y();
        double bcX = c.x() - b.x();
        double bcY = c.y() - b.y();
        return abX * bcY - abY * bcX;
    }
}
