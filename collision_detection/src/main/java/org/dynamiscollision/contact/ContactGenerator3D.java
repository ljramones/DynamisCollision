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

/**
 * Narrow-phase contact generation helpers for primitive volumes.
 */
public final class ContactGenerator3D {

    private ContactGenerator3D() {
    }

    public static Optional<ContactManifold3D> generate(Aabb a, Aabb b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("a and b must not be null");
        }
        if (!Intersection3D.intersects(a, b)) {
            return Optional.empty();
        }

        double overlapX = Math.min(a.maxX(), b.maxX()) - Math.max(a.minX(), b.minX());
        double overlapY = Math.min(a.maxY(), b.maxY()) - Math.max(a.minY(), b.minY());
        double overlapZ = Math.min(a.maxZ(), b.maxZ()) - Math.max(a.minZ(), b.minZ());

        double normalX = 0.0;
        double normalY = 0.0;
        double normalZ = 0.0;
        double penetration = overlapX;

        double centerDeltaX = b.centerX() - a.centerX();
        double centerDeltaY = b.centerY() - a.centerY();
        double centerDeltaZ = b.centerZ() - a.centerZ();

        Axis axis = Axis.X;
        if (overlapY < penetration) {
            penetration = overlapY;
            axis = Axis.Y;
        }
        if (overlapZ < penetration) {
            penetration = overlapZ;
            axis = Axis.Z;
        }

        ContactPoint3D contact;
        switch (axis) {
            case X -> {
                normalX = centerDeltaX >= 0.0 ? 1.0 : -1.0;
                double x = normalX > 0.0 ? (a.maxX() + b.minX()) * 0.5 : (a.minX() + b.maxX()) * 0.5;
                double y = overlapCenter(a.minY(), a.maxY(), b.minY(), b.maxY());
                double z = overlapCenter(a.minZ(), a.maxZ(), b.minZ(), b.maxZ());
                contact = new ContactPoint3D(x, y, z);
            }
            case Y -> {
                normalY = centerDeltaY >= 0.0 ? 1.0 : -1.0;
                double x = overlapCenter(a.minX(), a.maxX(), b.minX(), b.maxX());
                double y = normalY > 0.0 ? (a.maxY() + b.minY()) * 0.5 : (a.minY() + b.maxY()) * 0.5;
                double z = overlapCenter(a.minZ(), a.maxZ(), b.minZ(), b.maxZ());
                contact = new ContactPoint3D(x, y, z);
            }
            case Z -> {
                normalZ = centerDeltaZ >= 0.0 ? 1.0 : -1.0;
                double x = overlapCenter(a.minX(), a.maxX(), b.minX(), b.maxX());
                double y = overlapCenter(a.minY(), a.maxY(), b.minY(), b.maxY());
                double z = normalZ > 0.0 ? (a.maxZ() + b.minZ()) * 0.5 : (a.minZ() + b.maxZ()) * 0.5;
                contact = new ContactPoint3D(x, y, z);
            }
            default -> throw new IllegalStateException("Unknown axis");
        }

        CollisionManifold3D manifold = new CollisionManifold3D(normalX, normalY, normalZ, penetration);
        return Optional.of(new ContactManifold3D(manifold, List.of(contact)));
    }

    public static Optional<ContactManifold3D> generate(BoundingSphere a, BoundingSphere b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("a and b must not be null");
        }
        double dx = b.centerX() - a.centerX();
        double dy = b.centerY() - a.centerY();
        double dz = b.centerZ() - a.centerZ();
        double distSq = dx * dx + dy * dy + dz * dz;
        double radiusSum = a.radius() + b.radius();
        if (distSq > radiusSum * radiusSum) {
            return Optional.empty();
        }

        double dist = Math.sqrt(distSq);
        double nx;
        double ny;
        double nz;
        if (dist <= 1e-9) {
            nx = 1.0;
            ny = 0.0;
            nz = 0.0;
            dist = 0.0;
        } else {
            nx = dx / dist;
            ny = dy / dist;
            nz = dz / dist;
        }

        double penetration = radiusSum - dist;
        double ax = a.centerX() + nx * a.radius();
        double ay = a.centerY() + ny * a.radius();
        double az = a.centerZ() + nz * a.radius();
        double bx = b.centerX() - nx * b.radius();
        double by = b.centerY() - ny * b.radius();
        double bz = b.centerZ() - nz * b.radius();

        ContactPoint3D contact = new ContactPoint3D(
                (ax + bx) * 0.5,
                (ay + by) * 0.5,
                (az + bz) * 0.5);

        CollisionManifold3D manifold = new CollisionManifold3D(nx, ny, nz, penetration);
        return Optional.of(new ContactManifold3D(manifold, List.of(contact)));
    }

    private static double overlapCenter(double minA, double maxA, double minB, double maxB) {
        double min = Math.max(minA, minB);
        double max = Math.min(maxA, maxB);
        return (min + max) * 0.5;
    }

    private enum Axis {
        X, Y, Z
    }
}
