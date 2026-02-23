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
import org.dynamiscollision.bounds.BoundingSphere;
import org.dynamiscollision.bounds.Capsule;
import org.dynamiscollision.geometry.Ray3D;

/**
 * Utility methods for primitive 3D intersection checks.
 */
public final class Intersection3D {

    private Intersection3D() {
    }

    public static boolean intersects(Aabb a, Aabb b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("a and b must not be null");
        }
        return a.minX() <= b.maxX() && a.maxX() >= b.minX()
                && a.minY() <= b.maxY() && a.maxY() >= b.minY()
                && a.minZ() <= b.maxZ() && a.maxZ() >= b.minZ();
    }

    public static boolean intersects(BoundingSphere a, BoundingSphere b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("a and b must not be null");
        }
        double dx = a.centerX() - b.centerX();
        double dy = a.centerY() - b.centerY();
        double dz = a.centerZ() - b.centerZ();
        double radius = a.radius() + b.radius();
        return (dx * dx) + (dy * dy) + (dz * dz) <= radius * radius;
    }

    public static boolean intersects(BoundingSphere sphere, Aabb aabb) {
        if (sphere == null || aabb == null) {
            throw new IllegalArgumentException("sphere and aabb must not be null");
        }
        double x = clamp(sphere.centerX(), aabb.minX(), aabb.maxX());
        double y = clamp(sphere.centerY(), aabb.minY(), aabb.maxY());
        double z = clamp(sphere.centerZ(), aabb.minZ(), aabb.maxZ());

        double dx = sphere.centerX() - x;
        double dy = sphere.centerY() - y;
        double dz = sphere.centerZ() - z;
        return (dx * dx) + (dy * dy) + (dz * dz) <= sphere.radius() * sphere.radius();
    }

    public static boolean intersects(Aabb aabb, BoundingSphere sphere) {
        return intersects(sphere, aabb);
    }

    public static boolean intersects(Capsule a, Capsule b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("a and b must not be null");
        }
        double distanceSq = segmentSegmentDistanceSq(
                a.pointAX(), a.pointAY(), a.pointAZ(),
                a.pointBX(), a.pointBY(), a.pointBZ(),
                b.pointAX(), b.pointAY(), b.pointAZ(),
                b.pointBX(), b.pointBY(), b.pointBZ());
        double radius = a.radius() + b.radius();
        return distanceSq <= radius * radius;
    }

    public static boolean intersects(Capsule capsule, BoundingSphere sphere) {
        if (capsule == null || sphere == null) {
            throw new IllegalArgumentException("capsule and sphere must not be null");
        }
        double distanceSq = pointSegmentDistanceSq(
                sphere.centerX(), sphere.centerY(), sphere.centerZ(),
                capsule.pointAX(), capsule.pointAY(), capsule.pointAZ(),
                capsule.pointBX(), capsule.pointBY(), capsule.pointBZ());
        double radius = capsule.radius() + sphere.radius();
        return distanceSq <= radius * radius;
    }

    public static boolean intersects(BoundingSphere sphere, Capsule capsule) {
        return intersects(capsule, sphere);
    }

    public static boolean intersects(Capsule capsule, Aabb aabb) {
        if (capsule == null || aabb == null) {
            throw new IllegalArgumentException("capsule and aabb must not be null");
        }
        return Gjk3D.intersects(Gjk3D.fromCapsule(capsule), Gjk3D.fromAabb(aabb));
    }

    public static boolean intersects(Aabb aabb, Capsule capsule) {
        return intersects(capsule, aabb);
    }

    /**
     * Returns the nearest non-negative distance along the ray to the first AABB intersection.
     * Empty when no intersection exists.
     */
    public static OptionalDouble rayAabbIntersectionDistance(Ray3D ray, Aabb aabb) {
        if (ray == null || aabb == null) {
            throw new IllegalArgumentException("ray and aabb must not be null");
        }

        double tMin = 0.0;
        double tMax = Double.POSITIVE_INFINITY;

        AxisResult xAxis = axisInterval(ray.originX(), ray.dirX(), aabb.minX(), aabb.maxX());
        if (!xAxis.hit()) {
            return OptionalDouble.empty();
        }
        tMin = Math.max(tMin, xAxis.tMin());
        tMax = Math.min(tMax, xAxis.tMax());
        if (tMax < tMin) {
            return OptionalDouble.empty();
        }

        AxisResult yAxis = axisInterval(ray.originY(), ray.dirY(), aabb.minY(), aabb.maxY());
        if (!yAxis.hit()) {
            return OptionalDouble.empty();
        }
        tMin = Math.max(tMin, yAxis.tMin());
        tMax = Math.min(tMax, yAxis.tMax());
        if (tMax < tMin) {
            return OptionalDouble.empty();
        }

        AxisResult zAxis = axisInterval(ray.originZ(), ray.dirZ(), aabb.minZ(), aabb.maxZ());
        if (!zAxis.hit()) {
            return OptionalDouble.empty();
        }
        tMin = Math.max(tMin, zAxis.tMin());
        tMax = Math.min(tMax, zAxis.tMax());
        if (tMax < tMin) {
            return OptionalDouble.empty();
        }

        return OptionalDouble.of(tMin);
    }

    public static boolean intersects(Ray3D ray, Aabb aabb) {
        return rayAabbIntersectionDistance(ray, aabb).isPresent();
    }

    private static AxisResult axisInterval(double origin, double direction, double min, double max) {
        if (direction == 0.0) {
            return origin >= min && origin <= max
                    ? new AxisResult(true, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY)
                    : new AxisResult(false, 0.0, -1.0);
        }
        double inv = 1.0 / direction;
        double t0 = (min - origin) * inv;
        double t1 = (max - origin) * inv;
        if (t0 > t1) {
            double temp = t0;
            t0 = t1;
            t1 = temp;
        }
        return new AxisResult(true, t0, t1);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double pointSegmentDistanceSq(
            double px,
            double py,
            double pz,
            double ax,
            double ay,
            double az,
            double bx,
            double by,
            double bz) {
        double abx = bx - ax;
        double aby = by - ay;
        double abz = bz - az;
        double apx = px - ax;
        double apy = py - ay;
        double apz = pz - az;
        double abLenSq = (abx * abx) + (aby * aby) + (abz * abz);
        if (abLenSq <= 1e-12) {
            double dx = px - ax;
            double dy = py - ay;
            double dz = pz - az;
            return (dx * dx) + (dy * dy) + (dz * dz);
        }
        double t = clamp(((apx * abx) + (apy * aby) + (apz * abz)) / abLenSq, 0.0, 1.0);
        double cx = ax + abx * t;
        double cy = ay + aby * t;
        double cz = az + abz * t;
        double dx = px - cx;
        double dy = py - cy;
        double dz = pz - cz;
        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    // Closest-distance between two finite 3D segments.
    private static double segmentSegmentDistanceSq(
            double p1x,
            double p1y,
            double p1z,
            double q1x,
            double q1y,
            double q1z,
            double p2x,
            double p2y,
            double p2z,
            double q2x,
            double q2y,
            double q2z) {
        double d1x = q1x - p1x;
        double d1y = q1y - p1y;
        double d1z = q1z - p1z;
        double d2x = q2x - p2x;
        double d2y = q2y - p2y;
        double d2z = q2z - p2z;
        double rx = p1x - p2x;
        double ry = p1y - p2y;
        double rz = p1z - p2z;

        double a = dot(d1x, d1y, d1z, d1x, d1y, d1z);
        double e = dot(d2x, d2y, d2z, d2x, d2y, d2z);
        double f = dot(d2x, d2y, d2z, rx, ry, rz);
        double s;
        double t;

        if (a <= 1e-12 && e <= 1e-12) {
            return dot(rx, ry, rz, rx, ry, rz);
        }
        if (a <= 1e-12) {
            s = 0.0;
            t = clamp(f / e, 0.0, 1.0);
        } else {
            double c = dot(d1x, d1y, d1z, rx, ry, rz);
            if (e <= 1e-12) {
                t = 0.0;
                s = clamp(-c / a, 0.0, 1.0);
            } else {
                double b = dot(d1x, d1y, d1z, d2x, d2y, d2z);
                double denom = a * e - b * b;
                if (denom != 0.0) {
                    s = clamp((b * f - c * e) / denom, 0.0, 1.0);
                } else {
                    s = 0.0;
                }
                double tNom = b * s + f;
                if (tNom < 0.0) {
                    t = 0.0;
                    s = clamp(-c / a, 0.0, 1.0);
                } else if (tNom > e) {
                    t = 1.0;
                    s = clamp((b - c) / a, 0.0, 1.0);
                } else {
                    t = tNom / e;
                }
            }
        }

        double cx = rx + d1x * s - d2x * t;
        double cy = ry + d1y * s - d2y * t;
        double cz = rz + d1z * s - d2z * t;
        return dot(cx, cy, cz, cx, cy, cz);
    }

    private static double dot(double ax, double ay, double az, double bx, double by, double bz) {
        return ax * bx + ay * by + az * bz;
    }

    private record AxisResult(boolean hit, double tMin, double tMax) {
    }
}
