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

package org.dynamiscollision.contact;

import java.util.List;
import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.bounds.BoundingSphere;
import org.dynamiscollision.bounds.Capsule;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.narrowphase.Intersection3D;

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

    public static Optional<ContactManifold3D> generate(Capsule a, Capsule b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("a and b must not be null");
        }
        ClosestSegmentPoints closest = closestPointsBetweenSegments(
                vec(a.pointAX(), a.pointAY(), a.pointAZ()),
                vec(a.pointBX(), a.pointBY(), a.pointBZ()),
                vec(b.pointAX(), b.pointAY(), b.pointAZ()),
                vec(b.pointBX(), b.pointBY(), b.pointBZ()));
        return sphereLikeContact(closest.pointOnA(), a.radius(), closest.pointOnB(), b.radius());
    }

    public static Optional<ContactManifold3D> generate(Capsule capsule, BoundingSphere sphere) {
        if (capsule == null || sphere == null) {
            throw new IllegalArgumentException("capsule and sphere must not be null");
        }
        Vec3 capsulePoint = closestPointOnSegment(
                vec(capsule.pointAX(), capsule.pointAY(), capsule.pointAZ()),
                vec(capsule.pointBX(), capsule.pointBY(), capsule.pointBZ()),
                vec(sphere.centerX(), sphere.centerY(), sphere.centerZ()));
        Vec3 sphereCenter = vec(sphere.centerX(), sphere.centerY(), sphere.centerZ());
        return sphereLikeContact(capsulePoint, capsule.radius(), sphereCenter, sphere.radius());
    }

    public static Optional<ContactManifold3D> generate(Capsule capsule, Aabb aabb) {
        if (capsule == null || aabb == null) {
            throw new IllegalArgumentException("capsule and aabb must not be null");
        }
        SegmentAabbClosest closest = closestPointsSegmentAabb(
                vec(capsule.pointAX(), capsule.pointAY(), capsule.pointAZ()),
                vec(capsule.pointBX(), capsule.pointBY(), capsule.pointBZ()),
                aabb);
        double distSq = distanceSq(closest.segmentPoint(), closest.boxPoint());
        double radius = capsule.radius();
        if (distSq > radius * radius) {
            return Optional.empty();
        }

        Vec3 normal;
        Vec3 boxSurface = closest.boxPoint();
        if (distSq > EPS * EPS) {
            normal = normalize(sub(closest.boxPoint(), closest.segmentPoint()));
        } else {
            FaceProjection projection = nearestFaceProjection(closest.segmentPoint(), aabb);
            normal = projection.normal();
            boxSurface = projection.pointOnFace();
        }
        double distance = Math.sqrt(Math.max(0.0, distSq));
        double penetration = radius - distance;
        Vec3 capsuleSurface = add(closest.segmentPoint(), scale(normal, radius));
        ContactPoint3D contact = midpoint(capsuleSurface, boxSurface);
        CollisionManifold3D manifold = new CollisionManifold3D(normal.x(), normal.y(), normal.z(), penetration);
        return Optional.of(new ContactManifold3D(manifold, List.of(contact)));
    }

    public static Optional<ContactManifold3D> generate(BoundingSphere sphere, Capsule capsule) {
        Optional<ContactManifold3D> fromCapsule = generate(capsule, sphere);
        if (fromCapsule.isEmpty()) {
            return Optional.empty();
        }
        CollisionManifold3D original = fromCapsule.get().manifold();
        CollisionManifold3D flipped = new CollisionManifold3D(
                -original.normalX(),
                -original.normalY(),
                -original.normalZ(),
                original.penetrationDepth());
        return Optional.of(new ContactManifold3D(flipped, fromCapsule.get().contacts()));
    }

    public static Optional<ContactManifold3D> generate(Aabb aabb, Capsule capsule) {
        Optional<ContactManifold3D> fromCapsule = generate(capsule, aabb);
        if (fromCapsule.isEmpty()) {
            return Optional.empty();
        }
        CollisionManifold3D original = fromCapsule.get().manifold();
        CollisionManifold3D flipped = new CollisionManifold3D(
                -original.normalX(),
                -original.normalY(),
                -original.normalZ(),
                original.penetrationDepth());
        return Optional.of(new ContactManifold3D(flipped, fromCapsule.get().contacts()));
    }

    private static double overlapCenter(double minA, double maxA, double minB, double maxB) {
        double min = Math.max(minA, minB);
        double max = Math.min(maxA, maxB);
        return (min + max) * 0.5;
    }

    private static Optional<ContactManifold3D> sphereLikeContact(Vec3 centerA, double radiusA, Vec3 centerB, double radiusB) {
        double distSq = distanceSq(centerA, centerB);
        double radiusSum = radiusA + radiusB;
        if (distSq > radiusSum * radiusSum) {
            return Optional.empty();
        }

        double dist = Math.sqrt(Math.max(0.0, distSq));
        Vec3 normal;
        if (dist <= EPS) {
            normal = vec(1.0, 0.0, 0.0);
            dist = 0.0;
        } else {
            normal = scale(sub(centerB, centerA), 1.0 / dist);
        }
        double penetration = radiusSum - dist;
        Vec3 surfaceA = add(centerA, scale(normal, radiusA));
        Vec3 surfaceB = sub(centerB, scale(normal, radiusB));
        ContactPoint3D contact = midpoint(surfaceA, surfaceB);
        CollisionManifold3D manifold = new CollisionManifold3D(normal.x(), normal.y(), normal.z(), penetration);
        return Optional.of(new ContactManifold3D(manifold, List.of(contact)));
    }

    private static ClosestSegmentPoints closestPointsBetweenSegments(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2) {
        Vec3 d1 = sub(q1, p1);
        Vec3 d2 = sub(q2, p2);
        Vec3 r = sub(p1, p2);
        double a = dot(d1, d1);
        double e = dot(d2, d2);
        double f = dot(d2, r);

        double s;
        double t;
        if (a <= EPS && e <= EPS) {
            return new ClosestSegmentPoints(p1, p2);
        }
        if (a <= EPS) {
            s = 0.0;
            t = clamp01(f / Math.max(e, EPS));
        } else {
            double c = dot(d1, r);
            if (e <= EPS) {
                t = 0.0;
                s = clamp01(-c / a);
            } else {
                double b = dot(d1, d2);
                double denom = a * e - b * b;
                if (Math.abs(denom) <= EPS) {
                    s = 0.0;
                } else {
                    s = clamp01((b * f - c * e) / denom);
                }
                t = (b * s + f) / e;
                if (t < 0.0) {
                    t = 0.0;
                    s = clamp01(-c / a);
                } else if (t > 1.0) {
                    t = 1.0;
                    s = clamp01((b - c) / a);
                }
            }
        }

        Vec3 pointOnA = add(p1, scale(d1, s));
        Vec3 pointOnB = add(p2, scale(d2, t));
        return new ClosestSegmentPoints(pointOnA, pointOnB);
    }

    private static Vec3 closestPointOnSegment(Vec3 a, Vec3 b, Vec3 point) {
        Vec3 ab = sub(b, a);
        double lenSq = dot(ab, ab);
        if (lenSq <= EPS) {
            return a;
        }
        double t = clamp01(dot(sub(point, a), ab) / lenSq);
        return add(a, scale(ab, t));
    }

    private static SegmentAabbClosest closestPointsSegmentAabb(Vec3 a, Vec3 b, Aabb box) {
        if (distanceSq(a, b) <= EPS * EPS) {
            Vec3 boxPoint = closestPointOnAabb(a, box);
            return new SegmentAabbClosest(a, boxPoint);
        }

        double lo = 0.0;
        double hi = 1.0;
        for (int i = 0; i < 48; i++) {
            double third = (hi - lo) / 3.0;
            double t1 = lo + third;
            double t2 = hi - third;
            double d1 = distanceSqPointAabb(pointOnSegment(a, b, t1), box);
            double d2 = distanceSqPointAabb(pointOnSegment(a, b, t2), box);
            if (d1 <= d2) {
                hi = t2;
            } else {
                lo = t1;
            }
        }
        double bestT = (lo + hi) * 0.5;
        double bestDist = distanceSqPointAabb(pointOnSegment(a, b, bestT), box);

        double startDist = distanceSqPointAabb(a, box);
        if (startDist < bestDist) {
            bestDist = startDist;
            bestT = 0.0;
        }
        double endDist = distanceSqPointAabb(b, box);
        if (endDist < bestDist) {
            bestT = 1.0;
        }

        Vec3 segmentPoint = pointOnSegment(a, b, bestT);
        Vec3 boxPoint = closestPointOnAabb(segmentPoint, box);
        return new SegmentAabbClosest(segmentPoint, boxPoint);
    }

    private static FaceProjection nearestFaceProjection(Vec3 point, Aabb box) {
        double distMinX = Math.abs(point.x() - box.minX());
        double distMaxX = Math.abs(box.maxX() - point.x());
        double distMinY = Math.abs(point.y() - box.minY());
        double distMaxY = Math.abs(box.maxY() - point.y());
        double distMinZ = Math.abs(point.z() - box.minZ());
        double distMaxZ = Math.abs(box.maxZ() - point.z());

        double best = distMinX;
        Vec3 normal = vec(-1.0, 0.0, 0.0);
        Vec3 facePoint = vec(box.minX(), point.y(), point.z());

        if (distMaxX < best) {
            best = distMaxX;
            normal = vec(1.0, 0.0, 0.0);
            facePoint = vec(box.maxX(), point.y(), point.z());
        }
        if (distMinY < best) {
            best = distMinY;
            normal = vec(0.0, -1.0, 0.0);
            facePoint = vec(point.x(), box.minY(), point.z());
        }
        if (distMaxY < best) {
            best = distMaxY;
            normal = vec(0.0, 1.0, 0.0);
            facePoint = vec(point.x(), box.maxY(), point.z());
        }
        if (distMinZ < best) {
            best = distMinZ;
            normal = vec(0.0, 0.0, -1.0);
            facePoint = vec(point.x(), point.y(), box.minZ());
        }
        if (distMaxZ < best) {
            normal = vec(0.0, 0.0, 1.0);
            facePoint = vec(point.x(), point.y(), box.maxZ());
        }
        return new FaceProjection(normal, closestPointOnAabb(facePoint, box));
    }

    private static Vec3 pointOnSegment(Vec3 a, Vec3 b, double t) {
        return add(a, scale(sub(b, a), t));
    }

    private static Vec3 closestPointOnAabb(Vec3 point, Aabb box) {
        return vec(
                clamp(point.x(), box.minX(), box.maxX()),
                clamp(point.y(), box.minY(), box.maxY()),
                clamp(point.z(), box.minZ(), box.maxZ()));
    }

    private static double distanceSqPointAabb(Vec3 point, Aabb box) {
        Vec3 closest = closestPointOnAabb(point, box);
        return distanceSq(point, closest);
    }

    private static ContactPoint3D midpoint(Vec3 a, Vec3 b) {
        return new ContactPoint3D(
                (a.x() + b.x()) * 0.5,
                (a.y() + b.y()) * 0.5,
                (a.z() + b.z()) * 0.5);
    }

    private static Vec3 normalize(Vec3 v) {
        double lenSq = dot(v, v);
        if (lenSq <= EPS * EPS) {
            return vec(1.0, 0.0, 0.0);
        }
        double inv = 1.0 / Math.sqrt(lenSq);
        return scale(v, inv);
    }

    private static double distanceSq(Vec3 a, Vec3 b) {
        return dot(sub(a, b), sub(a, b));
    }

    private static double dot(Vec3 a, Vec3 b) {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    private static Vec3 add(Vec3 a, Vec3 b) {
        return vec(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
    }

    private static Vec3 sub(Vec3 a, Vec3 b) {
        return vec(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
    }

    private static Vec3 scale(Vec3 v, double scalar) {
        return vec(v.x() * scalar, v.y() * scalar, v.z() * scalar);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double clamp01(double value) {
        return clamp(value, 0.0, 1.0);
    }

    private static Vec3 vec(double x, double y, double z) {
        return new Vec3(x, y, z);
    }

    private static final double EPS = 1e-9;

    private enum Axis {
        X, Y, Z
    }

    private record Vec3(double x, double y, double z) {
    }

    private record ClosestSegmentPoints(Vec3 pointOnA, Vec3 pointOnB) {
    }

    private record SegmentAabbClosest(Vec3 segmentPoint, Vec3 boxPoint) {
    }

    private record FaceProjection(Vec3 normal, Vec3 pointOnFace) {
    }
}
