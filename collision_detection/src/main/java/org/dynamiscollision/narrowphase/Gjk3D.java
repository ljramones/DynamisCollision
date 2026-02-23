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

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.bounds.BoundingSphere;
import org.dynamiscollision.bounds.Capsule;
import org.vectrix.core.Vector3d;

/**
 * GJK intersection test for convex 3D shapes via support mappings.
 * Includes EPA-based manifold extraction for intersecting shapes.
 */
public final class Gjk3D {

    private static final int DEFAULT_MAX_ITERATIONS = 32;
    private static final int DEFAULT_EPA_MAX_ITERATIONS = 48;
    private static final double EPSILON = 1e-9;
    private static final double EPA_TOLERANCE = 1e-6;

    private Gjk3D() {
    }

    public static boolean intersects(ConvexSupport3D a, ConvexSupport3D b) {
        return intersects(a, b, DEFAULT_MAX_ITERATIONS);
    }

    public static boolean intersects(ConvexSupport3D a, ConvexSupport3D b, int maxIterations) {
        return runGjk(a, b, maxIterations).hit();
    }

    /**
     * Returns a collision manifold for intersecting convex shapes.
     * Empty when shapes do not intersect or manifold extraction does not converge.
     */
    public static Optional<CollisionManifold3D> intersectsWithManifold(ConvexSupport3D a, ConvexSupport3D b) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("support shapes must not be null");
        }
        GjkState state = runGjk(a, b, DEFAULT_MAX_ITERATIONS);
        if (!state.hit() || state.simplex().size() < 4) {
            return Optional.empty();
        }
        return runEpa(a, b, state.simplex(), DEFAULT_EPA_MAX_ITERATIONS);
    }

    public static ConvexSupport3D fromAabb(Aabb aabb) {
        if (aabb == null) {
            throw new IllegalArgumentException("aabb must not be null");
        }
        return direction -> new Vector3d(
                direction.x() >= 0.0 ? aabb.maxX() : aabb.minX(),
                direction.y() >= 0.0 ? aabb.maxY() : aabb.minY(),
                direction.z() >= 0.0 ? aabb.maxZ() : aabb.minZ());
    }

    public static ConvexSupport3D fromBoundingSphere(BoundingSphere sphere) {
        if (sphere == null) {
            throw new IllegalArgumentException("sphere must not be null");
        }
        return direction -> {
            Vec3 dir = Vec3.from(direction);
            if (dir.isNearZero()) {
                dir = new Vec3(1.0, 0.0, 0.0);
            } else {
                dir = dir.normalized();
            }
            return new Vector3d(
                    sphere.centerX() + dir.x() * sphere.radius(),
                    sphere.centerY() + dir.y() * sphere.radius(),
                    sphere.centerZ() + dir.z() * sphere.radius());
        };
    }

    public static ConvexSupport3D fromCapsule(Capsule capsule) {
        if (capsule == null) {
            throw new IllegalArgumentException("capsule must not be null");
        }
        return direction -> {
            Vec3 dir = Vec3.from(direction);
            if (dir.isNearZero()) {
                dir = new Vec3(1.0, 0.0, 0.0);
            } else {
                dir = dir.normalized();
            }

            double dotA = capsule.pointAX() * dir.x() + capsule.pointAY() * dir.y() + capsule.pointAZ() * dir.z();
            double dotB = capsule.pointBX() * dir.x() + capsule.pointBY() * dir.y() + capsule.pointBZ() * dir.z();
            if (dotA >= dotB) {
                return new Vector3d(
                        capsule.pointAX() + dir.x() * capsule.radius(),
                        capsule.pointAY() + dir.y() * capsule.radius(),
                        capsule.pointAZ() + dir.z() * capsule.radius());
            }
            return new Vector3d(
                    capsule.pointBX() + dir.x() * capsule.radius(),
                    capsule.pointBY() + dir.y() * capsule.radius(),
                    capsule.pointBZ() + dir.z() * capsule.radius());
        };
    }

    private static GjkState runGjk(ConvexSupport3D a, ConvexSupport3D b, int maxIterations) {
        if (a == null || b == null) {
            throw new IllegalArgumentException("support shapes must not be null");
        }
        if (maxIterations < 4) {
            throw new IllegalArgumentException("maxIterations must be >= 4");
        }

        Vec3[] direction = new Vec3[] {new Vec3(1.0, 0.0, 0.0)};
        List<Vec3> simplex = new ArrayList<>(4);

        Vec3 first = support(a, b, direction[0]);
        simplex.add(first);
        direction[0] = first.negate();
        if (direction[0].isNearZero()) {
            direction[0] = new Vec3(0.0, 1.0, 0.0);
        }

        for (int iteration = 0; iteration < maxIterations; iteration++) {
            Vec3 point = support(a, b, direction[0]);
            if (point.dot(direction[0]) <= 0.0) {
                return new GjkState(false, List.of());
            }
            simplex.add(0, point);
            if (updateSimplex(simplex, direction)) {
                return new GjkState(true, List.copyOf(simplex));
            }
        }
        return new GjkState(false, List.of());
    }

    private static Optional<CollisionManifold3D> runEpa(
            ConvexSupport3D a,
            ConvexSupport3D b,
            List<Vec3> initialSimplex,
            int maxIterations) {

        List<Vec3> points = new ArrayList<>(initialSimplex);
        List<EpaFace> faces = new ArrayList<>();

        faces.add(makeFace(0, 1, 2, points));
        faces.add(makeFace(0, 3, 1, points));
        faces.add(makeFace(0, 2, 3, points));
        faces.add(makeFace(1, 3, 2, points));

        for (int iteration = 0; iteration < maxIterations; iteration++) {
            EpaFace closest = findClosestFace(faces);
            Vec3 support = support(a, b, closest.normal());
            double supportDistance = support.dot(closest.normal());

            if ((supportDistance - closest.distance()) <= EPA_TOLERANCE) {
                Vec3 normal = closest.normal().normalized();
                return Optional.of(new CollisionManifold3D(normal.x(), normal.y(), normal.z(), supportDistance));
            }

            int supportIndex = points.size();
            points.add(support);

            List<EpaFace> visible = new ArrayList<>();
            for (EpaFace face : faces) {
                Vec3 pa = points.get(face.a());
                if (face.normal().dot(support.sub(pa)) > EPSILON) {
                    visible.add(face);
                }
            }

            List<Edge> boundary = new ArrayList<>();
            for (EpaFace face : visible) {
                addBoundaryEdge(boundary, new Edge(face.a(), face.b()));
                addBoundaryEdge(boundary, new Edge(face.b(), face.c()));
                addBoundaryEdge(boundary, new Edge(face.c(), face.a()));
            }

            faces.removeAll(visible);
            for (Edge edge : boundary) {
                faces.add(makeFace(edge.from(), edge.to(), supportIndex, points));
            }

            if (faces.isEmpty()) {
                return Optional.empty();
            }
        }
        return Optional.empty();
    }

    private static void addBoundaryEdge(List<Edge> boundary, Edge edge) {
        for (int i = 0; i < boundary.size(); i++) {
            Edge existing = boundary.get(i);
            if (existing.from() == edge.to() && existing.to() == edge.from()) {
                boundary.remove(i);
                return;
            }
        }
        boundary.add(edge);
    }

    private static EpaFace findClosestFace(List<EpaFace> faces) {
        EpaFace best = faces.get(0);
        for (int i = 1; i < faces.size(); i++) {
            if (faces.get(i).distance() < best.distance()) {
                best = faces.get(i);
            }
        }
        return best;
    }

    private static EpaFace makeFace(int a, int b, int c, List<Vec3> points) {
        Vec3 pa = points.get(a);
        Vec3 pb = points.get(b);
        Vec3 pc = points.get(c);

        Vec3 normal = pb.sub(pa).cross(pc.sub(pa));
        if (normal.isNearZero()) {
            normal = new Vec3(1.0, 0.0, 0.0);
        } else {
            normal = normal.normalized();
        }

        double distance = normal.dot(pa);
        if (distance < 0.0) {
            normal = normal.negate();
            distance = -distance;
            int tmp = b;
            b = c;
            c = tmp;
        }
        return new EpaFace(a, b, c, normal, distance);
    }

    private static boolean updateSimplex(List<Vec3> simplex, Vec3[] direction) {
        return switch (simplex.size()) {
            case 2 -> handleLine(simplex, direction);
            case 3 -> handleTriangle(simplex, direction);
            case 4 -> handleTetrahedron(simplex, direction);
            default -> false;
        };
    }

    private static boolean handleLine(List<Vec3> simplex, Vec3[] direction) {
        Vec3 a = simplex.get(0);
        Vec3 b = simplex.get(1);
        Vec3 ab = b.sub(a);
        Vec3 ao = a.negate();

        if (ab.dot(ao) > 0.0) {
            direction[0] = tripleCross(ab, ao, ab);
            if (direction[0].isNearZero()) {
                direction[0] = anyPerpendicular(ab);
            }
        } else {
            simplex.clear();
            simplex.add(a);
            direction[0] = ao;
        }
        return false;
    }

    private static boolean handleTriangle(List<Vec3> simplex, Vec3[] direction) {
        Vec3 a = simplex.get(0);
        Vec3 b = simplex.get(1);
        Vec3 c = simplex.get(2);

        Vec3 ab = b.sub(a);
        Vec3 ac = c.sub(a);
        Vec3 ao = a.negate();

        Vec3 abc = ab.cross(ac);
        Vec3 abPerp = ab.cross(abc);
        if (abPerp.dot(ao) > 0.0) {
            simplex.remove(2);
            direction[0] = tripleCross(ab, ao, ab);
            if (direction[0].isNearZero()) {
                direction[0] = anyPerpendicular(ab);
            }
            return false;
        }

        Vec3 acPerp = abc.cross(ac);
        if (acPerp.dot(ao) > 0.0) {
            simplex.remove(1);
            direction[0] = tripleCross(ac, ao, ac);
            if (direction[0].isNearZero()) {
                direction[0] = anyPerpendicular(ac);
            }
            return false;
        }

        if (abc.dot(ao) > 0.0) {
            direction[0] = abc;
        } else {
            simplex.set(1, c);
            simplex.set(2, b);
            direction[0] = abc.negate();
        }
        return false;
    }

    private static boolean handleTetrahedron(List<Vec3> simplex, Vec3[] direction) {
        Vec3 a = simplex.get(0);
        Vec3 b = simplex.get(1);
        Vec3 c = simplex.get(2);
        Vec3 d = simplex.get(3);
        Vec3 ao = a.negate();

        Face abc = orientFace(a, b, c, d);
        if (abc.normal().dot(ao) > 0.0) {
            simplex.clear();
            simplex.add(a);
            simplex.add(b);
            simplex.add(c);
            return handleTriangle(simplex, direction);
        }

        Face acd = orientFace(a, c, d, b);
        if (acd.normal().dot(ao) > 0.0) {
            simplex.clear();
            simplex.add(a);
            simplex.add(c);
            simplex.add(d);
            return handleTriangle(simplex, direction);
        }

        Face adb = orientFace(a, d, b, c);
        if (adb.normal().dot(ao) > 0.0) {
            simplex.clear();
            simplex.add(a);
            simplex.add(d);
            simplex.add(b);
            return handleTriangle(simplex, direction);
        }

        return true;
    }

    private static Face orientFace(Vec3 a, Vec3 b, Vec3 c, Vec3 opposite) {
        Vec3 normal = b.sub(a).cross(c.sub(a));
        if (normal.dot(opposite.sub(a)) > 0.0) {
            normal = normal.negate();
        }
        return new Face(a, b, c, normal);
    }

    private static Vec3 support(ConvexSupport3D a, ConvexSupport3D b, Vec3 direction) {
        Vector3d supportA = a.support(direction.toVector3D());
        Vector3d supportB = b.support(direction.negate().toVector3D());
        return Vec3.from(supportA).sub(Vec3.from(supportB));
    }

    private static Vec3 tripleCross(Vec3 a, Vec3 b, Vec3 c) {
        return a.cross(b).cross(c);
    }

    private static Vec3 anyPerpendicular(Vec3 v) {
        Vec3 axis = Math.abs(v.x()) < 0.9 ? new Vec3(1.0, 0.0, 0.0) : new Vec3(0.0, 1.0, 0.0);
        Vec3 p = v.cross(axis);
        return p.isNearZero() ? new Vec3(0.0, 0.0, 1.0) : p;
    }

    private record GjkState(boolean hit, List<Vec3> simplex) {
    }

    private record Face(Vec3 a, Vec3 b, Vec3 c, Vec3 normal) {
    }

    private record Edge(int from, int to) {
    }

    private record EpaFace(int a, int b, int c, Vec3 normal, double distance) {
    }

    private record Vec3(double x, double y, double z) {

        private static Vec3 from(Vector3d v) {
            return new Vec3(v.x(), v.y(), v.z());
        }

        private Vector3d toVector3D() {
            return new Vector3d(x, y, z);
        }

        private Vec3 sub(Vec3 other) {
            return new Vec3(x - other.x, y - other.y, z - other.z);
        }

        private double dot(Vec3 other) {
            return x * other.x + y * other.y + z * other.z;
        }

        private Vec3 cross(Vec3 other) {
            return new Vec3(
                    y * other.z - z * other.y,
                    z * other.x - x * other.z,
                    x * other.y - y * other.x);
        }

        private Vec3 negate() {
            return new Vec3(-x, -y, -z);
        }

        private double lengthSquared() {
            return x * x + y * y + z * z;
        }

        private boolean isNearZero() {
            return lengthSquared() <= EPSILON;
        }

        private Vec3 normalized() {
            double lenSq = lengthSquared();
            if (lenSq <= EPSILON) {
                return new Vec3(0.0, 0.0, 0.0);
            }
            double inv = 1.0 / Math.sqrt(lenSq);
            return new Vec3(x * inv, y * inv, z * inv);
        }
    }
}
