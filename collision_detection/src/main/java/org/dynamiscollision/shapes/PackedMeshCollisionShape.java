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

package org.dynamiscollision.shapes;

import java.util.Optional;
import java.util.OptionalDouble;
import org.dynamiscollision.adapters.MeshCollisionAdapter;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.dynamiscollision.narrowphase.Intersection3D;
import org.meshforge.pack.buffer.Meshlet;
import org.meshforge.pack.buffer.MeshletBufferView;
import org.meshforge.pack.buffer.PackedMesh;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4x3f;
import org.vectrix.core.Vector3d;
import org.vectrix.core.Vector3f;

/**
 * MeshForge-native collision shape backed by {@link PackedMesh}.
 * Current raycast is coarse and meshlet-aware (AABB + cone rejection, no triangle pass yet).
 */
public final class PackedMeshCollisionShape implements CollisionShape {

    
    public ShapeType shapeType() {
        return ShapeType.TRIANGLE_MESH;
    }


    private final PackedMesh mesh;
    private final Transformf localTransform;

    public PackedMeshCollisionShape(PackedMesh mesh, Transformf localTransform) {
        if (mesh == null) {
            throw new IllegalArgumentException("mesh must not be null");
        }
        this.mesh = mesh;
        this.localTransform = localTransform == null ? identity() : new Transformf(localTransform);
    }

    public PackedMesh mesh() {
        return mesh;
    }

    public Transformf localTransform() {
        return new Transformf(localTransform);
    }

    @Override
    public Aabb getWorldBounds(Transformf worldTransform) {
        Transformf combined = combined(worldTransform);
        Matrix4x3f matrix = combined.toAffineMat4Fast(new Matrix4x3f());
        Aabb localBounds = localMeshBounds();
        return transformAabb(localBounds, matrix);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf worldTransform) {
        if (ray == null) {
            throw new IllegalArgumentException("ray must not be null");
        }

        Transformf combined = combined(worldTransform);
        Matrix4x3f matrix = combined.toAffineMat4Fast(new Matrix4x3f());
        Aabb worldBounds = transformAabb(localMeshBounds(), matrix);
        if (!Intersection3D.intersects(ray, worldBounds)) {
            return Optional.empty();
        }

        MeshletBufferView meshlets = mesh.meshletsOrNull();
        if (meshlets == null || meshlets.meshletCount() == 0) {
            return coarseWorldBoundsHit(ray, worldBounds);
        }

        Vector3d rayDir = normalizedDirection(ray);
        double bestT = Double.POSITIVE_INFINITY;
        int bestMeshlet = -1;
        Vector3d bestNormal = null;
        Aabb bestMeshletBounds = null;

        for (int i = 0; i < meshlets.meshletCount(); i++) {
            Meshlet meshlet = meshlets.meshlet(i);
            Aabb meshletWorld = transformAabb(Aabb.fromAabbf(meshlet.bounds()), matrix);
            OptionalDouble distance = Intersection3D.rayAabbIntersectionDistance(ray, meshletWorld);
            if (distance.isEmpty()) {
                continue;
            }

            Vector3f transformedAxis = matrix.transformDirection(
                    new Vector3f(meshlet.coneAxisX(), meshlet.coneAxisY(), meshlet.coneAxisZ()),
                    new Vector3f());
            double axisLenSq = transformedAxis.lengthSquared();
            Vector3d candidateNormal = null;
            if (axisLenSq > 0.0) {
                transformedAxis.normalize();
                candidateNormal = new Vector3d(transformedAxis.x(), transformedAxis.y(), transformedAxis.z());
                double coneDot = rayDir.x() * transformedAxis.x() + rayDir.y() * transformedAxis.y() + rayDir.z() * transformedAxis.z();
                if (coneDot < meshlet.coneCutoffCos()) {
                    continue;
                }
            }

            double t = distance.getAsDouble();
            if (t < bestT) {
                bestT = t;
                bestMeshlet = i;
                bestNormal = candidateNormal;
                bestMeshletBounds = meshletWorld;
            }
        }

        if (bestMeshlet < 0) {
            return Optional.empty();
        }
        Vector3d normal = bestNormal;
        if (normal == null) {
            Vector3d hitPoint = pointOnRay(ray, bestT);
            normal = aabbFaceNormal(hitPoint, bestMeshletBounds);
        }
        normal = orientAgainstRay(normal, rayDir);
        return Optional.of(hitFromRay(ray, bestT, normal, bestMeshlet));
    }

    private Optional<RaycastResult> coarseWorldBoundsHit(Ray3D ray, Aabb worldBounds) {
        OptionalDouble distance = Intersection3D.rayAabbIntersectionDistance(ray, worldBounds);
        if (distance.isEmpty()) {
            return Optional.empty();
        }
        double t = distance.getAsDouble();
        Vector3d point = pointOnRay(ray, t);
        Vector3d normal = orientAgainstRay(aabbFaceNormal(point, worldBounds), normalizedDirection(ray));
        return Optional.of(hitFromRay(ray, t, normal, -1));
    }

    private RaycastResult hitFromRay(Ray3D ray, double t, Vector3d normal, int meshletIndex) {
        return new RaycastResult(t, pointOnRay(ray, t), normal, meshletIndex, -1);
    }

    private Aabb localMeshBounds() {
        MeshletBufferView meshlets = mesh.meshletsOrNull();
        if (meshlets != null && meshlets.meshletCount() > 0) {
            float minX = Float.POSITIVE_INFINITY;
            float minY = Float.POSITIVE_INFINITY;
            float minZ = Float.POSITIVE_INFINITY;
            float maxX = Float.NEGATIVE_INFINITY;
            float maxY = Float.NEGATIVE_INFINITY;
            float maxZ = Float.NEGATIVE_INFINITY;
            for (int i = 0; i < meshlets.meshletCount(); i++) {
                var b = meshlets.meshlet(i).bounds();
                minX = Math.min(minX, b.minX());
                minY = Math.min(minY, b.minY());
                minZ = Math.min(minZ, b.minZ());
                maxX = Math.max(maxX, b.maxX());
                maxY = Math.max(maxY, b.maxY());
                maxZ = Math.max(maxZ, b.maxZ());
            }
            return new Aabb(minX, minY, minZ, maxX, maxY, maxZ);
        }
        return MeshCollisionAdapter.bounds(mesh);
    }

    private Transformf combined(Transformf worldTransform) {
        Transformf world = worldTransform == null ? identity() : new Transformf(worldTransform);
        return Transformf.compose(world, localTransform, new Transformf());
    }

    private static Transformf identity() {
        return new Transformf().identity();
    }

    private static Aabb transformAabb(Aabb source, Matrix4x3f matrix) {
        Vector3f outMin = new Vector3f();
        Vector3f outMax = new Vector3f();
        matrix.transformAab(
                (float) source.minX(),
                (float) source.minY(),
                (float) source.minZ(),
                (float) source.maxX(),
                (float) source.maxY(),
                (float) source.maxZ(),
                outMin,
                outMax);
        return new Aabb(outMin.x(), outMin.y(), outMin.z(), outMax.x(), outMax.y(), outMax.z());
    }

    private static Vector3d normalizedDirection(Ray3D ray) {
        double len = Math.sqrt(ray.dirX() * ray.dirX() + ray.dirY() * ray.dirY() + ray.dirZ() * ray.dirZ());
        return new Vector3d(ray.dirX() / len, ray.dirY() / len, ray.dirZ() / len);
    }

    private static Vector3d pointOnRay(Ray3D ray, double t) {
        return new Vector3d(
                ray.originX() + ray.dirX() * t,
                ray.originY() + ray.dirY() * t,
                ray.originZ() + ray.dirZ() * t);
    }

    private static Vector3d aabbFaceNormal(Vector3d hitPoint, Aabb box) {
        if (box == null) {
            return new Vector3d(0.0, 1.0, 0.0);
        }
        double eps = 1e-6;
        if (Math.abs(hitPoint.x() - box.minX()) < eps) {
            return new Vector3d(-1.0, 0.0, 0.0);
        }
        if (Math.abs(hitPoint.x() - box.maxX()) < eps) {
            return new Vector3d(1.0, 0.0, 0.0);
        }
        if (Math.abs(hitPoint.y() - box.minY()) < eps) {
            return new Vector3d(0.0, -1.0, 0.0);
        }
        if (Math.abs(hitPoint.y() - box.maxY()) < eps) {
            return new Vector3d(0.0, 1.0, 0.0);
        }
        if (Math.abs(hitPoint.z() - box.minZ()) < eps) {
            return new Vector3d(0.0, 0.0, -1.0);
        }
        if (Math.abs(hitPoint.z() - box.maxZ()) < eps) {
            return new Vector3d(0.0, 0.0, 1.0);
        }
        return new Vector3d(0.0, 1.0, 0.0);
    }

    private static Vector3d orientAgainstRay(Vector3d normal, Vector3d rayDir) {
        double dot = normal.x() * rayDir.x() + normal.y() * rayDir.y() + normal.z() * rayDir.z();
        if (dot > 0.0) {
            return new Vector3d(-normal.x(), -normal.y(), -normal.z());
        }
        return normal;
    }
}
