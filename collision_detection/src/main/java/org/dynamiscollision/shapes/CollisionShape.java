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

import java.util.List;
import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;

/**
 * Generic collidable shape contract for world-space bounds and coarse ray tests.
 */
public interface CollisionShape {

    /**
     * Returns world-space bounds for this shape using the provided world transform.
     */
    Aabb getWorldBounds(Transformf worldTransform);

    /**
     * Coarse ray query for this shape.
     */
    Optional<RaycastResult> raycast(Ray3D ray, Transformf worldTransform);

    /**
     * Typed shape discriminator used by backend adapters.
     */
    ShapeType shapeType();

    static CollisionShape sphere(float radius) {
        return new SphereCollisionShape(radius);
    }

    static CollisionShape box(float halfX, float halfY, float halfZ) {
        return new BoxCollisionShape(halfX, halfY, halfZ);
    }

    static CollisionShape capsule(float radius, float height) {
        return new CapsuleCollisionShape(radius, height);
    }

    static CollisionShape cylinder(float radius, float height) {
        return new CylinderCollisionShape(radius, height);
    }

    static CollisionShape plane(float nx, float ny, float nz, float d) {
        return new PlaneCollisionShape(nx, ny, nz, d);
    }

    static CollisionShape planeY() {
        return new PlaneCollisionShape(0f, 1f, 0f, 0f);
    }

    static CollisionShape convexHull(float[] vertices, int[] indices) {
        return new ConvexHullCollisionShape(vertices, indices);
    }

    static CollisionShape triangleMesh(float[] vertices, int[] indices) {
        return new TriangleMeshCollisionShape(vertices, indices);
    }

    static CollisionShape heightfield(
            float[] heights,
            int widthSamples,
            int depthSamples,
            float worldWidth,
            float worldDepth,
            float maxHeight) {
        return new HeightfieldCollisionShape(
                heights,
                widthSamples,
                depthSamples,
                worldWidth,
                worldDepth,
                maxHeight);
    }

    static CollisionShape compound(List<CollisionShape> children, List<Transformf> localTransforms) {
        return new CompoundCollisionShape(children, localTransforms);
    }
}
