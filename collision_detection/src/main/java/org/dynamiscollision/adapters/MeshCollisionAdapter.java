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

import java.util.Map;
import java.util.WeakHashMap;
import org.meshforge.core.bounds.Boundsf;
import org.meshforge.core.bounds.Spheref;
import org.meshforge.core.mesh.MeshData;
import org.meshforge.pack.buffer.PackedMesh;

/**
 * MeshForge adapters for collision bounds and filters.
 */
public final class MeshCollisionAdapter {

    private static final Map<Object, CollisionFilter> FILTERS = new WeakHashMap<>();
    private static final Map<PackedMesh, Aabb> PACKED_BOUNDS = new WeakHashMap<>();

    private MeshCollisionAdapter() {
    }

    public static Aabb bounds(MeshData mesh) {
        if (mesh == null) {
            throw new IllegalArgumentException("mesh must not be null");
        }
        Boundsf bounds = mesh.boundsOrNull();
        if (bounds == null || bounds.aabb() == null) {
            throw new IllegalArgumentException("mesh bounds are not set");
        }
        return Aabb.fromAabbf(bounds.aabb());
    }

    public static void setBounds(MeshData mesh, Aabb bounds) {
        if (mesh == null || bounds == null) {
            throw new IllegalArgumentException("mesh and bounds must not be null");
        }
        mesh.setBounds(toBoundsf(bounds));
    }

    public static Aabb bounds(PackedMesh mesh) {
        if (mesh == null) {
            throw new IllegalArgumentException("mesh must not be null");
        }
        Aabb bounds = PACKED_BOUNDS.get(mesh);
        if (bounds == null) {
            throw new IllegalArgumentException("packed mesh bounds are not set");
        }
        return bounds;
    }

    public static void setBounds(PackedMesh mesh, Aabb bounds) {
        if (mesh == null || bounds == null) {
            throw new IllegalArgumentException("mesh and bounds must not be null");
        }
        PACKED_BOUNDS.put(mesh, bounds);
    }

    public static void setFilter(MeshData mesh, CollisionFilter filter) {
        setFilterInternal(mesh, filter);
    }

    public static CollisionFilter getFilter(MeshData mesh) {
        return getFilterInternal(mesh);
    }

    public static void setFilter(PackedMesh mesh, CollisionFilter filter) {
        setFilterInternal(mesh, filter);
    }

    public static CollisionFilter getFilter(PackedMesh mesh) {
        return getFilterInternal(mesh);
    }

    private static void setFilterInternal(Object mesh, CollisionFilter filter) {
        if (mesh == null || filter == null) {
            throw new IllegalArgumentException("mesh and filter must not be null");
        }
        FILTERS.put(mesh, filter);
    }

    private static CollisionFilter getFilterInternal(Object mesh) {
        if (mesh == null) {
            throw new IllegalArgumentException("mesh must not be null");
        }
        return FILTERS.getOrDefault(mesh, CollisionFilter.DEFAULT);
    }

    private static Boundsf toBoundsf(Aabb bounds) {
        float cx = (float) bounds.centerX();
        float cy = (float) bounds.centerY();
        float cz = (float) bounds.centerZ();
        float hx = (float) (bounds.sizeX() * 0.5);
        float hy = (float) (bounds.sizeY() * 0.5);
        float hz = (float) (bounds.sizeZ() * 0.5);
        float radius = (float) Math.sqrt(hx * hx + hy * hy + hz * hz);
        return new Boundsf(bounds.toAabbf(), new Spheref(cx, cy, cz, radius));
    }
}
