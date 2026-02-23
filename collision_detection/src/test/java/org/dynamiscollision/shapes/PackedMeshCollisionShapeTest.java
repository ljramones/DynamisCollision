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

import static org.junit.jupiter.api.Assertions.*;

import java.nio.ByteBuffer;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;
import org.dynamiscollision.adapters.MeshCollisionAdapter;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.junit.jupiter.api.Test;
import org.meshforge.core.bounds.Aabbf;
import org.meshforge.pack.buffer.Meshlet;
import org.meshforge.pack.buffer.MeshletBufferView;
import org.meshforge.pack.buffer.PackedMesh;
import org.meshforge.pack.layout.VertexLayout;
import org.vectrix.affine.Transformf;

class PackedMeshCollisionShapeTest {

    @Test
    void raycastHitsNearestMeshletBounds() {
        PackedMesh mesh = packedMesh(List.of(
                meshlet(new Aabbf(0, 0, 0, 1, 1, 1), 1f, 0f, 0f, -1f),
                meshlet(new Aabbf(3, 0, 0, 4, 1, 1), 1f, 0f, 0f, -1f)));
        PackedMeshCollisionShape shape = new PackedMeshCollisionShape(mesh, new Transformf().identity());
        Ray3D ray = new Ray3D(-1, 0.5, 0.5, 1, 0, 0);

        Optional<RaycastResult> hit = shape.raycast(ray, new Transformf().identity());

        assertTrue(hit.isPresent());
        assertEquals(1.0, hit.get().t(), 1e-9);
        assertEquals(0, hit.get().meshletIndex());
        assertEquals(0.0, hit.get().point().x(), 1e-9);
        assertEquals(-1.0, hit.get().normal().x(), 1e-9);
        assertEquals(0.0, hit.get().normal().y(), 1e-9);
        assertEquals(0.0, hit.get().normal().z(), 1e-9);
    }

    @Test
    void worldBoundsApplyWorldAndLocalTransforms() {
        PackedMesh mesh = packedMesh(List.of(meshlet(new Aabbf(0, 0, 0, 1, 1, 1), 1f, 0f, 0f, -1f)));
        PackedMeshCollisionShape shape = new PackedMeshCollisionShape(mesh, translation(1, 2, 3));

        Aabb bounds = shape.getWorldBounds(translation(4, 0, -1));

        assertEquals(5.0, bounds.minX(), 1e-6);
        assertEquals(2.0, bounds.minY(), 1e-6);
        assertEquals(2.0, bounds.minZ(), 1e-6);
        assertEquals(6.0, bounds.maxX(), 1e-6);
        assertEquals(3.0, bounds.maxY(), 1e-6);
        assertEquals(3.0, bounds.maxZ(), 1e-6);
    }

    @Test
    void fallsBackToAdapterBoundsWhenMeshletsAreMissing() {
        PackedMesh mesh = packedMesh(List.of());
        MeshCollisionAdapter.setBounds(mesh, new Aabb(0, 0, 0, 1, 1, 1));
        PackedMeshCollisionShape shape = new PackedMeshCollisionShape(mesh, new Transformf().identity());

        Optional<RaycastResult> hit = shape.raycast(new Ray3D(-1, 0.5, 0.5, 1, 0, 0), new Transformf().identity());

        assertTrue(hit.isPresent());
        assertEquals(-1, hit.get().meshletIndex());
        assertEquals(-1.0, hit.get().normal().x(), 1e-9);
        assertEquals(0.0, hit.get().normal().y(), 1e-9);
        assertEquals(0.0, hit.get().normal().z(), 1e-9);
    }

    @Test
    void raycastUsesMeshletConeAxisAsNormalApproximation() {
        PackedMesh mesh = packedMesh(List.of(meshlet(new Aabbf(0, 0, 0, 1, 1, 1), 0f, 0f, 1f, -1f)));
        PackedMeshCollisionShape shape = new PackedMeshCollisionShape(mesh, new Transformf().identity());
        Ray3D ray = new Ray3D(-1, 0.5, 0.5, 1, 0, 0);

        RaycastResult hit = shape.raycast(ray, new Transformf().identity()).orElseThrow();

        assertEquals(0.0, hit.normal().x(), 1e-9);
        assertEquals(0.0, hit.normal().y(), 1e-9);
        assertEquals(1.0, hit.normal().z(), 1e-9);
    }

    private static Transformf translation(float x, float y, float z) {
        Transformf transform = new Transformf().identity();
        transform.translation.set(x, y, z);
        return transform;
    }

    private static Meshlet meshlet(Aabbf bounds, float coneAxisX, float coneAxisY, float coneAxisZ, float cutoffCos) {
        return new Meshlet(0, 0, 0, 0, 0, bounds, coneAxisX, coneAxisY, coneAxisZ, cutoffCos);
    }

    private static PackedMesh packedMesh(List<Meshlet> meshlets) {
        VertexLayout layout = new VertexLayout(0, new LinkedHashMap<>());
        PackedMesh.IndexBufferView index = new PackedMesh.IndexBufferView(
                PackedMesh.IndexType.UINT16,
                ByteBuffer.allocate(0),
                0);
        return new PackedMesh(
                layout,
                ByteBuffer.allocate(0),
                index,
                List.of(),
                MeshletBufferView.of(meshlets));
    }
}
