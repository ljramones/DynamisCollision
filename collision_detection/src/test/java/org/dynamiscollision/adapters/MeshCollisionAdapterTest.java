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

package org.dynamiscollision.adapters;

import static org.junit.jupiter.api.Assertions.*;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.filtering.CollisionFilter;
import org.dynamiscollision.filtering.CollisionKind;
import org.junit.jupiter.api.Test;
import org.meshforge.core.attr.VertexSchema;
import org.meshforge.core.mesh.MeshData;

class MeshCollisionAdapterTest {

    @Test
    void storesAndRetrievesFilter() {
        MeshData mesh = mesh();
        CollisionFilter filter = new CollisionFilter(0b0010, 0b0101, CollisionKind.TRIGGER);

        MeshCollisionAdapter.setFilter(mesh, filter);

        assertEquals(filter, MeshCollisionAdapter.getFilter(mesh));
    }

    @Test
    void defaultsFilterWhenUnset() {
        MeshData mesh = mesh();

        assertEquals(CollisionFilter.DEFAULT, MeshCollisionAdapter.getFilter(mesh));
    }

    @Test
    void storesAndRetrievesBoundsForMeshData() {
        MeshData mesh = mesh();
        Aabb expected = new Aabb(0, 1, 2, 3, 4, 5);

        MeshCollisionAdapter.setBounds(mesh, expected);

        assertEquals(expected, MeshCollisionAdapter.bounds(mesh));
    }

    @Test
    void throwsWhenMeshDataBoundsUnset() {
        assertThrows(IllegalArgumentException.class, () -> MeshCollisionAdapter.bounds(mesh()));
    }

    private static MeshData mesh() {
        return new MeshData(VertexSchema.standardLit(), 0);
    }
}
