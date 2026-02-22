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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.meshforge.core.attr.VertexSchema;
import org.meshforge.core.mesh.MeshData;
import org.junit.jupiter.api.Test;

class CollisionWorld3DTest {

    @Test
    void emitsEnterStayExitForPersistentPair() {
        Body a = new Body("a", new Aabb(0, 0, 0, 2, 2, 2), CollisionFilter.DEFAULT);
        Body b = new Body("b", new Aabb(1, 0, 0, 3, 2, 2), CollisionFilter.DEFAULT);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::bounds,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.bounds(), right.bounds()));

        List<CollisionEvent<Body>> frame1 = world.update(List.of(a, b));
        assertEquals(1, frame1.size());
        assertEquals(CollisionEventType.ENTER, frame1.get(0).type());

        List<CollisionEvent<Body>> frame2 = world.update(List.of(a, b));
        assertEquals(1, frame2.size());
        assertEquals(CollisionEventType.STAY, frame2.get(0).type());

        b.setBounds(new Aabb(5, 0, 0, 7, 2, 2));
        List<CollisionEvent<Body>> frame3 = world.update(List.of(a, b));
        assertEquals(1, frame3.size());
        assertEquals(CollisionEventType.EXIT, frame3.get(0).type());
    }

    @Test
    void classifiesTriggerAsNonResponse() {
        Body ship = new Body("ship", new Aabb(0, 0, 0, 2, 2, 2),
                new CollisionFilter(0b0001, 0b0110, CollisionKind.SOLID));
        Body sensor = new Body("sensor", new Aabb(1, 0, 0, 3, 2, 2),
                new CollisionFilter(0b0010, 0b1111, CollisionKind.TRIGGER));

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::bounds,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.bounds(), right.bounds()));

        List<CollisionEvent<Body>> events = world.update(List.of(ship, sensor));
        assertEquals(1, events.size());
        assertEquals(CollisionEventType.ENTER, events.get(0).type());
        assertFalse(events.get(0).responseEnabled());
    }

    @Test
    void reusesManifoldCacheAcrossStayFrames() {
        Body a = new Body("a", new Aabb(0, 0, 0, 2, 2, 2), CollisionFilter.DEFAULT);
        Body b = new Body("b", new Aabb(1, 0, 0, 3, 2, 2), CollisionFilter.DEFAULT);
        CollisionPair<Body> pair = new CollisionPair<>(a, b);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::bounds,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.bounds(), right.bounds()));

        world.update(List.of(a, b));
        assertTrue(world.manifoldCache().get(pair).isPresent());
        world.update(List.of(a, b));
        assertTrue(world.manifoldCache().get(pair).isPresent());
    }

    @Test
    void supportsMeshAdapterWorld() {
        MeshData m1 = mesh();
        MeshData m2 = mesh();
        MeshCollisionAdapter.setBounds(m1, new Aabb(0, 0, 0, 2, 2, 2));
        MeshCollisionAdapter.setBounds(m2, new Aabb(1, 0, 0, 3, 2, 2));
        MeshCollisionAdapter.setFilter(m1, CollisionFilter.DEFAULT);
        MeshCollisionAdapter.setFilter(m2, CollisionFilter.DEFAULT);

        CollisionWorld3D<MeshData> world = CollisionWorld3D.forMeshDataDefault();

        List<CollisionEvent<MeshData>> events = world.update(List.of(m1, m2));
        assertEquals(1, events.size());
        assertEquals(CollisionEventType.ENTER, events.get(0).type());
    }

    private static MeshData mesh() {
        return new MeshData(VertexSchema.standardLit(), 0);
    }

    private static final class Body {
        private final String id;
        private Aabb bounds;
        private final CollisionFilter filter;

        private Body(String id, Aabb bounds, CollisionFilter filter) {
            this.id = id;
            this.bounds = bounds;
            this.filter = filter;
        }

        private Aabb bounds() {
            return bounds;
        }

        private void setBounds(Aabb bounds) {
            this.bounds = bounds;
        }

        private CollisionFilter filter() {
            return filter;
        }

        @Override
        public String toString() {
            return id;
        }
    }
}
