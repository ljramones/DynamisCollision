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

package org.dynamiscollision.world;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import org.dynamiscollision.adapters.MeshCollisionAdapter;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.broadphase.SweepAndPrune3D;
import org.dynamiscollision.contact.ContactGenerator3D;
import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.filtering.CollisionFilter;
import org.dynamiscollision.filtering.CollisionKind;
import org.dynamiscollision.pipeline.CollisionPair;
import org.junit.jupiter.api.Test;
import org.meshforge.core.attr.VertexSchema;
import org.meshforge.core.mesh.MeshData;
import org.meshforge.pack.buffer.PackedMesh;
import org.meshforge.pack.layout.VertexLayout;

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

    @Test
    void supportsPackedMeshWorld() {
        PackedMesh m1 = packedMesh();
        PackedMesh m2 = packedMesh();
        MeshCollisionAdapter.setBounds(m1, new Aabb(0, 0, 0, 2, 2, 2));
        MeshCollisionAdapter.setBounds(m2, new Aabb(1, 0, 0, 3, 2, 2));
        MeshCollisionAdapter.setFilter(m1, CollisionFilter.DEFAULT);
        MeshCollisionAdapter.setFilter(m2, CollisionFilter.DEFAULT);

        CollisionWorld3D<PackedMesh> world = CollisionWorld3D.forPackedMeshesDefault();

        List<CollisionEvent<PackedMesh>> events = world.update(List.of(m1, m2));
        assertEquals(1, events.size());
        assertEquals(CollisionEventType.ENTER, events.get(0).type());
    }

    @Test
    void stepValidatesDtAndAcceptsLargeDt() {
        Body a = new Body("a", new Aabb(0, 0, 0, 1, 1, 1), CollisionFilter.DEFAULT);
        Body b = new Body("b", new Aabb(0.5, 0, 0, 1.5, 1, 1), CollisionFilter.DEFAULT);
        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::bounds,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.bounds(), right.bounds()));

        world.setBodyAdapter(new RigidBodyAdapter3D<>() {
            @Override
            public org.vectrix.core.Vector3d getPosition(Body body) {
                return new org.vectrix.core.Vector3d(0, 0, 0);
            }

            @Override
            public void setPosition(Body body, org.vectrix.core.Vector3d position) {
            }

            @Override
            public org.vectrix.core.Vector3d getVelocity(Body body) {
                return new org.vectrix.core.Vector3d(0, 0, 0);
            }

            @Override
            public void setVelocity(Body body, org.vectrix.core.Vector3d velocity) {
            }

            @Override
            public double getInverseMass(Body body) {
                return 1.0;
            }

            @Override
            public double getRestitution(Body body) {
                return 0.0;
            }

            @Override
            public double getFriction(Body body) {
                return 0.0;
            }
        });

        assertThrows(IllegalArgumentException.class, () -> world.step(List.of(a, b), 0.0));
        assertThrows(IllegalArgumentException.class, () -> world.step(List.of(a, b), -0.01));
        assertDoesNotThrow(() -> world.step(List.of(a, b), 10.0));
    }

    @Test
    void eventOrderingIsDeterministicForFixedInsertionOrder() {
        Body a = new Body("a", new Aabb(0, 0, 0, 3, 1, 1), CollisionFilter.DEFAULT);
        Body b = new Body("b", new Aabb(1, 0, 0, 4, 1, 1), CollisionFilter.DEFAULT);
        Body c = new Body("c", new Aabb(2, 0, 0, 5, 1, 1), CollisionFilter.DEFAULT);
        List<Body> ordered = List.of(a, b, c);

        List<String> first = eventKeysFromFreshWorld(ordered);
        List<String> second = eventKeysFromFreshWorld(ordered);
        assertEquals(first, second);
    }

    private static List<String> eventKeysFromFreshWorld(List<Body> items) {
        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::bounds,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.bounds(), right.bounds()));
        List<CollisionEvent<Body>> events = world.update(items);
        List<String> keys = new ArrayList<>(events.size());
        for (CollisionEvent<Body> event : events) {
            keys.add(event.pair().first().id + "-" + event.pair().second().id + "-" + event.type());
        }
        return keys;
    }

    private static MeshData mesh() {
        return new MeshData(VertexSchema.standardLit(), 0);
    }

    private static PackedMesh packedMesh() {
        VertexLayout layout = new VertexLayout(0, new java.util.LinkedHashMap<>());
        PackedMesh.IndexBufferView index = new PackedMesh.IndexBufferView(
                PackedMesh.IndexType.UINT16,
                ByteBuffer.allocate(0),
                0);
        return new PackedMesh(layout, ByteBuffer.allocate(0), index, List.of());
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
