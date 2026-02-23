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

package org.dynamiscollision.debug;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.contact.ContactManifold3D;
import org.dynamiscollision.contact.ContactPoint3D;
import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.pipeline.CollisionPair;
import org.junit.jupiter.api.Test;

class CollisionDebugSnapshot3DTest {

    @Test
    void buildsSnapshotFromItemsAndEvents() {
        Body a = new Body("a", new Aabb(0, 0, 0, 2, 2, 2));
        Body b = new Body("b", new Aabb(1, 1, 1, 3, 3, 3));
        CollisionPair<Body> pair = new CollisionPair<>(a, b);
        CollisionManifold3D manifold3D = new CollisionManifold3D(1, 0, 0, 0.5);
        ContactManifold3D manifold = new ContactManifold3D(
                manifold3D,
                List.of(new ContactPoint3D(1, 1, 1), new ContactPoint3D(2, 2, 2)));
        CollisionEvent<Body> event = new CollisionEvent<>(pair, CollisionEventType.ENTER, true, manifold);

        CollisionDebugSnapshot3D<Body> snapshot = CollisionDebugSnapshot3D.from(
                List.of(a, b),
                Body::bounds,
                List.of(event));

        assertEquals(2, snapshot.items().size());
        assertEquals(2, snapshot.contacts().size());
        assertEquals(a, snapshot.items().get(0).item());
        assertEquals(CollisionEventType.ENTER, snapshot.contacts().get(0).type());
    }

    @Test
    void rejectsNullArguments() {
        assertThrows(IllegalArgumentException.class, () ->
                CollisionDebugSnapshot3D.from(null, Body::bounds, List.of()));
        assertThrows(IllegalArgumentException.class, () ->
                CollisionDebugSnapshot3D.from(List.of(), null, List.of()));
        assertThrows(IllegalArgumentException.class, () ->
                CollisionDebugSnapshot3D.from(List.of(), Body::bounds, null));
    }

    @Test
    void ignoresNullEventsAndEventsWithoutManifold() {
        Body a = new Body("a", new Aabb(0, 0, 0, 1, 1, 1));
        Body b = new Body("b", new Aabb(0, 0, 0, 1, 1, 1));
        CollisionPair<Body> pair = new CollisionPair<>(a, b);
        CollisionEvent<Body> noManifold = new CollisionEvent<>(pair, CollisionEventType.STAY, true, null);

        CollisionDebugSnapshot3D<Body> snapshot = CollisionDebugSnapshot3D.from(
                List.of(a, b),
                Body::bounds,
                java.util.Arrays.asList(null, noManifold));

        assertEquals(2, snapshot.items().size());
        assertTrue(snapshot.contacts().isEmpty());
    }

    @Test
    void snapshotCollectionsAreImmutableCopies() {
        Body a = new Body("a", new Aabb(0, 0, 0, 1, 1, 1));
        CollisionDebugSnapshot3D<Body> snapshot = CollisionDebugSnapshot3D.from(
                List.of(a),
                Body::bounds,
                List.of());

        assertThrows(UnsupportedOperationException.class, () -> snapshot.items().add(null));
        assertThrows(UnsupportedOperationException.class, () -> snapshot.contacts().add(null));
    }

    private static final class Body {
        private final String id;
        private final Aabb bounds;

        private Body(String id, Aabb bounds) {
            this.id = id;
            this.bounds = bounds;
        }

        private Aabb bounds() {
            return bounds;
        }

        @Override
        public String toString() {
            return id;
        }
    }
}
