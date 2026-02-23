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

import static org.junit.jupiter.api.Assertions.*;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.bounds.BoundingSphere;
import org.dynamiscollision.bounds.Capsule;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.junit.jupiter.api.Test;

class ContactGenerator3DTest {

    @Test
    void generatesAabbContactWithAxisManifold() {
        Aabb a = new Aabb(0, 0, 0, 2, 2, 2);
        Aabb b = new Aabb(1.5, 0.5, 0.5, 3, 1.5, 1.5);

        ContactManifold3D manifold = ContactGenerator3D.generate(a, b).orElseThrow();
        CollisionManifold3D m = manifold.manifold();

        assertEquals(1.0, m.normalX(), 1e-9);
        assertEquals(0.0, m.normalY(), 1e-9);
        assertEquals(0.0, m.normalZ(), 1e-9);
        assertEquals(0.5, m.penetrationDepth(), 1e-9);
        assertEquals(1, manifold.contacts().size());
        ContactPoint3D cp = manifold.contacts().get(0);
        assertTrue(cp.x() >= 1.5 && cp.x() <= 2.0);
        assertTrue(cp.y() >= 0.5 && cp.y() <= 1.5);
        assertTrue(cp.z() >= 0.5 && cp.z() <= 1.5);
    }

    @Test
    void generatesSphereContact() {
        BoundingSphere a = new BoundingSphere(0, 0, 0, 1.0);
        BoundingSphere b = new BoundingSphere(1.5, 0, 0, 1.0);

        ContactManifold3D manifold = ContactGenerator3D.generate(a, b).orElseThrow();
        assertEquals(1.0, manifold.manifold().normalX(), 1e-9);
        assertEquals(0.5, manifold.manifold().penetrationDepth(), 1e-9);
        assertEquals(1, manifold.contacts().size());
    }

    @Test
    void returnsEmptyWhenNoContact() {
        Aabb a = new Aabb(0, 0, 0, 1, 1, 1);
        Aabb b = new Aabb(2, 2, 2, 3, 3, 3);
        assertTrue(ContactGenerator3D.generate(a, b).isEmpty());
    }

    @Test
    void generatesCapsuleCapsuleContactForParallelSegments() {
        Capsule a = new Capsule(0, 0, 0, 0, 2, 0, 0.5);
        Capsule b = new Capsule(0.8, 0, 0, 0.8, 2, 0, 0.5);

        ContactManifold3D manifold = ContactGenerator3D.generate(a, b).orElseThrow();
        assertEquals(1.0, manifold.manifold().normalX(), 1e-9);
        assertEquals(0.0, manifold.manifold().normalY(), 1e-9);
        assertEquals(0.0, manifold.manifold().normalZ(), 1e-9);
        assertEquals(0.2, manifold.manifold().penetrationDepth(), 1e-9);
        assertEquals(1, manifold.contacts().size());
    }

    @Test
    void capsuleCapsuleHandlesDegeneratePointCapsules() {
        Capsule a = new Capsule(0, 0, 0, 0, 0, 0, 1.0);
        Capsule b = new Capsule(1.5, 0, 0, 1.5, 0, 0, 1.0);

        ContactManifold3D manifold = ContactGenerator3D.generate(a, b).orElseThrow();
        assertEquals(1.0, manifold.manifold().normalX(), 1e-9);
        assertEquals(0.5, manifold.manifold().penetrationDepth(), 1e-9);
    }

    @Test
    void generatesCapsuleSphereContact() {
        Capsule capsule = new Capsule(0, 0, 0, 0, 2, 0, 0.5);
        BoundingSphere sphere = new BoundingSphere(0.8, 1.0, 0.0, 0.5);

        ContactManifold3D manifold = ContactGenerator3D.generate(capsule, sphere).orElseThrow();
        assertEquals(1.0, manifold.manifold().normalX(), 1e-9);
        assertEquals(0.2, manifold.manifold().penetrationDepth(), 1e-9);
    }

    @Test
    void generatesCapsuleAabbContact() {
        Capsule capsule = new Capsule(-0.4, 0.5, 0.5, -0.4, 1.5, 0.5, 0.5);
        Aabb box = new Aabb(0, 0, 0, 1, 2, 1);

        ContactManifold3D manifold = ContactGenerator3D.generate(capsule, box).orElseThrow();
        assertEquals(1.0, manifold.manifold().normalX(), 1e-9);
        assertEquals(0.1, manifold.manifold().penetrationDepth(), 1e-9);
        assertEquals(1, manifold.contacts().size());
    }

    @Test
    void returnsEmptyWhenCapsuleSphereSeparated() {
        Capsule capsule = new Capsule(0, 0, 0, 0, 2, 0, 0.25);
        BoundingSphere sphere = new BoundingSphere(2.0, 1.0, 0.0, 0.25);
        assertTrue(ContactGenerator3D.generate(capsule, sphere).isEmpty());
    }
}
