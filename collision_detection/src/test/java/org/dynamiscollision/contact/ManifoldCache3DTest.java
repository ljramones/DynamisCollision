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
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

class ManifoldCache3DTest {

    @Test
    void storesAndRetrievesManifoldByPair() {
        ManifoldCache3D<String> cache = new ManifoldCache3D<>();
        CollisionPair<String> pair = new CollisionPair<>("a", "b");
        ContactManifold3D manifold = new ContactManifold3D(
                new CollisionManifold3D(1, 0, 0, 0.25),
                List.of(new ContactPoint3D(1, 2, 3)));

        cache.put(pair, manifold);
        ContactManifold3D found = cache.get(pair).orElseThrow();
        assertEquals(0.25, found.manifold().penetrationDepth(), 1e-9);
        assertEquals(0.0, cache.getWarmStart(pair).orElseThrow().normalImpulse(), 1e-9);
    }

    @Test
    void prunesEntriesByFrameAge() {
        ManifoldCache3D<String> cache = new ManifoldCache3D<>();
        CollisionPair<String> pair = new CollisionPair<>("a", "b");
        cache.put(pair, new ContactManifold3D(
                new CollisionManifold3D(1, 0, 0, 0.1),
                List.of(new ContactPoint3D(0, 0, 0))));

        cache.nextFrame();
        cache.nextFrame();
        cache.pruneStale(1);

        assertTrue(cache.get(pair).isEmpty());
        assertEquals(0, cache.size());
    }

    @Test
    void keepsFreshEntriesAfterUpdate() {
        ManifoldCache3D<String> cache = new ManifoldCache3D<>();
        CollisionPair<String> pair = new CollisionPair<>("a", "b");

        cache.put(pair, new ContactManifold3D(
                new CollisionManifold3D(1, 0, 0, 0.1),
                List.of(new ContactPoint3D(0, 0, 0))));
        cache.nextFrame();
        cache.put(pair, new ContactManifold3D(
                new CollisionManifold3D(0, 1, 0, 0.2),
                List.of(new ContactPoint3D(1, 0, 0))));
        cache.nextFrame();
        cache.pruneStale(1);

        assertTrue(cache.get(pair).isPresent());
        assertEquals(1, cache.size());
    }

    @Test
    void storesWarmStartPerPair() {
        ManifoldCache3D<String> cache = new ManifoldCache3D<>();
        CollisionPair<String> pair = new CollisionPair<>("a", "b");

        cache.put(pair, new ContactManifold3D(
                new CollisionManifold3D(1, 0, 0, 0.1),
                List.of(new ContactPoint3D(0, 0, 0))));
        cache.setWarmStart(pair, new WarmStartImpulse(0.4, -0.1));

        WarmStartImpulse warm = cache.getWarmStart(pair).orElseThrow();
        assertEquals(0.4, warm.normalImpulse(), 1e-9);
        assertEquals(-0.1, warm.tangentImpulse(), 1e-9);
    }
}
