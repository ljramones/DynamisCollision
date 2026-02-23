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

package org.dynamiscollision.stress;

import static org.junit.jupiter.api.Assertions.*;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.bounds.BoundingSphere;
import org.dynamiscollision.bounds.Capsule;
import org.dynamiscollision.broadphase.BroadPhase3D;
import org.dynamiscollision.broadphase.SpatialHash3D;
import org.dynamiscollision.broadphase.SweepAndPrune3D;
import org.dynamiscollision.contact.ContactGenerator3D;
import org.dynamiscollision.contact.ContactManifold3D;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.narrowphase.ConvexSupport3D;
import org.dynamiscollision.narrowphase.Gjk3D;
import org.dynamiscollision.narrowphase.Intersection3D;
import org.dynamiscollision.pipeline.CollisionPair;
import org.junit.jupiter.api.Test;

class CollisionStressFuzzTest {

    @Test
    void degenerateAndCoincidentPrimitiveContactsProduceValidFiniteManifolds() {
        Optional<ContactManifold3D> ss = ContactGenerator3D.generate(
                new BoundingSphere(0, 0, 0, 0.0),
                new BoundingSphere(0, 0, 0, 0.0));
        assertTrue(ss.isPresent());
        assertValidManifold(ss.get().manifold(), ss.get());

        Optional<ContactManifold3D> aa = ContactGenerator3D.generate(
                new Aabb(1, 1, 1, 1, 1, 1),
                new Aabb(1, 1, 1, 1, 1, 1));
        assertTrue(aa.isPresent());
        assertValidManifold(aa.get().manifold(), aa.get());

        Optional<ContactManifold3D> cc = ContactGenerator3D.generate(
                new Capsule(2, 2, 2, 2, 2, 2, 0.0),
                new Capsule(2, 2, 2, 2, 2, 2, 0.0));
        assertTrue(cc.isPresent());
        assertValidManifold(cc.get().manifold(), cc.get());
    }

    @Test
    void extremeCoordinateContactRemainsFinite() {
        double base = 1e9;
        Capsule capsule = new Capsule(base, base, base, base + 1000, base, base, 10.0);
        Aabb box = new Aabb(base + 1005, base - 5, base - 5, base + 1030, base + 5, base + 5);
        Optional<ContactManifold3D> manifold = ContactGenerator3D.generate(capsule, box);
        assertTrue(manifold.isPresent());
        assertValidManifold(manifold.get().manifold(), manifold.get());
    }

    @Test
    void broadPhaseAtThousandObjectsContainsAllBruteForceOverlaps() {
        List<Body> bodies = generateBodies(1000, 101L);
        Set<CollisionPair<Body>> brute = bruteForceOverlaps(bodies);
        BroadPhase3D<Body> sap = new SweepAndPrune3D<>();
        BroadPhase3D<Body> hash = new SpatialHash3D<>(16.0);

        Set<CollisionPair<Body>> sapPairs = sap.findPotentialPairs(bodies, Body::bounds);
        Set<CollisionPair<Body>> hashPairs = hash.findPotentialPairs(bodies, Body::bounds);

        assertTrue(sapPairs.containsAll(brute));
        assertTrue(hashPairs.containsAll(brute));
    }

    @Test
    void randomConvexPairsRemainFiniteAndRespectGjkIterationBound() {
        assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
            Random random = new Random(20260223L);
            for (int i = 0; i < 500; i++) {
                ConvexSupport3D a = randomSupport(random);
                ConvexSupport3D b = randomSupport(random);
                assertDoesNotThrow(() -> Gjk3D.intersects(a, b, 32));
                Optional<CollisionManifold3D> manifold = Gjk3D.intersectsWithManifold(a, b);
                if (manifold.isPresent()) {
                    CollisionManifold3D m = manifold.get();
                    assertTrue(Double.isFinite(m.normalX()));
                    assertTrue(Double.isFinite(m.normalY()));
                    assertTrue(Double.isFinite(m.normalZ()));
                    assertTrue(Double.isFinite(m.penetrationDepth()));
                    assertTrue(m.penetrationDepth() >= 0.0);
                    double len = Math.sqrt(m.normalX() * m.normalX() + m.normalY() * m.normalY() + m.normalZ() * m.normalZ());
                    assertEquals(1.0, len, 1e-6);
                }
            }
        });
    }

    @Test
    void capsuleContactGeneratorHandlesDegenerateSpecificCases() {
        Optional<ContactManifold3D> pointCapsuleVsSphere = ContactGenerator3D.generate(
                new Capsule(0, 0, 0, 0, 0, 0, 0.5),
                new BoundingSphere(0.25, 0, 0, 0.5));
        assertTrue(pointCapsuleVsSphere.isPresent());
        assertValidManifold(pointCapsuleVsSphere.get().manifold(), pointCapsuleVsSphere.get());

        Optional<ContactManifold3D> coaxialCapsules = ContactGenerator3D.generate(
                new Capsule(0, 0, 0, 0, 4, 0, 0.5),
                new Capsule(0, 3.6, 0, 0, 8, 0, 0.5));
        assertTrue(coaxialCapsules.isPresent());
        assertValidManifold(coaxialCapsules.get().manifold(), coaxialCapsules.get());

        Optional<ContactManifold3D> coincidentCapsules = ContactGenerator3D.generate(
                new Capsule(1, 2, 3, 4, 5, 6, 0.75),
                new Capsule(1, 2, 3, 4, 5, 6, 0.75));
        assertTrue(coincidentCapsules.isPresent());
        assertValidManifold(coincidentCapsules.get().manifold(), coincidentCapsules.get());

        Optional<ContactManifold3D> capsuleVsPointAabb = ContactGenerator3D.generate(
                new Capsule(-0.3, 0, 0, -0.3, 0, 0, 0.5),
                new Aabb(0, 0, 0, 0, 0, 0));
        assertTrue(capsuleVsPointAabb.isPresent());
        assertValidManifold(capsuleVsPointAabb.get().manifold(), capsuleVsPointAabb.get());
    }

    @Test
    void randomCapsulePairsThroughContactGeneratorAlwaysFinite() {
        assertTimeoutPreemptively(Duration.ofSeconds(2), () -> {
            Random random = new Random(88991L);
            for (int i = 0; i < 500; i++) {
                Capsule a = randomCapsule(random);
                Capsule b = randomCapsule(random);
                Optional<ContactManifold3D> manifold = ContactGenerator3D.generate(a, b);
                if (manifold.isPresent()) {
                    assertValidManifold(manifold.get().manifold(), manifold.get());
                }
            }
        });
    }

    private static ConvexSupport3D randomSupport(Random random) {
        int type = random.nextInt(3);
        return switch (type) {
            case 0 -> Gjk3D.fromAabb(randomAabb(random));
            case 1 -> Gjk3D.fromBoundingSphere(new BoundingSphere(
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, 0.01, 20.0)));
            default -> Gjk3D.fromCapsule(new Capsule(
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, -200, 200),
                    randomRange(random, 0.01, 20.0)));
        };
    }

    private static Capsule randomCapsule(Random random) {
        return new Capsule(
                randomRange(random, -500, 500),
                randomRange(random, -500, 500),
                randomRange(random, -500, 500),
                randomRange(random, -500, 500),
                randomRange(random, -500, 500),
                randomRange(random, -500, 500),
                randomRange(random, 0.0, 25.0));
    }

    private static Aabb randomAabb(Random random) {
        double cx = randomRange(random, -500, 500);
        double cy = randomRange(random, -500, 500);
        double cz = randomRange(random, -500, 500);
        double sx = randomRange(random, 0.01, 40.0);
        double sy = randomRange(random, 0.01, 40.0);
        double sz = randomRange(random, 0.01, 40.0);
        return new Aabb(cx - sx * 0.5, cy - sy * 0.5, cz - sz * 0.5, cx + sx * 0.5, cy + sy * 0.5, cz + sz * 0.5);
    }

    private static double randomRange(Random random, double min, double max) {
        return min + (max - min) * random.nextDouble();
    }

    private static List<Body> generateBodies(int count, long seed) {
        Random random = new Random(seed);
        List<Body> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            out.add(new Body(i, randomAabb(random)));
        }
        return out;
    }

    private static Set<CollisionPair<Body>> bruteForceOverlaps(List<Body> bodies) {
        java.util.LinkedHashSet<CollisionPair<Body>> out = new java.util.LinkedHashSet<>();
        for (int i = 0; i < bodies.size(); i++) {
            for (int j = i + 1; j < bodies.size(); j++) {
                if (Intersection3D.intersects(bodies.get(i).bounds(), bodies.get(j).bounds())) {
                    out.add(new CollisionPair<>(bodies.get(i), bodies.get(j)));
                }
            }
        }
        return out;
    }

    private static void assertValidManifold(CollisionManifold3D manifold, ContactManifold3D contactManifold) {
        assertTrue(manifold.penetrationDepth() >= 0.0);
        assertTrue(Double.isFinite(manifold.penetrationDepth()));
        double len = Math.sqrt(manifold.normalX() * manifold.normalX()
                + manifold.normalY() * manifold.normalY()
                + manifold.normalZ() * manifold.normalZ());
        assertEquals(1.0, len, 1e-6);
        contactManifold.contacts().forEach(cp -> {
            assertTrue(Double.isFinite(cp.x()));
            assertTrue(Double.isFinite(cp.y()));
            assertTrue(Double.isFinite(cp.z()));
        });
    }

    private record Body(int id, Aabb bounds) {
    }
}
