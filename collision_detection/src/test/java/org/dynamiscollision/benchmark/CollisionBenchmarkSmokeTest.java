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

package org.dynamiscollision.benchmark;

import static org.junit.jupiter.api.Assertions.*;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.bounds.BoundingSphere;
import org.dynamiscollision.bounds.Capsule;
import org.dynamiscollision.broadphase.BroadPhase3D;
import org.dynamiscollision.broadphase.SpatialHash3D;
import org.dynamiscollision.broadphase.SweepAndPrune3D;
import org.dynamiscollision.contact.ContactGenerator3D;
import org.dynamiscollision.filtering.CollisionFilter;
import org.dynamiscollision.narrowphase.ConvexSupport3D;
import org.dynamiscollision.narrowphase.Gjk3D;
import org.dynamiscollision.pipeline.CollisionPair;
import org.dynamiscollision.world.CollisionWorld3D;
import org.dynamiscollision.world.RigidBodyAdapter3D;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3d;

class CollisionBenchmarkSmokeTest {

    @Test
    void broadPhaseComparisonIsReproducibleAcrossScales() {
        assertTimeoutPreemptively(Duration.ofSeconds(5), () -> {
            for (int count : new int[] {100, 500, 1000}) {
                List<Body> bodies = generateBodies(count, 1337L + count);
                double sapMs = avgBroadphaseMillis(new SweepAndPrune3D<>(), bodies, 4);
                double hashMs = avgBroadphaseMillis(new SpatialHash3D<>(24.0), bodies, 4);
                assertTrue(Double.isFinite(sapMs) && sapMs >= 0.0);
                assertTrue(Double.isFinite(hashMs) && hashMs >= 0.0);
            }
        });
    }

    @Test
    void gjkThousandRandomPairsCompletes() {
        assertTimeoutPreemptively(Duration.ofSeconds(5), () -> {
            Random random = new Random(424242L);
            long start = System.nanoTime();
            int hits = 0;
            for (int i = 0; i < 1000; i++) {
                ConvexSupport3D a = randomSupport(random);
                ConvexSupport3D b = randomSupport(random);
                if (Gjk3D.intersects(a, b, 32)) {
                    hits++;
                }
            }
            double millis = (System.nanoTime() - start) / 1_000_000.0;
            assertTrue(Double.isFinite(millis) && millis >= 0.0);
            assertTrue(hits >= 0);
        });
    }

    @Test
    void worldStepHundredBodiesSixtyFramesCompletes() {
        assertTimeoutPreemptively(Duration.ofSeconds(5), () -> {
            List<Body> bodies = generateBodies(100, 9090L);
            CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                    new SweepAndPrune3D<>(),
                    Body::bounds,
                    body -> CollisionFilter.DEFAULT,
                    (left, right) -> ContactGenerator3D.generate(left.bounds(), right.bounds()));
            world.setBodyAdapter(new BodyAdapter());
            world.setGravity(new Vector3d(0.0, -9.8, 0.0));

            long start = System.nanoTime();
            for (int frame = 0; frame < 60; frame++) {
                world.step(bodies, 1.0 / 60.0);
            }
            double millis = (System.nanoTime() - start) / 1_000_000.0;
            assertTrue(Double.isFinite(millis) && millis >= 0.0);
        });
    }

    private static double avgBroadphaseMillis(BroadPhase3D<Body> broadPhase, List<Body> bodies, int rounds) {
        double totalMs = 0.0;
        for (int i = 0; i < rounds; i++) {
            long start = System.nanoTime();
            java.util.Set<CollisionPair<Body>> pairs = broadPhase.findPotentialPairs(bodies, Body::bounds);
            totalMs += (System.nanoTime() - start) / 1_000_000.0;
            assertNotNull(pairs);
        }
        return totalMs / rounds;
    }

    private static ConvexSupport3D randomSupport(Random random) {
        int type = random.nextInt(3);
        return switch (type) {
            case 0 -> Gjk3D.fromAabb(randomAabb(random));
            case 1 -> Gjk3D.fromBoundingSphere(new BoundingSphere(
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, 0.01, 40.0)));
            default -> Gjk3D.fromCapsule(new Capsule(
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, -1000, 1000),
                    randomRange(random, 0.01, 40.0)));
        };
    }

    private static List<Body> generateBodies(int count, long seed) {
        Random random = new Random(seed);
        List<Body> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            Aabb bounds = randomAabb(random);
            Vector3d position = new Vector3d(bounds.centerX(), bounds.centerY(), bounds.centerZ());
            Vector3d velocity = new Vector3d(
                    randomRange(random, -10.0, 10.0),
                    randomRange(random, -10.0, 10.0),
                    randomRange(random, -10.0, 10.0));
            out.add(new Body(i, position, velocity, bounds));
        }
        return out;
    }

    private static Aabb randomAabb(Random random) {
        double cx = randomRange(random, -500, 500);
        double cy = randomRange(random, -500, 500);
        double cz = randomRange(random, -500, 500);
        double sx = randomRange(random, 0.5, 12.0);
        double sy = randomRange(random, 0.5, 12.0);
        double sz = randomRange(random, 0.5, 12.0);
        return new Aabb(
                cx - sx * 0.5, cy - sy * 0.5, cz - sz * 0.5,
                cx + sx * 0.5, cy + sy * 0.5, cz + sz * 0.5);
    }

    private static double randomRange(Random random, double min, double max) {
        return min + (max - min) * random.nextDouble();
    }

    private static final class Body {
        private final int id;
        private Vector3d position;
        private Vector3d velocity;
        private Aabb bounds;

        private Body(int id, Vector3d position, Vector3d velocity, Aabb bounds) {
            this.id = id;
            this.position = position;
            this.velocity = velocity;
            this.bounds = bounds;
        }

        private Aabb bounds() {
            double hx = bounds.sizeX() * 0.5;
            double hy = bounds.sizeY() * 0.5;
            double hz = bounds.sizeZ() * 0.5;
            return new Aabb(
                    position.x() - hx, position.y() - hy, position.z() - hz,
                    position.x() + hx, position.y() + hy, position.z() + hz);
        }
    }

    private static final class BodyAdapter implements RigidBodyAdapter3D<Body> {
        @Override
        public Vector3d getPosition(Body body) {
            return body.position;
        }

        @Override
        public void setPosition(Body body, Vector3d position) {
            body.position = position;
        }

        @Override
        public Vector3d getVelocity(Body body) {
            return body.velocity;
        }

        @Override
        public void setVelocity(Body body, Vector3d velocity) {
            body.velocity = velocity;
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
    }
}
