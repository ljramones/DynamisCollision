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
import org.vectrix.core.Vector3d;
import org.junit.jupiter.api.Test;

class ContactSolver3DTest {

    private static final RigidBodyAdapter3D<Body> BODY_ADAPTER = new RigidBodyAdapter3D<>() {
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
            return body.inverseMass;
        }

        @Override
        public double getRestitution(Body body) {
            return body.restitution;
        }

        @Override
        public double getFriction(Body body) {
            return body.friction;
        }
    };

    @Test
    void separatesOverlapForSolidPairs() {
        Body a = body("a", 0, 0, 0, 1, 1, 1, 1.0, CollisionFilter.DEFAULT);
        Body b = body("b", 1.5, 0, 0, 1, 1, 1, 1.0, CollisionFilter.DEFAULT);

        ContactManifold3D contact = ContactGenerator3D.generate(a.aabb(), b.aabb()).orElseThrow();
        ContactSolver3D<Body> solver = new ContactSolver3D<>(BODY_ADAPTER);
        solver.setPositionCorrectionPercent(1.0);
        solver.setPositionCorrectionSlop(0.0);

        solver.resolve(new CollisionEvent<>(
                new CollisionPair<>(a, b),
                CollisionEventType.ENTER,
                true,
                contact));

        double overlapX = Math.min(a.aabb().maxX(), b.aabb().maxX()) - Math.max(a.aabb().minX(), b.aabb().minX());
        assertTrue(overlapX <= 1e-9);
    }

    @Test
    void doesNotResolveTriggerPairs() {
        Body a = body("a", 0, 0, 0, 1, 1, 1, 1.0,
                new CollisionFilter(0b01, 0b11, CollisionKind.SOLID));
        Body b = body("b", 1.5, 0, 0, 1, 1, 1, 1.0,
                new CollisionFilter(0b10, 0b11, CollisionKind.TRIGGER));

        Vector3d startA = a.position;
        Vector3d startB = b.position;
        ContactManifold3D contact = ContactGenerator3D.generate(a.aabb(), b.aabb()).orElseThrow();
        ContactSolver3D<Body> solver = new ContactSolver3D<>(BODY_ADAPTER);
        solver.setPositionCorrectionPercent(1.0);
        solver.setPositionCorrectionSlop(0.0);

        solver.resolve(new CollisionEvent<>(
                new CollisionPair<>(a, b),
                CollisionEventType.ENTER,
                false,
                contact));

        assertEquals(startA.x(), a.position.x(), 1e-9);
        assertEquals(startB.x(), b.position.x(), 1e-9);
    }

    @Test
    void keepsRestingContactStableAcrossFrames() {
        Body floor = body("floor", 0, 0, 0, 5, 0.5, 5, 0.0, CollisionFilter.DEFAULT);
        Body box = body("box", 0, 0.9, 0, 0.5, 0.5, 0.5, 1.0, CollisionFilter.DEFAULT);

        ContactSolver3D<Body> solver = new ContactSolver3D<>(BODY_ADAPTER);
        solver.setPositionCorrectionPercent(1.0);
        solver.setPositionCorrectionSlop(0.0);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::aabb,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.aabb(), right.aabb()));
        world.setResponder(solver);

        List<CollisionEvent<Body>> frame1 = world.update(List.of(floor, box));
        assertEquals(CollisionEventType.ENTER, frame1.get(0).type());

        for (int i = 0; i < 5; i++) {
            List<CollisionEvent<Body>> events = world.update(List.of(floor, box));
            assertEquals(1, events.size());
            assertEquals(CollisionEventType.STAY, events.get(0).type());
        }

        // resting: floor top at y=0.5, box half-height=0.5 => center y should be 1.0
        assertEquals(1.0, box.position.y(), 1e-6);
        assertTrue(Intersection3D.intersects(floor.aabb(), box.aabb()));
    }

    @Test
    void iterativeSolverImprovesStackStability() {
        Scenario oneIter = runStackScenario(1, 40);
        Scenario manyIter = runStackScenario(8, 40);

        assertTrue(Double.isFinite(oneIter.top.position.y()));
        assertTrue(Double.isFinite(manyIter.top.position.y()));
        assertTrue(manyIter.top.position.y() > 1.0);
    }

    @Test
    void iterativeSolverReducesRestingJitter() {
        double jitterOne = jitterMetric(runStackScenario(1, 60).topHistory);
        double jitterMany = jitterMetric(runStackScenario(8, 60).topHistory);
        assertTrue(Double.isFinite(jitterOne));
        assertTrue(Double.isFinite(jitterMany));
        assertTrue(jitterMany < 10.0);
    }

    @Test
    void fixedStepSimulationIsDeterministic() {
        Scenario runA = runStackScenario(6, 50);
        Scenario runB = runStackScenario(6, 50);

        assertEquals(runA.bottom.position.y(), runB.bottom.position.y(), 1e-9);
        assertEquals(runA.top.position.y(), runB.top.position.y(), 1e-9);
        assertEquals(runA.bottom.velocity.y(), runB.bottom.velocity.y(), 1e-9);
        assertEquals(runA.top.velocity.y(), runB.top.velocity.y(), 1e-9);
    }

    private static Scenario runStackScenario(int iterations, int frames) {
        Body floor = body("floor", 0, 0, 0, 5, 0.5, 5, 0.0, CollisionFilter.DEFAULT);
        Body bottom = body("bottom", 0, 0.9, 0, 0.5, 0.5, 0.5, 1.0, CollisionFilter.DEFAULT);
        Body top = body("top", 0, 1.7, 0, 0.5, 0.5, 0.5, 1.0, CollisionFilter.DEFAULT);

        ContactSolver3D<Body> solver = new ContactSolver3D<>(BODY_ADAPTER);
        solver.setPositionCorrectionPercent(1.0);
        solver.setPositionCorrectionSlop(0.0);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::aabb,
                Body::filter,
                (left, right) -> ContactGenerator3D.generate(left.aabb(), right.aabb()));
        world.setResponder(solver);
        world.setSolverIterations(iterations);

        double[] topHistory = new double[frames];
        for (int i = 0; i < frames; i++) {
            // fixed-step gravity integration
            if (bottom.inverseMass > 0) {
                bottom.velocity = new Vector3d(bottom.velocity.x(), bottom.velocity.y() - 0.01, bottom.velocity.z());
                bottom.position = new Vector3d(bottom.position.x(), bottom.position.y() + bottom.velocity.y(), bottom.position.z());
            }
            if (top.inverseMass > 0) {
                top.velocity = new Vector3d(top.velocity.x(), top.velocity.y() - 0.01, top.velocity.z());
                top.position = new Vector3d(top.position.x(), top.position.y() + top.velocity.y(), top.position.z());
            }
            world.update(List.of(floor, bottom, top));
            topHistory[i] = top.position.y();
        }
        return new Scenario(bottom, top, topHistory);
    }

    private static double jitterMetric(double[] values) {
        double sum = 0.0;
        for (int i = Math.max(1, values.length - 20); i < values.length; i++) {
            sum += Math.abs(values[i] - values[i - 1]);
        }
        return sum;
    }

    private static Body body(
            String id,
            double x,
            double y,
            double z,
            double hx,
            double hy,
            double hz,
            double inverseMass,
            CollisionFilter filter) {
        Body body = new Body();
        body.id = id;
        body.position = new Vector3d(x, y, z);
        body.velocity = new Vector3d(0, 0, 0);
        body.halfX = hx;
        body.halfY = hy;
        body.halfZ = hz;
        body.inverseMass = inverseMass;
        body.restitution = 0.0;
        body.friction = 0.5;
        body.filter = filter;
        return body;
    }

    private static final class Body {
        private String id;
        private Vector3d position;
        private Vector3d velocity;
        private double halfX;
        private double halfY;
        private double halfZ;
        private double inverseMass;
        private double restitution;
        private double friction;
        private CollisionFilter filter;

        private Aabb aabb() {
            return new Aabb(
                    position.x() - halfX, position.y() - halfY, position.z() - halfZ,
                    position.x() + halfX, position.y() + halfY, position.z() + halfZ);
        }

        private CollisionFilter filter() {
            return filter;
        }

        @Override
        public String toString() {
            return id;
        }
    }

    private record Scenario(Body bottom, Body top, double[] topHistory) {
    }
}
