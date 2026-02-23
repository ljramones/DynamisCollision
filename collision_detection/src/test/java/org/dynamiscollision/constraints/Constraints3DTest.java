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

package org.dynamiscollision.constraints;

import static org.junit.jupiter.api.Assertions.*;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.broadphase.SweepAndPrune3D;
import org.dynamiscollision.contact.ContactGenerator3D;
import org.dynamiscollision.filtering.CollisionFilter;
import org.dynamiscollision.world.CollisionWorld3D;
import org.dynamiscollision.world.RigidBodyAdapter3D;
import org.junit.jupiter.api.Test;
import org.vectrix.core.Vector3d;

class Constraints3DTest {

    private static final RigidBodyAdapter3D<Body> ADAPTER = new RigidBodyAdapter3D<>() {
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
            return 0;
        }

        @Override
        public double getFriction(Body body) {
            return 0;
        }
    };

    @Test
    void distanceConstraintConvergesTowardTarget() {
        Body a = new Body(new Vector3d(0, 0, 0), 1.0);
        Body b = new Body(new Vector3d(4, 0, 0), 1.0);
        DistanceConstraint3D<Body> c = new DistanceConstraint3D<>(a, b, 2.0, 1.0);

        c.solve(ADAPTER, 0.016);
        double dx = b.position.x() - a.position.x();
        assertEquals(2.0, Math.abs(dx), 1e-9);
    }

    @Test
    void pointConstraintPullsBodyToAnchor() {
        Body a = new Body(new Vector3d(10, 0, 0), 1.0);
        PointConstraint3D<Body> c = new PointConstraint3D<>(a, new Vector3d(1, 2, 3), 1.0);
        c.solve(ADAPTER, 0.016);

        assertEquals(1.0, a.position.x(), 1e-9);
        assertEquals(2.0, a.position.y(), 1e-9);
        assertEquals(3.0, a.position.z(), 1e-9);
    }

    @Test
    void worldStepMaintainsDistanceConstraintUnderGravity() {
        Body a = new Body(new Vector3d(0, 2, 0), 1.0);
        Body b = new Body(new Vector3d(0, 4, 0), 1.0);

        CollisionWorld3D<Body> world = new CollisionWorld3D<>(
                new SweepAndPrune3D<>(),
                Body::aabb,
                body -> CollisionFilter.DEFAULT,
                (left, right) -> ContactGenerator3D.generate(left.aabb(), right.aabb()));
        world.setBodyAdapter(ADAPTER);
        world.setGravity(new Vector3d(0, -9.8, 0));
        world.setConstraintIterations(4);
        world.addConstraint(new DistanceConstraint3D<>(a, b, 2.0, 0.8));

        for (int i = 0; i < 30; i++) {
            world.step(java.util.List.of(a, b), 1.0 / 120.0);
        }
        double dx = b.position.x() - a.position.x();
        double dy = b.position.y() - a.position.y();
        double dz = b.position.z() - a.position.z();
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        assertTrue(Math.abs(dist - 2.0) < 0.2);
    }

    @Test
    void distanceConstraintLeavesBodiesWhenBothStatic() {
        Body a = new Body(new Vector3d(0, 0, 0), 0.0);
        Body b = new Body(new Vector3d(10, 0, 0), 0.0);
        DistanceConstraint3D<Body> c = new DistanceConstraint3D<>(a, b, 1.0, 1.0);

        c.solve(ADAPTER, 0.016);
        assertEquals(0.0, a.position.x(), 1e-9);
        assertEquals(10.0, b.position.x(), 1e-9);
    }

    @Test
    void pointConstraintIgnoresStaticBody() {
        Body a = new Body(new Vector3d(5, 5, 5), 0.0);
        PointConstraint3D<Body> c = new PointConstraint3D<>(a, new Vector3d(0, 0, 0), 1.0);
        c.solve(ADAPTER, 0.016);
        assertEquals(5.0, a.position.x(), 1e-9);
        assertEquals(5.0, a.position.y(), 1e-9);
        assertEquals(5.0, a.position.z(), 1e-9);
    }

    @Test
    void validatesConstraintConstruction() {
        Body a = new Body(new Vector3d(0, 0, 0), 1.0);
        Body b = new Body(new Vector3d(1, 0, 0), 1.0);

        assertThrows(IllegalArgumentException.class, () -> new DistanceConstraint3D<>(null, b, 1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new DistanceConstraint3D<>(a, b, -1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new DistanceConstraint3D<>(a, b, 1.0, 2.0));

        assertThrows(IllegalArgumentException.class, () -> new PointConstraint3D<>(null, new Vector3d(0, 0, 0), 1.0));
        assertThrows(IllegalArgumentException.class, () -> new PointConstraint3D<>(a, null, 1.0));
        assertThrows(IllegalArgumentException.class, () -> new PointConstraint3D<>(a, new Vector3d(0, 0, 0), -0.1));
    }

    private static final class Body {
        private Vector3d position;
        private Vector3d velocity = new Vector3d(0, 0, 0);
        private final double inverseMass;

        private Body(Vector3d position, double inverseMass) {
            this.position = position;
            this.inverseMass = inverseMass;
        }

        private Aabb aabb() {
            return new Aabb(
                    position.x() - 0.25, position.y() - 0.25, position.z() - 0.25,
                    position.x() + 0.25, position.y() + 0.25, position.z() + 0.25);
        }
    }
}
