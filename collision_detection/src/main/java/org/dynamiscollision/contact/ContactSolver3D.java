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

import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.pipeline.CollisionPair;
import org.dynamiscollision.world.CollisionResponder3D;
import org.dynamiscollision.world.RigidBodyAdapter3D;
import org.vectrix.core.Vector3d;

/**
 * Basic position/velocity response solver for collision contacts.
 */
public final class ContactSolver3D<T> implements CollisionResponder3D<T> {

    private final RigidBodyAdapter3D<T> adapter;
    private double positionCorrectionPercent = 0.8;
    private double positionCorrectionSlop = 0.001;

    public ContactSolver3D(RigidBodyAdapter3D<T> adapter) {
        if (adapter == null) {
            throw new IllegalArgumentException("adapter must not be null");
        }
        this.adapter = adapter;
    }

    public void setPositionCorrectionPercent(double value) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException("positionCorrectionPercent must be in [0,1]");
        }
        this.positionCorrectionPercent = value;
    }

    public void setPositionCorrectionSlop(double value) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException("positionCorrectionSlop must be >= 0");
        }
        this.positionCorrectionSlop = value;
    }

    @Override
    public void resolve(CollisionEvent<T> event) {
        if (event == null || !event.responseEnabled() || event.type() == CollisionEventType.EXIT || event.manifold() == null) {
            return;
        }
        solvePosition(event.pair(), event.manifold());
        solveVelocity(event.pair(), event.manifold(), WarmStartImpulse.ZERO);
    }

    public void solve(CollisionPair<T> pair, ContactManifold3D contact) {
        solvePosition(pair, contact);
        solveVelocity(pair, contact, WarmStartImpulse.ZERO);
    }

    public void solvePosition(CollisionEvent<T> event) {
        if (event == null || !event.responseEnabled() || event.type() == CollisionEventType.EXIT || event.manifold() == null) {
            return;
        }
        solvePosition(event.pair(), event.manifold());
    }

    public WarmStartImpulse solveVelocity(CollisionEvent<T> event, WarmStartImpulse warmStart) {
        if (event == null || !event.responseEnabled() || event.type() == CollisionEventType.EXIT || event.manifold() == null) {
            return WarmStartImpulse.ZERO;
        }
        return solveVelocity(event.pair(), event.manifold(), warmStart == null ? WarmStartImpulse.ZERO : warmStart);
    }

    public void solvePosition(CollisionPair<T> pair, ContactManifold3D contact) {
        if (pair == null || contact == null) {
            throw new IllegalArgumentException("pair and contact must not be null");
        }
        T bodyA = pair.first();
        T bodyB = pair.second();
        CollisionManifold3D manifold = contact.manifold();

        double invMassA = Math.max(0.0, adapter.getInverseMass(bodyA));
        double invMassB = Math.max(0.0, adapter.getInverseMass(bodyB));
        double invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0) {
            return;
        }

        Vector3d normal = new Vector3d(manifold.normalX(), manifold.normalY(), manifold.normalZ());
        solvePosition(bodyA, bodyB, normal, manifold.penetrationDepth(), invMassA, invMassB, invMassSum);
    }

    public WarmStartImpulse solveVelocity(CollisionPair<T> pair, ContactManifold3D contact, WarmStartImpulse warmStart) {
        if (pair == null || contact == null) {
            throw new IllegalArgumentException("pair and contact must not be null");
        }
        T bodyA = pair.first();
        T bodyB = pair.second();
        CollisionManifold3D manifold = contact.manifold();

        double invMassA = Math.max(0.0, adapter.getInverseMass(bodyA));
        double invMassB = Math.max(0.0, adapter.getInverseMass(bodyB));
        double invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0) {
            return WarmStartImpulse.ZERO;
        }

        Vector3d normal = new Vector3d(manifold.normalX(), manifold.normalY(), manifold.normalZ());
        return solveVelocity(bodyA, bodyB, normal, invMassA, invMassB, invMassSum,
                warmStart == null ? WarmStartImpulse.ZERO : warmStart);
    }

    private void solvePosition(
            T bodyA,
            T bodyB,
            Vector3d normal,
            double penetrationDepth,
            double invMassA,
            double invMassB,
            double invMassSum) {

        double correctionMagnitude = Math.max(0.0, penetrationDepth - positionCorrectionSlop)
                * positionCorrectionPercent / invMassSum;
        if (correctionMagnitude <= 0.0) {
            return;
        }
        Vector3d correction = scale(normal, correctionMagnitude);

        Vector3d positionA = adapter.getPosition(bodyA);
        Vector3d positionB = adapter.getPosition(bodyB);
        adapter.setPosition(bodyA, sub(positionA, scale(correction, invMassA)));
        adapter.setPosition(bodyB, add(positionB, scale(correction, invMassB)));
    }

    private WarmStartImpulse solveVelocity(
            T bodyA,
            T bodyB,
            Vector3d normal,
            double invMassA,
            double invMassB,
            double invMassSum,
            WarmStartImpulse warmStart) {

        Vector3d velocityA = adapter.getVelocity(bodyA);
        Vector3d velocityB = adapter.getVelocity(bodyB);

        Vector3d tangentDir = tangentDirection(sub(velocityB, velocityA), normal);
        if (Math.abs(warmStart.normalImpulse()) > 0.0 || Math.abs(warmStart.tangentImpulse()) > 0.0) {
            Vector3d warmImpulse = add(
                    scale(normal, warmStart.normalImpulse()),
                    scale(tangentDir, warmStart.tangentImpulse()));
            velocityA = sub(velocityA, scale(warmImpulse, invMassA));
            velocityB = add(velocityB, scale(warmImpulse, invMassB));
        }

        Vector3d relativeVelocity = sub(velocityB, velocityA);
        double velocityAlongNormal = dot(relativeVelocity, normal);
        double accumulatedNormal = warmStart.normalImpulse();
        double accumulatedTangent = warmStart.tangentImpulse();
        if (velocityAlongNormal > 0.0) {
            adapter.setVelocity(bodyA, velocityA);
            adapter.setVelocity(bodyB, velocityB);
            return new WarmStartImpulse(accumulatedNormal, accumulatedTangent);
        }

        double restitution = Math.min(
                clamp01(adapter.getRestitution(bodyA)),
                clamp01(adapter.getRestitution(bodyB)));

        double impulseScalar = -(1.0 + restitution) * velocityAlongNormal / invMassSum;
        double oldAccumulatedNormal = accumulatedNormal;
        accumulatedNormal = Math.max(0.0, accumulatedNormal + impulseScalar);
        double clampedNormalDelta = accumulatedNormal - oldAccumulatedNormal;
        Vector3d impulse = scale(normal, clampedNormalDelta);
        velocityA = sub(velocityA, scale(impulse, invMassA));
        velocityB = add(velocityB, scale(impulse, invMassB));

        Vector3d rvAfterNormal = sub(velocityB, velocityA);
        Vector3d tangent = tangentDirection(rvAfterNormal, normal);
        double jt = -dot(rvAfterNormal, tangent) / invMassSum;
        double friction = Math.sqrt(
                Math.max(0.0, adapter.getFriction(bodyA))
                        * Math.max(0.0, adapter.getFriction(bodyB)));
        double maxFriction = accumulatedNormal * friction;
        double desiredTangent = accumulatedTangent + jt;
        double clampedTangent = clamp(desiredTangent, -maxFriction, maxFriction);
        double tangentDelta = clampedTangent - accumulatedTangent;
        accumulatedTangent = clampedTangent;
        if (Math.abs(tangentDelta) > 1e-12) {
            Vector3d frictionImpulse = scale(tangent, tangentDelta);
            velocityA = sub(velocityA, scale(frictionImpulse, invMassA));
            velocityB = add(velocityB, scale(frictionImpulse, invMassB));
        }

        adapter.setVelocity(bodyA, velocityA);
        adapter.setVelocity(bodyB, velocityB);
        return new WarmStartImpulse(accumulatedNormal, accumulatedTangent);
    }

    private static Vector3d tangentDirection(Vector3d relativeVelocity, Vector3d normal) {
        double tangentX = relativeVelocity.x() - normal.x() * dot(relativeVelocity, normal);
        double tangentY = relativeVelocity.y() - normal.y() * dot(relativeVelocity, normal);
        double tangentZ = relativeVelocity.z() - normal.z() * dot(relativeVelocity, normal);
        double tangentLen = Math.sqrt(tangentX * tangentX + tangentY * tangentY + tangentZ * tangentZ);
        if (tangentLen > 1e-9) {
            return new Vector3d(tangentX / tangentLen, tangentY / tangentLen, tangentZ / tangentLen);
        }
        return anyPerpendicular(normal);
    }

    private static Vector3d anyPerpendicular(Vector3d normal) {
        Vector3d axis = Math.abs(normal.x()) < 0.9 ? new Vector3d(1, 0, 0) : new Vector3d(0, 1, 0);
        Vector3d tangent = cross(normal, axis);
        double len = Math.sqrt(dot(tangent, tangent));
        if (len <= 1e-9) {
            return new Vector3d(0, 0, 1);
        }
        return scale(tangent, 1.0 / len);
    }

    private static Vector3d cross(Vector3d a, Vector3d b) {
        return new Vector3d(
                a.y() * b.z() - a.z() * b.y(),
                a.z() * b.x() - a.x() * b.z(),
                a.x() * b.y() - a.y() * b.x());
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double clamp01(double value) {
        if (value < 0.0) {
            return 0.0;
        }
        return Math.min(1.0, value);
    }

    private static double dot(Vector3d a, Vector3d b) {
        return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
    }

    private static Vector3d add(Vector3d a, Vector3d b) {
        return new Vector3d(a.x() + b.x(), a.y() + b.y(), a.z() + b.z());
    }

    private static Vector3d sub(Vector3d a, Vector3d b) {
        return new Vector3d(a.x() - b.x(), a.y() - b.y(), a.z() - b.z());
    }

    private static Vector3d scale(Vector3d v, double scale) {
        return new Vector3d(v.x() * scale, v.y() * scale, v.z() * scale);
    }
}
