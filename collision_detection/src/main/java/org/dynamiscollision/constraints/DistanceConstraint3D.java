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

import org.dynamiscollision.world.RigidBodyAdapter3D;
import org.vectrix.core.Vector3d;

/**
 * Keeps two bodies near a target distance.
 */
public final class DistanceConstraint3D<T> implements Constraint3D<T> {

    private final T bodyA;
    private final T bodyB;
    private final double targetDistance;
    private final double stiffness;

    public DistanceConstraint3D(T bodyA, T bodyB, double targetDistance, double stiffness) {
        if (bodyA == null || bodyB == null) {
            throw new IllegalArgumentException("bodies must not be null");
        }
        if (!Double.isFinite(targetDistance) || targetDistance < 0.0) {
            throw new IllegalArgumentException("targetDistance must be >= 0");
        }
        if (!Double.isFinite(stiffness) || stiffness < 0.0 || stiffness > 1.0) {
            throw new IllegalArgumentException("stiffness must be in [0,1]");
        }
        this.bodyA = bodyA;
        this.bodyB = bodyB;
        this.targetDistance = targetDistance;
        this.stiffness = stiffness;
    }

    @Override
    public void solve(RigidBodyAdapter3D<T> adapter, double dtSeconds) {
        Vector3d pa = adapter.getPosition(bodyA);
        Vector3d pb = adapter.getPosition(bodyB);
        double dx = pb.x() - pa.x();
        double dy = pb.y() - pa.y();
        double dz = pb.z() - pa.z();
        double dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        if (dist <= 1e-9) {
            return;
        }

        double invMassA = Math.max(0.0, adapter.getInverseMass(bodyA));
        double invMassB = Math.max(0.0, adapter.getInverseMass(bodyB));
        double invMassSum = invMassA + invMassB;
        if (invMassSum <= 0.0) {
            return;
        }

        double error = dist - targetDistance;
        if (Math.abs(error) <= 1e-9) {
            return;
        }
        double nx = dx / dist;
        double ny = dy / dist;
        double nz = dz / dist;
        double correctionMag = error * stiffness;

        Vector3d correction = new Vector3d(nx * correctionMag, ny * correctionMag, nz * correctionMag);
        adapter.setPosition(bodyA, new Vector3d(
                pa.x() + correction.x() * (invMassA / invMassSum),
                pa.y() + correction.y() * (invMassA / invMassSum),
                pa.z() + correction.z() * (invMassA / invMassSum)));
        adapter.setPosition(bodyB, new Vector3d(
                pb.x() - correction.x() * (invMassB / invMassSum),
                pb.y() - correction.y() * (invMassB / invMassSum),
                pb.z() - correction.z() * (invMassB / invMassSum)));
    }
}
