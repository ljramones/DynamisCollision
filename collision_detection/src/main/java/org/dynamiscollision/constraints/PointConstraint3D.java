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
 * Pulls a body toward a fixed world-space anchor.
 */
public final class PointConstraint3D<T> implements Constraint3D<T> {

    private final T body;
    private final Vector3d anchor;
    private final double stiffness;

    public PointConstraint3D(T body, Vector3d anchor, double stiffness) {
        if (body == null || anchor == null) {
            throw new IllegalArgumentException("body and anchor must not be null");
        }
        if (!Double.isFinite(stiffness) || stiffness < 0.0 || stiffness > 1.0) {
            throw new IllegalArgumentException("stiffness must be in [0,1]");
        }
        this.body = body;
        this.anchor = anchor;
        this.stiffness = stiffness;
    }

    @Override
    public void solve(RigidBodyAdapter3D<T> adapter, double dtSeconds) {
        double invMass = Math.max(0.0, adapter.getInverseMass(body));
        if (invMass <= 0.0) {
            return;
        }
        Vector3d p = adapter.getPosition(body);
        Vector3d corrected = new Vector3d(
                p.x() + (anchor.x() - p.x()) * stiffness,
                p.y() + (anchor.y() - p.y()) * stiffness,
                p.z() + (anchor.z() - p.z()) * stiffness);
        adapter.setPosition(body, corrected);
    }
}
