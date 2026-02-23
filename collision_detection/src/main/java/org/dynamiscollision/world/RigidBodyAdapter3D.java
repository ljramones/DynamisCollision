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

package org.dynamiscollision.world;

import org.vectrix.core.Vector3d;

/**
 * Adapter for reading and writing rigid-body state from user objects.
 *
 * <p>This interface is also the integration boundary for animation-driven collision updates.
 * Typical flow for character animation integration:
 *
 * <ol>
 *   <li>Animation system evaluates a pose and computes world-space bone transforms.</li>
 *   <li>Caller maps bone transforms to collision shape transforms/bounds for each body.</li>
 *   <li>Adapter returns those world-space positions/velocities to {@link CollisionWorld3D}.</li>
 *   <li>After collision solve, caller may consume resolved positions/velocities to drive gameplay state.</li>
 * </ol>
 *
 * <p>Contract requirements:
 * <ul>
 *   <li>All returned vectors/scalars must be finite.</li>
 *   <li>`getPosition`/`setPosition` and `getVelocity`/`setVelocity` must reference the same world-space frame.</li>
 *   <li>For kinematic/animation-authored bodies, return `0` inverse mass.</li>
 *   <li>If animation is authoritative, apply animation pose first, then run collision, then resolve final pose/state in caller code.</li>
 * </ul>
 */
public interface RigidBodyAdapter3D<T> {

    Vector3d getPosition(T body);

    void setPosition(T body, Vector3d position);

    Vector3d getVelocity(T body);

    void setVelocity(T body, Vector3d velocity);

    double getInverseMass(T body);

    double getRestitution(T body);

    double getFriction(T body);
}
