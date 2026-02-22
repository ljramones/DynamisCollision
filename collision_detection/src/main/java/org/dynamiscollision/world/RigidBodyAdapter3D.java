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

import org.vectrix.core.Vector3d;

/**
 * Adapter for reading and writing rigid-body state from user objects.
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
