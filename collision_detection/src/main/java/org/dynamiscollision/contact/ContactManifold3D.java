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

import java.util.List;
import org.dynamiscollision.narrowphase.CollisionManifold3D;

/**
 * Collision manifold with contact points.
 */
public record ContactManifold3D(
        CollisionManifold3D manifold,
        List<ContactPoint3D> contacts) {

    public ContactManifold3D {
        if (manifold == null) {
            throw new IllegalArgumentException("manifold must not be null");
        }
        if (contacts == null || contacts.isEmpty()) {
            throw new IllegalArgumentException("contacts must not be null or empty");
        }
        contacts = List.copyOf(contacts);
    }
}
