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

/**
 * Layer/mask filter and trigger/solid classification.
 */
public record CollisionFilter(int layerBits, int maskBits, CollisionKind kind) {

    public static final CollisionFilter DEFAULT = new CollisionFilter(1, 0xFFFFFFFF, CollisionKind.SOLID);

    public CollisionFilter {
        if (layerBits == 0) {
            throw new IllegalArgumentException("layerBits must not be 0");
        }
        if (kind == null) {
            throw new IllegalArgumentException("kind must not be null");
        }
    }

    public boolean canInteract(CollisionFilter other) {
        if (other == null) {
            throw new IllegalArgumentException("other must not be null");
        }
        return (this.layerBits & other.maskBits) != 0
                && (other.layerBits & this.maskBits) != 0;
    }

    public boolean responseEnabledWith(CollisionFilter other) {
        return canInteract(other)
                && this.kind == CollisionKind.SOLID
                && other.kind == CollisionKind.SOLID;
    }
}
