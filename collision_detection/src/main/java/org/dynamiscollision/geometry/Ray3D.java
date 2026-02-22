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
 * Immutable 3D ray defined by origin and direction.
 */
public record Ray3D(
        double originX,
        double originY,
        double originZ,
        double dirX,
        double dirY,
        double dirZ) {

    public Ray3D {
        validateFinite(originX, "originX");
        validateFinite(originY, "originY");
        validateFinite(originZ, "originZ");
        validateFinite(dirX, "dirX");
        validateFinite(dirY, "dirY");
        validateFinite(dirZ, "dirZ");
        if (dirX == 0.0 && dirY == 0.0 && dirZ == 0.0) {
            throw new IllegalArgumentException("Ray direction must be non-zero");
        }
    }

    private static void validateFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }
}
