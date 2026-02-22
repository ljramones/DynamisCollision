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
 * SAT narrow-phase result containing minimum translation vector direction and depth.
 */
public record CollisionManifold2D(
        double normalX,
        double normalY,
        double penetrationDepth) {

    public CollisionManifold2D {
        if (!Double.isFinite(normalX) || !Double.isFinite(normalY) || !Double.isFinite(penetrationDepth)) {
            throw new IllegalArgumentException("values must be finite");
        }
        if (penetrationDepth < 0.0) {
            throw new IllegalArgumentException("penetrationDepth must be >= 0");
        }
        double lenSq = normalX * normalX + normalY * normalY;
        if (Math.abs(lenSq - 1.0) > 1e-6) {
            throw new IllegalArgumentException("normal must be unit length");
        }
    }
}
