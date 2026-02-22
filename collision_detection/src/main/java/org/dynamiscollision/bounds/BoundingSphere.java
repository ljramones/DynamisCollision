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
 * Immutable bounding sphere in 3D.
 */
public record BoundingSphere(
        double centerX,
        double centerY,
        double centerZ,
        double radius) {

    public BoundingSphere {
        validateFinite(centerX, "centerX");
        validateFinite(centerY, "centerY");
        validateFinite(centerZ, "centerZ");
        validateFinite(radius, "radius");
        if (radius < 0.0) {
            throw new IllegalArgumentException("radius must be >= 0");
        }
    }

    public boolean intersects(BoundingSphere other) {
        return Intersection3D.intersects(this, other);
    }

    public boolean intersects(Aabb other) {
        return Intersection3D.intersects(this, other);
    }

    public Aabb toAabb() {
        return new Aabb(
                centerX - radius,
                centerY - radius,
                centerZ - radius,
                centerX + radius,
                centerY + radius,
                centerZ + radius);
    }

    private static void validateFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }
}
