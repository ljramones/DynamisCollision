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

import org.meshforge.core.bounds.Aabbf;

/**
 * Immutable axis-aligned bounding box in 3D.
 */
public record Aabb(
        double minX,
        double minY,
        double minZ,
        double maxX,
        double maxY,
        double maxZ) {

    public Aabb {
        validateFinite(minX, "minX");
        validateFinite(minY, "minY");
        validateFinite(minZ, "minZ");
        validateFinite(maxX, "maxX");
        validateFinite(maxY, "maxY");
        validateFinite(maxZ, "maxZ");
        if (minX > maxX || minY > maxY || minZ > maxZ) {
            throw new IllegalArgumentException("Aabb min values must be <= max values");
        }
    }

    public static Aabb fromAabbf(Aabbf bounds) {
        if (bounds == null) {
            throw new IllegalArgumentException("bounds must not be null");
        }
        return new Aabb(
                bounds.minX(),
                bounds.minY(),
                bounds.minZ(),
                bounds.maxX(),
                bounds.maxY(),
                bounds.maxZ());
    }

    public Aabbf toAabbf() {
        return new Aabbf(
                (float) minX,
                (float) minY,
                (float) minZ,
                (float) maxX,
                (float) maxY,
                (float) maxZ);
    }

    public double centerX() {
        return (minX + maxX) * 0.5;
    }

    public double centerY() {
        return (minY + maxY) * 0.5;
    }

    public double centerZ() {
        return (minZ + maxZ) * 0.5;
    }

    public double sizeX() {
        return maxX - minX;
    }

    public double sizeY() {
        return maxY - minY;
    }

    public double sizeZ() {
        return maxZ - minZ;
    }

    public boolean intersects(Aabb other) {
        return Intersection3D.intersects(this, other);
    }

    private static void validateFinite(double value, String name) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(name + " must be finite");
        }
    }
}
