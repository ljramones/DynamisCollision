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

package org.dynamiscollision.bounds;

import org.dynamiscollision.narrowphase.Intersection3D;

/**
 * Immutable capsule in 3D defined by segment endpoints and radius.
 */
public record Capsule(
        double pointAX,
        double pointAY,
        double pointAZ,
        double pointBX,
        double pointBY,
        double pointBZ,
        double radius) {

    public Capsule {
        validateFinite(pointAX, "pointAX");
        validateFinite(pointAY, "pointAY");
        validateFinite(pointAZ, "pointAZ");
        validateFinite(pointBX, "pointBX");
        validateFinite(pointBY, "pointBY");
        validateFinite(pointBZ, "pointBZ");
        validateFinite(radius, "radius");
        if (radius < 0.0) {
            throw new IllegalArgumentException("radius must be >= 0");
        }
    }

    public boolean intersects(Capsule other) {
        return Intersection3D.intersects(this, other);
    }

    public boolean intersects(BoundingSphere other) {
        return Intersection3D.intersects(this, other);
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
