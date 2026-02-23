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

package org.dynamiscollision.narrowphase;

/**
 * 1D projection interval used by SAT tests.
 */
public record ProjectionInterval(double min, double max) {

    public ProjectionInterval {
        if (!Double.isFinite(min) || !Double.isFinite(max)) {
            throw new IllegalArgumentException("min/max must be finite");
        }
        if (min > max) {
            throw new IllegalArgumentException("min must be <= max");
        }
    }

    public boolean overlaps(ProjectionInterval other) {
        if (other == null) {
            throw new IllegalArgumentException("other must not be null");
        }
        return min <= other.max && max >= other.min;
    }

    /**
     * Returns overlap amount, or a negative value when intervals are separated.
     */
    public double overlapDepth(ProjectionInterval other) {
        if (other == null) {
            throw new IllegalArgumentException("other must not be null");
        }
        return Math.min(max, other.max) - Math.max(min, other.min);
    }
}
