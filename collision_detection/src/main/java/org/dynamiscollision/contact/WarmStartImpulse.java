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
 * Cached scalar impulses used for warm starting iterative solvers.
 */
public record WarmStartImpulse(double normalImpulse, double tangentImpulse) {

    public static final WarmStartImpulse ZERO = new WarmStartImpulse(0.0, 0.0);

    public WarmStartImpulse {
        if (!Double.isFinite(normalImpulse) || !Double.isFinite(tangentImpulse)) {
            throw new IllegalArgumentException("impulses must be finite");
        }
    }
}
