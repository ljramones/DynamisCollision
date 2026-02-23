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

package org.dynamiscollision.pipeline;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.BiPredicate;

/**
 * Utility for applying narrow-phase checks to broad-phase candidate pairs.
 */
public final class CollisionPipeline {

    private CollisionPipeline() {
    }

    public static <T> Set<CollisionPair<T>> findCollisions(
            Set<CollisionPair<T>> candidates,
            BiPredicate<T, T> narrowPhase) {
        if (candidates == null || narrowPhase == null) {
            throw new IllegalArgumentException("candidates and narrowPhase must not be null");
        }
        Set<CollisionPair<T>> collisions = new LinkedHashSet<>();
        for (CollisionPair<T> pair : candidates) {
            if (pair == null) {
                continue;
            }
            if (narrowPhase.test(pair.first(), pair.second())) {
                collisions.add(pair);
            }
        }
        return collisions;
    }
}
