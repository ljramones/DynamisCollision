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

package org.dynamiscollision.filtering;

import java.util.LinkedHashSet;
import java.util.Set;
import java.util.function.Function;
import org.dynamiscollision.pipeline.CollisionPair;
import org.dynamiscollision.pipeline.FilteredCollisionPair;

/**
 * Filter utilities for broad-phase candidate pairs.
 */
public final class CollisionFiltering {

    private CollisionFiltering() {
    }

    public static <T> Set<FilteredCollisionPair<T>> filterPairs(
            Set<CollisionPair<T>> candidates,
            Function<T, CollisionFilter> filterProvider) {
        if (candidates == null || filterProvider == null) {
            throw new IllegalArgumentException("candidates and filterProvider must not be null");
        }
        Set<FilteredCollisionPair<T>> result = new LinkedHashSet<>();
        for (CollisionPair<T> pair : candidates) {
            if (pair == null) {
                continue;
            }
            CollisionFilter left = safeFilter(filterProvider.apply(pair.first()));
            CollisionFilter right = safeFilter(filterProvider.apply(pair.second()));
            if (left.canInteract(right)) {
                result.add(new FilteredCollisionPair<>(pair, left.responseEnabledWith(right)));
            }
        }
        return result;
    }

    private static CollisionFilter safeFilter(CollisionFilter filter) {
        return filter == null ? CollisionFilter.DEFAULT : filter;
    }
}
