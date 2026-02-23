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

package org.dynamiscollision.broadphase;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.pipeline.CollisionPair;

/**
 * 3D sweep-and-prune broad phase using X-axis sweep and Y/Z interval checks.
 */
public final class SweepAndPrune3D<T> implements BroadPhase3D<T> {

    @Override
    public Set<CollisionPair<T>> findPotentialPairs(Collection<T> items, Function<T, Aabb> aabbProvider) {
        if (items == null || aabbProvider == null) {
            throw new IllegalArgumentException("items and aabbProvider must not be null");
        }

        List<ItemBounds<T>> sorted = new ArrayList<>(items.size());
        for (T item : items) {
            if (item == null) {
                continue;
            }
            Aabb aabb = aabbProvider.apply(item);
            if (aabb == null) {
                continue;
            }
            sorted.add(new ItemBounds<>(item, aabb));
        }
        sorted.sort(Comparator.comparingDouble(entry -> entry.bounds().minX()));

        Set<CollisionPair<T>> pairs = new LinkedHashSet<>();
        List<ItemBounds<T>> active = new ArrayList<>();

        for (ItemBounds<T> current : sorted) {
            double currentMinX = current.bounds().minX();
            active.removeIf(candidate -> candidate.bounds().maxX() < currentMinX);

            for (ItemBounds<T> candidate : active) {
                if (overlapsYZ(candidate.bounds(), current.bounds())) {
                    pairs.add(new CollisionPair<>(candidate.item(), current.item()));
                }
            }
            active.add(current);
        }
        return pairs;
    }

    private boolean overlapsYZ(Aabb left, Aabb right) {
        return left.minY() <= right.maxY() && left.maxY() >= right.minY()
                && left.minZ() <= right.maxZ() && left.maxZ() >= right.minZ();
    }

    private record ItemBounds<T>(T item, Aabb bounds) {
    }
}
