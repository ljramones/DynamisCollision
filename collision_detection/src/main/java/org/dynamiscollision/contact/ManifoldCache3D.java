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

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

/**
 * Frame-based manifold persistence cache keyed by collision pair.
 */
public final class ManifoldCache3D<T> {

    private final Map<CollisionPair<T>, CachedManifold> cache = new HashMap<>();
    private long frameIndex;

    public void nextFrame() {
        frameIndex++;
    }

    public void put(CollisionPair<T> pair, ContactManifold3D manifold) {
        if (pair == null || manifold == null) {
            throw new IllegalArgumentException("pair and manifold must not be null");
        }
        CachedManifold prior = cache.get(pair);
        WarmStartImpulse warmStart = prior == null ? WarmStartImpulse.ZERO : prior.warmStart();
        cache.put(pair, new CachedManifold(manifold, frameIndex, warmStart));
    }

    public Optional<ContactManifold3D> get(CollisionPair<T> pair) {
        if (pair == null) {
            throw new IllegalArgumentException("pair must not be null");
        }
        CachedManifold cached = cache.get(pair);
        return cached == null ? Optional.empty() : Optional.of(cached.manifold());
    }

    public Optional<WarmStartImpulse> getWarmStart(CollisionPair<T> pair) {
        if (pair == null) {
            throw new IllegalArgumentException("pair must not be null");
        }
        CachedManifold cached = cache.get(pair);
        return cached == null ? Optional.empty() : Optional.of(cached.warmStart());
    }

    public void setWarmStart(CollisionPair<T> pair, WarmStartImpulse warmStart) {
        if (pair == null || warmStart == null) {
            throw new IllegalArgumentException("pair and warmStart must not be null");
        }
        CachedManifold cached = cache.get(pair);
        if (cached == null) {
            return;
        }
        cache.put(pair, new CachedManifold(cached.manifold(), cached.lastUpdatedFrame(), warmStart));
    }

    public void pruneStale(long maxFramesWithoutUpdate) {
        if (maxFramesWithoutUpdate < 0) {
            throw new IllegalArgumentException("maxFramesWithoutUpdate must be >= 0");
        }
        cache.entrySet().removeIf(entry -> (frameIndex - entry.getValue().lastUpdatedFrame()) > maxFramesWithoutUpdate);
    }

    public int size() {
        return cache.size();
    }

    private record CachedManifold(ContactManifold3D manifold, long lastUpdatedFrame, WarmStartImpulse warmStart) {
    }
}
