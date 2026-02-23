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

import java.util.Objects;

/**
 * Unordered candidate pair of colliders.
 */
public final class CollisionPair<T> {

    private final T first;
    private final T second;

    public CollisionPair(T first, T second) {
        if (first == null || second == null) {
            throw new IllegalArgumentException("pair values must not be null");
        }
        if (first == second) {
            throw new IllegalArgumentException("pair values must be different");
        }
        this.first = first;
        this.second = second;
    }

    public T first() {
        return first;
    }

    public T second() {
        return second;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof CollisionPair<?> other)) {
            return false;
        }
        return (Objects.equals(first, other.first) && Objects.equals(second, other.second))
                || (Objects.equals(first, other.second) && Objects.equals(second, other.first));
    }

    @Override
    public int hashCode() {
        return Objects.hashCode(first) ^ Objects.hashCode(second);
    }

    @Override
    public String toString() {
        return "CollisionPair[" + first + ", " + second + "]";
    }
}
