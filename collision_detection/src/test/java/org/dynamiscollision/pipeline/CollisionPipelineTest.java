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

import static org.junit.jupiter.api.Assertions.*;

import java.util.LinkedHashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

class CollisionPipelineTest {

    @Test
    void filtersCandidatesWithNarrowPhase() {
        CollisionPair<String> ab = new CollisionPair<>("a", "b");
        CollisionPair<String> ac = new CollisionPair<>("a", "c");

        Set<CollisionPair<String>> collisions = CollisionPipeline.findCollisions(
                Set.of(ab, ac),
                (left, right) -> ("a".equals(left) && "b".equals(right))
                        || ("b".equals(left) && "a".equals(right)));

        assertEquals(1, collisions.size());
        assertTrue(collisions.contains(ab));
    }

    @Test
    void ignoresNullCandidateEntries() {
        CollisionPair<String> ab = new CollisionPair<>("a", "b");
        Set<CollisionPair<String>> candidates = new LinkedHashSet<>();
        candidates.add(ab);
        candidates.add(null);
        Set<CollisionPair<String>> collisions = CollisionPipeline.findCollisions(
                candidates,
                (left, right) -> true);
        assertEquals(1, collisions.size());
        assertTrue(collisions.contains(ab));
    }

    @Test
    void validatesArguments() {
        assertTrue(assertThrows(IllegalArgumentException.class,
                () -> CollisionPipeline.findCollisions(null, (l, r) -> true))
                .getMessage().contains("must not be null"));
        assertTrue(assertThrows(IllegalArgumentException.class,
                () -> CollisionPipeline.findCollisions(Set.of(), null))
                .getMessage().contains("must not be null"));
    }

    @Test
    void handlesEmptyAndSingleCandidateSets() {
        Set<CollisionPair<String>> empty = CollisionPipeline.findCollisions(Set.of(), (l, r) -> true);
        assertTrue(empty.isEmpty());

        CollisionPair<String> only = new CollisionPair<>("x", "y");
        Set<CollisionPair<String>> single = CollisionPipeline.findCollisions(Set.of(only), (l, r) -> true);
        assertEquals(Set.of(only), single);
    }

    @Test
    void returnsEmptyWhenAllCandidatesFailNarrowPhase() {
        CollisionPair<String> ab = new CollisionPair<>("a", "b");
        CollisionPair<String> ac = new CollisionPair<>("a", "c");
        Set<CollisionPair<String>> collisions = CollisionPipeline.findCollisions(Set.of(ab, ac), (l, r) -> false);
        assertTrue(collisions.isEmpty());
    }
}
