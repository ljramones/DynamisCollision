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

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;

class CollisionFilteringTest {

    @Test
    void filtersByLayerMaskAndClassifiesResponse() {
        CollisionPair<String> solidPair = new CollisionPair<>("ship", "asteroid");
        CollisionPair<String> triggerPair = new CollisionPair<>("ship", "sensor");
        CollisionPair<String> ignoredPair = new CollisionPair<>("ship", "ui");

        Map<String, CollisionFilter> filters = Map.of(
                "ship", new CollisionFilter(0b0001, 0b0110, CollisionKind.SOLID),
                "asteroid", new CollisionFilter(0b0010, 0b1111, CollisionKind.SOLID),
                "sensor", new CollisionFilter(0b0100, 0b1111, CollisionKind.TRIGGER),
                "ui", new CollisionFilter(0b1000, 0b1000, CollisionKind.SOLID));

        Set<FilteredCollisionPair<String>> result = CollisionFiltering.filterPairs(
                Set.of(solidPair, triggerPair, ignoredPair),
                filters::get);

        assertEquals(2, result.size());
        assertTrue(result.contains(new FilteredCollisionPair<>(solidPair, true)));
        assertTrue(result.contains(new FilteredCollisionPair<>(triggerPair, false)));
        assertFalse(result.contains(new FilteredCollisionPair<>(ignoredPair, false)));
    }

    @Test
    void defaultsFilterWhenProviderReturnsNull() {
        CollisionPair<String> pair = new CollisionPair<>("a", "b");
        Set<FilteredCollisionPair<String>> result = CollisionFiltering.filterPairs(
                Set.of(pair), item -> null);
        assertEquals(1, result.size());
        assertTrue(result.iterator().next().responseEnabled());
    }
}
