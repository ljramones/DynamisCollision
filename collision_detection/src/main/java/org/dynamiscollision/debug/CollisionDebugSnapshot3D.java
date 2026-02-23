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

package org.dynamiscollision.debug;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.Function;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.contact.ContactPoint3D;
import org.dynamiscollision.events.CollisionEvent;
import org.dynamiscollision.events.CollisionEventType;
import org.dynamiscollision.narrowphase.CollisionManifold3D;
import org.dynamiscollision.pipeline.CollisionPair;

/**
 * Builds a frame snapshot that can be used by debug overlays and diagnostics.
 */
public record CollisionDebugSnapshot3D<T>(
        List<ItemBounds<T>> items,
        List<Contact<T>> contacts) {

    public CollisionDebugSnapshot3D {
        if (items == null || contacts == null) {
            throw new IllegalArgumentException("items and contacts must not be null");
        }
        items = List.copyOf(items);
        contacts = List.copyOf(contacts);
    }

    public static <T> CollisionDebugSnapshot3D<T> from(
            Collection<T> items,
            Function<T, Aabb> boundsProvider,
            Collection<CollisionEvent<T>> events) {
        if (items == null || boundsProvider == null || events == null) {
            throw new IllegalArgumentException("items, boundsProvider and events must not be null");
        }

        List<ItemBounds<T>> itemBounds = new ArrayList<>(items.size());
        for (T item : items) {
            itemBounds.add(new ItemBounds<>(item, boundsProvider.apply(item)));
        }

        List<Contact<T>> contacts = new ArrayList<>();
        for (CollisionEvent<T> event : events) {
            if (event == null || event.manifold() == null) {
                continue;
            }
            for (ContactPoint3D point : event.manifold().contacts()) {
                contacts.add(new Contact<>(
                        event.pair(),
                        event.type(),
                        event.responseEnabled(),
                        event.manifold().manifold(),
                        point));
            }
        }

        return new CollisionDebugSnapshot3D<>(itemBounds, contacts);
    }

    public record ItemBounds<T>(T item, Aabb bounds) {

        public ItemBounds {
            if (item == null || bounds == null) {
                throw new IllegalArgumentException("item and bounds must not be null");
            }
        }
    }

    public record Contact<T>(
            CollisionPair<T> pair,
            CollisionEventType type,
            boolean responseEnabled,
            CollisionManifold3D manifold,
            ContactPoint3D point) {

        public Contact {
            if (pair == null || type == null || manifold == null || point == null) {
                throw new IllegalArgumentException("pair, type, manifold and point must not be null");
            }
        }
    }
}
