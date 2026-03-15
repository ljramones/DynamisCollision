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

package org.dynamisengine.collision.world;

import java.util.List;
import org.dynamisengine.collision.events.CollisionEvent;

/**
 * Policy gate for collision response path selection in {@link CollisionWorld3D}.
 *
 * <p>This allows explicit preference of non-legacy responder handling for representative flows,
 * while preserving legacy ContactSolver3D special-path compatibility by default.
 */
@FunctionalInterface
public interface CollisionResponsePathPolicy3D<T> {

    boolean useLegacyContactSolverPath(CollisionResponder3D<T> responder, List<CollisionEvent<T>> responseEvents);
}
