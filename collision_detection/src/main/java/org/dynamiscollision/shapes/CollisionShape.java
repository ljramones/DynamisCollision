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

package org.dynamiscollision.shapes;

import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;

/**
 * Generic collidable shape contract for world-space bounds and coarse ray tests.
 */
public interface CollisionShape {

    /**
     * Returns world-space bounds for this shape using the provided world transform.
     */
    Aabb getWorldBounds(Transformf worldTransform);

    /**
     * Coarse ray query for this shape.
     */
    java.util.Optional<RaycastResult> raycast(Ray3D ray, Transformf worldTransform);
}
