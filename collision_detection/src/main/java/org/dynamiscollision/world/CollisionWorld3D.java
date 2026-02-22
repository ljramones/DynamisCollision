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

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BiFunction;
import java.util.function.Function;
import org.meshforge.core.mesh.MeshData;
import org.meshforge.pack.buffer.PackedMesh;
import org.vectrix.core.Vector3d;

/**
 * Runtime orchestration for broad phase, filtering, narrow phase and event generation.
 */
public final class CollisionWorld3D<T> {

    private final BroadPhase3D<T> broadPhase;
    private final Function<T, Aabb> boundsProvider;
    private final Function<T, CollisionFilter> filterProvider;
    private final BiFunction<T, T, Optional<ContactManifold3D>> narrowPhase;
    private final ManifoldCache3D<T> manifoldCache = new ManifoldCache3D<>();
    private final List<Constraint3D<T>> constraints = new CopyOnWriteArrayList<>();

    private final Map<CollisionPair<T>, FrameCollision> previousFrame = new HashMap<>();
    private long manifoldRetentionFrames = 2;
    private int solverIterations = 1;
    private int constraintIterations = 1;
    private CollisionResponder3D<T> responder;
    private RigidBodyAdapter3D<T> bodyAdapter;
    private Vector3d gravity = new Vector3d(0.0, 0.0, 0.0);

    public CollisionWorld3D(
            BroadPhase3D<T> broadPhase,
            Function<T, Aabb> boundsProvider,
            Function<T, CollisionFilter> filterProvider,
            BiFunction<T, T, Optional<ContactManifold3D>> narrowPhase) {
        if (broadPhase == null || boundsProvider == null || filterProvider == null || narrowPhase == null) {
            throw new IllegalArgumentException("constructor arguments must not be null");
        }
        this.broadPhase = broadPhase;
        this.boundsProvider = boundsProvider;
        this.filterProvider = filterProvider;
        this.narrowPhase = narrowPhase;
    }

    public static CollisionWorld3D<MeshData> forMeshData(BroadPhase3D<MeshData> broadPhase) {
        return new CollisionWorld3D<>(
                broadPhase,
                MeshCollisionAdapter::bounds,
                MeshCollisionAdapter::getFilter,
                (left, right) -> ContactGenerator3D.generate(
                        MeshCollisionAdapter.bounds(left),
                        MeshCollisionAdapter.bounds(right)));
    }

    public static CollisionWorld3D<MeshData> forMeshDataDefault() {
        return forMeshData(new SweepAndPrune3D<>());
    }

    public static CollisionWorld3D<PackedMesh> forPackedMeshes(BroadPhase3D<PackedMesh> broadPhase) {
        return new CollisionWorld3D<>(
                broadPhase,
                MeshCollisionAdapter::bounds,
                MeshCollisionAdapter::getFilter,
                (left, right) -> ContactGenerator3D.generate(
                        MeshCollisionAdapter.bounds(left),
                        MeshCollisionAdapter.bounds(right)));
    }

    public static CollisionWorld3D<PackedMesh> forPackedMeshesDefault() {
        return forPackedMeshes(new SweepAndPrune3D<>());
    }

    public void setManifoldRetentionFrames(long manifoldRetentionFrames) {
        if (manifoldRetentionFrames < 0) {
            throw new IllegalArgumentException("manifoldRetentionFrames must be >= 0");
        }
        this.manifoldRetentionFrames = manifoldRetentionFrames;
    }

    public ManifoldCache3D<T> manifoldCache() {
        return manifoldCache;
    }

    public void setResponder(CollisionResponder3D<T> responder) {
        this.responder = responder;
    }

    public void setSolverIterations(int solverIterations) {
        if (solverIterations < 1) {
            throw new IllegalArgumentException("solverIterations must be >= 1");
        }
        this.solverIterations = solverIterations;
    }

    public void setConstraintIterations(int constraintIterations) {
        if (constraintIterations < 1) {
            throw new IllegalArgumentException("constraintIterations must be >= 1");
        }
        this.constraintIterations = constraintIterations;
    }

    public void setBodyAdapter(RigidBodyAdapter3D<T> bodyAdapter) {
        this.bodyAdapter = bodyAdapter;
    }

    public void setGravity(Vector3d gravity) {
        if (gravity == null) {
            throw new IllegalArgumentException("gravity must not be null");
        }
        this.gravity = gravity;
    }

    public void addConstraint(Constraint3D<T> constraint) {
        if (constraint == null) {
            throw new IllegalArgumentException("constraint must not be null");
        }
        constraints.add(constraint);
    }

    public void clearConstraints() {
        constraints.clear();
    }

    public List<Constraint3D<T>> constraints() {
        return List.copyOf(constraints);
    }

    public List<CollisionEvent<T>> update(Collection<T> items) {
        if (items == null) {
            throw new IllegalArgumentException("items must not be null");
        }

        manifoldCache.nextFrame();

        Set<CollisionPair<T>> candidates = broadPhase.findPotentialPairs(items, boundsProvider);
        Set<FilteredCollisionPair<T>> filteredPairs = CollisionFiltering.filterPairs(candidates, filterProvider);

        Map<CollisionPair<T>, FrameCollision> currentFrame = new HashMap<>();
        for (FilteredCollisionPair<T> filtered : filteredPairs) {
            CollisionPair<T> pair = filtered.pair();
            Optional<ContactManifold3D> contact = narrowPhase.apply(pair.first(), pair.second());
            if (contact.isEmpty()) {
                continue;
            }
            ContactManifold3D manifold = contact.get();
            currentFrame.put(pair, new FrameCollision(filtered.responseEnabled(), manifold));
            manifoldCache.put(pair, manifold);
        }

        List<CollisionEvent<T>> events = new ArrayList<>();

        List<CollisionEvent<T>> responseEvents = new ArrayList<>();
        for (Map.Entry<CollisionPair<T>, FrameCollision> entry : currentFrame.entrySet()) {
            CollisionPair<T> pair = entry.getKey();
            FrameCollision current = entry.getValue();
            CollisionEventType type = previousFrame.containsKey(pair) ? CollisionEventType.STAY : CollisionEventType.ENTER;
            CollisionEvent<T> event = new CollisionEvent<>(pair, type, current.responseEnabled(), current.manifold());
            events.add(event);
            if (event.responseEnabled()) {
                responseEvents.add(event);
            }
        }

        for (Map.Entry<CollisionPair<T>, FrameCollision> entry : previousFrame.entrySet()) {
            if (!currentFrame.containsKey(entry.getKey())) {
                FrameCollision prior = entry.getValue();
                events.add(new CollisionEvent<>(
                        entry.getKey(),
                        CollisionEventType.EXIT,
                        prior.responseEnabled(),
                        prior.manifold()));
            }
        }

        previousFrame.clear();
        previousFrame.putAll(currentFrame);
        manifoldCache.pruneStale(manifoldRetentionFrames);

        applyResponses(responseEvents);

        return events;
    }

    public List<CollisionEvent<T>> step(Collection<T> items, double dtSeconds) {
        if (items == null) {
            throw new IllegalArgumentException("items must not be null");
        }
        if (!Double.isFinite(dtSeconds) || dtSeconds <= 0.0) {
            throw new IllegalArgumentException("dtSeconds must be > 0");
        }
        if (bodyAdapter == null) {
            throw new IllegalStateException("bodyAdapter must be set to use step()");
        }

        // 1) integrate velocities (external acceleration only)
        for (T body : items) {
            double invMass = Math.max(0.0, bodyAdapter.getInverseMass(body));
            if (invMass <= 0.0) {
                continue;
            }
            Vector3d v = bodyAdapter.getVelocity(body);
            bodyAdapter.setVelocity(body, new Vector3d(
                    v.x() + gravity.x() * dtSeconds,
                    v.y() + gravity.y() * dtSeconds,
                    v.z() + gravity.z() * dtSeconds));
        }

        // 2) solve constraints
        for (int i = 0; i < constraintIterations; i++) {
            for (Constraint3D<T> constraint : constraints) {
                constraint.solve(bodyAdapter, dtSeconds);
            }
        }

        // 3) solve collisions at current predicted state
        List<CollisionEvent<T>> events = update(items);

        // 4) integrate positions
        for (T body : items) {
            double invMass = Math.max(0.0, bodyAdapter.getInverseMass(body));
            if (invMass <= 0.0) {
                continue;
            }
            Vector3d p = bodyAdapter.getPosition(body);
            Vector3d v = bodyAdapter.getVelocity(body);
            bodyAdapter.setPosition(body, new Vector3d(
                    p.x() + v.x() * dtSeconds,
                    p.y() + v.y() * dtSeconds,
                    p.z() + v.z() * dtSeconds));
        }
        return events;
    }

    @SuppressWarnings("unchecked")
    private void applyResponses(List<CollisionEvent<T>> responseEvents) {
        if (responder == null || responseEvents.isEmpty()) {
            return;
        }

        responseEvents.sort(Comparator.comparing(event ->
                String.valueOf(event.pair().first()) + "|" + String.valueOf(event.pair().second())));

        if (responder instanceof ContactSolver3D<?> anySolver) {
            ContactSolver3D<T> solver = (ContactSolver3D<T>) anySolver;
            for (int i = 0; i < solverIterations; i++) {
                for (CollisionEvent<T> event : responseEvents) {
                    solver.solvePosition(event);
                }
            }
            for (int i = 0; i < solverIterations; i++) {
                for (CollisionEvent<T> event : responseEvents) {
                    WarmStartImpulse warmStart = i == 0
                            ? manifoldCache.getWarmStart(event.pair()).orElse(WarmStartImpulse.ZERO)
                            : WarmStartImpulse.ZERO;
                    WarmStartImpulse solved = solver.solveVelocity(event, warmStart);
                    if (i == solverIterations - 1) {
                        manifoldCache.setWarmStart(event.pair(), solved);
                    }
                }
            }
            return;
        }

        for (CollisionEvent<T> event : responseEvents) {
            responder.resolve(event);
        }
    }

    private record FrameCollision(boolean responseEnabled, ContactManifold3D manifold) {
    }
}
