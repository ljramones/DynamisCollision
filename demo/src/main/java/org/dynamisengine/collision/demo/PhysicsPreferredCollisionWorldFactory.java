package org.dynamisengine.collision.demo;

import org.dynamisengine.collision.bounds.Aabb;
import org.dynamisengine.collision.broadphase.BroadPhase3D;
import org.dynamisengine.collision.contact.ContactManifold3D;
import org.dynamisengine.collision.filtering.CollisionFilter;
import org.dynamisengine.collision.world.CollisionResponder3D;
import org.dynamisengine.collision.world.CollisionWorld3D;
import org.dynamisengine.physics.api.collision.PhysicsCollisionWorldAssemblies;
import org.dynamisengine.physics.api.collision.PhysicsContactBodyAdapter;
import org.dynamisengine.physics.api.collision.PhysicsSeamSelectionPolicy;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.Function;

/**
 * Demo runtime composition entrypoint that adopts Physics-preferred collision flow defaults.
 *
 * <p>Usage is opt-in for demo/runtime scenarios. Existing non-adopting flows remain unchanged.
 */
public final class PhysicsPreferredCollisionWorldFactory {

    private PhysicsPreferredCollisionWorldFactory() {
    }

    public static <T> CollisionWorld3D<T> create(
            BroadPhase3D<T> broadPhase,
            Function<T, Aabb> boundsProvider,
            Function<T, CollisionFilter> filterProvider,
            BiFunction<T, T, Optional<ContactManifold3D>> narrowPhase,
            PhysicsContactBodyAdapter<T> bodyAdapter,
            CollisionResponder3D<T> fallbackResponder) {
        return PhysicsCollisionWorldAssemblies.createWithPreferredDefaults(
                broadPhase,
                boundsProvider,
                filterProvider,
                narrowPhase,
                bodyAdapter,
                fallbackResponder);
    }

    public static <T> CollisionWorld3D<T> create(
            BroadPhase3D<T> broadPhase,
            Function<T, Aabb> boundsProvider,
            Function<T, CollisionFilter> filterProvider,
            BiFunction<T, T, Optional<ContactManifold3D>> narrowPhase,
            PhysicsContactBodyAdapter<T> bodyAdapter,
            CollisionResponder3D<T> fallbackResponder,
            PhysicsSeamSelectionPolicy<T> seamSelectionPolicy) {
        return PhysicsCollisionWorldAssemblies.createWithPreferredDefaults(
                broadPhase,
                boundsProvider,
                filterProvider,
                narrowPhase,
                bodyAdapter,
                fallbackResponder,
                seamSelectionPolicy);
    }
}
