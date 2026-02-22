# DynamisCollision Core Collision Foundations

## Scope

Implemented primitives and utilities:

- `BroadPhase3D<T>`: broad-phase strategy interface
- `Aabb`: immutable axis-aligned bounding box
- `BoundingSphere`: immutable sphere bounds
- `Ray3D`: immutable ray (origin + direction)
- `Intersection3D`: primitive intersection checks
- `SpatialHash3D<T>`: broad-phase candidate pair generation
- `SweepAndPrune3D<T>`: broad-phase candidate generation via axis sweep
- `CollisionPair<T>`: unordered candidate pair container
- `ConvexPolygon2D`: immutable convex polygon (SAT input)
- `ProjectionInterval`: 1D projected interval for SAT
- `Sat2D`: SAT narrow-phase checks (`intersects`, manifold output)
- `CollisionManifold2D`: normal + penetration depth
- `ConvexSupport3D`: support mapping interface for convex 3D shapes
- `Gjk3D`: GJK narrow-phase for convex 3D shapes, with EPA manifold extraction
- `CollisionManifold3D`: normal + penetration depth
- `ContactPoint3D`: individual contact location
- `ContactManifold3D`: manifold + contact point list
- `ContactGenerator3D`: primitive contact generation
- `ManifoldCache3D<T>`: frame-based manifold persistence
- `WarmStartImpulse`: cached solver impulse state
- `CollisionKind`: `SOLID` vs `TRIGGER`
- `CollisionFilter`: layer/mask + behavior filter
- `CollisionFiltering`: filter + classify candidate pairs
- `FilteredCollisionPair<T>`: pair annotated with response flag
- `NodeCollisionAdapter`: JavaFX `Node` bounds/filter adapter hooks
- `CollisionEventType`: `ENTER`, `STAY`, `EXIT`
- `CollisionEvent<T>`: per-frame event payload
- `CollisionWorld3D<T>`: orchestration loop for broad-phase/filter/narrow-phase/cache/events
- `CollisionDebugSnapshot3D<T>`: per-frame debug data extraction (bounds + contacts)
- `Constraint3D<T>`: constraint interface
- `DistanceConstraint3D<T>`: pair distance constraint
- `PointConstraint3D<T>`: body-to-anchor constraint
- `PhysicsStep3D`: fixed timestep accumulator helper
- `RigidBodyAdapter3D<T>`: pluggable body state adapter
- `CollisionResponder3D<T>`: response hook interface
- `ContactSolver3D<T>`: basic positional + impulse response solver
- `Ccd3D`: continuous collision detection helpers (time-of-impact)
- `CollisionPipeline`: applies narrow-phase tests to broad-phase candidates

## Implemented Checks

- `Aabb` vs `Aabb`
- `BoundingSphere` vs `BoundingSphere`
- `BoundingSphere` vs `Aabb`
- `Ray3D` vs `Aabb` with hit distance (`OptionalDouble`)
- `ConvexPolygon2D` vs `ConvexPolygon2D` via SAT (2D)

## Broad-Phase Behavior

`SpatialHash3D` uses a uniform grid (`cellSize`) and returns potential pairs that share at least one occupied cell.

`SweepAndPrune3D` sorts by X-axis interval start and checks active overlaps in Y/Z for candidate generation.

Important: broad-phase results are candidate pairs only. Final collision validation should use narrow-phase checks from `Intersection3D`.

Use `CollisionPipeline` to filter broad-phase candidate pairs with narrow-phase checks.

## Narrow-Phase Behavior

- SAT (`Sat2D`) supports 2D convex polygons and can return MTV-like manifold data.
- GJK (`Gjk3D`) supports convex 3D shapes through support mappings (`ConvexSupport3D`).
- EPA (`Gjk3D.intersectsWithManifold`) provides manifold normal and penetration depth for intersecting convex pairs.
- `ContactGenerator3D` currently provides explicit contact points for:
    - `Aabb` vs `Aabb`
    - `BoundingSphere` vs `BoundingSphere`

## CCD Behavior

- `Ccd3D.segmentAabbTimeOfImpact(start, end, box)` returns first `t` in `[0,1]` for segment-vs-AABB.
- `Ccd3D.sweptAabbTimeOfImpact(moving, delta, target)` returns first `t` in `[0,1]` for moving-AABB-vs-static-AABB.
- `Ccd3D.sweptConvexTimeOfImpact(shapeA, deltaA, shapeB, deltaB)` returns approximate first `t` in `[0,1]` for moving convex shapes using sampled bracketing + binary refinement over GJK.

## Manifold Persistence

- `ManifoldCache3D<T>` stores contact manifolds by `CollisionPair<T>`.
- Entries can be retained across frames and pruned by age using `pruneStale(...)`.
- Warm-start state can be stored/retrieved per pair:
    - `getWarmStart(...)`
    - `setWarmStart(...)`

## Filtering & Scene Integration Hooks

- `CollisionFilter` provides layer/mask checks and `SOLID`/`TRIGGER` behavior.
- `CollisionFiltering.filterPairs(...)` filters broad-phase pairs and annotates response eligibility.
- `NodeCollisionAdapter` exposes:
    - `boundsInParent(Node)` -> `Aabb`
    - `setFilter(Node, CollisionFilter)`
    - `getFilter(Node)` with defaults/property fallback

## Collision World Orchestration

- `CollisionWorld3D<T>` performs per-frame:
    - broad-phase candidate generation
    - layer/mask + trigger/solid filtering
    - narrow-phase contact generation
    - manifold cache update/pruning
    - event generation (`ENTER`, `STAY`, `EXIT`)
- JavaFX convenience factories:
    - `CollisionWorld3D.forJavaFxNodes(...)`
    - `CollisionWorld3D.forJavaFxNodesDefault()`
- Optional response hook:
    - `CollisionWorld3D.setResponder(CollisionResponder3D<T>)`
    - `ContactSolver3D<T>` can be used as the responder.
- Iteration control:
    - `CollisionWorld3D.setSolverIterations(int)`
    - `CollisionWorld3D.setConstraintIterations(int)`
    - with `ContactSolver3D`, world executes split passes:
        - position correction pass (iterative)
        - velocity impulse pass (iterative with first-pass warm start from cache)
- Physics stepping API:
    - `CollisionWorld3D.step(items, dtSeconds)`:
        - integrate velocities (gravity)
        - solve constraints
        - run collision detection/response
        - integrate positions

## Debugging and Visualization

- `CollisionDebugSnapshot3D.from(items, boundsProvider, events)` builds a lightweight frame snapshot:
    - per-item bounds
    - per-contact points with event type and manifold metadata
- This is intended for debug overlays, instrumentation, and diagnostics tools.
- `DynamisFX-Demo` includes `org.dynamisfx.samples.utilities.CollisionDebugWorld`:
    - moving-body collision world demo
    - wireframe AABB overlays
    - contact-point markers (`ENTER`/`STAY`/`EXIT`)
    - controls:
        - `D` toggle debug overlay
        - `SPACE` pause/resume simulation

## Demo Coverage (DynamisFX-Demo)

Sampler divisions in `DynamisFX-Client` route collision demos into:

- `Collision Detection`
- `Collision Detection Solver`
- `Pipeline Physics`

Implemented demos currently include:

- `BroadPhaseComparisonDemo` (spatial hash vs sweep-and-prune, stats overlays)
- `GjkEpaVisualizerDemo` (GJK simplex and EPA penetration visualization)
- `Sat2dPolygonPlaygroundDemo` (2D SAT axes and MTV behavior)
- `CcdTunnelingDemo` (discrete vs swept vs CCD TOI behavior)
- `ContactSolverStackingDemo` (solver iterations, warm start, friction/baumgarte tuning)
- `RayCastingSceneDemo` (single ray/flashlight, brute force vs broad-phase culling)
- `CollisionFilterLayerDemo` (layer matrix filtering with pair counters)
- `BoundingVolumeComparisonDemo` (AABB vs sphere vs convex/GJK comparison)
- `CollisionPipelineMonitorDemo` (broad/filter/narrow/manifold flow monitoring)
- `MixedPrimitiveStressTestDemo` (AABB/AABB, sphere/sphere, sphere/AABB cost mix)
- `ManifoldPersistenceVisualizerDemo` (contact age and warm-start persistence)

## Benchmark Harness

`DynamisFX-Core/src/test/java/org/dynamisfx/collision/broadphase/BroadPhase3DBenchmark.java` provides a simple runtime comparison for:

- `SpatialHash3D`
- `SweepAndPrune3D`

It is intended for local tuning/regression checks (not as a strict microbenchmark framework).

## Current Shortcomings (Intentional v1)

- No oriented bounding boxes (OBB)
- No triangle-mesh narrow-phase
- No full rigid-body engine (friction models, joint stacks, sleeping/islands, restitution tuning)
- No automatic shape decomposition or mesh-to-convex conversion pipeline
- SAT support is currently 2D and convex-only (concave requires decomposition)
- Convex-convex CCD is currently approximate (sampled + bisection), not full conservative advancement.
- Contact generation is currently primitive-focused; generic GJK/EPA contact point generation is not yet implemented.
- Solver is baseline iterative (position/velocity split with warm-start), not a full production constraint/stacking solver.
- Constraints are currently positional constraints only (no dedicated velocity-level joint constraints).

## Testing

Coverage added in `DynamisFX-Core`:

- `AabbAndSphereValidationTest`
- `Intersection3DTest`
- `SpatialHash3DTest`
- `Sat2DTest`
- `SweepAndPrune3DTest`
- `Gjk3DTest`
- `Ccd3DTest`
- `CollisionPipelineTest`
- `BroadPhase3DBenchmark` (manual benchmark harness)
- `ContactGenerator3DTest`
- `ManifoldCache3DTest`
- `CollisionFilteringTest`
- `NodeCollisionAdapterTest`
- `CollisionWorld3DTest`
- `CollisionDebugSnapshot3DTest`
- `ContactSolver3DTest`
- `PhysicsStep3DTest`
- `Constraints3DTest`
