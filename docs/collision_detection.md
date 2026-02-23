# DynamisCollision Core Collision Foundations

## Scope

Implemented primitives and utilities:

- `BroadPhase3D<T>`: broad-phase strategy interface
- `Aabb`: immutable axis-aligned bounding box
- `BoundingSphere`: immutable sphere bounds
- `Capsule`: immutable capsule (segment endpoints + radius)
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
- `MeshCollisionAdapter`: MeshForge `MeshData`/`PackedMesh` bounds/filter adapter hooks
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
- `CollisionShape`: common shape contract for bounds + ray queries
- `PackedMeshCollisionShape`: MeshForge-native collision shape with meshlet-aware coarse raycast
- `RaycastResult`: coarse hit payload (`t`, point, normal, meshlet/triangle indices)

## Implemented Checks

- `Aabb` vs `Aabb`
- `BoundingSphere` vs `BoundingSphere`
- `BoundingSphere` vs `Aabb`
- `Capsule` vs `Capsule`
- `Capsule` vs `BoundingSphere`
- `Capsule` vs `Aabb`
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
    - `Capsule` vs `Capsule`
    - `Capsule` vs `BoundingSphere`
    - `Capsule` vs `Aabb`

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
- `MeshCollisionAdapter` exposes:
    - `bounds(MeshData)` / `bounds(PackedMesh)` -> `Aabb`
    - `setBounds(MeshData, Aabb)` / `setBounds(PackedMesh, Aabb)`
    - `setFilter(...)` and `getFilter(...)` with default fallback

## Collision World Orchestration

- `CollisionWorld3D<T>` performs per-frame:
    - broad-phase candidate generation
    - layer/mask + trigger/solid filtering
    - narrow-phase contact generation
    - manifold cache update/pruning
    - event generation (`ENTER`, `STAY`, `EXIT`)
- Mesh convenience factories:
    - `CollisionWorld3D.forMeshData(...)`
    - `CollisionWorld3D.forMeshDataDefault()`
    - `CollisionWorld3D.forPackedMeshes(...)`
    - `CollisionWorld3D.forPackedMeshesDefault()`
- Mesh shape APIs:
    - `PackedMeshCollisionShape.getWorldBounds(...)`
    - `PackedMeshCollisionShape.raycast(...)` (meshlet AABB + cone coarse pass, triangle pass pending)
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
        - predict positions
        - run collision detection on predicted state
        - solve constraints and collision response

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

## Demo Coverage

This repository includes a `demo` module intended for integration/visualization experiments.
Specific sample class names are not maintained in this document; treat this section as a
high-level note that demo scenarios should cover broad-phase behavior, narrow-phase correctness,
filtering, solver behavior, and ray queries.

## Benchmark Harness

`collision_detection/src/test/java/org/dynamiscollision/broadphase/BroadPhase3DBenchmark.java`
provides a simple runtime comparison for:

- `SpatialHash3D`
- `SweepAndPrune3D`

It is intended for local tuning/regression checks (not as a strict microbenchmark framework).

Additional benchmark-smoke coverage is in:

- `collision_detection/src/test/java/org/dynamiscollision/benchmark/CollisionBenchmarkSmokeTest.java`
  - broad phase at 100/500/1000 objects (fixed seed)
  - GJK random-pair throughput smoke
  - world-step throughput smoke

## Current Shortcomings (Intentional v1)

- No oriented bounding boxes (OBB)
- No triangle-mesh narrow-phase
- No full rigid-body engine (friction models, joint stacks, sleeping/islands, restitution tuning)
- No automatic shape decomposition or mesh-to-convex conversion pipeline
- SAT support is currently 2D and convex-only (concave requires decomposition)
- Convex-convex CCD is currently approximate (sampled + bisection), not full conservative advancement.
- No conservative advancement CCD implementation yet.
- Contact generation is currently primitive-focused; generic GJK/EPA contact point generation is not yet implemented.
- Solver is baseline iterative (position/velocity split with warm-start), not a full production constraint/stacking solver.
- Constraints are currently positional constraints only (no dedicated velocity-level joint constraints).

## Testing

Coverage in this module:

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
- `MeshCollisionAdapterTest`
- `CollisionWorld3DTest`
- `PackedMeshCollisionShapeTest`
- `CollisionDebugSnapshot3DTest`
- `ContactSolver3DTest`
- `PhysicsStep3DTest`
- `Constraints3DTest`
