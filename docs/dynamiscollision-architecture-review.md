# DynamisCollision Architecture Review

## Repo Overview

- Repository: `DynamisCollision`
- Modules:
  - `collision_detection` (core library)
  - `demo` (examples/experiments)
- Language/build: Maven multi-module, Java 25 (`pom.xml`)
- Module export surface (`collision_detection/src/main/java/module-info.java`) currently exports:
  - substrate-like packages: `bounds`, `shapes`, `geometry`, `broadphase`, `narrowphase`, `filtering`, `pipeline`
  - simulation-leaning packages: `world`, `contact`, `constraints`, `events`, `debug`, `adapters`

Grounded observation: this repo currently contains both collision-query substrate capabilities and physics-style simulation/runtime behavior.

## Strict Ownership Statement

### What DynamisCollision should own

DynamisCollision should own **collision/query substrate authority**:

- Shape descriptors and geometric bounds primitives.
- Broadphase and narrowphase query mechanics.
- Query-focused intersection/raycast/sweep/overlap APIs.
- Collision filtering/mask/layer classification primitives.
- Collision acceleration/query data structures and pair-generation pipeline.
- Mesh-oriented collision query adapters at the boundary (consuming prepared geometry metadata, not shaping it).

This aligns best with packages like:

- `org.dynamisengine.collision.bounds`
- `org.dynamisengine.collision.shapes`
- `org.dynamisengine.collision.geometry`
- `org.dynamisengine.collision.broadphase`
- `org.dynamisengine.collision.narrowphase`
- `org.dynamisengine.collision.filtering`
- `org.dynamisengine.collision.pipeline`

### What DynamisCollision must not own

DynamisCollision should **not** own:

- Simulation stepping/solver authority.
- Contact lifecycle authority as part of runtime simulation orchestration.
- Constraint solving authority.
- World-level orchestration/tick authority.
- Gameplay/system policy.
- SceneGraph hierarchy ownership.
- ECS/world/session authority.
- Render/GPU policy.
- Geometry preparation authority (MeshForge/AssetPipeline concerns).

## Dependency Rules

### Allowed dependencies for DynamisCollision

- Foundational math/substrate libraries (e.g., `vectrix`).
- Prepared geometry read-only consumption contracts when needed for query adapters (e.g., MeshForge mesh data access), without taking ownership of prep/bake policy.
- DynamisCore-level minimal abstractions if introduced later.

### Forbidden dependencies for DynamisCollision

- Dependence on WorldEngine authority/orchestration APIs.
- Dependence on Physics simulation authority APIs for core substrate behavior.
- Dependence on LightEngine render-planning/policy APIs.
- Dependence on Session/Content/Scripting policy layers.

### Who may depend on DynamisCollision

- DynamisPhysics (as query substrate provider).
- DynamisWorldEngine (through Physics or explicit query façade).
- Debug/feature consumers that need collision queries (through stable query contracts).

## Public vs Internal Boundary Assessment

### Canonical public boundary (recommended)

Public surface should prioritize substrate contracts and data types:

- bounds/shapes/geometry primitives
- broadphase/narrowphase/query APIs
- filtering and pair/query pipeline contracts

### Currently exposed but likely too broad

The module currently exports simulation-oriented packages that are likely beyond strict collision substrate scope:

- `org.dynamisengine.collision.world`
- `org.dynamisengine.collision.contact`
- `org.dynamisengine.collision.constraints`
- `org.dynamisengine.collision.events`
- `org.dynamisengine.collision.debug`

Evidence examples:

- `CollisionWorld3D` performs update loops, event lifecycle (`ENTER/STAY/EXIT`), and `step(...)` including gravity integration and constraint/response passes.
- `ContactSolver3D` applies position and velocity impulses using restitution/friction and warm-start state.
- `PhysicsStep3D` is a fixed-timestep accumulator utility.
- Constraint interfaces/implementations (`Constraint3D`, `DistanceConstraint3D`, `PointConstraint3D`) are simulation concerns.

These are functionally useful but architecturally overlap with the ratified Physics authority.

### Internal/implementation candidates

If this boundary is tightened later, world/contact/constraints/debug runtime implementations should be treated as:

- either internal transitional scaffolding,
- or moved/owned under Physics-side runtime modules.

## Policy Leakage / Overlap Findings

### DynamisPhysics overlap (major hotspot)

Strong overlap exists with physics simulation authority:

- Collision currently includes solver (`ContactSolver3D`) and simulation stepping (`CollisionWorld3D.step`, `PhysicsStep3D`) behaviors that Physics should own per prior ratification.
- Contact manifold lifecycle and warm-start persistence used for iterative simulation are physics-adjacent runtime behaviors, not pure query substrate.

### DynamisSceneGraph overlap (currently low)

- No direct SceneGraph ownership detected.
- Risk remains if collision world objects become scene-node authorities rather than shape/query consumers.

### MeshForge overlap (moderate, manageable)

- `MeshCollisionAdapter` and `PackedMeshCollisionShape` correctly consume MeshForge data.
- This is acceptable as read-only query integration.
- Boundary risk: collision should not absorb mesh preparation/shaping responsibilities.

### DynamisWorldEngine overlap (moderate risk)

- `CollisionWorld3D` naming/behavior resembles world-runtime orchestration.
- WorldEngine should orchestrate; Collision should provide substrate queries and shape/filter mechanics.

### Debug/VFX overlap (minor)

- `CollisionDebugSnapshot3D` is diagnostics-focused and can remain if strictly data-only.
- Avoid turning collision debug helpers into render/system policy APIs.

## Ratification Result

**Result: needs boundary tightening**

Why:

- The repo has a solid collision substrate core (broadphase/narrowphase/shapes/filtering/pipeline).
- But public/module surface currently includes simulation/runtime orchestration concerns that overlap with DynamisPhysics authority.
- The Physics↔Collision seam is not yet strict enough in current exported structure.

## Recommended Next Step

1. Define and document the explicit seam contract:
   - Collision owns query substrate APIs and shape/filter/query data models.
   - Physics owns simulation stepping, constraint resolution, contact response lifecycle, and solver policy.
2. Treat world/contact/constraint runtime pieces in Collision as transitional until ownership is consolidated.
3. Perform the next deep review on **DynamisVFX** (or another graphics/feature consumer) to ensure debug/visual consumers stay downstream of the Collision/Physics boundary instead of pulling ownership upward.

This review is a boundary-ratification document only and does not propose immediate package moves or API-breaking refactors.
