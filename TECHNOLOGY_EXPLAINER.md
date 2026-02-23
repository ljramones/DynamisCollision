# DynamisCollision Technology Explainer

## Purpose

DynamisCollision provides deterministic, engine-agnostic collision detection and basic collision response for 3D runtime integration.

The library is designed around a clear pipeline:
1. Broad phase candidate generation
2. Filter classification
3. Narrow phase contact generation
4. Event generation (`ENTER`, `STAY`, `EXIT`)
5. Optional iterative contact response

## Core Dependencies

- `org.vectrix:vectrix`: immutable math primitives (`Vector3d`, transforms)
- `org.meshforge:meshforge`: mesh and bounds data integration

## Collision Pipeline Architecture

### Broad Phase

- `SpatialHash3D<T>`: uniform grid candidate generation
- `SweepAndPrune3D<T>`: axis sweep candidate generation

Both return potential pairs only. They are intentionally conservative and should be followed by narrow phase checks.

### Filtering

- `CollisionFilter`: layer/mask and `SOLID`/`TRIGGER` classification
- `CollisionFiltering`: pair filtering and response eligibility classification

### Narrow Phase

- Primitive checks in `Intersection3D` (`Aabb`, `BoundingSphere`, `Capsule`, ray/AABB)
- Convex-convex checks in `Gjk3D` using support mappings (`ConvexSupport3D`)
- EPA in `Gjk3D.intersectsWithManifold(...)` for penetration manifold extraction
- 2D SAT support in `Sat2D`

### Contact Generation

`ContactGenerator3D` converts overlap tests into contact manifolds:
- `Aabb` vs `Aabb`
- `BoundingSphere` vs `BoundingSphere`
- `Capsule` vs `Capsule`
- `Capsule` vs `BoundingSphere`
- `Capsule` vs `Aabb`

### Runtime World

`CollisionWorld3D<T>` orchestrates frame updates:
- deterministic event lifecycle
- manifold cache and warm-start state
- configurable solver and constraint iterations
- `step(items, dt)` for velocity integration, predicted-state collision pass, and response

## Integration Patterns

### Generic Bodies

Provide:
- bounds function (`T -> Aabb`)
- filter function (`T -> CollisionFilter`)
- narrow phase function (`(T, T) -> Optional<ContactManifold3D>`)

### MeshForge Objects

- `MeshCollisionAdapter` for bounds/filter storage
- `PackedMeshCollisionShape` for world bounds and coarse meshlet-aware raycast

## Determinism and Validation

Current coverage includes:
- lifecycle determinism checks
- warm-start regression checks
- stress/fuzz invariant suite
- benchmark smoke tests (fixed-seed reproducibility)

## Intentional v2 Gaps

- OBB primitives
- triangle-level mesh narrow phase
- full conservative advancement CCD
- velocity-level joint constraints
- automatic convex decomposition
