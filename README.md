# DynamisCollision

Collision detection library for rendering-engine integration, organized as a JDK 25 multi-module Maven project.

This project is powered by:
- `Vectrix` for vector/matrix math primitives
- `MeshForge` for mesh/bounds integration (`MeshData` and `PackedMesh`)

## Modules

- `collision_detection`: core library module for collision detection APIs.
- `demo`: companion module for experiments and examples.

## Requirements

- JDK `25` (already pinned in `.java-version`)
- Maven `3.9+`
- Local Maven artifacts:
  - `org.vectrix:vectrix:1.10.9`
  - `org.meshforge:meshforge:1.1.0`

## Project Layout

```text
.
├── pom.xml
├── collision_detection
│   ├── pom.xml
│   └── src/main/java/org/dynamiscollision/...
└── demo
    ├── pom.xml
    └── src/main/java/org/dynamiscollision/...
```

Use `org.dynamiscollision` as the base package for all new code.

## Build and Test

Run from repository root:

```bash
mvn clean verify
```

Useful targeted commands:

```bash
mvn -pl collision_detection test
mvn -pl demo -am compile
```

## Current Capabilities

- Broad phase: `SpatialHash3D`, `SweepAndPrune3D`
- Narrow phase: `Intersection3D`, `Gjk3D` (+ EPA manifold), `Sat2D`
- Contact generation: `Aabb`, `BoundingSphere`, `Capsule` combinations
- Collision world/runtime: filtering, event lifecycle (`ENTER/STAY/EXIT`), manifold cache, iterative response solver
- Mesh integration: `MeshCollisionAdapter`, `PackedMeshCollisionShape`, coarse meshlet-aware raycast
- Testing: unit, deterministic lifecycle checks, stress/fuzz invariants, benchmark smoke coverage

## Known Gaps (Intentional v2 Scope)

- OBB primitives
- Triangle-level mesh narrow phase (current mesh raycast is coarse, not triangle-accurate)
- Full conservative advancement CCD (current convex CCD is sampled + refinement)
- Velocity-level joint constraints
- Automatic convex decomposition

## Dependency Notes

```xml
<dependency>
  <groupId>org.vectrix</groupId>
  <artifactId>vectrix</artifactId>
  <version>1.10.9</version>
</dependency>
<dependency>
  <groupId>org.meshforge</groupId>
  <artifactId>meshforge</artifactId>
  <version>1.1.0</version>
</dependency>
```

This keeps Vectrix available for vector/matrix math and MeshForge available for mesh/bounds integration as the library expands.
