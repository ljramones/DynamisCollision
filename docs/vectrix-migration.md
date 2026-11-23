# DynamisCollision → Vectrix Migration (Phase 1), and Verified Defects

Status: **planned**. Vectrix 2.4.0 adds the kernels this note maps to.

The ownership rule is `/Users/larrymitchell/Dynamis/docs/architecture/math-gpu-ownership.md`, section 2:
**Collision detects, Physics decides, and vectrix supplies the pure functions.** Shapes, manifolds, the broadphase
structures and the solvers stay here.

The defects in section 3 were verified on 2026-11-22 against `main` at `c919291`. Paths are relative to
`collision_detection/src/main/java/org/dynamisengine/collision/`.

## 1. Where things stand

- Collision depends on vectrix **2.0.0** (`pom.xml`, `module-info.java: requires org.dynamisengine.vectrix`).
- It uses vectrix only for basic types: `core.Vector*` and `Matrix*` (17 imports) and `affine.Transformf` (11
  imports), in 23 files.
- None of vectrix's geometry code is used. Collision reimplements it, and most helpers create a new immutable
  object per operation, including two private `Vec3` records.

## 2. Replace with vectrix (double precision, allocation-free)

| Collision code | Vectrix 2.4.0 |
|---|---|
| `contact/ContactGenerator3D` AABB–AABB contact (lines 35-94) | `geometry.Contactsd.aabbAabb`: the exact minimum translation (see defect 2) |
| `ContactGenerator3D` sphere–sphere (`sphereLikeContact`) | `Contactsd.sphereSphere` (coincident centers → `+X`, as today) |
| `ContactGenerator3D` segment–segment and point–segment closest points | `Intersectiond.findClosestPointsLineSegments`, `findClosestPointOnLineSegment` (zero-length segments are handled from 2.4.0) |
| `ContactGenerator3D` segment–AABB by a 48-step ternary search (hundreds of allocations per call) | `ClosestPointsd.segmentAab`: exact, with no iteration and no allocation |
| `ContactGenerator3D` capsule–AABB | `Contactsd.capsuleAabb` (fixes defect 1) |
| `ContactGenerator3D` nearest face, closest point on AABB, point–AABB distance | `Contactsd.nearestFace`, `ClosestPointsd.closestPointAab`, `distanceSquaredPointAab` |
| `narrowphase/Intersection3D` solid sphere overlap | `Contactsd.testSphereSphere`. **Not** `Intersectiond.testSphereSphere`, which asks whether the *surfaces* meet and returns false for a sphere inside another. |
| `Intersection3D` AABB overlap, sphere–AABB | `Intersectiond.testAabAab`, `testAabSphere` |
| `Intersection3D` ray–AABB slab | `Intersectiond.intersectRayAab`. vectrix returns a negative `tNear` when the ray starts inside, so clamp it to 0 to keep today's behaviour. |
| Shape `getWorldBounds` | `BoundingVolumesd.aabOfObb`, `aabOfCapsule`, `aabOfCylinder`; `BoundingVolumesf.aabOfTransformedPoints` (fixes defect 5) |
| Private `Vec3` records (`ContactGenerator3D`, `Gjk3D`) | `Vector3d` (mutable; reuse scratch instances in hot loops) |
| `Gjk3D` any-perpendicular | `GeometryUtils.perpendicular(double…)` |
| `constraints/DistanceConstraint3D` | `physics.PbdConstraintsd.distanceConstraint` |
| `ConvexPolygon2D` convexity and winding | `RobustPredicates.orient2d`, `MeshMath.signedArea2` |

For an oriented box, transform the other shape into the box frame, call the AABB kernel, and rotate the normal and
point back.

**Not in vectrix:**
- GJK/EPA with support functions, and the GJK-based time of impact. This is the undecided "Phase 2" in the
  architecture page.
- Swept-AABB time of impact.
- 2D SAT with a minimum translation vector.
- A signed-cell spatial hash key.

## 3. Verified defects

Each defect was reproduced by running the real classes, and the inputs are given so each one can become a
regression test. "Vectrix" in the expected column means the matching 2.4.0 kernel gives the correct answer on the
same input. The contact solver moves body A along −n and body B along +n (`ContactSolver3D:140-147`), so contact
normals must point from A to B.

| # | Defect | Reproduction (inputs → observed vs expected) | Status |
|---|---|---|---|
| 1 | **Capsule–AABB deep case pushes the capsule deeper, with too small a depth.** `ContactGenerator3D.generate(Capsule, Aabb)` uses the box's *outward* face normal once the segment touches the box (lines 184-189). Its shallow branch points capsule → box, so the normal flips sign as the segment enters. The penetration is `radius − 0`, which ignores how far the segment is inside. | Box `[−1, 1]³`, radius 0.1. Segment at x = 1.05 (shallow): normal −X, depth 0.05 ✓. Segment at x = 0.90 (deep): normal **+X**, depth **0.1**. Expected −X, 0.2 (vectrix `Contactsd.capsuleAabb` gives −X, 0.2). | **Confirmed** |
| 2 | **AABB–AABB contact does not separate contained boxes.** The penetration is the overlap width on the axis of least overlap, with its sign taken from the box centers. When one box contains the other along that axis, pushing by that amount leaves them overlapping. | A `[0, 10]³`, B `[4, 5]×[1, 9.5]×[1, 9]` → normal −X, depth **1**. After pushing B by 1 the boxes still overlap. Expected: the minimum separating push is 5 along −X (vectrix `Contactsd.aabbAabb`). | **Confirmed** |
| 3 | **GJK misses overlaps of small shapes.** `Gjk3D` compares unnormalised squared lengths with an absolute `EPSILON = 1e-9` (lines 35, 423, 428). Triple-product search directions scale with size³, so they are treated as zero for millimetre shapes, and the test reports no intersection. | Capsule (0.5s, ±s, 0.3s), radius 0.6s, against box `[−s, 0]×[−s, s]×[−s, s]`; the two overlap by 0.1s. `Intersection3D.intersects` returns true for s = 1, 0.1 and 0.01, but **false for s = 1e-3 and 1e-4**. Separated pairs are correctly false at every scale. | **Confirmed** |
| 4 | **EPA returns no manifold for overlapping spheres.** `Gjk3D.intersectsWithManifold` gives up after 48 iterations against an absolute tolerance of 1e-6. A curved support never gets that close in 48 polytope refinements, so EPA returns `Optional.empty()` even though GJK reports the hit. Boxes work. Only tests call this today. | Unit spheres at (0, 0, 0) and (0.5, 0, 0): GJK true, EPA **none** (exact depth 1.5). The same happens for centers (1.5, 0, 0), (1.5, 0.4, 0.1) and (1, 1, 1). Boxes overlapping by 0.5 → (1, 0, 0), 0.5 ✓. | **Confirmed** |
| 5 | **World bounds ignore rotation and scale.** Box, capsule and cylinder `getWorldBounds` use only the translation; sphere bounds ignore scale. A rotated body's bounds are wrong, so the broadphase misses pairs. | Box half extents (2, 0.1, 0.1), rotated 90° about Z: bounds x `[−2, 2]`, y `[−0.1, 0.1]`. True: x `[−0.1, 0.1]`, y `[−2, 2]`. A body at (0, 1.55, 0), inside the rotated box, does not overlap its bounds. A sphere of radius 1 with scale 3 has bounds x `[−1, 1]`. | **Confirmed** |
| 6 | **Spatial hash never terminates with a plane.** `PlaneCollisionShape.getWorldBounds` returns `Aabb.infinite()` (±`Double.MAX_VALUE`). `SpatialHash3D.cell()` saturates to `Integer.MAX_VALUE`, and the `x <= maxCellX; x++` loops (lines 72-78) then overflow and never end. Even without the overflow, it would allocate a `Cell` for each of 2³² cells per axis. | `new SpatialHash3D<>(1.0).findPotentialPairs([plane, box], …)` is still running after 3 s. | **Confirmed** |
| 7 | **Heightfield bounds exclude negative heights.** `HeightfieldCollisionShape.getWorldBounds` spans `y .. y + maxHeight` from the corner origin, ignoring the actual height values. | Heights `{−3, −3, 2, 2}`, `maxHeight` 2 → bounds y `[0, 2]`; the surface goes down to −3. | **Confirmed.** Physics' ODE adapter also centres heightfields while these bounds start at the corner; that part is unverified. |
| 8 | **The packed-mesh raycast skips meshlets that face the ray.** `PackedMeshCollisionShape.raycast` (line 114) skips a meshlet when `rayDir · coneAxis < coneCutoffCos`. MeshForge stores the mean unit normal and the minimum normal-to-axis cosine (`MeshletClusters:231-286`), so the test keeps only back-facing meshlets. The existing tests all use `cutoffCos = −1`, a cone that never skips, so this path is untested. | Meshlet bounds `[0, 1]³`, ray from (−1, 0.5, 0.5) along +X. A cone with axis −X (facing the ray), cutoff 0.9 → **miss**. A cone with axis +X (facing away) → **hit**. For front faces, a meshlet can be skipped only when `rayDir · axis ≥ sin α`; vectrix `gpu.MeshletCones` has the exact test. | **Confirmed** |
| 9 | **Raycasts against most shapes always miss.** `raycast` is a stub returning `Optional.empty()` for box, capsule, cylinder, convex hull, triangle mesh and heightfield shapes. Only sphere, plane, packed mesh (meshlet bounds only) and compound shapes implement it. The README mentions only that the mesh raycast is coarse. | A ray straight down at a unit box at the origin → `Optional.empty`. | **Confirmed** (an unimplemented feature) |
| — | *Touching contacts disagree between GJK and the contact generator* (suspected from `Gjk3D:139`, `<= 0`) | A capsule at exactly radius distance from a box: GJK true, contact generator depth 0. They agree. | **Not reproduced** |

Fixing order:
1. Defect 6, the hang. Clamp infinite bounds, or keep unbounded shapes such as planes out of the hash.
2. Defects 1 and 2, the contact generator. Switch to `Contactsd`.
3. Defect 5, the world bounds. Switch to `BoundingVolumesd`.
4. Defects 3 and 4, GJK/EPA: relative tolerances, an iteration budget, and a fallback for curved supports. If GJK/EPA
   moves to vectrix (Phase 2), this work happens there.
5. Defects 7, 8 and 9 are independent.

Every fix should land with the reproduction above as a regression test.

## 4. Migration order

1. Fix defect 6 (independent of vectrix).
2. Bump `<vectrix.version>` to 2.4.0 once it is released.
3. Switch the contact generator to `Contactsd` and `ClosestPointsd`, which fixes defects 1 and 2. Keep the existing
   contact tests as regressions, and add those from section 3.
4. Switch world bounds to `BoundingVolumesd`, which fixes defect 5.
5. Replace the private `Vec3` records, reusing scratch instances in hot loops.
6. Decide GJK/EPA ownership (Phase 2) before fixing defects 3 and 4.
