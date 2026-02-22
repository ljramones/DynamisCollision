# Collision Package Layout

Collision code is organized by responsibility while keeping package names stable (`org.dynamiscollision`).

## Main Source Folders

- `bounds/`: AABB and sphere bounds primitives.
- `broadphase/`: broad-phase interfaces and implementations.
- `narrowphase/`: SAT/GJK/CCD/intersection primitives.
- `contact/`: contact generation, manifolds, caching, solving.
- `constraints/`: point/distance and generic constraint contracts.
- `pipeline/`: pairing/filtering pipeline orchestration.
- `world/`: world stepping, responders, body adapters.
- `filtering/`: collision kinds, filters, mask/layer logic.
- `events/`: collision event payloads and event types.
- `debug/`: debug snapshot models.
- `adapters/`: MeshForge mesh collision adapters.
- `geometry/`: ray and related helpers.

## Test Source Mirrors

Tests under `collision_detection/src/test/java/org/dynamiscollision/` mirror the same folder structure for easier navigation.
