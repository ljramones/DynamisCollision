# Animation Integration Contract

## Goal

Define how animation pose output should feed DynamisCollision runtime input through `RigidBodyAdapter3D<T>`.

## Data Flow

1. Evaluate animation pose for the current frame.
2. Compute world-space bone transforms.
3. Build/update collision body state (position, velocity, bounds/shape transform) from those transforms.
4. Run `CollisionWorld3D.update(...)` or `CollisionWorld3D.step(...)`.
5. Consume collision outputs (events and/or solved state) in gameplay/animation reconciliation.

## Adapter Expectations

- `getPosition` and `getVelocity` are world-space values.
- `setPosition` and `setVelocity` write world-space values back to the same body state.
- `getInverseMass`:
  - `0.0` for kinematic animation-driven bodies
  - `> 0.0` for dynamic bodies that should be moved by solver impulses
- `getRestitution` and `getFriction` must be finite and non-negative.

## Caller Responsibilities

- Apply animation pose before collision update each frame.
- Ensure bounds/shape transforms match the latest pose before broad phase.
- If animation is authoritative, decide reconciliation strategy:
  - animation-only (ignore solver writeback for those bodies), or
  - blended/constraint-aware (consume solver corrections to avoid interpenetration artifacts).

## Determinism Notes

- Keep object insertion order stable when deterministic event ordering matters.
- Use fixed-step update loops for simulation consistency.
