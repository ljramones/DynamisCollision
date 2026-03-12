This is the right call, and it confirms the seam you were already worried about:

DynamisCollision is not cleanly ratified yet. It has a strong substrate core — bounds, shapes, geometry, broadphase, narrowphase, filtering, and pipeline — but it is still carrying simulation-style responsibilities that belong on the Physics side, not the Collision side. The review is explicit about that: CollisionWorld3D.step, ContactSolver3D, PhysicsStep3D, and constraint solving push it past “query substrate” into “simulation authority,” which is why “needs boundary tightening” is the correct result. 

dynamiscollision-architecture-r…

That lines up cleanly with the Physics review too: Physics has already been ratified as the simulation authority, and the main remaining hotspot identified there was exactly the Physics ↔ Collision seam. 

dynamisphysics-architecture-rev…

What this means

You now have a clear asymmetry:

DynamisPhysics: ratified with constraints, and should own dynamic simulation, contacts, solver stepping, and simulation-state queries. 

dynamisphysics-architecture-rev…

DynamisCollision: not yet cleanly ratified, because it still includes world/contact/constraint runtime behavior that overlaps with Physics authority. 

dynamiscollision-architecture-r…

So the next reviews should avoid pretending the seam is settled. The review’s recommendation to go to DynamisVFX next is sensible because it checks that visual/debug consumers remain downstream of the Collision/Physics split instead of becoming a back door that re-hardens the wrong ownership.
