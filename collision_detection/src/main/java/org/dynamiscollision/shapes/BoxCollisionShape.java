package org.dynamiscollision.shapes;

import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;

public final class BoxCollisionShape implements CollisionShape {
    private final float halfX;
    private final float halfY;
    private final float halfZ;

    public BoxCollisionShape(float halfX, float halfY, float halfZ) {
        if (!Float.isFinite(halfX) || !Float.isFinite(halfY) || !Float.isFinite(halfZ)
                || halfX <= 0f || halfY <= 0f || halfZ <= 0f) {
            throw new IllegalArgumentException("Box half extents must be positive finite values");
        }
        this.halfX = halfX;
        this.halfY = halfY;
        this.halfZ = halfZ;
    }

    public float halfExtentX() { return halfX; }
    public float halfExtentY() { return halfY; }
    public float halfExtentZ() { return halfZ; }

    @Override
    public ShapeType shapeType() {
        return ShapeType.BOX;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        double cx = tx.translation.x();
        double cy = tx.translation.y();
        double cz = tx.translation.z();
        return new Aabb(cx - halfX, cy - halfY, cz - halfZ, cx + halfX, cy + halfY, cz + halfZ);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        return Optional.empty();
    }
}
