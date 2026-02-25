package org.dynamiscollision.shapes;

import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;

public final class CylinderCollisionShape implements CollisionShape {
    private final float radius;
    private final float height;

    public CylinderCollisionShape(float radius, float height) {
        if (!Float.isFinite(radius) || radius <= 0f || !Float.isFinite(height) || height <= 0f) {
            throw new IllegalArgumentException("Cylinder radius/height invalid");
        }
        this.radius = radius;
        this.height = height;
    }

    public float radius() { return radius; }
    public float height() { return height; }

    @Override
    public ShapeType shapeType() {
        return ShapeType.CYLINDER;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        double cx = tx.translation.x();
        double cy = tx.translation.y();
        double cz = tx.translation.z();
        double halfH = height * 0.5;
        return new Aabb(cx - radius, cy - halfH, cz - radius, cx + radius, cy + halfH, cz + radius);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        return Optional.empty();
    }
}
