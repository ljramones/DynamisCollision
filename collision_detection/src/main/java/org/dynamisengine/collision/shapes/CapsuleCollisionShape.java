package org.dynamisengine.collision.shapes;

import java.util.Optional;
import org.dynamisengine.collision.bounds.Aabb;
import org.dynamisengine.collision.geometry.Ray3D;
import org.dynamisengine.collision.geometry.RaycastResult;
import org.dynamisengine.vectrix.affine.Transformf;

public final class CapsuleCollisionShape implements CollisionShape {
    private final float radius;
    private final float height;

    public CapsuleCollisionShape(float radius, float height) {
        if (!Float.isFinite(radius) || radius <= 0f || !Float.isFinite(height) || height < 0f) {
            throw new IllegalArgumentException("Capsule radius/height invalid");
        }
        this.radius = radius;
        this.height = height;
    }

    public float radius() { return radius; }
    public float height() { return height; }

    @Override
    public ShapeType shapeType() {
        return ShapeType.CAPSULE;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        double cx = tx.translation.x();
        double cy = tx.translation.y();
        double cz = tx.translation.z();
        double halfH = (height * 0.5) + radius;
        return new Aabb(cx - radius, cy - halfH, cz - radius, cx + radius, cy + halfH, cz + radius);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        return Optional.empty();
    }
}
