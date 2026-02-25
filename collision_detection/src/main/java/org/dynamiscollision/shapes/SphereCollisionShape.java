package org.dynamiscollision.shapes;

import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Vector3d;

public final class SphereCollisionShape implements CollisionShape {
    private final float radius;

    public SphereCollisionShape(float radius) {
        if (!Float.isFinite(radius) || radius <= 0f) {
            throw new IllegalArgumentException("Sphere radius must be positive, got: " + radius);
        }
        this.radius = radius;
    }

    public float radius() {
        return radius;
    }

    @Override
    public ShapeType shapeType() {
        return ShapeType.SPHERE;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        double cx = tx.translation.x();
        double cy = tx.translation.y();
        double cz = tx.translation.z();
        return new Aabb(cx - radius, cy - radius, cz - radius, cx + radius, cy + radius, cz + radius);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        double cx = tx.translation.x();
        double cy = tx.translation.y();
        double cz = tx.translation.z();

        double ocx = ray.originX() - cx;
        double ocy = ray.originY() - cy;
        double ocz = ray.originZ() - cz;

        double a = ray.dirX() * ray.dirX() + ray.dirY() * ray.dirY() + ray.dirZ() * ray.dirZ();
        double b = 2.0 * (ocx * ray.dirX() + ocy * ray.dirY() + ocz * ray.dirZ());
        double c = ocx * ocx + ocy * ocy + ocz * ocz - radius * radius;

        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0) {
            return Optional.empty();
        }

        double sqrt = Math.sqrt(disc);
        double t0 = (-b - sqrt) / (2.0 * a);
        double t1 = (-b + sqrt) / (2.0 * a);
        double hitT = t0 >= 0.0 ? t0 : t1;
        if (hitT < 0.0) {
            return Optional.empty();
        }

        Vector3d hit = new Vector3d(
                ray.originX() + ray.dirX() * hitT,
                ray.originY() + ray.dirY() * hitT,
                ray.originZ() + ray.dirZ() * hitT);
        Vector3d normal = new Vector3d(hit.x() - cx, hit.y() - cy, hit.z() - cz).normalize();
        return Optional.of(new RaycastResult(hitT, hit, normal, -1, -1));
    }
}
