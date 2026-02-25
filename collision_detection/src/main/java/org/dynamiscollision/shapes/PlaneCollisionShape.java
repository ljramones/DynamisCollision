package org.dynamiscollision.shapes;

import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Vector3d;

public final class PlaneCollisionShape implements CollisionShape {
    private final float nx;
    private final float ny;
    private final float nz;
    private final float d;

    public PlaneCollisionShape(float nx, float ny, float nz, float d) {
        double len = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (!Float.isFinite(nx) || !Float.isFinite(ny) || !Float.isFinite(nz) || !Float.isFinite(d) || len <= 0.0) {
            throw new IllegalArgumentException("Plane normal/d invalid");
        }
        this.nx = (float) (nx / len);
        this.ny = (float) (ny / len);
        this.nz = (float) (nz / len);
        this.d = d;
    }

    public float normalX() { return nx; }
    public float normalY() { return ny; }
    public float normalZ() { return nz; }
    public float distance() { return d; }

    @Override
    public ShapeType shapeType() {
        return ShapeType.PLANE;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        return Aabb.infinite();
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        double denom = nx * ray.dirX() + ny * ray.dirY() + nz * ray.dirZ();
        if (Math.abs(denom) < 1e-9) {
            return Optional.empty();
        }
        double hitT = (d - (nx * ray.originX() + ny * ray.originY() + nz * ray.originZ())) / denom;
        if (hitT < 0.0) {
            return Optional.empty();
        }
        Vector3d point = new Vector3d(
                ray.originX() + ray.dirX() * hitT,
                ray.originY() + ray.dirY() * hitT,
                ray.originZ() + ray.dirZ() * hitT);
        Vector3d normal = new Vector3d(nx, ny, nz);
        return Optional.of(new RaycastResult(hitT, point, normal, -1, -1));
    }
}
