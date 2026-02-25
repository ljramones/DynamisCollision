package org.dynamiscollision.shapes;

import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;
import org.vectrix.core.Matrix4x3f;
import org.vectrix.core.Vector3f;

public final class ConvexHullCollisionShape implements CollisionShape {
    private final float[] vertices;
    private final int[] indices;

    public ConvexHullCollisionShape(float[] vertices, int[] indices) {
        if (vertices == null || vertices.length < 3 || vertices.length % 3 != 0) {
            throw new IllegalArgumentException("Convex hull vertices must be non-empty xyz triplets");
        }
        this.vertices = vertices.clone();
        this.indices = indices == null ? new int[0] : indices.clone();
    }

    public float[] vertices() { return vertices.clone(); }
    public int[] indices() { return indices.clone(); }

    @Override
    public ShapeType shapeType() {
        return ShapeType.CONVEX_HULL;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        Matrix4x3f m = tx.toAffineMat4Fast(new Matrix4x3f());

        float minX = Float.MAX_VALUE;
        float minY = Float.MAX_VALUE;
        float minZ = Float.MAX_VALUE;
        float maxX = -Float.MAX_VALUE;
        float maxY = -Float.MAX_VALUE;
        float maxZ = -Float.MAX_VALUE;

        for (int i = 0; i < vertices.length; i += 3) {
            Vector3f p = m.transformPosition(new Vector3f(vertices[i], vertices[i + 1], vertices[i + 2]));
            minX = Math.min(minX, p.x());
            minY = Math.min(minY, p.y());
            minZ = Math.min(minZ, p.z());
            maxX = Math.max(maxX, p.x());
            maxY = Math.max(maxY, p.y());
            maxZ = Math.max(maxZ, p.z());
        }
        return new Aabb(minX, minY, minZ, maxX, maxY, maxZ);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        return Optional.empty();
    }
}
