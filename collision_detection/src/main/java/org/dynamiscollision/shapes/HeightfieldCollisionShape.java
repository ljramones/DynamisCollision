package org.dynamiscollision.shapes;

import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;

public final class HeightfieldCollisionShape implements CollisionShape {
    private final float[] heights;
    private final int widthSamples;
    private final int depthSamples;
    private final float worldWidth;
    private final float worldDepth;
    private final float maxHeight;

    public HeightfieldCollisionShape(
            float[] heights,
            int widthSamples,
            int depthSamples,
            float worldWidth,
            float worldDepth,
            float maxHeight) {
        if (heights == null || heights.length == 0) {
            throw new IllegalArgumentException("Heightfield heights must be non-empty");
        }
        this.heights = heights;
        this.widthSamples = widthSamples;
        this.depthSamples = depthSamples;
        this.worldWidth = worldWidth;
        this.worldDepth = worldDepth;
        this.maxHeight = maxHeight;
    }

    public float[] heights() { return heights; }
    public int widthSamples() { return widthSamples; }
    public int depthSamples() { return depthSamples; }
    public float worldWidth() { return worldWidth; }
    public float worldDepth() { return worldDepth; }
    public float maxHeight() { return maxHeight; }

    @Override
    public ShapeType shapeType() {
        return ShapeType.HEIGHTFIELD;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        Transformf tx = t == null ? new Transformf().identity() : t;
        double x = tx.translation.x();
        double y = tx.translation.y();
        double z = tx.translation.z();
        return new Aabb(x, y, z, x + worldWidth, y + maxHeight, z + worldDepth);
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        return Optional.empty();
    }
}
