package org.dynamiscollision.shapes;

import java.util.List;
import java.util.Optional;
import org.dynamiscollision.bounds.Aabb;
import org.dynamiscollision.geometry.Ray3D;
import org.dynamiscollision.geometry.RaycastResult;
import org.vectrix.affine.Transformf;

public final class CompoundCollisionShape implements CollisionShape {
    private final List<CollisionShape> children;
    private final List<Transformf> localTransforms;

    public CompoundCollisionShape(List<CollisionShape> children, List<Transformf> localTransforms) {
        if (children == null || localTransforms == null || children.size() != localTransforms.size()) {
            throw new IllegalArgumentException("children.size() must equal localTransforms.size()");
        }
        this.children = List.copyOf(children);
        this.localTransforms = List.copyOf(localTransforms);
    }

    public List<CollisionShape> children() { return children; }
    public List<Transformf> localTransforms() { return localTransforms; }
    public int childCount() { return children.size(); }

    @Override
    public ShapeType shapeType() {
        return ShapeType.COMPOUND;
    }

    @Override
    public Aabb getWorldBounds(Transformf t) {
        if (children.isEmpty()) {
            return Aabb.empty();
        }

        Transformf world = t == null ? new Transformf().identity() : t;
        Aabb result = null;
        for (int i = 0; i < children.size(); i++) {
            Transformf childWorld = Transformf.compose(world, localTransforms.get(i), new Transformf());
            Aabb childBounds = children.get(i).getWorldBounds(childWorld);
            result = result == null ? childBounds : result.union(childBounds);
        }
        return result == null ? Aabb.empty() : result;
    }

    @Override
    public Optional<RaycastResult> raycast(Ray3D ray, Transformf t) {
        Transformf world = t == null ? new Transformf().identity() : t;
        RaycastResult closest = null;
        for (int i = 0; i < children.size(); i++) {
            Transformf childWorld = Transformf.compose(world, localTransforms.get(i), new Transformf());
            Optional<RaycastResult> hit = children.get(i).raycast(ray, childWorld);
            if (hit.isPresent() && (closest == null || hit.get().t() < closest.t())) {
                closest = hit.get();
            }
        }
        return Optional.ofNullable(closest);
    }
}
