package org.dynamiscollision.shapes;

import org.dynamiscollision.bounds.Aabb;
import org.junit.jupiter.api.Test;
import org.vectrix.affine.Transformf;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

class TypedCollisionShapeTest {

    @Test
    void factoriesReturnTypedShapes() {
        assertEquals(ShapeType.SPHERE, CollisionShape.sphere(1f).shapeType());
        assertEquals(ShapeType.BOX, CollisionShape.box(1f, 2f, 3f).shapeType());
        assertEquals(ShapeType.CAPSULE, CollisionShape.capsule(0.5f, 2f).shapeType());
        assertEquals(ShapeType.CYLINDER, CollisionShape.cylinder(0.5f, 2f).shapeType());
        assertEquals(ShapeType.PLANE, CollisionShape.planeY().shapeType());
    }

    @Test
    void aabbHelpersExistAndWork() {
        Aabb a = new Aabb(-1, -1, -1, 1, 1, 1);
        Aabb b = new Aabb(2, 2, 2, 3, 3, 3);
        Aabb u = a.union(b);
        assertEquals(-1.0, u.minX());
        assertEquals(3.0, u.maxZ());
        assertNotNull(Aabb.empty());
        assertNotNull(Aabb.infinite());
    }

    @Test
    void compoundComputesUnionBounds() {
        CollisionShape compound = CollisionShape.compound(
            java.util.List.of(CollisionShape.sphere(1f), CollisionShape.box(1f, 1f, 1f)),
            java.util.List.of(new Transformf().identity(), new Transformf().identity())
        );
        Aabb bounds = compound.getWorldBounds(new Transformf().identity());
        assertNotNull(bounds);
    }
}
