module org.dynamisengine.collision {
    requires org.dynamisengine.vectrix;
    requires meshforge;

    exports org.dynamisengine.collision.bounds;
    exports org.dynamisengine.collision.shapes;
    exports org.dynamisengine.collision.geometry;
    exports org.dynamisengine.collision.events;
    exports org.dynamisengine.collision.filtering;
    exports org.dynamisengine.collision.pipeline;
    exports org.dynamisengine.collision.world;
    exports org.dynamisengine.collision.contact;
    exports org.dynamisengine.collision.broadphase;
    exports org.dynamisengine.collision.narrowphase;
    exports org.dynamisengine.collision.constraints;
    exports org.dynamisengine.collision.debug;
    exports org.dynamisengine.collision.adapters;
}
