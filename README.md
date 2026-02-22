# DynamisCollision

Collision detection library for rendering-engine integration, organized as a JDK 25 multi-module Maven project.

## Modules

- `collision_detection`: core library module for collision detection APIs.
- `demo`: companion module for experiments and examples.

## Requirements

- JDK `25` (already pinned in `.java-version`)
- Maven `3.9+`
- Local Maven artifact for Vectrix:
  - `org.vectrix:vectrix:1.10.9`

## Project Layout

```text
.
├── pom.xml
├── collision_detection
│   ├── pom.xml
│   └── src/main/java/org/dynamiscollision/...
└── demo
    ├── pom.xml
    └── src/main/java/org/dynamiscollision/...
```

Use `org.dynamiscollision` as the base package for all new code.

## Build and Test

Run from repository root:

```bash
mvn clean verify
```

Useful targeted commands:

```bash
mvn -pl collision_detection test
mvn -pl demo -am compile
```

## Dependency Notes

`collision_detection` already declares:

```xml
<dependency>
  <groupId>org.vectrix</groupId>
  <artifactId>vectrix</artifactId>
  <version>1.10.9</version>
</dependency>
```

This keeps Vectrix available for vector/matrix-backed collision features as the library expands.
