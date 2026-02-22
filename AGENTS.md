# Repository Guidelines

## Project Structure & Module Organization
This repository is currently a minimal scaffold and only includes `.java-version` (pinned JDK). As code is added, keep a predictable Java layout:
- `src/main/java/` for production code.
- `src/test/java/` for automated tests.
- `src/main/resources/` for non-code assets and config templates.
- `docs/` for design notes and architecture decisions.

Keep package names lowercase (for example, `org.dynamiscollision.core`) and align directories with package paths.

## Build, Test, and Development Commands
No build tool wrapper (`gradlew` or `mvnw`) is committed yet. Until one is added:
- `java --version` confirms the local JDK matches `.java-version`.
- `git status` checks working tree cleanliness before commits.

After adopting a build tool, standardize and document commands here (for example, `./gradlew test` or `./mvnw test`) and prefer wrapper-based commands over globally installed tooling.

## Coding Style & Naming Conventions
Use standard Java conventions:
- 4-space indentation, no tabs.
- `PascalCase` for classes, `camelCase` for methods/fields, `UPPER_SNAKE_CASE` for constants.
- One top-level public class per file; filename must match class name.

If formatting/lint tooling is added (for example, Spotless or Checkstyle), run it before opening a PR and commit formatting-only changes separately when possible.

## Testing Guidelines
There is no committed test framework yet. When tests are introduced:
- Place unit tests under `src/test/java/` mirroring package structure.
- Name test classes `*Test` (example: `CollisionResolverTest`).
- Keep tests deterministic and independent; avoid hidden ordering dependencies.

Document coverage targets once CI is configured.

## Commit & Pull Request Guidelines
Current history uses short, imperative commit subjects (example: `locked in jdk version`). Continue with concise, action-oriented messages under ~72 characters.

For pull requests:
- Explain what changed and why.
- Link related issues (`#123`) when applicable.
- Include test evidence (command + result), even for small changes.
- Keep PRs focused; separate refactors from functional changes.

## Security & Configuration Tips
Do not commit secrets, local `.env` files, or machine-specific IDE settings. Prefer checked-in example configs (for example, `application.example.properties`) and document required environment variables in this file or `docs/`.
