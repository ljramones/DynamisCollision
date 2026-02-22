/*
 * Copyright 2024-2026 DynamisFX Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.dynamiscollision;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.function.Function;

/**
 * Lightweight benchmark harness for broad-phase candidate generation.
 *
 * Run:
 * {@code mvn -pl DynamisFX-Core -DskipTests test-compile}
 * then execute this class from your IDE or test classpath runner.
 */
public final class BroadPhase3DBenchmark {

    private BroadPhase3DBenchmark() {
    }

    public static void main(String[] args) {
        int objectCount = args.length > 0 ? Integer.parseInt(args[0]) : 25_000;
        int rounds = args.length > 1 ? Integer.parseInt(args[1]) : 8;
        long seed = args.length > 2 ? Long.parseLong(args[2]) : 1337L;

        List<Body> bodies = generateBodies(objectCount, seed);
        Function<Body, Aabb> bounds = Body::bounds;

        BroadPhase3D<Body> spatialHash = new SpatialHash3D<>(24.0);
        BroadPhase3D<Body> sweepAndPrune = new SweepAndPrune3D<>();

        // warmup
        run(spatialHash, bodies, bounds, 2);
        run(sweepAndPrune, bodies, bounds, 2);

        Result hashResult = run(spatialHash, bodies, bounds, rounds);
        Result sapResult = run(sweepAndPrune, bodies, bounds, rounds);

        System.out.println("BroadPhase3DBenchmark");
        System.out.println("objects=" + objectCount + " rounds=" + rounds + " seed=" + seed);
        System.out.printf("SpatialHash3D  avg=%.3f ms pairs=%d%n", hashResult.avgMillis(), hashResult.pairs());
        System.out.printf("SweepAndPrune3D avg=%.3f ms pairs=%d%n", sapResult.avgMillis(), sapResult.pairs());
    }

    private static Result run(
            BroadPhase3D<Body> algorithm,
            List<Body> bodies,
            Function<Body, Aabb> bounds,
            int rounds) {
        long totalNanos = 0L;
        int pairCount = 0;
        for (int i = 0; i < rounds; i++) {
            long start = System.nanoTime();
            Set<CollisionPair<Body>> pairs = algorithm.findPotentialPairs(bodies, bounds);
            totalNanos += (System.nanoTime() - start);
            pairCount = pairs.size();
        }
        return new Result((totalNanos / 1_000_000.0) / rounds, pairCount);
    }

    private static List<Body> generateBodies(int count, long seed) {
        Random rng = new Random(seed);
        List<Body> out = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            double cx = rng.nextDouble() * 8_000.0 - 4_000.0;
            double cy = rng.nextDouble() * 8_000.0 - 4_000.0;
            double cz = rng.nextDouble() * 8_000.0 - 4_000.0;
            double sx = 2.0 + rng.nextDouble() * 16.0;
            double sy = 2.0 + rng.nextDouble() * 16.0;
            double sz = 2.0 + rng.nextDouble() * 16.0;
            Aabb bounds = new Aabb(
                    cx - sx * 0.5, cy - sy * 0.5, cz - sz * 0.5,
                    cx + sx * 0.5, cy + sy * 0.5, cz + sz * 0.5);
            out.add(new Body(i, bounds));
        }
        return out;
    }

    private record Body(int id, Aabb bounds) {
    }

    private record Result(double avgMillis, int pairs) {
    }
}
