/*
 * Copyright 2024-2026 DynamisCollision Contributors
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

package org.dynamiscollision.world;

import java.util.function.DoubleConsumer;

/**
 * Fixed-timestep accumulator utility for deterministic stepping.
 */
public final class PhysicsStep3D {

    private final double fixedDtSeconds;
    private int maxSubsteps;
    private double accumulatorSeconds;

    public PhysicsStep3D(double fixedDtSeconds, int maxSubsteps) {
        if (!Double.isFinite(fixedDtSeconds) || fixedDtSeconds <= 0.0) {
            throw new IllegalArgumentException("fixedDtSeconds must be > 0");
        }
        if (maxSubsteps < 1) {
            throw new IllegalArgumentException("maxSubsteps must be >= 1");
        }
        this.fixedDtSeconds = fixedDtSeconds;
        this.maxSubsteps = maxSubsteps;
    }

    public int advance(double frameDeltaSeconds, DoubleConsumer stepConsumer) {
        if (!Double.isFinite(frameDeltaSeconds) || frameDeltaSeconds < 0.0) {
            throw new IllegalArgumentException("frameDeltaSeconds must be finite and >= 0");
        }
        if (stepConsumer == null) {
            throw new IllegalArgumentException("stepConsumer must not be null");
        }
        accumulatorSeconds += frameDeltaSeconds;

        int steps = 0;
        while (accumulatorSeconds >= fixedDtSeconds && steps < maxSubsteps) {
            stepConsumer.accept(fixedDtSeconds);
            accumulatorSeconds -= fixedDtSeconds;
            steps++;
        }
        if (steps == maxSubsteps && accumulatorSeconds > fixedDtSeconds * maxSubsteps) {
            accumulatorSeconds = fixedDtSeconds * maxSubsteps;
        }
        return steps;
    }

    public double fixedDtSeconds() {
        return fixedDtSeconds;
    }

    public int maxSubsteps() {
        return maxSubsteps;
    }

    public void setMaxSubsteps(int maxSubsteps) {
        if (maxSubsteps < 1) {
            throw new IllegalArgumentException("maxSubsteps must be >= 1");
        }
        this.maxSubsteps = maxSubsteps;
    }

    public double accumulatorSeconds() {
        return accumulatorSeconds;
    }

    public void reset() {
        accumulatorSeconds = 0.0;
    }
}
