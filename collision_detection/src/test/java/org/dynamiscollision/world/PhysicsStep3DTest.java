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

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class PhysicsStep3DTest {

    @Test
    void advancesInFixedSubsteps() {
        PhysicsStep3D stepper = new PhysicsStep3D(0.01, 8);
        AtomicInteger calls = new AtomicInteger();

        int steps = stepper.advance(0.025, dt -> calls.incrementAndGet());
        assertEquals(2, steps);
        assertEquals(2, calls.get());
        assertEquals(0.005, stepper.accumulatorSeconds(), 1e-9);
    }

    @Test
    void deterministicAcrossSameInputSequence() {
        double[] deltas = new double[] {0.016, 0.017, 0.015, 0.016, 0.020};
        int totalA = runSequence(deltas);
        int totalB = runSequence(deltas);
        assertEquals(totalA, totalB);
    }

    private static int runSequence(double[] deltas) {
        PhysicsStep3D stepper = new PhysicsStep3D(0.01, 8);
        AtomicInteger calls = new AtomicInteger();
        for (double delta : deltas) {
            stepper.advance(delta, dt -> calls.incrementAndGet());
        }
        return calls.get();
    }
}
