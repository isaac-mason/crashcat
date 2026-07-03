import { describe, expect, test } from 'vitest';
import {
    addBroadphaseLayer,
    addObjectLayer,
    box,
    broadphase,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    registerAll,
    rigidBody,
} from '../src';

registerAll();

/**
 * Exhaustive property test for broadphase pair reporting.
 *
 * Enumerates the full cross-product of pair-relevant body state (motion type, sleeping,
 * sensor, collideKinematicVsNonDynamic, layer-pair enabled) for two overlapping boxes,
 * runs the real findCollidingPairs, and compares against an INDEPENDENTLY written
 * reference predicate. This pins the pair-reporting truth table: any change to
 * shouldReportPair must update the reference here deliberately.
 */

type PairBodySpec = {
    motionType: MotionType;
    sleeping: boolean;
    sensor: boolean;
    optIn: boolean; // collideKinematicVsNonDynamic
};

/**
 * the reference predicate — written from the intended contract, NOT from the
 * implementation. pinned contract rows (jolt-faithful, accepted as intended):
 * - two sleeping/static bodies are never (re)detected: nobody queries the broadphase.
 *   consequence: a body that falls asleep inside a static sensor goes stale.
 * - a kinematic SENSOR does not detect static geometry without the opt-in flag
 *   (the sensor exception only fires when the OTHER body is the sensor).
 * - sensors have no other effect on pair reporting; sensor semantics live in narrowphase.
 */
function referenceShouldReport(a: PairBodySpec, b: PairBodySpec, layersCollide: boolean): boolean {
    // at least one body must query the broadphase (awake and non-static)
    const queriesA = a.motionType !== MotionType.STATIC && !a.sleeping;
    const queriesB = b.motionType !== MotionType.STATIC && !b.sleeping;
    if (!queriesA && !queriesB) return false;

    // object layer pair table must allow the pair
    if (!layersCollide) return false;

    // motion type / sensor gate: opt-in, or a dynamic body, or kinematic-meets-sensor
    return (
        a.optIn ||
        b.optIn ||
        a.motionType === MotionType.DYNAMIC ||
        b.motionType === MotionType.DYNAMIC ||
        (a.motionType === MotionType.KINEMATIC && b.sensor) ||
        (b.motionType === MotionType.KINEMATIC && a.sensor)
    );
}

function specLabel(s: PairBodySpec): string {
    const motion = s.motionType === MotionType.STATIC ? 'static' : s.motionType === MotionType.KINEMATIC ? 'kinematic' : 'dynamic';
    return `${motion}${s.sleeping ? '+asleep' : ''}${s.sensor ? '+sensor' : ''}${s.optIn ? '+optIn' : ''}`;
}

function runCombo(a: PairBodySpec, b: PairBodySpec, layersCollide: boolean): number {
    const settings = createWorldSettings();
    const bpLayer = addBroadphaseLayer(settings);
    const layerA = addObjectLayer(settings, bpLayer);
    const layerB = addObjectLayer(settings, bpLayer);
    if (layersCollide) {
        enableCollision(settings, layerA, layerB);
    }
    const world = createWorld(settings);

    const shape = box.create({ halfExtents: [0.5, 0.5, 0.5] });

    const bodyA = rigidBody.create(world, {
        shape,
        objectLayer: layerA,
        motionType: a.motionType,
        position: [0, 0, 0],
        sensor: a.sensor,
        collideKinematicVsNonDynamic: a.optIn,
        mass: a.motionType === MotionType.DYNAMIC ? 1 : undefined,
    });
    const bodyB = rigidBody.create(world, {
        shape,
        objectLayer: layerB,
        motionType: b.motionType,
        position: [0.4, 0, 0], // overlapping
        sensor: b.sensor,
        collideKinematicVsNonDynamic: b.optIn,
        mass: b.motionType === MotionType.DYNAMIC ? 1 : undefined,
    });

    if (a.sleeping) rigidBody.sleep(world, bodyA);
    if (b.sleeping) rigidBody.sleep(world, bodyB);

    broadphase.findCollidingPairs(world, 0.02, undefined);
    return world.broadphase.pairs.n;
}

describe('broadphase pair logic', () => {
    test('findCollidingPairs matches the reference truth table over the full state cross-product', () => {
        const motionTypes = [MotionType.STATIC, MotionType.KINEMATIC, MotionType.DYNAMIC];
        const bools = [false, true];

        let combos = 0;
        for (const motionA of motionTypes) {
            for (const motionB of motionTypes) {
                for (const sleepA of bools) {
                    // sleeping is only reachable for dynamic bodies (the engine never
                    // sleeps kinematics; statics are excluded by sleep())
                    if (sleepA && motionA !== MotionType.DYNAMIC) continue;
                    for (const sleepB of bools) {
                        if (sleepB && motionB !== MotionType.DYNAMIC) continue;
                        for (const sensorA of bools) {
                            for (const sensorB of bools) {
                                for (const optInA of bools) {
                                    for (const optInB of bools) {
                                        for (const layersCollide of bools) {
                                            const a: PairBodySpec = { motionType: motionA, sleeping: sleepA, sensor: sensorA, optIn: optInA };
                                            const b: PairBodySpec = { motionType: motionB, sleeping: sleepB, sensor: sensorB, optIn: optInB };

                                            const expected = referenceShouldReport(a, b, layersCollide);
                                            const pairs = runCombo(a, b, layersCollide);

                                            const label = `A=${specLabel(a)} B=${specLabel(b)} layersCollide=${layersCollide}`;
                                            expect(pairs, label).toBe(expected ? 1 : 0);
                                            combos++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // sanity: the cross-product actually enumerated a meaningful state space
        expect(combos).toBeGreaterThan(500);
    });
});
