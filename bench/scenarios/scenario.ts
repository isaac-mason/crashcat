import type { World } from 'crashcat';

export type ScenarioInstance = {
    world: World;
    /**
     * returns the world to the state `create()` left it in, so every measured window starts the
     * same way. this runs inside the timed region, so it must stay cheap — one pass over the
     * bodies, not a rebuild.
     */
    reset(): void;
    /** advances exactly one step, including any per-step scenario work (queries, character input) */
    step(stepIndex: number): void;
};

export type Scenario = {
    /** matches the file stem, and is the name labs reports under */
    name: string;
    /** what this scenario is derived from, and which subsystem it loads */
    description: string;
    /** steps in one measured window */
    steps: number;
    /** builds the world. called once, outside the timed region */
    create(): ScenarioInstance;
};

export function defineScenario(scenario: Scenario): Scenario {
    return scenario;
}

/** run one measured window: back to the start, then `steps` steps */
export function runWindow(scenario: Scenario, instance: ScenarioInstance): number {
    instance.reset();
    for (let i = 0; i < scenario.steps; i++) {
        instance.step(i);
    }
    // the awake body count depends on the whole window, so it doubles as the labs snapshot and
    // keeps the work out of reach of dead code elimination
    return instance.world.bodies.activeBodyCount;
}

/**
 * runs a few windows untimed.
 *
 * `reset` cannot rewind the contact cache's warm-start lambdas or the shape of the broadphase tree,
 * so the first window off a fresh world is not quite the same work as the ones after it. running a
 * couple here lets that converge before anything is measured.
 */
export function warm(scenario: Scenario, instance: ScenarioInstance, windows = 2): void {
    for (let w = 0; w < windows; w++) {
        runWindow(scenario, instance);
    }
}
