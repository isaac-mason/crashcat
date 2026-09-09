import type { World } from 'crashcat';

export type ScenarioInstance = {
    world: World;
    /** advances exactly one step, including any per-step scenario work (queries, churn, character input) */
    step(stepIndex: number): void;
};

export type Scenario = {
    /** matches the file stem, and is the name labs reports under */
    name: string;
    /** what this scenario is derived from, and which subsystem it loads */
    description: string;
    /** steps the measured op runs after building the world */
    steps: number;
    /** builds a fresh world. called once per measured op, so it must be deterministic */
    create(): ScenarioInstance;
};

export function defineScenario(scenario: Scenario): Scenario {
    return scenario;
}

/**
 * runs a whole scenario once: build, then `steps` steps.
 *
 * this is the measured unit. the return value is the awake body count, which depends on the whole
 * simulation, so it doubles as the labs snapshot and keeps the work out of reach of dce. building every op is what keeps the workload stationary across
 * labs' repeated sampling: a physics scenario that kept stepping one long-lived world would get
 * cheaper as it settled, and the samples would measure a different simulation each time.
 */
export function runScenario(scenario: Scenario): number {
    const instance = scenario.create();
    for (let i = 0; i < scenario.steps; i++) {
        instance.step(i);
    }
    return instance.world.bodies.activeBodyCount;
}
