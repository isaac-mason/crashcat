import { createWorld, createWorldSettings, registerAll, updateWorld, type Listener } from 'crashcat';

registerAll();

const worldSettings = createWorldSettings();
const world = createWorld(worldSettings);

/* SNIPPET_START: variable-timestep */
let lastTime = performance.now();
const maxDelta = 1 / 30;

function gameLoopVariableTimestep() {
    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    updateWorld(world, undefined, delta);

    // ... render ...

    requestAnimationFrame(gameLoopVariableTimestep);
}
/* SNIPPET_END: variable-timestep */

/* SNIPPET_START: fixed-timestep */
const PHYSICS_DT = 1 / 60;
let accumulator = 0;
let lastTimeFixed = performance.now();

function gameLoopFixedTimestep() {
    const currentTime = performance.now();
    const frameTime = Math.min((currentTime - lastTimeFixed) / 1000, 0.25);
    lastTimeFixed = currentTime;

    accumulator += frameTime;

    // step physics at fixed rate
    while (accumulator >= PHYSICS_DT) {
        updateWorld(world, undefined, PHYSICS_DT);
        accumulator -= PHYSICS_DT;
    }

    // ... render with interpolation ...
    // const alpha = accumulator / PHYSICS_DT;
    // interpolate body positions using alpha for smooth rendering

    requestAnimationFrame(gameLoopFixedTimestep);
}
/* SNIPPET_END: fixed-timestep */
