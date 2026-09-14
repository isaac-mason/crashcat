import { box, capsule, filter, type KCC, kcc, MotionType, rigidBody, type Shape, updateWorld } from 'crashcat';
import { quat, vec3, vec4 } from 'math';
import {
    captureBodyState,
    createStandardWorld,
    createTerrainShape,
    LAYER_MOVING,
    LAYER_STATIC,
    makeRng,
    restoreBodyState,
    TIME_STEP,
    terrainHeight,
} from './common';
import { defineScenario } from './scenario';

// Jolt's PerformanceTest CharacterVirtualScene: kinematic characters roaming a triangle-mesh
// terrain with loose props to bump into. The KCC drives shape casts and collide-shape queries
// against the mesh BVH every step, which is the dominant cost in a real character-driven frame and
// is not touched by any of the rigid-body scenarios.

const CHARACTER_COUNT = 24;
const PROP_COUNT = 40;
const TERRAIN_QUADS = 64;
const TERRAIN_EXTENT = 60;
const ROAM_HALF = 22;
const AGENT_RADIUS = 0.4;
const AGENT_HALF_HEIGHT = 0.5;
const AGENT_SPEED = 3;
const GRAVITY: [number, number, number] = [0, -20, 0];
const RNG_SEED = 0x94d049bb;
const ROAM_SEED = 0x1b873593;

const agentShape: Shape = capsule.create({ halfHeightOfCylinder: AGENT_HALF_HEIGHT, radius: AGENT_RADIUS });
const propShape: Shape = box.create({ halfExtents: [0.3, 0.3, 0.3] });

export const characterTerrain = defineScenario({
    name: 'character-terrain',
    description: 'Jolt PerformanceTest CharacterVirtualScene — 24 KCC agents roaming a triangle mesh',
    steps: 120,
    create() {
        const rng = makeRng(RNG_SEED);
        const world = createStandardWorld(GRAVITY);

        rigidBody.create(world, {
            shape: createTerrainShape(TERRAIN_QUADS, TERRAIN_EXTENT),
            objectLayer: LAYER_STATIC,
            motionType: MotionType.STATIC,
            position: [0, 0, 0],
            friction: 0.5,
            restitution: 0,
        });

        const characterFilter = filter.create(world.settings.layers);
        const updateSettings = kcc.createDefaultUpdateSettings();

        const agents: {
            character: KCC;
            heading: number;
            turnTimer: number;
            spawn: [number, number, number];
            spawnHeading: number;
            spawnTurnTimer: number;
        }[] = [];
        for (let i = 0; i < CHARACTER_COUNT; i++) {
            const x = (rng() * 2 - 1) * ROAM_HALF;
            const z = (rng() * 2 - 1) * ROAM_HALF;
            const character = kcc.create(
                {
                    ...kcc.DEFAULT_KCC_SETTINGS,
                    shape: agentShape,
                    mass: 70,
                    maxSlopeAngle: (45 * Math.PI) / 180,
                    characterPadding: 0.02,
                    innerRigidBody: { shape: agentShape, objectLayer: LAYER_MOVING },
                    supportingVolumePlane: vec4.fromValues(0, 1, 0, -AGENT_RADIUS),
                },
                [x, terrainHeight(x, z) + 0.5, z],
                quat.create(),
            );
            kcc.add(world, character);
            const heading = rng() * Math.PI * 2;
            const turnTimer = rng() * 2;
            agents.push({
                character,
                heading,
                turnTimer,
                spawn: [x, terrainHeight(x, z) + 0.5, z],
                spawnHeading: heading,
                spawnTurnTimer: turnTimer,
            });
        }

        for (let i = 0; i < PROP_COUNT; i++) {
            const x = (rng() * 2 - 1) * ROAM_HALF;
            const z = (rng() * 2 - 1) * ROAM_HALF;
            rigidBody.create(world, {
                shape: propShape,
                objectLayer: LAYER_MOVING,
                motionType: MotionType.DYNAMIC,
                position: [x, terrainHeight(x, z) + 1 + rng() * 2, z],
                mass: 2,
                friction: 0.5,
                restitution: 0,
            });
        }

        const velocity: [number, number, number] = [0, 0, 0];
        const captured = captureBodyState(world);

        // characters roam under a seeded rng, so a window only repeats if the stream restarts too
        let roamRng = makeRng(ROAM_SEED);

        return {
            world,
            reset() {
                restoreBodyState(world, captured);
                roamRng = makeRng(ROAM_SEED);
                for (const agent of agents) {
                    vec3.copy(agent.character.position, agent.spawn);
                    vec3.zero(agent.character.linearVelocity);
                    agent.heading = agent.spawnHeading;
                    agent.turnTimer = agent.spawnTurnTimer;
                }
            },
            step() {
                for (const agent of agents) {
                    const character = agent.character;

                    agent.turnTimer -= TIME_STEP;
                    if (agent.turnTimer <= 0) {
                        agent.heading += (roamRng() - 0.5) * 1.5;
                        agent.turnTimer = 0.5 + roamRng() * 1.5;
                    }
                    // turn back before walking off the edge of the terrain
                    if (Math.abs(character.position[0]) > ROAM_HALF || Math.abs(character.position[2]) > ROAM_HALF) {
                        agent.heading = Math.atan2(-character.position[2], -character.position[0]);
                    }

                    kcc.updateGroundVelocity(world, character);
                    velocity[0] = Math.cos(agent.heading) * AGENT_SPEED;
                    velocity[2] = Math.sin(agent.heading) * AGENT_SPEED;
                    velocity[1] =
                        character.ground.state === kcc.GroundState.ON_GROUND
                            ? 0
                            : character.linearVelocity[1] + GRAVITY[1] * TIME_STEP;
                    vec3.copy(character.linearVelocity, velocity);

                    kcc.update(world, character, TIME_STEP, GRAVITY, updateSettings, undefined, characterFilter);
                }
                updateWorld(world, undefined, TIME_STEP);
            },
        };
    },
});
