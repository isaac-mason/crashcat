import {
    addBroadphaseLayer,
    addObjectLayer,
    createWorld,
    createWorldSettings,
    enableCollision,
    MotionType,
    plane,
    registerAll,
    rigidBody,
    type Shape,
    triangleMesh,
    type World,
} from 'crashcat';
import type { Vec3 } from 'math';

registerAll();

/** object layer for static geometry — ground, terrain, walls */
export const LAYER_STATIC = 0;
/** object layer for everything that moves */
export const LAYER_MOVING = 1;

export const TIME_STEP = 1 / 60;

/**
 * a world with the two object layers every scenario uses. static-vs-static is left disabled so
 * the sea-of-static-boxes field costs what it would in a game.
 */
export function createStandardWorld(gravity: Vec3 = [0, -9.81, 0]): World {
    const settings = createWorldSettings();
    const broadphaseStatic = addBroadphaseLayer(settings);
    const broadphaseMoving = addBroadphaseLayer(settings);
    const layerStatic = addObjectLayer(settings, broadphaseStatic);
    const layerMoving = addObjectLayer(settings, broadphaseMoving);
    enableCollision(settings, layerMoving, layerMoving);
    enableCollision(settings, layerMoving, layerStatic);
    settings.gravity = gravity;
    if (layerStatic !== LAYER_STATIC || layerMoving !== LAYER_MOVING) {
        throw new Error('layer ids drifted from the LAYER_STATIC/LAYER_MOVING constants');
    }
    return createWorld(settings);
}

/** mulberry32 — the same generator every scenario seeds, so runs replay exactly */
export function makeRng(seed: number): () => number {
    let state = seed >>> 0;
    return () => {
        state = (state + 0x6d2b79f5) >>> 0;
        let t = state;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
}

export function addGroundPlane(world: World, halfExtent = 200, friction = 0.5): void {
    rigidBody.create(world, {
        shape: plane.create({ plane: { normal: [0, 1, 0], constant: 0 }, halfExtent }),
        objectLayer: LAYER_STATIC,
        motionType: MotionType.STATIC,
        position: [0, 0, 0],
        friction,
        restitution: 0,
    });
}

/** smooth rolling terrain, amplitudes chosen to sum to +/- 2m */
export function terrainHeight(x: number, z: number): number {
    return (
        Math.sin(x * 0.18) * Math.cos(z * 0.18) * 1.2 +
        Math.sin(x * 0.42 + 1.0) * Math.sin(z * 0.37) * 0.5 +
        Math.cos((x + z) * 0.11) * 0.3
    );
}

export function createTerrainShape(quads: number, extent: number): Shape {
    const cell = extent / quads;
    const half = extent / 2;
    const row = quads + 1;
    const positions: number[] = [];
    for (let iz = 0; iz < row; iz++) {
        for (let ix = 0; ix < row; ix++) {
            const x = ix * cell - half;
            const z = iz * cell - half;
            positions.push(x, terrainHeight(x, z), z);
        }
    }
    const indices: number[] = [];
    for (let iz = 0; iz < quads; iz++) {
        for (let ix = 0; ix < quads; ix++) {
            const bottomLeft = iz * row + ix;
            const bottomRight = bottomLeft + 1;
            const topLeft = bottomLeft + row;
            const topRight = topLeft + 1;
            indices.push(bottomLeft, topLeft, bottomRight);
            indices.push(bottomRight, topLeft, topRight);
        }
    }
    return triangleMesh.create({ positions, indices });
}

/** a lumpy convex blob — points on a jittered sphere, which is what PEEL's convex tests use */
export function convexBlobPositions(pointCount: number, radius: number, rng: () => number): number[] {
    const positions: number[] = [];
    for (let i = 0; i < pointCount; i++) {
        const theta = rng() * Math.PI * 2;
        const phi = Math.acos(2 * rng() - 1);
        const r = radius * (0.7 + rng() * 0.3);
        positions.push(r * Math.sin(phi) * Math.cos(theta), r * Math.cos(phi), r * Math.sin(phi) * Math.sin(theta));
    }
    return positions;
}
