import {
    type BodyVisitor,
    CastRayStatus,
    CastShapeStatus,
    MotionType,
    type RigidBody,
    type World,
    box,
    broadphase,
    capsule,
    castRay,
    castRayVsShape,
    castShape,
    castShapeVsShape,
    collidePoint,
    collidePointVsShape,
    collideShape,
    collideShapeVsShape,
    createAllCastRayCollector,
    createAllCastShapeCollector,
    createAllCollidePointCollector,
    createAllCollideShapeCollector,
    createAnyCastRayCollector,
    createAnyCastShapeCollector,
    createAnyCollidePointCollector,
    createAnyCollideShapeCollector,
    createClosestCastRayCollector,
    createClosestCastShapeCollector,
    createDefaultCastRaySettings,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    filter,
    rigidBody,
    sphere,
} from 'crashcat';
import type { Box3, Vec3 } from 'mathcat';
import { quat, raycast3, vec3 } from 'mathcat';

declare const world: World;
declare const OBJECT_LAYER_DEBRIS: number;
declare const OBJECT_LAYER_MOVING: number;
declare const BROADPHASE_LAYER_MOVING: number;
declare const BROADPHASE_LAYER_NOT_MOVING: number;
declare const playerId: number;

/* SNIPPET_START: cast-ray */
// cast a ray through the world to find bodies
const rayOrigin = vec3.fromValues(0, 5, 0);
const rayDirection = vec3.fromValues(0, -1, 0);
const rayLength = 100;

// create a filter to control what the ray can hit
const queryFilter = filter.create(world.settings.layers);

// closest: finds the nearest hit along the ray
const closestCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();
castRay(world, closestCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

if (closestCollector.hit.status === CastRayStatus.COLLIDING) {
    const hitDistance = closestCollector.hit.fraction * rayLength;
    const hitPoint = vec3.create();
    vec3.scaleAndAdd(hitPoint, rayOrigin, rayDirection, hitDistance);
    console.log('closest hit at', hitPoint);
}

// any: finds the first hit (fast early-out, useful for line-of-sight checks)
const anyCollector = createAnyCastRayCollector();
anyCollector.reset();
castRay(world, anyCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

if (anyCollector.hit.status === CastRayStatus.COLLIDING) {
    console.log('ray hit something');
}

// all: collects every hit along the ray
const allCollector = createAllCastRayCollector();
allCollector.reset();
castRay(world, allCollector, raySettings, rayOrigin, rayDirection, rayLength, queryFilter);

for (const hit of allCollector.hits) {
    if (hit.status === CastRayStatus.COLLIDING) {
        console.log('hit at fraction', hit.fraction);
    }
}
/* SNIPPET_END: cast-ray */

/* SNIPPET_START: cast-shape */
// sweep a shape through the world (useful for character movement, projectiles)
const castPosition = vec3.fromValues(0, 5, 0);
const castQuaternion = quat.create();
const castScale = vec3.fromValues(1, 1, 1);
const castDisplacement = vec3.fromValues(0, -10, 0);

const sweepShape = sphere.create({ radius: 0.5 });

// closest: finds the nearest hit along the sweep
const closestShapeCollector = createClosestCastShapeCollector();
const shapeSettings = createDefaultCastShapeSettings();
castShape(world, closestShapeCollector, shapeSettings, sweepShape, castPosition, castQuaternion, castScale, castDisplacement, queryFilter);

if (closestShapeCollector.hit.status === CastShapeStatus.COLLIDING) {
    const hitFraction = closestShapeCollector.hit.fraction;
    const hitPosition = vec3.create();
    vec3.scaleAndAdd(hitPosition, castPosition, castDisplacement, hitFraction);
    console.log('shape hit at', hitPosition);
}

// any: finds the first hit (fast early-out)
const anyShapeCollector = createAnyCastShapeCollector();
anyShapeCollector.reset();
castShape(world, anyShapeCollector, shapeSettings, sweepShape, castPosition, castQuaternion, castScale, castDisplacement, queryFilter);

if (anyShapeCollector.hit.status === CastShapeStatus.COLLIDING) {
    console.log('shape hit something');
}

// all: collects every hit along the sweep
const allShapeCollector = createAllCastShapeCollector();
allShapeCollector.reset();
castShape(world, allShapeCollector, shapeSettings, sweepShape, castPosition, castQuaternion, castScale, castDisplacement, queryFilter);

for (const hit of allShapeCollector.hits) {
    if (hit.status === CastShapeStatus.COLLIDING) {
        console.log('shape hit at fraction', hit.fraction);
    }
}
/* SNIPPET_END: cast-shape */

/* SNIPPET_START: collide-point */
// test if a point is inside any bodies (useful for triggers, pickups)
const point = vec3.fromValues(0, 2, 0);

// any: checks if the point is inside any body (fast early-out)
const anyPointCollector = createAnyCollidePointCollector();
const pointSettings = createDefaultCollidePointSettings();
collidePoint(world, anyPointCollector, pointSettings, point, queryFilter);

if (anyPointCollector.hit !== null) {
    console.log('point is inside body', anyPointCollector.hit.bodyIdB);
}

// all: finds every body containing the point
const allPointCollector = createAllCollidePointCollector();
allPointCollector.reset();
collidePoint(world, allPointCollector, pointSettings, point, queryFilter);

console.log('point is inside', allPointCollector.hits.length, 'bodies');
/* SNIPPET_END: collide-point */

/* SNIPPET_START: collide-shape */
// test if a shape overlaps any bodies (useful for area triggers, placement validation)
const queryShape = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });
const queryPosition = vec3.fromValues(0, 2, 0);
const queryQuaternion = quat.create();
const queryScale = vec3.fromValues(1, 1, 1);

// any: checks if the shape overlaps any body (fast early-out)
const anyShapeOverlapCollector = createAnyCollideShapeCollector();
const shapeOverlapSettings = createDefaultCollideShapeSettings();
collideShape(world, anyShapeOverlapCollector, shapeOverlapSettings, queryShape, queryPosition, queryQuaternion, queryScale, queryFilter);

if (anyShapeOverlapCollector.hit !== null) {
    console.log('shape overlaps body', anyShapeOverlapCollector.hit.bodyIdB);
}

// all: finds every body overlapping the shape
const allShapeOverlapCollector = createAllCollideShapeCollector();
allShapeOverlapCollector.reset();
collideShape(world, allShapeOverlapCollector, shapeOverlapSettings, queryShape, queryPosition, queryQuaternion, queryScale, queryFilter);

console.log('shape overlaps', allShapeOverlapCollector.hits.length, 'bodies');
/* SNIPPET_END: collide-shape */

/* SNIPPET_START: broadphase-query */
// for advanced scenarios: query the broadphase spatial acceleration structure directly
// useful when you need custom traversal logic or want to avoid narrowphase overhead

// intersectAABB: find all bodies whose AABBs overlap a box
const queryAABB: Box3 = [vec3.fromValues(-5, -5, -5), vec3.fromValues(5, 5, 5)];

const aabbVisitor: BodyVisitor = {
    shouldExit: false,
    visit(body: RigidBody) {
        console.log('body AABB overlaps query AABB:', body.id);
        // set shouldExit = true to stop traversal early
    },
};

broadphase.intersectAABB(world, queryAABB, queryFilter, aabbVisitor);

// intersectPoint: find all bodies whose AABBs contain a point
const queryPoint = vec3.fromValues(0, 5, 0);

const pointVisitor: BodyVisitor = {
    shouldExit: false,
    visit(body: RigidBody) {
        console.log('body AABB contains point:', body.id);
    },
};

broadphase.intersectPoint(world, queryPoint, queryFilter, pointVisitor);
/* SNIPPET_END: broadphase-query */

/* SNIPPET_START: shape-vs-shape */
// for advanced scenarios: query shape-vs-shape directly without the world
// useful for custom spatial queries, editor tools, or when you manage bodies yourself

const shapeA = capsule.create({ radius: 0.5, halfHeightOfCylinder: 1 });
const shapeB = box.create({ halfExtents: vec3.fromValues(1, 1, 1) });

const posA = vec3.fromValues(0, 0, 0);
const quatA = quat.create();
const scaleA = vec3.fromValues(1, 1, 1);

const posB = vec3.fromValues(2, 0, 0);
const quatB = quat.create();
const scaleB = vec3.fromValues(1, 1, 1);

// castRayVsShape: cast a ray against a specific shape
const ray = raycast3.create();
vec3.set(ray.origin, -5, 0, 0);
vec3.set(ray.direction, 1, 0, 0);
ray.length = 10;

const rayVsShapeCollector = createClosestCastRayCollector();
const rayVsShapeSettings = createDefaultCastRaySettings();

// biome-ignore format: pretty
castRayVsShape(
    rayVsShapeCollector,
    rayVsShapeSettings,
    ray,
    // shape b
    shapeB,
    0, 0, // subShapeId, subShapeIdBits
    posB[0], posB[1], posB[2],
    quatB[0], quatB[1], quatB[2], quatB[3],
    scaleB[0], scaleB[1], scaleB[2],
);

// collidePointVsShape: test if a point is inside a specific shape
const testPoint = vec3.fromValues(0.5, 0, 0);

const pointVsShapeCollector = createAnyCollidePointCollector();
const pointVsShapeSettings = createDefaultCollidePointSettings();

// biome-ignore format: pretty
collidePointVsShape(
    pointVsShapeCollector,
    pointVsShapeSettings,
    // point
    testPoint[0], testPoint[1], testPoint[2],
    // shape b
    shapeB,
    0, 0, // subShapeId, subShapeIdBits
    posB[0], posB[1], posB[2],
    quatB[0], quatB[1], quatB[2], quatB[3],
    scaleB[0], scaleB[1], scaleB[2],
);

// castShapeVsShape: sweep one shape against another
const displacement = vec3.fromValues(5, 0, 0);

const castShapeVsShapeCollector = createClosestCastShapeCollector();
const castShapeVsShapeSettings = createDefaultCastShapeSettings();

// biome-ignore format: pretty
castShapeVsShape(
    castShapeVsShapeCollector,
    castShapeVsShapeSettings,
    // shape a
    shapeA,
    0, 0, // subShapeIdA, subShapeIdBitsA
    posA[0], posA[1], posA[2],
    quatA[0], quatA[1], quatA[2], quatA[3],
    scaleA[0], scaleA[1], scaleA[2],
    displacement[0], displacement[1], displacement[2],
    // shape b
    shapeB,
    0, 0, // subShapeIdB, subShapeIdBitsB
    posB[0], posB[1], posB[2],
    quatB[0], quatB[1], quatB[2], quatB[3],
    scaleB[0], scaleB[1], scaleB[2],
);

// collideShapeVsShape: test if two shapes overlap
const collideShapeVsShapeCollector = createAllCollideShapeCollector();
const collideShapeVsShapeSettings = createDefaultCollideShapeSettings();

// biome-ignore format: pretty
collideShapeVsShape(
    collideShapeVsShapeCollector,
    collideShapeVsShapeSettings,
    // shape a
    shapeA,
    0, 0, // subShapeIdA, subShapeIdBitsA
    posA[0], posA[1], posA[2],
    quatA[0], quatA[1], quatA[2], quatA[3],
    scaleA[0], scaleA[1], scaleA[2],
    // shape b
    shapeB,
    0, 0, // subShapeIdB, subShapeIdBitsB
    posB[0], posB[1], posB[2],
    quatB[0], quatB[1], quatB[2], quatB[3],
    scaleB[0], scaleB[1], scaleB[2],
);
/* SNIPPET_END: shape-vs-shape */

/* SNIPPET_START: filter */
// filters control what queries can hit, using object layers and collision groups/masks

// basic: create a filter with all layers enabled
const worldQueryFilter = filter.create(world.settings.layers);

// filter specific object layers
filter.disableObjectLayer(worldQueryFilter, world.settings.layers, OBJECT_LAYER_DEBRIS);
filter.enableObjectLayer(worldQueryFilter, world.settings.layers, OBJECT_LAYER_MOVING);

// filter specific broadphase layers
filter.disableBroadphaseLayer(worldQueryFilter, world.settings.layers, BROADPHASE_LAYER_MOVING);
filter.enableBroadphaseLayer(worldQueryFilter, world.settings.layers, BROADPHASE_LAYER_NOT_MOVING);

// filter everything, then selectively enable
filter.disableAllLayers(worldQueryFilter, world.settings.layers);
filter.enableObjectLayer(worldQueryFilter, world.settings.layers, OBJECT_LAYER_MOVING);

// collision groups and masks (works alongside layer filtering)
worldQueryFilter.collisionGroup = 0b0001; // query belongs to group 1
worldQueryFilter.collisionMask = 0b0010 | 0b0100; // query hits groups 2 and 4

// custom body filter callback
worldQueryFilter.bodyFilter = (body: RigidBody) => {
    // custom logic - exclude specific bodies
    if (body.userData === 'player') return false;
    
    // only hit dynamic bodies
    if (body.motionType !== MotionType.DYNAMIC) return false;
    
    return true;
};

// setFromBody: configure filter to match what a body can collide with
const playerBody = rigidBody.get(world, playerId)!;
filter.setFromBody(worldQueryFilter, world.settings.layers, playerBody);

// copy filter settings
const rayFilter = filter.create(world.settings.layers);
filter.copy(rayFilter, worldQueryFilter);

// use filter in queries
castRay(world, closestCollector, raySettings, rayOrigin, rayDirection, rayLength, worldQueryFilter);
/* SNIPPET_END: filter */
