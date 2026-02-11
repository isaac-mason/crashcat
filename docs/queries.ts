import {
    type BodyVisitor,
    box,
    broadphase,
    CastRayStatus,
    CastShapeStatus,
    castRay,
    castShape,
    collidePoint,
    collideShape,
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
    MotionType,
    type RigidBody,
    rigidBody,
    sphere,
    type World,
} from 'crashcat';
import type { Box3 } from 'mathcat';
import { quat, vec3 } from 'mathcat';

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
castShape(
    world,
    closestShapeCollector,
    shapeSettings,
    sweepShape,
    castPosition,
    castQuaternion,
    castScale,
    castDisplacement,
    queryFilter,
);

if (closestShapeCollector.hit.status === CastShapeStatus.COLLIDING) {
    const hitFraction = closestShapeCollector.hit.fraction;
    const hitPosition = vec3.create();
    vec3.scaleAndAdd(hitPosition, castPosition, castDisplacement, hitFraction);
    console.log('shape hit at', hitPosition);
}

// any: finds the first hit (fast early-out)
const anyShapeCollector = createAnyCastShapeCollector();
anyShapeCollector.reset();
castShape(
    world,
    anyShapeCollector,
    shapeSettings,
    sweepShape,
    castPosition,
    castQuaternion,
    castScale,
    castDisplacement,
    queryFilter,
);

if (anyShapeCollector.hit.status === CastShapeStatus.COLLIDING) {
    console.log('shape hit something');
}

// all: collects every hit along the sweep
const allShapeCollector = createAllCastShapeCollector();
allShapeCollector.reset();
castShape(
    world,
    allShapeCollector,
    shapeSettings,
    sweepShape,
    castPosition,
    castQuaternion,
    castScale,
    castDisplacement,
    queryFilter,
);

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
collideShape(
    world,
    anyShapeOverlapCollector,
    shapeOverlapSettings,
    queryShape,
    queryPosition,
    queryQuaternion,
    queryScale,
    queryFilter,
);

if (anyShapeOverlapCollector.hit !== null) {
    console.log('shape overlaps body', anyShapeOverlapCollector.hit.bodyIdB);
}

// all: finds every body overlapping the shape
const allShapeOverlapCollector = createAllCollideShapeCollector();
allShapeOverlapCollector.reset();
collideShape(
    world,
    allShapeOverlapCollector,
    shapeOverlapSettings,
    queryShape,
    queryPosition,
    queryQuaternion,
    queryScale,
    queryFilter,
);

console.log('shape overlaps', allShapeOverlapCollector.hits.length, 'bodies');
/* SNIPPET_END: collide-shape */

/* SNIPPET_START: broadphase-query */
// for advanced scenarios: query the broadphase spatial acceleration structure directly
// useful when you need custom traversal logic or want to avoid narrowphase overhead

// intersectAABB: find all bodies whose AABBs overlap a box
const queryAABB: Box3 = [
    [-5, -5, -5],
    [5, 5, 5],
];

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
worldQueryFilter.collisionGroups = 0b0001; // query belongs to group 1
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
