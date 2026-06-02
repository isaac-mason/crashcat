import { type Box3, type Vec3 } from 'mathcat';
import type { RigidBody } from '../body/rigid-body.js';
import type { Filter } from '../filter.js';
import { type Layers } from '../layers.js';
import type { Listener } from '../listener.js';
import type { World } from '../world.js';
import type { BodyVisitor } from './body-visitor.js';
import * as dbvt from './dbvt.js';
/** broadphase state for a physics world */
export type Broadphase = {
    /** dynamic bounding volume trees, one per broadphase layer */
    dbvts: dbvt.DBVT[];
    /** pooled pairs storage */
    pairs: Pairs;
};
/** a body pair */
export type BodyPair = [a: RigidBody, b: RigidBody];
/** pooled pairs storage [bodyIndexA, bodyIndexB, ...] */
export type Pairs = {
    /** flat array of body indices */
    pool: number[];
    /** number of valid pairs */
    n: number;
};
/** initializes broadphase state */
export declare function init(layers: Layers): Broadphase;
/** adds a body to the broadphase */
export declare function addBody(broadphase: Broadphase, body: RigidBody, layers: Layers): void;
/** removes a body from the broadphase */
export declare function removeBody(broadphase: Broadphase, body: RigidBody): void;
/** updates a body's AABB in the broadphase */
export declare function updateBody(broadphase: Broadphase, body: RigidBody): void;
/** removes and re-adds a body in the broadphase when its layer changes */
export declare function reinsertBody(broadphase: Broadphase, body: RigidBody, layers: Layers): void;
/** find potentially colliding body pairs, updates broadphase.pairs */
export declare function findCollidingPairs(world: World, speculativeContactDistance: number, listener: Listener | undefined): void;
/** finds bodies with AABBs that intersect the given ray */
export declare function castRay(world: World, origin: Vec3, direction: Vec3, length: number, queryFilter: Filter, visitor: BodyVisitor): void;
/** finds bodies with AABBs that intersect the given AABB */
export declare function intersectAABB(world: World, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void;
/** finds bodies with AABBs that contain the given point */
export declare function intersectPoint(world: World, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void;
/** finds bodies with AABBs that intersect the given swept AABB */
export declare function castAABB(world: World, bounds: Box3, displacement: Vec3, queryFilter: Filter, visitor: BodyVisitor): void;
/** get the bounds of all DBVTs in the broadphase */
export declare function bounds(out: Box3, broadphase: Broadphase): Box3;
