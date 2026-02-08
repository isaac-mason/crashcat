import { type Box3, box3, type Vec3 } from 'mathcat';
import { MotionType } from '../body/motion-type';
import type { RigidBody } from '../body/rigid-body';
import type { Filter } from '../filter';
import * as filter from '../filter';
import {
    broadphaseLayerCollidesWithBroadphaseLayer,
    type Layers,
    objectLayerCollidesWithBroadphaseLayer,
} from '../layers';
import type { Listener } from '../listener';
import { assert } from '../utils/assert';
import type { World } from '../world';
import type { BodyVisitor } from './body-visitor';
import * as dbvt from './dbvt';
import { INACTIVE_BODY_INDEX } from '../body/sleep';

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

/**
 * Determines if a body will query the broadphase during findCollidingPairs.
 * Bodies that don't query rely on other bodies to find collision pairs with them.
 *
 * Static bodies never query (they are found by active bodies).
 * Sleeping bodies never query (they are found by active bodies, which wakes them).
 */
function shouldQueryBroadphase(body: RigidBody): boolean {
    // static bodies never query - they are found by active bodies
    if (body.motionType === MotionType.STATIC) {
        return false;
    }
    // sleeping bodies never query - active bodies will find them and wake them
    if (body.sleeping) {
        return false;
    }
    // all active dynamic and kinematic bodies query
    return true;
}

/** gets a deduplication index for skipping processing of duplicate pairs */
function getDeduplicationIndex(body: RigidBody): number {
    // bodies that don't query get max value - this ensures querying bodies always report pairs with them
    if (!shouldQueryBroadphase(body)) {
        return INACTIVE_BODY_INDEX;
    }

    // querying bodies use their array index for deduplication between themselves
    return body.index;
}

/** visitor for finding colliding body pairs, does pair deduplication and motion type filtering */
class CollisionBodyPairVisitor implements BodyVisitor {
    shouldExit = false;

    activeBody: RigidBody = null!;
    pairs: Pairs = null!;
    listener: Listener | undefined = undefined;

    setup(activeBody: RigidBody, pairs: Pairs, listener: Listener | undefined): void {
        this.activeBody = activeBody;
        this.pairs = pairs;
        this.listener = listener;
        this.shouldExit = false;
    }

    visit(otherBody: RigidBody): void {
        // avoid self-collision
        if (this.activeBody.id === otherBody.id) return;

        // avoid duplicate pairs
        if (getDeduplicationIndex(this.activeBody) >= getDeduplicationIndex(otherBody)) return;

        // motion type filtering: static-static never collide
        const motionTypeA = this.activeBody.motionType;
        const motionTypeB = otherBody.motionType;

        if (motionTypeA === MotionType.STATIC && motionTypeB === MotionType.STATIC) {
            return;
        }

        // kinematic-kinematic and kinematic-static pairs require opt-in, EXCEPT:
        // - a kinematic body can always collide with a sensor
        if (
            !this.activeBody.collideKinematicVsNonDynamic &&
            !otherBody.collideKinematicVsNonDynamic &&
            motionTypeA !== MotionType.DYNAMIC &&
            motionTypeB !== MotionType.DYNAMIC &&
            !(motionTypeA === MotionType.KINEMATIC && otherBody.sensor) &&
            !(motionTypeB === MotionType.KINEMATIC && this.activeBody.sensor)
        ) {
            return;
        }

        // user body pair filter - called after all built-in checks pass
        if (this.listener?.onBodyPairValidate) {
            // sort bodies for consistent ordering (higher motion type first, then by id)
            let bodyA = this.activeBody;
            let bodyB = otherBody;
            if (bodyA.motionType > bodyB.motionType || (bodyA.motionType === bodyB.motionType && bodyB.id < bodyA.id)) {
                bodyA = otherBody;
                bodyB = this.activeBody;
            }
            if (!this.listener.onBodyPairValidate(bodyA, bodyB)) {
                return;
            }
        }

        // report pair by writing body indices to flat array
        const pairIndex = this.pairs.n * 2;

        // grow array if necessary by pushing
        if (pairIndex >= this.pairs.pool.length) {
            this.pairs.pool.push(this.activeBody.index, otherBody.index);
        } else {
            // write to existing slots
            this.pairs.pool[pairIndex] = this.activeBody.index;
            this.pairs.pool[pairIndex + 1] = otherBody.index;
        }

        this.pairs.n++;
    }
}

const _collisionBodyPairVisitor = /* @__PURE__ */ new CollisionBodyPairVisitor();

/** initializes broadphase state */
export function init(layers: Layers): Broadphase {
    const numBroadphaseLayers = layers.broadphaseLayers;
    const dbvts: dbvt.DBVT[] = [];

    // create one DBVH per broadphase layer
    for (let i = 0; i < numBroadphaseLayers; i++) {
        dbvts.push(
            // future: options?
            dbvt.create(),
        );
    }

    return {
        dbvts,
        pairs: { pool: [], n: 0 },
    };
}

/** adds a body to the broadphase */
export function addBody(broadphase: Broadphase, body: RigidBody, layers: Layers): void {
    const objectLayer = body.objectLayer;

    // map object layer to broadphase layer
    const broadphaseLayer = layers.objectLayerToBroadphaseLayer[objectLayer];

    if (broadphaseLayer === undefined) {
        assert(false, `Object layer ${objectLayer} not mapped to broadphase layer`);
        return;
    }

    // get the dbvt for this broadphase layer
    const tree = broadphase.dbvts[broadphaseLayer];

    // insert into dbvt
    const node = dbvt.add(tree, body);

    // store broadphase layer and dbvt node on the body
    body.broadphaseLayer = broadphaseLayer;
    body.dbvtNode = node;
}

/** removes a body from the broadphase */
export function removeBody(broadphase: Broadphase, body: RigidBody): void {
    if (body.broadphaseLayer === -1) return;

    const tree = broadphase.dbvts[body.broadphaseLayer];

    if (body.dbvtNode !== -1) {
        dbvt.remove(tree, body);
    }

    body.broadphaseLayer = -1;
    body.dbvtNode = -1;
}

/** updates a body's AABB in the broadphase */
export function updateBody(broadphase: Broadphase, body: RigidBody): void {
    if (body.dbvtNode === -1 || body.broadphaseLayer === -1) return;
    const tree = broadphase.dbvts[body.broadphaseLayer];

    dbvt.update(tree, body, -1); // -1 = use tree root for reinsertion
}

/** removes and re-adds a body in the broadphase when its layer changes */
export function reinsertBody(broadphase: Broadphase, body: RigidBody, layers: Layers): void {
    removeBody(broadphase, body);
    addBody(broadphase, body, layers);
}

const _findCollidingPairs_filter = /* @__PURE__ */ filter.createEmpty();
const _findCollidingPairs_expandedAABB = /* @__PURE__ */ box3.create();

/** find potentially colliding body pairs, updates broadphase.pairs */
export function findCollidingPairs(world: World, speculativeContactDistance: number, listener: Listener | undefined): void {
    const layers = world.settings.layers;
    const broadphase = world.broadphase;

    // reset pair count
    broadphase.pairs.n = 0;

    // optimize trees incrementally (matching Bullet's btDbvtBroadphase::collide)
    // bullet default: 1 + (m_leaves * m_dupdates / 100), where m_dupdates = 0
    // this means minimum 1 node per frame is optimized even with 0% setting
    for (let i = 0; i < broadphase.dbvts.length; i++) {
        const tree = broadphase.dbvts[i];
        // optimize 1% of tree nodes per frame (minimum 1)
        const passes = Math.max(1, Math.floor((tree.nodes.length - tree.freeNodeIndices.length) * 0.01));
        dbvt.optimizeIncremental(tree, passes);
    }

    // sort bodies by broadphase layer for cache efficiency
    const sortedBodies = [...world.bodies.pool].sort((a, b) => {
        const layerA = layers.objectLayerToBroadphaseLayer[a.objectLayer];
        const layerB = layers.objectLayerToBroadphaseLayer[b.objectLayer];
        return layerA - layerB;
    });

    for (const body of sortedBodies) {
        // skip pooled bodies and bodies that don't query the broadphase
        if (body._pooled || !shouldQueryBroadphase(body)) continue;

        const activeObjectLayer = body.objectLayer;
        const activeBroadphaseLayer = layers.objectLayerToBroadphaseLayer[activeObjectLayer];

        // expand AABB by speculative contact distance
        box3.expandByMargin(_findCollidingPairs_expandedAABB, body.aabb, speculativeContactDistance);

        // determine which broadphase layers this body can collide with
        for (let otherBroadphaseLayer = 0; otherBroadphaseLayer < broadphase.dbvts.length; otherBroadphaseLayer++) {
            // check if active body's object layer can collide with this broadphase layer
            if (!objectLayerCollidesWithBroadphaseLayer(layers, activeObjectLayer, otherBroadphaseLayer)) {
                continue;
            }

            // check if the two broadphase layers can collide with each other
            if (!broadphaseLayerCollidesWithBroadphaseLayer(layers, activeBroadphaseLayer, otherBroadphaseLayer)) {
                continue;
            }

            // query this broadphase layer's dynamic bounding volume tree
            const tree = broadphase.dbvts[otherBroadphaseLayer];

            // setup filter based on active body's collision properties
            // this enables only object layers that can collide with the active body
            filter.setFromBody(_findCollidingPairs_filter, layers, body);

            // setup body pair visitor for this query
            _collisionBodyPairVisitor.setup(body, broadphase.pairs, listener);

            // query bvh with filter
            dbvt.intersectAABB(world, tree, _findCollidingPairs_expandedAABB, _findCollidingPairs_filter, _collisionBodyPairVisitor);
        }
    }
}

/** finds bodies with AABBs that intersect the given ray */
export function castRay(
    world: World,
    origin: Vec3,
    direction: Vec3,
    length: number,
    queryFilter: Filter,
    visitor: BodyVisitor,
): void {
    // query each broadphase layer that passes the filter
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.castRay(world, tree, origin, direction, length, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** finds bodies with AABBs that intersect the given AABB */
export function intersectAABB(world: World, aabb: Box3, queryFilter: Filter, visitor: BodyVisitor): void {
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.intersectAABB(world, tree, aabb, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** finds bodies with AABBs that contain the given point */
export function intersectPoint(world: World, point: Vec3, queryFilter: Filter, visitor: BodyVisitor): void {
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.intersectPoint(world, tree, point, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** finds bodies with AABBs that intersect the given swept AABB */
export function castAABB(
    world: World,
    bounds: Box3,
    displacement: Vec3,
    queryFilter: Filter,
    visitor: BodyVisitor,
): void {
    for (let broadphaseLayer = 0; broadphaseLayer < world.broadphase.dbvts.length; broadphaseLayer++) {
        if (!filter.filterBroadphaseLayer(queryFilter, broadphaseLayer)) continue;

        const tree = world.broadphase.dbvts[broadphaseLayer];

        dbvt.castAABB(world, tree, bounds, displacement, queryFilter, visitor);

        if (visitor.shouldExit) break;
    }
}

/** get the bounds of all DBVTs in the broadphase */
export function bounds(out: Box3, broadphase: Broadphase): Box3 {
    box3.empty(out);

    for (const tree of broadphase.dbvts) {
        if (tree.root === -1) continue;

        const rootNode = tree.nodes[tree.root];
        box3.union(out, out, rootNode.aabb);
    }

    return out;
}
