import type { Box3, Vec3 } from 'mathcat';
import { box3, vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body';
import type { Bodies } from './body/bodies';
import type { ContactSettings } from './constraints/contact-constraints';
import { createContactSettings } from './constraints/contact-constraints';
import { pool } from './utils/pool';

/** continuous collision detection (CCD) state for one physics step */
export type CCD = {
    /** CCD bodies detected this frame */
    ccdBodies: CCDBody[];
};

/**
 * CCD body data for one frame.
 * Each CCDBody represents a dynamic body that needs continuous collision detection
 * this frame because its movement exceeds the threshold.
 */
export type CCDBody = {
    /** index of body performing CCD (body.index) */
    bodyIndex: number;

    /** index of body we hit (-1 if no hit) */
    hitBodyIndex: number;

    /** desired movement this frame (velocity * dt) */
    deltaPosition: Vec3;

    /** normal at collision point (world space) */
    contactNormal: Vec3;

    /** contact point in world space */
    contactPoint: Vec3;

    /** time of impact [0, 1] where 0 = start, 1 = end */
    fraction: number;

    /** time of impact + penetration allowance */
    fractionPlusSlop: number;

    /** squared threshold for doing CCD (threshold * innerRadius)² */
    linearCastThresholdSq: number;

    /** maximum allowed penetration distance */
    maxPenetration: number;

    /** sub-shape index on body performing CCD */
    subShapeId1: number;

    /** sub-shape index on body we hit */
    subShapeId2: number;

    /** contact settings for this collision (material properties and scales) */
    contactSettings: ContactSettings;
};

function createCCDBody(): CCDBody {
    return {
        bodyIndex: -1,
        hitBodyIndex: -1,
        deltaPosition: vec3.create(),
        contactNormal: vec3.create(),
        contactPoint: vec3.create(),
        fraction: 1.0,
        fractionPlusSlop: 1.0,
        linearCastThresholdSq: 0,
        maxPenetration: 0,
        subShapeId1: 0,
        subShapeId2: 0,
        contactSettings: createContactSettings(),
    };
}

export function resetCCDBody(ccdBody: CCDBody): void {
    ccdBody.bodyIndex = -1;
    ccdBody.hitBodyIndex = -1;
    vec3.zero(ccdBody.deltaPosition);
    vec3.zero(ccdBody.contactNormal);
    vec3.zero(ccdBody.contactPoint);
    ccdBody.fraction = 1.0;
    ccdBody.fractionPlusSlop = 1.0;
    ccdBody.linearCastThresholdSq = 0;
    ccdBody.maxPenetration = 0;
    ccdBody.subShapeId1 = 0;
    ccdBody.subShapeId2 = 0;
    ccdBody.contactSettings.combinedFriction = 0;
    ccdBody.contactSettings.combinedRestitution = 0;
    ccdBody.contactSettings.isSensor = false;
    ccdBody.contactSettings.invMassScale1 = 1.0;
    ccdBody.contactSettings.invMassScale2 = 1.0;
    ccdBody.contactSettings.invInertiaScale1 = 1.0;
    ccdBody.contactSettings.invInertiaScale2 = 1.0;
    vec3.zero(ccdBody.contactSettings.relativeLinearSurfaceVelocity);
    vec3.zero(ccdBody.contactSettings.relativeAngularSurfaceVelocity);
}

export function init(): CCD {
    return {
        ccdBodies: [],
    };
}

export const ccdBodyPool = /* @__PURE__ */ pool(createCCDBody);

/** clear CCD state for a new physics step, clears state from previous frame */
export function clear(state: CCD, bodies: Bodies): void {
    // release ccd bodies
    for (let i = 0; i < state.ccdBodies.length; i++) {
        const ccdBody = state.ccdBodies[i];
        ccdBodyPool.release(ccdBody);
    }

    // reset pointers
    for (let i = 0; i < bodies.pool.length; i++) {
        const body = bodies.pool[i];
        if (body._pooled) continue;
        body.ccdBodyIndex = -1;
    }

    // empty ccd bodies array
    state.ccdBodies.length = 0;
}

const _computeSweptAABB_endAABB = /* @__PURE__ */ box3.create();

/** compute AABB that encompasses both start and end positions (swept AABB) */
export function computeSweptAABB(out: Box3, body: RigidBody, displacement: Vec3): void {
    // start AABB (current body AABB)
    const startAABB = body.aabb;

    // end AABB (body AABB translated by displacement)
    // translate the AABB by adding displacement to both min and max
    _computeSweptAABB_endAABB[0][0] = startAABB[0][0] + displacement[0];
    _computeSweptAABB_endAABB[0][1] = startAABB[0][1] + displacement[1];
    _computeSweptAABB_endAABB[0][2] = startAABB[0][2] + displacement[2];
    _computeSweptAABB_endAABB[1][0] = startAABB[1][0] + displacement[0];
    _computeSweptAABB_endAABB[1][1] = startAABB[1][1] + displacement[1];
    _computeSweptAABB_endAABB[1][2] = startAABB[1][2] + displacement[2];

    // create union of start and end AABBs (swept AABB)
    box3.union(out, startAABB, _computeSweptAABB_endAABB);
}
