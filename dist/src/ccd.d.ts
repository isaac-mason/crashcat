import type { Vec3 } from 'math';
import type { Box3 } from 'math/shapes';
import type { Bodies } from './body/bodies.js';
import type { RigidBody } from './body/rigid-body.js';
import type { ContactSettings } from './constraints/contact-constraints.js';
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
export declare function resetCCDBody(ccdBody: CCDBody): void;
export declare function init(): CCD;
export declare const ccdBodyPool: {
    request: () => CCDBody;
    release: (item: CCDBody) => void;
    reset: () => void;
};
/** clear CCD state for a new physics step, clears state from previous frame */
export declare function clear(state: CCD, bodies: Bodies): void;
/** compute AABB that encompasses both start and end positions (swept AABB) */
export declare function computeSweptAABB(out: Box3, body: RigidBody, displacement: Vec3): void;
