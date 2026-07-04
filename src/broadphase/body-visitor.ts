import type { RigidBody } from '../body/rigid-body';

/** type for a broadphase body visitor */
export type BodyVisitor = {
    /** whether the broadphase traversal should exit early */
    shouldExit: boolean;
    /**
     * closest-hit fraction found so far, in [0, 1] of the ray length (jolt's GetEarlyOutFraction).
     * castRay prunes any node whose fat-AABB entry fraction is >= this. Optional — visitors that
     * don't cast (intersectAABB etc.) omit it, and castRay then falls back to no distance pruning.
     */
    earlyOutFraction?: number;
    /** called when visiting a body */
    visit(body: RigidBody): void;
};
