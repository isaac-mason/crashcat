import type { ConstraintId } from './constraint-id';
import { INVALID_CONSTRAINT_ID } from './constraint-id';

/** constraint space enum - where are constraint points specified */
export enum ConstraintSpace {
    /** points specified in world space */
    WORLD = 0,
    /** points specified relative to body */
    LOCAL = 1,
}

/** base constraint fields shared by all constraint types */
export type ConstraintBase = {
    /** @internal whether this constraint is currently pooled, in which case it should be ignored */
    _pooled: boolean;

    /** @internal whether both bodies are sleeping (set each frame before constraint solving) */
    _sleeping: boolean;

    /** unique constraint identifier */
    id: ConstraintId;

    /** index from constraint ID */
    index: number;

    /** sequence number from constraint ID */
    sequence: number;

    /** whether constraint is enabled */
    enabled: boolean;

    /** constraint priority (higher = solved first) */
    constraintPriority: number;

    /** velocity iteration override (0 = use default) */
    numVelocityStepsOverride: number;

    /** position iteration override (0 = use default) */
    numPositionStepsOverride: number;

    /** user data */
    userData: bigint;

    /** index of first body */
    bodyIndexA: number;

    /** index of second body */
    bodyIndexB: number;
};

/** helper to remove a constraint ID from a body's constraintIds array */
export function removeConstraintIdFromBody(body: { constraintIds: ConstraintId[] }, constraintId: ConstraintId): void {
    const idx = body.constraintIds.indexOf(constraintId);
    if (idx !== -1) {
        // swap-remove for O(1)
        const last = body.constraintIds.length - 1;
        if (idx !== last) {
            body.constraintIds[idx] = body.constraintIds[last];
        }
        body.constraintIds.pop();
    }
}

/** default constraint base fields */
export function makeConstraintBase(): ConstraintBase {
    return {
        _pooled: true,
        id: INVALID_CONSTRAINT_ID,
        index: -1,
        sequence: -1,
        enabled: true,
        constraintPriority: 0,
        numVelocityStepsOverride: 0,
        _sleeping: false,
        numPositionStepsOverride: 0,
        userData: 0n,
        bodyIndexA: -1,
        bodyIndexB: -1,
    };
}
