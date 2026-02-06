import type { RigidBody } from './rigid-body';

/** physics world bodies state */
export type Bodies = {
    /** pool of bodies in the world */
    pool: RigidBody[];
    /** pool of freed body indices for reuse */
    freeIndices: number[];
    /** next body sequence number */
    nextSequence: number;
    /** array of active body indices (body.index values), maintained incrementally via swap-remove */
    activeBodyIndices: number[];
    /** current count of active bodies */
    activeBodyCount: number;
};

export function init(): Bodies {
    return {
        pool: [],
        freeIndices: [],
        nextSequence: 0,
        activeBodyIndices: [],
        activeBodyCount: 0,
    };
}
