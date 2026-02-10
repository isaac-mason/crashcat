import type { Bodies } from '../body/bodies';
import type { World } from '../world';
import type { ConeConstraint } from './cone-constraint';
import {
    type ConstraintId,
    ConstraintType,
    getConstraintIdIndex,
    getConstraintIdType,
    INVALID_CONSTRAINT_ID,
} from './constraint-id';
import type { DistanceConstraint } from './distance-constraint';
import type { FixedConstraint } from './fixed-constraint';
import type { HingeConstraint } from './hinge-constraint';
import type { PointConstraint } from './point-constraint';
import type { SixDOFConstraint } from './six-dof-constraint';
import type { SliderConstraint } from './slider-constraint';
import type { SwingTwistConstraint } from './swing-twist-constraint';

/**
 * constraint type registry for discriminated unions (extensible).
 * custom constraints can extend this via declaration merging:
 * @example
 * ```typescript
 * declare module 'crashcat' {
 *   interface ConstraintTypeRegistry {
 *     [ConstraintType.USER_1]: MyCustomConstraint;
 *   }
 * }
 * ```
 */
export interface ConstraintTypeRegistry {
    [ConstraintType.POINT]: PointConstraint;
    [ConstraintType.DISTANCE]: DistanceConstraint;
    [ConstraintType.HINGE]: HingeConstraint;
    [ConstraintType.SLIDER]: SliderConstraint;
    [ConstraintType.FIXED]: FixedConstraint;
    [ConstraintType.CONE]: ConeConstraint;
    [ConstraintType.SWING_TWIST]: SwingTwistConstraint;
    [ConstraintType.SIX_DOF]: SixDOFConstraint;
}

/** union type of all constraint data types, derived from registry interface */
export type Constraint = ConstraintTypeRegistry[keyof ConstraintTypeRegistry];

/** per-type constraint pool */
export type ConstraintPool<C extends ConstraintBase = ConstraintBase> = {
    type: ConstraintType;
    constraints: C[];
    freeIndices: number[];
    nextSequence: number;
};

/** the constraint pools container - keyed by type for monomorphic iteration */
export type Constraints = {
    pools: Partial<Record<ConstraintType, ConstraintPool>>;
};

/** initialize empty constraints */
export function init(): Constraints {
    return {
        pools: {},
    };
}

/** result type for constraint iteration overrides */
export type ConstraintIterationOverrides = {
    velocity: number;
    position: number;
};

/** result type for constraint sorting fields */
export type ConstraintSortFields = {
    priority: number;
    index: number;
};

/** function signatures for constraint operations */
export type SetupVelocityFn<C extends ConstraintBase> = (constraint: C, bodies: Bodies, deltaTime: number) => void;
export type WarmStartVelocityFn<C extends ConstraintBase> = (
    constraint: C,
    bodies: Bodies,
    warmStartImpulseRatio: number,
) => void;
export type SolveVelocityFn<C extends ConstraintBase> = (constraint: C, bodies: Bodies, deltaTime: number) => boolean;
export type SolvePositionFn<C extends ConstraintBase> = (
    constraint: C,
    bodies: Bodies,
    deltaTime: number,
    baumgarte: number,
) => boolean;
export type ResetWarmStartFn<C extends ConstraintBase> = (constraint: C) => void;
export type GetIterationOverridesFn<C extends ConstraintBase> = (out: ConstraintIterationOverrides, constraint: C) => void;
export type GetSortFieldsFn<C extends ConstraintBase> = (out: ConstraintSortFields, constraint: C) => void;

/** definition for a user constraint */
export type ConstraintDef<C extends ConstraintBase = ConstraintBase> = {
    /** constraint type enum value */
    type: ConstraintType;

    /** setup velocity constraints for this constraint */
    setupVelocity: SetupVelocityFn<C>;

    /** warm start velocity constraints */
    warmStartVelocity: WarmStartVelocityFn<C>;

    /** solve velocity constraints, returns true if impulse applied */
    solveVelocity: SolveVelocityFn<C>;

    /** solve position constraints, returns true if impulse applied */
    solvePosition: SolvePositionFn<C>;

    /** reset warm start state (called when bodies wake up) */
    resetWarmStart: ResetWarmStartFn<C>;

    /** get iteration overrides for solver */
    getIterationOverrides: GetIterationOverridesFn<C>;

    /** get sort fields for constraint ordering */
    getSortFields: GetSortFieldsFn<C>;
};

/** options for defining a user constraint */
export type ConstraintDefOptions<C extends ConstraintBase> = ConstraintDef<C>;

/** define a user constraint def - ensures consistent shape */
export function defineConstraint<C extends ConstraintBase>(options: ConstraintDefOptions<C>): ConstraintDef<C> {
    return {
        type: options.type,
        setupVelocity: options.setupVelocity,
        warmStartVelocity: options.warmStartVelocity,
        solveVelocity: options.solveVelocity,
        solvePosition: options.solvePosition,
        resetWarmStart: options.resetWarmStart,
        getIterationOverrides: options.getIterationOverrides,
        getSortFields: options.getSortFields,
    };
}

/** global registry of constraint definitions keyed by ConstraintType */
export const constraintDefs: Partial<Record<ConstraintType, ConstraintDef>> = {};

/** register a constraint definition */
export function registerConstraintDef(def: ConstraintDef): void {
    constraintDefs[def.type] = def;
}

/** get or create a constraint pool for a given type */
export function ensurePool<C extends ConstraintBase>(constraints: Constraints, type: ConstraintType): ConstraintPool<C> {
    let pool = constraints.pools[type] as ConstraintPool<C> | undefined;
    if (!pool) {
        pool = { type, constraints: [], freeIndices: [], nextSequence: 0 };
        constraints.pools[type] = pool;
    }
    return pool;
}

/** create a constraint iteration overrides result object */
export function createConstraintIterationOverrides(): ConstraintIterationOverrides {
    return { velocity: 0, position: 0 };
}

/** create a constraint sort fields result object */
export function createConstraintSortFields(): ConstraintSortFields {
    return { priority: 0, index: 0 };
}

/**
 * remove a constraint by its ID. handles type dispatch automatically.
 * used for cleanup when a body is removed.
 */
export function removeConstraintById(world: World, constraintId: ConstraintId): void {
    const type = getConstraintIdType(constraintId);
    const index = getConstraintIdIndex(constraintId);
    const pool = world.constraints.pools[type] as ConstraintPool | undefined;
    if (!pool) return;

    const constraint = pool.constraints[index];
    if (constraint && !constraint._pooled && constraint.id === constraintId) {
        // remove from bodies' constraintIds arrays
        const bodyA = world.bodies.pool[constraint.bodyIndexA];
        const bodyB = world.bodies.pool[constraint.bodyIndexB];
        if (bodyA && !bodyA._pooled) {
            removeConstraintIdFromBody(bodyA, constraint.id);
        }
        if (constraint.bodyIndexA !== constraint.bodyIndexB && bodyB && !bodyB._pooled) {
            removeConstraintIdFromBody(bodyB, constraint.id);
        }

        // return to pool
        pool.freeIndices.push(index);
        constraint._pooled = true;
    }
}

/** destroy all constraints involving a body */
export function destroyBodyConstraints(world: World, body: { constraintIds: ConstraintId[] }): void {
    // iterate backwards since removeConstraintById modifies the other body's array
    for (let i = body.constraintIds.length - 1; i >= 0; i--) {
        const constraintId = body.constraintIds[i];
        removeConstraintById(world, constraintId);
    }
    body.constraintIds.length = 0;
}

/** get iteration overrides for a constraint by ID, returns velocity and position step overrides */
export function getConstraintIterationOverrides(
    out: ConstraintIterationOverrides,
    constraints: Constraints,
    constraintId: ConstraintId,
): void {
    const type = getConstraintIdType(constraintId);
    const index = getConstraintIdIndex(constraintId);
    const def = constraintDefs[type];
    const pool = constraints.pools[type];
    if (!pool || !def) {
        out.velocity = 0;
        out.position = 0;
        return;
    }
    const constraint = pool.constraints[index];
    def.getIterationOverrides(out, constraint);
}

/** get sorting fields for a constraint by id */
export function getConstraintSortFields(out: ConstraintSortFields, constraints: Constraints, constraintId: ConstraintId): void {
    const type = getConstraintIdType(constraintId);
    const index = getConstraintIdIndex(constraintId);
    const def = constraintDefs[type];
    const pool = constraints.pools[type];
    if (!pool || !def) {
        out.priority = 0;
        out.index = 0;
        return;
    }
    const constraint = pool.constraints[index];
    def.getSortFields(out, constraint);
}

const _sortFieldsA = /* @__PURE__ */ createConstraintSortFields();
const _sortFieldsB = /* @__PURE__ */ createConstraintSortFields();

/**
 * sort constraint IDs for deterministic solving.
 * sorts by:
 * - 1. priority (lower = solved first)
 * - 2. constraint index (for determinism when priorities equal)
 */
export function sortConstraintIds(constraints: Constraints, constraintIds: ConstraintId[]): void {
    constraintIds.sort((aId, bId) => {
        getConstraintSortFields(_sortFieldsA, constraints, aId);
        getConstraintSortFields(_sortFieldsB, constraints, bId);

        // primary sort: priority (lower first)
        if (_sortFieldsA.priority !== _sortFieldsB.priority) {
            return _sortFieldsA.priority - _sortFieldsB.priority;
        }

        // secondary sort: constraint index
        return _sortFieldsA.index - _sortFieldsB.index;
    });
}

/** update sleeping state for all constraints based on their connected bodies sleeping states */
export function updateSleeping(constraintsState: Constraints, bodies: Bodies): void {
    for (const type in constraintsState.pools) {
        const pool = constraintsState.pools[type as unknown as ConstraintType]!;
        for (const constraint of pool.constraints) {
            if (constraint._pooled) continue;
            const bodyA = bodies.pool[constraint.bodyIndexA];
            const bodyB = bodies.pool[constraint.bodyIndexB];
            constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
        }
    }
}

/** setup velocity constraints for all active user constraints, called before warm starting and solving */
export function setupVelocityConstraints(constraintsState: Constraints, bodies: Bodies, deltaTime: number): void {
    for (const type in constraintsState.pools) {
        const pool = constraintsState.pools[type as unknown as ConstraintType]!;
        const def = constraintDefs[pool.type]!;
        for (const constraint of pool.constraints) {
            if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
                def.setupVelocity(constraint, bodies, deltaTime);
            }
        }
    }
}

/** warm start velocity constraints for all active user constraints */
export function warmStartVelocityConstraints(constraintsState: Constraints, bodies: Bodies, warmStartImpulseRatio: number): void {
    for (const type in constraintsState.pools) {
        const pool = constraintsState.pools[type as unknown as ConstraintType]!;
        const def = constraintDefs[pool.type]!;
        for (const constraint of pool.constraints) {
            if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
                def.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
            }
        }
    }
}

/** solve velocity constraints for an island, called once per velocity iteration */
export function solveVelocityConstraintsForIsland(
    constraints: Constraints,
    bodies: Bodies,
    constraintIds: ConstraintId[],
    deltaTime: number,
): void {
    for (const constraintId of constraintIds) {
        const type = getConstraintIdType(constraintId);
        const index = getConstraintIdIndex(constraintId);
        const def = constraintDefs[type];
        const pool = constraints.pools[type];
        if (!pool || !def) continue;
        const constraint = pool.constraints[index];
        def.solveVelocity(constraint, bodies, deltaTime);
    }
}

/** solve position constraints for an island, called once per position iteration, true if any constraint applied an impulse (not yet converged) */
export function solvePositionConstraintsForIsland(
    constraints: Constraints,
    bodies: Bodies,
    constraintIds: ConstraintId[],
    baumgarteFactor: number,
    deltaTime: number,
): boolean {
    let appliedImpulse = false;

    for (const constraintId of constraintIds) {
        const type = getConstraintIdType(constraintId);
        const index = getConstraintIdIndex(constraintId);
        const def = constraintDefs[type];
        const pool = constraints.pools[type];
        if (!pool || !def) continue;
        const constraint = pool.constraints[index];
        const constraintAppliedImpulse = def.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
        appliedImpulse = appliedImpulse || constraintAppliedImpulse;
    }

    return appliedImpulse;
} /** constraint space enum - where are constraint points specified */

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
