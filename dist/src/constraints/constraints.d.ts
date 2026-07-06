import type { Bodies } from '../body/bodies.js';
import type { World } from '../world.js';
import type { ConeConstraint } from './cone-constraint.js';
import { type ConstraintId, ConstraintType } from './constraint-id.js';
import type { DistanceConstraint } from './distance-constraint.js';
import type { FixedConstraint } from './fixed-constraint.js';
import type { HingeConstraint } from './hinge-constraint.js';
import type { PointConstraint } from './point-constraint.js';
import type { SixDOFConstraint } from './six-dof-constraint.js';
import type { SliderConstraint } from './slider-constraint.js';
import type { SwingTwistConstraint } from './swing-twist-constraint.js';
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
export declare function init(): Constraints;
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
export type WarmStartVelocityFn<C extends ConstraintBase> = (constraint: C, bodies: Bodies, warmStartImpulseRatio: number) => void;
export type SolveVelocityFn<C extends ConstraintBase> = (constraint: C, bodies: Bodies, deltaTime: number) => boolean;
export type SolvePositionFn<C extends ConstraintBase> = (constraint: C, bodies: Bodies, deltaTime: number, baumgarte: number) => boolean;
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
export declare function defineConstraint<C extends ConstraintBase>(options: ConstraintDefOptions<C>): ConstraintDef<C>;
/** global registry of constraint definitions keyed by ConstraintType */
export declare const constraintDefs: Partial<Record<ConstraintType, ConstraintDef>>;
/** register a constraint definition */
export declare function registerConstraintDef(def: ConstraintDef): void;
/** get or create a constraint pool for a given type */
export declare function ensurePool<C extends ConstraintBase>(constraints: Constraints, type: ConstraintType): ConstraintPool<C>;
/** create a constraint iteration overrides result object */
export declare function createConstraintIterationOverrides(): ConstraintIterationOverrides;
/** create a constraint sort fields result object */
export declare function createConstraintSortFields(): ConstraintSortFields;
/**
 * remove a constraint by its ID. handles type dispatch automatically.
 * used for cleanup when a body is removed.
 */
export declare function removeConstraintById(world: World, constraintId: ConstraintId): void;
/** destroy all constraints involving a body */
export declare function destroyBodyConstraints(world: World, body: {
    constraintIds: ConstraintId[];
}): void;
/** get iteration overrides for a constraint by ID, returns velocity and position step overrides */
export declare function getConstraintIterationOverrides(out: ConstraintIterationOverrides, constraints: Constraints, constraintId: ConstraintId): void;
/** get sorting fields for a constraint by id */
export declare function getConstraintSortFields(out: ConstraintSortFields, constraints: Constraints, constraintId: ConstraintId): void;
/**
 * sort constraint IDs for deterministic solving.
 * sorts by:
 * - 1. priority (lower = solved first)
 * - 2. constraint index (for determinism when priorities equal)
 */
export declare function sortConstraintIds(constraints: Constraints, constraintIds: ConstraintId[]): void;
/** update sleeping state for all constraints based on their connected bodies sleeping states */
export declare function updateSleeping(constraintsState: Constraints, bodies: Bodies): void;
/** setup velocity constraints for all active user constraints, called before warm starting and solving */
export declare function setupVelocityConstraints(constraintsState: Constraints, bodies: Bodies, deltaTime: number): void;
/** warm start velocity constraints for all active user constraints */
export declare function warmStartVelocityConstraints(constraintsState: Constraints, bodies: Bodies, warmStartImpulseRatio: number): void;
/** solve velocity constraints for an island, called once per velocity iteration, returns true if any constraint applied an impulse */
export declare function solveVelocityConstraintsForIsland(constraints: Constraints, bodies: Bodies, constraintIds: ConstraintId[], deltaTime: number): boolean;
/** solve position constraints for an island, called once per position iteration, returns true if any constraint applied an impulse */
export declare function solvePositionConstraintsForIsland(constraints: Constraints, bodies: Bodies, constraintIds: ConstraintId[], baumgarteFactor: number, deltaTime: number): boolean; /** constraint space enum - where are constraint points specified */
export declare enum ConstraintSpace {
    /** points specified in world space */
    WORLD = 0,
    /** points specified relative to body */
    LOCAL = 1
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
export declare function removeConstraintIdFromBody(body: {
    constraintIds: ConstraintId[];
}, constraintId: ConstraintId): void;
/** default constraint base fields */
export declare function makeConstraintBase(): ConstraintBase;
