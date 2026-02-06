import type { Bodies } from '../body/bodies';
import type { World } from '../world';
import * as coneConstraint from './cone-constraint';
import { type ConstraintId, ConstraintType, getConstraintIdIndex, getConstraintIdType } from './constraint-id';
import * as distanceConstraint from './distance-constraint';
import * as fixedConstraint from './fixed-constraint';
import * as hingeConstraint from './hinge-constraint';
import * as pointConstraint from './point-constraint';
import * as sixDOFConstraint from './six-dof-constraint';
import * as sliderConstraint from './slider-constraint';
import * as swingTwistConstraint from './swing-twist-constraint';

type ConstraintPool<T> = {
    constraints: T[];
    freeIndices: number[];
    nextSequence: number;
};

/** state for all user constraints in the world */
export type Constraints = {
    pointConstraints: ConstraintPool<pointConstraint.PointConstraint>;
    distanceConstraints: ConstraintPool<distanceConstraint.DistanceConstraint>;
    hingeConstraints: ConstraintPool<hingeConstraint.HingeConstraint>;
    fixedConstraints: ConstraintPool<fixedConstraint.FixedConstraint>;
    swingTwistConstraints: ConstraintPool<swingTwistConstraint.SwingTwistConstraint>;
    sliderConstraints: ConstraintPool<sliderConstraint.SliderConstraint>;
    coneConstraints: ConstraintPool<coneConstraint.ConeConstraint>;
    sixDOFConstraints: ConstraintPool<sixDOFConstraint.SixDOFConstraint>;
};

/** initialize empty constraints */
export function init(): Constraints {
    return {
        pointConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        distanceConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        hingeConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        fixedConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        swingTwistConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        sliderConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        coneConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
        sixDOFConstraints: {
            constraints: [],
            freeIndices: [],
            nextSequence: 0,
        },
    };
}

/**
 * Remove a constraint by its ID. Handles type dispatch automatically.
 * Used for cleanup when a body is removed.
 */
export function removeConstraintById(world: World, constraintId: ConstraintId): void {
    const type = getConstraintIdType(constraintId);
    const index = getConstraintIdIndex(constraintId);

    switch (type) {
        case ConstraintType.POINT: {
            const constraint = world.constraints.pointConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                pointConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.DISTANCE: {
            const constraint = world.constraints.distanceConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                distanceConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.HINGE: {
            const constraint = world.constraints.hingeConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                hingeConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.FIXED: {
            const constraint = world.constraints.fixedConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                fixedConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.SWING_TWIST: {
            const constraint = world.constraints.swingTwistConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                swingTwistConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.SLIDER: {
            const constraint = world.constraints.sliderConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                sliderConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.CONE: {
            const constraint = world.constraints.coneConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                coneConstraint.remove(world, constraint);
            }
            break;
        }
        case ConstraintType.SIX_DOF: {
            const constraint = world.constraints.sixDOFConstraints.constraints[index];
            if (constraint && !constraint._pooled && constraint.id === constraintId) {
                sixDOFConstraint.remove(world, constraint);
            }
            break;
        }
    }
}

/** result type for constraint iteration overrides */
export type ConstraintIterationOverrides = {
    velocity: number;
    position: number;
};

/** create a constraint iteration overrides result object */
export function createConstraintIterationOverrides(): ConstraintIterationOverrides {
    return { velocity: 0, position: 0 };
}

/** get iteration overrides for a constraint by ID, returns velocity and position step overrides */
export function getConstraintIterationOverrides(
    out: ConstraintIterationOverrides,
    constraints: Constraints,
    constraintId: ConstraintId,
): void {
    const type = getConstraintIdType(constraintId);
    const index = getConstraintIdIndex(constraintId);

    switch (type) {
        case ConstraintType.POINT: {
            const constraint = constraints.pointConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.DISTANCE: {
            const constraint = constraints.distanceConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.HINGE: {
            const constraint = constraints.hingeConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.FIXED: {
            const constraint = constraints.fixedConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.SWING_TWIST: {
            const constraint = constraints.swingTwistConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.SLIDER: {
            const constraint = constraints.sliderConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.CONE: {
            const constraint = constraints.coneConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
        case ConstraintType.SIX_DOF: {
            const constraint = constraints.sixDOFConstraints.constraints[index];
            out.velocity = constraint.numVelocityStepsOverride;
            out.position = constraint.numPositionStepsOverride;
            break;
        }
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

/** result type for constraint sorting fields */
export type ConstraintSortFields = {
    priority: number;
    index: number;
};

/** create a constraint sort fields result object */
export function createConstraintSortFields(): ConstraintSortFields {
    return { priority: 0, index: 0 };
}

/** get sorting fields for a constraint by id */
export function getConstraintSortFields(out: ConstraintSortFields, constraints: Constraints, constraintId: ConstraintId): void {
    const type = getConstraintIdType(constraintId);
    const index = getConstraintIdIndex(constraintId);

    switch (type) {
        case ConstraintType.POINT: {
            const constraint = constraints.pointConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.DISTANCE: {
            const constraint = constraints.distanceConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.HINGE: {
            const constraint = constraints.hingeConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.FIXED: {
            const constraint = constraints.fixedConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.SWING_TWIST: {
            const constraint = constraints.swingTwistConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.SLIDER: {
            const constraint = constraints.sliderConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.CONE: {
            const constraint = constraints.coneConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
        case ConstraintType.SIX_DOF: {
            const constraint = constraints.sixDOFConstraints.constraints[index];
            out.priority = constraint.constraintPriority;
            out.index = constraint.index;
            break;
        }
    }
}

const _sortFieldsA = createConstraintSortFields();
const _sortFieldsB = createConstraintSortFields();

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
export function updateSleeping(constraints: Constraints, bodies: Bodies): void {
    // update point constraints
    for (const constraint of constraints.pointConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update distance constraints
    for (const constraint of constraints.distanceConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update hinge constraints
    for (const constraint of constraints.hingeConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update fixed constraints
    for (const constraint of constraints.fixedConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update swing-twist constraints
    for (const constraint of constraints.swingTwistConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update slider constraints
    for (const constraint of constraints.sliderConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update cone constraints
    for (const constraint of constraints.coneConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }

    // update sixDOF constraints
    for (const constraint of constraints.sixDOFConstraints.constraints) {
        if (constraint._pooled) continue;
        const bodyA = bodies.pool[constraint.bodyIndexA];
        const bodyB = bodies.pool[constraint.bodyIndexB];
        constraint._sleeping = bodyA.sleeping && bodyB.sleeping;
    }
}

/** setup velocity constraints for all active user constraints, called before warm starting and solving */
export function setupVelocityConstraints(constraints: Constraints, bodies: Bodies, deltaTime: number): void {
    // setup point constraints
    for (const constraint of constraints.pointConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            pointConstraint.setupVelocity(constraint, bodies);
        }
    }

    // setup distance constraints
    for (const constraint of constraints.distanceConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            distanceConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }

    // setup hinge constraints
    for (const constraint of constraints.hingeConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            hingeConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }

    // setup fixed constraints
    for (const constraint of constraints.fixedConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            fixedConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }

    // setup swing-twist constraints
    for (const constraint of constraints.swingTwistConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            swingTwistConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }

    // setup slider constraints
    for (const constraint of constraints.sliderConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            sliderConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }

    // setup cone constraints
    for (const constraint of constraints.coneConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            coneConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }

    // setup sixDOF constraints
    for (const constraint of constraints.sixDOFConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled) {
            sixDOFConstraint.setupVelocity(constraint, bodies, deltaTime);
        }
    }
}

/** warm start velocity constraints for all active user constraints */
export function warmStartVelocityConstraints(constraints: Constraints, bodies: Bodies, warmStartImpulseRatio: number): void {
    // warm start point constraints
    for (const constraint of constraints.pointConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            pointConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start distance constraints
    for (const constraint of constraints.distanceConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            distanceConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start hinge constraints
    for (const constraint of constraints.hingeConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            hingeConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start fixed constraints
    for (const constraint of constraints.fixedConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            fixedConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start swing-twist constraints
    for (const constraint of constraints.swingTwistConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            swingTwistConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start slider constraints
    for (const constraint of constraints.sliderConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            sliderConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start cone constraints
    for (const constraint of constraints.coneConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            coneConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
        }
    }

    // warm start sixDOF constraints
    for (const constraint of constraints.sixDOFConstraints.constraints) {
        if (!constraint._pooled && constraint.enabled && !constraint._sleeping) {
            sixDOFConstraint.warmStartVelocity(constraint, bodies, warmStartImpulseRatio);
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

        switch (type) {
            case ConstraintType.POINT: {
                const constraint = constraints.pointConstraints.constraints[index];
                pointConstraint.solveVelocity(constraint, bodies);
                break;
            }
            case ConstraintType.DISTANCE: {
                const constraint = constraints.distanceConstraints.constraints[index];
                distanceConstraint.solveVelocity(constraint, bodies);
                break;
            }
            case ConstraintType.HINGE: {
                const constraint = constraints.hingeConstraints.constraints[index];
                hingeConstraint.solveVelocity(constraint, bodies, deltaTime);
                break;
            }
            case ConstraintType.FIXED: {
                const constraint = constraints.fixedConstraints.constraints[index];
                fixedConstraint.solveVelocity(constraint, bodies, deltaTime);
                break;
            }
            case ConstraintType.SWING_TWIST: {
                const constraint = constraints.swingTwistConstraints.constraints[index];
                swingTwistConstraint.solveVelocity(constraint, bodies, deltaTime);
                break;
            }
            case ConstraintType.SLIDER: {
                const constraint = constraints.sliderConstraints.constraints[index];
                sliderConstraint.solveVelocity(constraint, bodies, deltaTime);
                break;
            }
            case ConstraintType.CONE: {
                const constraint = constraints.coneConstraints.constraints[index];
                coneConstraint.solveVelocity(constraint, bodies, deltaTime);
                break;
            }
            case ConstraintType.SIX_DOF: {
                const constraint = constraints.sixDOFConstraints.constraints[index];
                sixDOFConstraint.solveVelocity(constraint, bodies, deltaTime);
                break;
            }
        }
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

        switch (type) {
            case ConstraintType.POINT: {
                const constraint = constraints.pointConstraints.constraints[index];
                const constraintAppliedImpulse = pointConstraint.solvePosition(constraint, bodies, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.DISTANCE: {
                const constraint = constraints.distanceConstraints.constraints[index];
                const constraintAppliedImpulse = distanceConstraint.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.HINGE: {
                const constraint = constraints.hingeConstraints.constraints[index];
                const constraintAppliedImpulse = hingeConstraint.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.FIXED: {
                const constraint = constraints.fixedConstraints.constraints[index];
                const constraintAppliedImpulse = fixedConstraint.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.SWING_TWIST: {
                const constraint = constraints.swingTwistConstraints.constraints[index];
                const constraintAppliedImpulse = swingTwistConstraint.solvePosition(
                    constraint,
                    bodies,
                    deltaTime,
                    baumgarteFactor,
                );
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.SLIDER: {
                const constraint = constraints.sliderConstraints.constraints[index];
                const constraintAppliedImpulse = sliderConstraint.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.CONE: {
                const constraint = constraints.coneConstraints.constraints[index];
                const constraintAppliedImpulse = coneConstraint.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
            case ConstraintType.SIX_DOF: {
                const constraint = constraints.sixDOFConstraints.constraints[index];
                const constraintAppliedImpulse = sixDOFConstraint.solvePosition(constraint, bodies, deltaTime, baumgarteFactor);
                appliedImpulse = appliedImpulse || constraintAppliedImpulse;
                break;
            }
        }
    }

    return appliedImpulse;
}
