import type { RigidBody } from '../body/rigid-body';

/** type for a broadphase body visitor */
export type BodyVisitor = {
    /** whether the broadphase traversal should exit early */
    shouldExit: boolean;
    /** called when visiting a body */
    visit(body: RigidBody): void;
};
