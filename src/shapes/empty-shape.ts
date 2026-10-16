import { mat4, type Vec3, vec3 } from 'math';
import { type Box3, box3 } from 'math/shapes';
import type { MassProperties } from '../body/mass-properties';
import { defineShape, ShapeType, type SupportingFaceResult, type SurfaceNormalResult } from './shapes';

/**
 * Empty shape - has no volume, no collision geometry.
 * Used for anchor points in constraints or as placeholders.
 *
 * EmptyShape bodies:
 * - Do not produce any collision contacts (no-op collision/cast functions)
 * - Have unit mass and identity inertia
 * - Have zero-size AABB
 * - Are added to broadphase but filtered out during narrowphase via dispatch
 * - Can still be used in constraints (e.g., for "fixed to world" constraints)
 */
export type EmptyShape = {
    type: ShapeType.EMPTY;
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

/**
 * Create an empty shape.
 * Useful for creating static anchor bodies that don't need collision geometry.
 */
export function create(): EmptyShape {
    return {
        type: ShapeType.EMPTY,
        // zero-size AABB at origin
        aabb: box3.set(box3.create(), 0, 0, 0, 0, 0, 0),
        // center of mass at origin
        centerOfMass: vec3.create(),
        // zero volume
        volume: 0,
    };
}

export const def = /* @__PURE__ */ (() =>
    defineShape<EmptyShape>({
        type: ShapeType.EMPTY,
        computeMassProperties(out: MassProperties, _shape: EmptyShape): void {
            // unit mass and inertia, so a dynamic body without a shape still integrates
            out.mass = 1;
            mat4.identity(out.inertia);
        },
        getSurfaceNormal(ioResult: SurfaceNormalResult, _shape: EmptyShape, _subShapeId: number): void {
            // no surface, return zero normal
            vec3.zero(ioResult.normal);
            vec3.zero(ioResult.position);
            vec3.set(ioResult.scale, 1, 1, 1);
        },
        getSupportingFace(ioResult: SupportingFaceResult, _direction: Vec3, _shape: EmptyShape, _subShapeId: number): void {
            // no faces, return empty face
            ioResult.face.numVertices = 0;
        },
        getMaterial: (): number => -1,
        castRay: () => {
            /* no-op */
        },
        collidePoint: () => {
            /* no-op */
        },
        register: () => {
            /* no-op */
        },
    }))();
