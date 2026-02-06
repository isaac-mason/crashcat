import { type Box3, quat, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import {
	defineShape,
	ShapeCategory,
	ShapeType,
	type SupportingFaceResult,
	type SurfaceNormalResult,
} from './shapes';

/**
 * Empty shape - has no volume, no collision geometry.
 * Used for anchor points in constraints or as placeholders.
 *
 * EmptyShape bodies:
 * - Do not produce any collision contacts (no-op collision/cast functions)
 * - Have zero mass and zero inertia
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
		// zero-size AABB at origin [min, max] both at [0,0,0]
		aabb: [vec3.create(), vec3.create()],
		// center of mass at origin
		centerOfMass: vec3.create(),
		// zero volume
		volume: 0,
	};
}

export const def = defineShape<EmptyShape>({
	type: ShapeType.EMPTY,
	category: ShapeCategory.OTHER,
	computeMassProperties(out: MassProperties, _shape: EmptyShape): void {
		// empty shape has zero mass and zero inertia
		out.mass = 0;
		// inertia matrix is already initialized to zeros in massProperties.create()
	},

	getSurfaceNormal(ioResult: SurfaceNormalResult, _shape: EmptyShape, _subShapeId: number): void {
		// no surface, return zero normal
		vec3.zero(ioResult.normal);
		vec3.zero(ioResult.position);
		quat.identity(ioResult.quaternion);
		vec3.set(ioResult.scale, 1, 1, 1);
	},

	getSupportingFace(ioResult: SupportingFaceResult, _direction: Vec3, _shape: EmptyShape, _subShapeId: number): void {
		// no faces, return empty face
		ioResult.face.numVertices = 0;
	},

	getInnerRadius(_shape: EmptyShape): number {
		return 0;
	},
	getLeafShape(out, shape, subShapeId): void {
		// leaf shape returns itself
		out.shape = shape;
		out.remainder = subShapeId;
	},
    getSubShapeTransformedShape(out, shape, subShapeId): void {
        // leaf shape returns itself with accumulated transforms
        out.shape = shape;
        out.remainder = subShapeId;
    },
    castRay: () => {
        /* no-op */
    },
    collidePoint: () => {
        /* no-op */
    },
	register: () => {
		/* no-op */
	},
});
