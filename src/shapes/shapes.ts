import type { Box3, Raycast3 } from 'mathcat';
import { type Quat, quat, type Vec3, vec3 } from 'mathcat';
import type { MassProperties } from '../body/mass-properties';
import type { SubShapeId } from '../body/sub-shape';
import type { CastRayCollector, CastRaySettings } from '../collision/cast-ray-vs-shape';
import type { CastShapeCollector, CastShapeSettings } from '../collision/cast-shape-vs-shape';
import type { CollidePointCollector, CollidePointSettings } from '../collision/collide-point-vs-shape';
import type { CollideShapeCollector, CollideShapeSettings } from '../collision/collide-shape-vs-shape';
import type { SupportFunctionMode } from '../collision/support';
import type { Face } from '../utils/face';
import type { BoxShape } from './box';
import type { CapsuleShape } from './capsule';
import type { CompoundShape } from './compound';
import type { ConvexHullShape } from './convex-hull';
import type { CylinderShape } from './cylinder';
import type { EmptyShape } from './empty-shape';
import type { OffsetCenterOfMassShape } from './offset-center-of-mass';
import type { PlaneShape } from './plane';
import type { ScaledShape } from './scaled';
import type { SphereShape } from './sphere';
import type { TransformedShape } from './transformed';
import type { TriangleMeshShape } from './triangle-mesh';

/** base shape type */
export type ShapeBase = {
    /** shape type discriminator */
    type: ShapeType;
    /** shape local bounds */
    aabb: Box3;
    /** shape center of mass */
    centerOfMass: Vec3;
    /** shape volume */
    volume: number;
};

/** shape types enum */
export enum ShapeType {
    // built-in shapes: 0-100
    SPHERE = 0,
    BOX = 1,
    CAPSULE = 2,
    CONVEX_HULL = 3,
    TRIANGLE_MESH = 4,
    COMPOUND = 5,
    TRANSFORMED = 6,
    SCALED = 7,
    EMPTY = 8,
    CYLINDER = 9,
    OFFSET_CENTER_OF_MASS = 10,
    PLANE = 11,

    // user-defined shapes: 101-110
    USER_1 = 101,
    USER_2 = 102,
    USER_3 = 103,
    USER_4 = 104,
    USER_5 = 105,
    USER_6 = 106,
    USER_7 = 107,
    USER_8 = 108,
    USER_9 = 109,
    USER_10 = 110,
}

/** shape categories enum */
export enum ShapeCategory {
    /** Convex shapes (Sphere, Box, Capsule, ConvexHull) */
    CONVEX = 0,
    /** Mesh shapes (TriangleMesh) */
    MESH = 1,
    /** Decorator shapes that transform other shapes (Transformed, Scaled) */
    DECORATOR = 2,
    /** Composite shapes that contain other shapes (Compound) */
    COMPOSITE = 3,
    /** Shapes that don't fit into the above categories */
    OTHER = 4,
}

/**
 * Shape type registry for discriminated unions (extensible).
 * Custom shapes can extend this via declaration merging:
 * @example
 * ```typescript
 * declare module 'crashcat' {
 *   interface ShapeTypeRegistry {
 *     [ShapeType.USER_1]: MyCustomShape;
 *   }
 * }
 * ```
 */
export interface ShapeTypeRegistry {
    [ShapeType.SPHERE]: SphereShape;
    [ShapeType.BOX]: BoxShape;
    [ShapeType.CAPSULE]: CapsuleShape;
    [ShapeType.CONVEX_HULL]: ConvexHullShape;
    [ShapeType.CYLINDER]: CylinderShape;
    [ShapeType.TRIANGLE_MESH]: TriangleMeshShape;
    [ShapeType.COMPOUND]: CompoundShape;
    [ShapeType.TRANSFORMED]: TransformedShape;
    [ShapeType.SCALED]: ScaledShape;
    [ShapeType.EMPTY]: EmptyShape;
    [ShapeType.OFFSET_CENTER_OF_MASS]: OffsetCenterOfMassShape;
    [ShapeType.PLANE]: PlaneShape;
}

/** shape type, union derived from registry interface */
export type Shape = ShapeTypeRegistry[keyof ShapeTypeRegistry];

/** shape type registry for convex shapes */
export interface ConvexShapeTypeRegistry {
    [ShapeType.SPHERE]: SphereShape;
    [ShapeType.BOX]: BoxShape;
    [ShapeType.CAPSULE]: CapsuleShape;
    [ShapeType.CONVEX_HULL]: ConvexHullShape;
    [ShapeType.CYLINDER]: CylinderShape;
}

/** shape type, constrained to convex shapes */
export type ConvexShape = ConvexShapeTypeRegistry[keyof ConvexShapeTypeRegistry];

export const DEFAULT_SHAPE_DENSITY = 1000; // kg/m³

export type SurfaceNormalResult = {
    normal: Vec3;
    position: Vec3;
    quaternion: Quat;
    scale: Vec3;
};

export type SupportingFaceResult = {
    face: Face;
    position: Vec3;
    quaternion: Quat;
    scale: Vec3;
};

export function createSupportingFaceResult(): SupportingFaceResult {
    return {
        face: { vertices: [], numVertices: 0 },
        position: vec3.create(),
        quaternion: quat.create(),
        scale: vec3.fromValues(1, 1, 1),
    };
}

export type GetLeafShapeResult = {
    /** The leaf shape found, or null if navigation failed */
    shape: Shape | null;
    /** Remaining SubShapeId after navigation */
    remainder: SubShapeId;
};

export type GetSubShapeTransformedShapeResult = {
    /** The transformed shape found, or null if navigation failed */
    shape: Shape | null;
    /** Accumulated position transform */
    position: Vec3;
    /** Accumulated rotation transform */
    rotation: Quat;
    /** Accumulated scale transform */
    scale: Vec3;
    /** Remaining SubShapeId after navigation */
    remainder: SubShapeId;
};

export type CastRayVsShapeFn<S> = (
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: S,
    subShapeId: number,
    subShapeIdBits: number,
    posX: number,
    posY: number,
    posZ: number,
    quatX: number,
    quatY: number,
    quatZ: number,
    quatW: number,
    scaleX: number,
    scaleY: number,
    scaleZ: number,
) => void;

export type CollidePointVsShapeFn<S> = (
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: S,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
) => void;

/** Collision handler function type for colliding two shapes */
export type CollideShapeVsShapeFn = (
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    quatAX: number,
    quatAY: number,
    quatAZ: number,
    quatAW: number,
    scaleAX: number,
    scaleAY: number,
    scaleAZ: number,
    shapeB: Shape,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
) => void;

/** Cast handler function type for casting one shape against another */
export type CastShapeVsShapeFn = (
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    subShapeIdBitsA: number,
    posAX: number,
    posAY: number,
    posAZ: number,
    quatAX: number,
    quatAY: number,
    quatAZ: number,
    quatAW: number,
    scaleAX: number,
    scaleAY: number,
    scaleAZ: number,
    dispAX: number,
    dispAY: number,
    dispAZ: number,
    shapeB: Shape,
    subShapeIdB: number,
    subShapeIdBitsB: number,
    posBX: number,
    posBY: number,
    posBZ: number,
    quatBX: number,
    quatBY: number,
    quatBZ: number,
    quatBW: number,
    scaleBX: number,
    scaleBY: number,
    scaleBZ: number,
) => void;

type OptionalShapeDef = 'computeMassProperties' | 'getInnerRadius' | 'createSupportPool' | 'getSupportFunction' | 'getLeafShape' | 'getSubShapeTransformedShape';

export type ShapeDefOptions<S extends ShapeBase> = Omit<ShapeDef<S>, OptionalShapeDef> &
    Partial<Pick<ShapeDef<S>, OptionalShapeDef>>;

export function defineShape<S extends ShapeBase>(shapeDef: ShapeDefOptions<S>): ShapeDef<S> {
    const computeMassProperties = shapeDef.computeMassProperties ?? ((out: MassProperties, _shape: S) => {
        // default: unit mass, identity inertia (don't change)
        out.mass = 1;
    });

    const getInnerRadius = shapeDef.getInnerRadius ?? ((_shape: S) => {
        // default: zero inner radius
        return 0;
    });
    
    const getLeafShape =
        shapeDef.getLeafShape ??
        ((out, shape, subShapeId) => {
            // default: leaf shape returns itself
            out.shape = shape as Shape;
            out.remainder = subShapeId;
        });

    const getSubShapeTransformedShape =
        shapeDef.getSubShapeTransformedShape ??
        ((out, shape, subShapeId) => {
            // default: leaf shape returns itself with accumulated transforms
            out.shape = shape as Shape;
            out.remainder = subShapeId;
        });

    // default: no support pool
    const createSupportPool = shapeDef.createSupportPool ?? (() => undefined);

    // default: no-op support function
    const getSupportFunction = shapeDef.getSupportFunction ?? (() => undefined);

    return {
        type: shapeDef.type,
        category: shapeDef.category,
        computeMassProperties,
        getSurfaceNormal: shapeDef.getSurfaceNormal,
        getSupportingFace: shapeDef.getSupportingFace,
        getInnerRadius,
        getLeafShape,
        getSubShapeTransformedShape,
        castRay: shapeDef.castRay,
        collidePoint: shapeDef.collidePoint,
        createSupportPool,
        getSupportFunction,
        register: shapeDef.register,
    };
}

export type ComputeMassPropertiesImpl<S extends ShapeBase> = (out: MassProperties, shape: S) => void;

export type GetSurfaceNormalImpl<S extends ShapeBase> = (ioResult: SurfaceNormalResult, shape: S, subShapeId: number) => void;

export type GetSupportingFaceImpl<S extends ShapeBase> = (
    ioResult: SupportingFaceResult,
    direction: Vec3,
    shape: S,
    subShapeId: number,
) => void;

export type GetInnerRadiusImpl<S extends ShapeBase> = (shape: S) => number;

export type GetLeafShapeImpl<S extends ShapeBase> = (
    outResult: GetLeafShapeResult,
    shape: S,
    subShapeId: SubShapeId,
) => void;

export type GetSubShapeTransformedShapeImpl<S extends ShapeBase> = (
    outResult: GetSubShapeTransformedShapeResult,
    shape: S,
    subShapeId: SubShapeId,
) => void;

export type GetSupportFunctionImpl<S extends ShapeBase> = (
    pool: any,
    shape: S,
    mode: SupportFunctionMode,
    scale: Vec3,
) => any | undefined;

export type ShapeDef<S extends ShapeBase> = {
    type: ShapeType;

    /** Shape category used for collision dispatch registration */
    category: ShapeCategory;

    /** get mass properties for the shape */
    computeMassProperties: ComputeMassPropertiesImpl<S>;

    /** get surface normal */
    getSurfaceNormal: GetSurfaceNormalImpl<S>;

    /** get supporting face */
    getSupportingFace: GetSupportingFaceImpl<S>;

    /** get inner radius - radius of the biggest sphere that fits entirely in the shape */
    getInnerRadius: GetInnerRadiusImpl<S>;

    /** navigate to leaf shape following sub shape id hierarchy */
    getLeafShape: GetLeafShapeImpl<S>;

    /** navigate to transformed shape following sub shape id hierarchy, accumulating transforms */
    getSubShapeTransformedShape: GetSubShapeTransformedShapeImpl<S>;

    /** cast a ray against the shape */
    castRay: CastRayVsShapeFn<S>;

    /** test if a point collides with the shape */
    collidePoint: CollidePointVsShapeFn<S>;

    /**
     * Create a support pool for this shape type.
     * return undefined for non-convex shapes.
     * Called once per ShapeSupportPool when pools are created or shapes are registered.
     */
    createSupportPool(): any | undefined;

    /**
     * Get a support function for this shape.
     * return undefined for non-convex shapes.
     * @param pool the pool created by createSupportPool() for this shape type
     * @param shape the shape instance
     * @param mode support function mode (INCLUDE/EXCLUDE_CONVEX_RADIUS)
     * @param scale scale to apply to the shape
     */
    getSupportFunction: GetSupportFunctionImpl<S>;

    /** Register collision and cast handlers for this shape type */
    register(): void;
};

export const shapeDefs = {} as Record<ShapeType, ShapeDef<Shape>>;

export type CollisionDispatch = {
    collideFns: Map<ShapeType, Map<ShapeType, CollideShapeVsShapeFn>>;
    castFns: Map<ShapeType, Map<ShapeType, CastShapeVsShapeFn>>;
};

export const collisionDispatch: CollisionDispatch = {
    collideFns: new Map(),
    castFns: new Map(),
};

/**
 * Register a collision handler for a shape type pair.
 * If a handler already exists for the pair, it will be overwritten.
 * @param typeA first shape type
 * @param typeB second shape type
 * @param fn collision handler function
 */
export function setCollideShapeFn(typeA: ShapeType, typeB: ShapeType, fn: CollideShapeVsShapeFn): void {
    if (!collisionDispatch.collideFns.has(typeA)) {
        collisionDispatch.collideFns.set(typeA, new Map());
    }
    collisionDispatch.collideFns.get(typeA)!.set(typeB, fn);
}

/**
 * Register a cast handler for a shape type pair.
 * If a handler already exists for the pair, it will be overwritten.
 * @param typeA first shape type
 * @param typeB second shape type
 * @param fn cast handler function
 */
export function setCastShapeFn(typeA: ShapeType, typeB: ShapeType, fn: CastShapeVsShapeFn): void {
    if (!collisionDispatch.castFns.has(typeA)) {
        collisionDispatch.castFns.set(typeA, new Map());
    }
    collisionDispatch.castFns.get(typeA)!.set(typeB, fn);
}

const _surfaceNormalResult: SurfaceNormalResult = {
    normal: vec3.create(),
    position: vec3.create(),
    quaternion: quat.create(),
    scale: vec3.fromValues(1, 1, 1),
};

export function getShapeSurfaceNormal(out: Vec3, shape: Shape, position: Vec3, subShapeId: number): Vec3 {
    // set up result with initial transforms
    vec3.copy(_surfaceNormalResult.position, position);
    quat.identity(_surfaceNormalResult.quaternion);
    vec3.set(_surfaceNormalResult.scale, 1, 1, 1);

    // compute surface normal with accumulated transforms
    shapeDefs[shape.type].getSurfaceNormal(_surfaceNormalResult, shape, subShapeId);

    // copy result to output
    return vec3.copy(out, _surfaceNormalResult.normal);
}

export function computeMassProperties(out: MassProperties, shape: Shape): MassProperties {
    const shapeDef = shapeDefs[shape.type];
    shapeDef.computeMassProperties(out, shape);
    return out;
}

const _supportingFaceResult = /* @__PURE__ */ createSupportingFaceResult();

/**
 * Computes the supporting face, takes a local-space direction and returns face vertices in world space.
 * @param out output face with vertices in world space
 * @param shape shape to query
 * @param subShapeId sub-shape ID
 * @param localDirection query direction in local space to the shape
 * @param position world position of shape
 * @param quaternion world rotation of shape
 * @param scale scale in local space of the shape
 */
export function getShapeSupportingFace(
    out: Face,
    shape: Shape,
    subShapeId: number,
    localDirection: Vec3,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
): void {
    vec3.copy(_supportingFaceResult.position, position);
    quat.copy(_supportingFaceResult.quaternion, quaternion);
    vec3.copy(_supportingFaceResult.scale, scale);

    shapeDefs[shape.type].getSupportingFace(_supportingFaceResult, localDirection, shape, subShapeId);

    out.numVertices = _supportingFaceResult.face.numVertices;
    for (let i = 0; i < out.numVertices * 3; i++) {
        out.vertices[i] = _supportingFaceResult.face.vertices[i];
    }
}

/**
 * Get the inner radius of a shape.
 * Returns the radius of the biggest sphere that fits entirely in the shape.
 * For shapes with multiple sub-shapes, returns the smallest inner radius of the parts.
 * This can be used as a measure of how far the shape can be moved without risking going through geometry.
 *
 * Note: This is calculated on-demand, not cached. For performance-critical paths,
 * consider caching the result if calling repeatedly on the same shape.
 *
 * @param shape the shape to query
 * @returns the inner radius in meters
 */
export function getShapeInnerRadius(shape: Shape): number {
    const shapeDef = shapeDefs[shape.type];
    return shapeDef.getInnerRadius(shape);
}
