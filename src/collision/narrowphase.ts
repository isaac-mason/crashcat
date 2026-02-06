import type { Raycast3 } from 'mathcat';
import { collisionDispatch, shapeDefs, type Shape } from '../shapes/shapes';
import type { CastRayCollector, CastRaySettings } from './cast-ray-vs-shape';
import type { CastShapeCollector, CastShapeSettings } from './cast-shape-vs-shape';
import type { CollidePointCollector, CollidePointSettings } from './collide-point-vs-shape';
import {
    copyCollideShapeSettings,
    createDefaultCollideShapeSettings,
    type CollideShapeCollector,
    type CollideShapeSettings,
} from './collide-shape-vs-shape';
import { InternalEdgeRemovingCollector } from './internal-edge-removing-collector';

export {
    CastRayStatus,
    copyCastRayHit,
    createAllCastRayCollector,
    createAnyCastRayCollector,
    createCastRayHit,
    createClosestCastRayCollector,
    createDefaultCastRaySettings,
} from './cast-ray-vs-shape';
export type { CastRayCollector, CastRayHit, CastRaySettings } from './cast-ray-vs-shape';

export function castRayVsShape(
    collector: CastRayCollector,
    settings: CastRaySettings,
    ray: Raycast3,
    shape: Shape,
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
): void {
    const def = shapeDefs[shape.type];

    if (!def) {
        throw new Error(`Unknown shape type in castRayVsShape: ${shape.type}`);
    }

    def.castRay(
        collector,
        settings,
        ray,
        shape,
        subShapeId,
        subShapeIdBits,
        posX,
        posY,
        posZ,
        quatX,
        quatY,
        quatZ,
        quatW,
        scaleX,
        scaleY,
        scaleZ,
    );
}

export {
    CastShapeStatus,
    copyCastShapeHit,
    createAllCastShapeCollector,
    createAnyCastShapeCollector,
    createCastShapeHit,
    createClosestCastShapeCollector,
    createDefaultCastShapeSettings,
} from './cast-shape-vs-shape';
export type { CastShapeCollector, CastShapeHit, CastShapeSettings } from './cast-shape-vs-shape';

export function castShapeVsShape(
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
): void {
    const fn = collisionDispatch.castFns.get(shapeA.type)?.get(shapeB.type);

    if (!fn) {
        return;
    }

    fn(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        scaleAX,
        scaleAY,
        scaleAZ,
        dispAX,
        dispAY,
        dispAZ,
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}

export {
    copyCollideShapeHit,
    createAllCollideShapeCollector,
    createAnyCollideShapeCollector,
    createClosestCollideShapeCollector,
    createCollideShapeHit,
    createDefaultCollideShapeSettings,
} from './collide-shape-vs-shape';
export type { CollideShapeCollector, CollideShapeHit, CollideShapeSettings } from './collide-shape-vs-shape';
export { InternalEdgeRemovingCollector, type VoidedFeature } from './internal-edge-removing-collector';

const _internalEdgeRemovingCollector = new InternalEdgeRemovingCollector();
const _internalEdgeRemoval_modifiedSettings = createDefaultCollideShapeSettings();

/**
 * Collide two shapes using the registered collision dispatch handlers.
 * If no handler is registered for the shape pair, treats as no-op (no collision).
 */
export function collideShapeVsShape(
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
): void {
    const fn = collisionDispatch.collideFns.get(shapeA.type)?.get(shapeB.type);

    if (!fn) {
        return;
    }

    fn(
        collector,
        settings,
        shapeA,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        scaleAX,
        scaleAY,
        scaleAZ,
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}

/**
 * collide two shapes with enhanced internal edge removal to eliminate "ghost collisions"
 * when sliding across internal edges of triangle meshes or compound shapes.
 *
 * this wrapper:
 * - forces collectFaces=true and collideOnlyWithActiveEdges=false (required for algorithm)
 * - wraps the collector with InternalEdgeRemovingCollector
 * - automatically calls flush() after collision
 *
 * use this for objects that need smooth sliding across multi-primitive surfaces
 * (e.g., character controllers, rolling objects on triangle meshes).
 */
export function collideShapeVsShapeWithInternalEdgeRemoval(
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
): void {
    // copy settings and force required flags
    copyCollideShapeSettings(_internalEdgeRemoval_modifiedSettings, settings);
    _internalEdgeRemoval_modifiedSettings.collideOnlyWithActiveEdges = false; // must collide with all edges
    _internalEdgeRemoval_modifiedSettings.collectFaces = true; // must collect faces

    // setup reusable wrapper collector
    _internalEdgeRemovingCollector.reset();
    _internalEdgeRemovingCollector.set(collector);

    // delegate to standard collision
    collideShapeVsShape(
        _internalEdgeRemovingCollector,
        _internalEdgeRemoval_modifiedSettings,
        shapeA,
        subShapeIdA,
        subShapeIdBitsA,
        posAX,
        posAY,
        posAZ,
        quatAX,
        quatAY,
        quatAZ,
        quatAW,
        scaleAX,
        scaleAY,
        scaleAZ,
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        scaleBX,
        scaleBY,
        scaleBZ,
    );

    // flush delayed results
    _internalEdgeRemovingCollector.flush();
}

export {
    copyCollidePointHit,
    createAllCollidePointCollector,
    createAnyCollidePointCollector,
    createCollidePointHit,
    createDefaultCollidePointSettings,
} from './collide-point-vs-shape';
export type { CollidePointCollector, CollidePointHit, CollidePointSettings } from './collide-point-vs-shape';

export function collidePointVsShape(
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
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
): void {
    const def = shapeDefs[shapeB.type];

    if (!def) {
        throw new Error(`collidePointVsShape: Unsupported shape type ${shapeB.type}`);
    }

    def.collidePoint(
        collector,
        settings,
        pointX,
        pointY,
        pointZ,
        shapeB,
        subShapeIdB,
        subShapeIdBitsB,
        posBX,
        posBY,
        posBZ,
        quatBX,
        quatBY,
        quatBZ,
        quatBW,
        scaleBX,
        scaleBY,
        scaleBZ,
    );
}
