import { type Box3, box3, createSimplex2D, mat4, quat, randomFloat, type Raycast3, type Vec3, vec3 } from 'mathcat';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/Addons.js';
import type {
    CastRayCollector,
    CastRaySettings,
    CollidePointCollector,
    CollidePointSettings,
    CollideShapeCollector,
    CollideShapeSettings,
    RigidBody,
    Shape,
} from 'crashcat';
import {
    addBroadphaseLayer,
    addObjectLayer,
    ALL_SHAPE_DEFS,
    box,
    capsule,
    castRay,
    CastRayStatus,
    collideConvexVsConvexLocal,
    createCastRayHit,
    createClosestCastRayCollector,
    createCollidePointHit,
    createDefaultCastRaySettings,
    createWorld,
    createWorldSettings,
    defineShape,
    enableCollision,
    filter,
    getShapeSurfaceNormal,
    MotionType,
    registerShapes,
    reversedCollideShapeVsShape,
    rigidBody,
    setCollideShapeFn,
    ShapeCategory,
    shapeDefs,
    ShapeType,
    sphere,
    subShape,
    transformFace,
    updateWorld,
} from 'crashcat';
import { debugRenderer } from 'crashcat/three';
import * as debugUI from './debug/debug-ui';

/* voxel shape */

const CHUNK_BITS = 4;
const CHUNK_SIZE = 1 << CHUNK_BITS; // 16
const CHUNK_VOXEL_COUNT = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE; // 4096

enum VoxelFace {
    NEGATIVE_X = 0,
    POSITIVE_X = 1,
    NEGATIVE_Y = 2,
    POSITIVE_Y = 3,
    NEGATIVE_Z = 4,
    POSITIVE_Z = 5,
}

const VOXEL_FACE_COUNT = 6;

const VOXEL_FACE_NORMALS: readonly (readonly [number, number, number])[] = [
    [-1, 0, 0], // NEGATIVE_X
    [1, 0, 0], // POSITIVE_X
    [0, -1, 0], // NEGATIVE_Y
    [0, 1, 0], // POSITIVE_Y
    [0, 0, -1], // NEGATIVE_Z
    [0, 0, 1], // POSITIVE_Z
];

// get face from contact normal (for collision detection)
function getFaceFromNormal(nx: number, ny: number, nz: number): VoxelFace {
    const ax = Math.abs(nx);
    const ay = Math.abs(ny);
    const az = Math.abs(nz);

    if (ax > ay && ax > az) {
        return nx > 0 ? VoxelFace.POSITIVE_X : VoxelFace.NEGATIVE_X;
    } else if (ay > az) {
        return ny > 0 ? VoxelFace.POSITIVE_Y : VoxelFace.NEGATIVE_Y;
    } else {
        return nz > 0 ? VoxelFace.POSITIVE_Z : VoxelFace.NEGATIVE_Z;
    }
}

// derive voxel coordinates from hit position and face
// the face tells us which side was hit, helping resolve boundary ambiguity
const VOXEL_BOUNDARY_EPSILON = 1e-5;

function getVoxelFromHitPosition(outVoxel: Vec3, hitX: number, hitY: number, hitZ: number, face: VoxelFace): void {
    let vx = Math.floor(hitX);
    let vy = Math.floor(hitY);
    let vz = Math.floor(hitZ);

    // when hit is exactly on a boundary (integer coordinate),
    // for positive-facing hits the voxel is one less than floor would give
    switch (face) {
        case VoxelFace.POSITIVE_X:
            if (Math.abs(hitX - Math.round(hitX)) < VOXEL_BOUNDARY_EPSILON) {
                vx = Math.round(hitX) - 1;
            }
            break;
        case VoxelFace.POSITIVE_Y:
            if (Math.abs(hitY - Math.round(hitY)) < VOXEL_BOUNDARY_EPSILON) {
                vy = Math.round(hitY) - 1;
            }
            break;
        case VoxelFace.POSITIVE_Z:
            if (Math.abs(hitZ - Math.round(hitZ)) < VOXEL_BOUNDARY_EPSILON) {
                vz = Math.round(hitZ) - 1;
            }
            break;
    }

    outVoxel[0] = vx;
    outVoxel[1] = vy;
    outVoxel[2] = vz;
}

// get closest face for a point inside a voxel (for collidePoint)
function getClosestFaceForPoint(
    relX: number, // position relative to voxel center
    relY: number,
    relZ: number,
): VoxelFace {
    // find distance to each face (half extent is 0.5)
    const distX = 0.5 - Math.abs(relX);
    const distY = 0.5 - Math.abs(relY);
    const distZ = 0.5 - Math.abs(relZ);

    // smallest distance = closest face
    if (distX <= distY && distX <= distZ) {
        return relX > 0 ? VoxelFace.POSITIVE_X : VoxelFace.NEGATIVE_X;
    } else if (distY <= distZ) {
        return relY > 0 ? VoxelFace.POSITIVE_Y : VoxelFace.NEGATIVE_Y;
    } else {
        return relZ > 0 ? VoxelFace.POSITIVE_Z : VoxelFace.NEGATIVE_Z;
    }
}

type VoxelShapeSettings = {
    chunkBounds: Box3;
};

type VoxelShape = {
    type: ShapeType.USER_1;
    chunkSize: number;
    chunkBounds: Box3;
    chunkCount: Vec3;
    chunks: (VoxelShapeChunk | null)[];
    aabb: Box3;
    centerOfMass: Vec3;
    volume: number;
};

type VoxelShapeChunk = {
    cx: number;
    cy: number;
    cz: number;
    voxels: Uint8Array;
    sum: number;
};

function createVoxelShape(settings: VoxelShapeSettings): VoxelShape {
    const chunkBounds = settings.chunkBounds;

    // calculate chunk count from bounds
    const chunkCount = vec3.create();
    chunkCount[0] = chunkBounds[1][0] - chunkBounds[0][0];
    chunkCount[1] = chunkBounds[1][1] - chunkBounds[0][1];
    chunkCount[2] = chunkBounds[1][2] - chunkBounds[0][2];

    // flat array for chunks
    const totalChunks = chunkCount[0] * chunkCount[1] * chunkCount[2];
    const chunks = new Array<VoxelShapeChunk | null>(totalChunks).fill(null);

    // initialize aabb from chunk bounds
    const aabb = box3.create();
    const minVX = chunkBounds[0][0] * CHUNK_SIZE;
    const minVY = chunkBounds[0][1] * CHUNK_SIZE;
    const minVZ = chunkBounds[0][2] * CHUNK_SIZE;
    const maxVX = chunkBounds[1][0] * CHUNK_SIZE;
    const maxVY = chunkBounds[1][1] * CHUNK_SIZE;
    const maxVZ = chunkBounds[1][2] * CHUNK_SIZE;
    vec3.set(aabb[0], minVX, minVY, minVZ);
    vec3.set(aabb[1], maxVX, maxVY, maxVZ);

    const centerOfMass = vec3.create();
    const volume = 0;

    return {
        type: ShapeType.USER_1,
        chunkSize: CHUNK_SIZE,
        chunkBounds: box3.clone(chunkBounds),
        chunkCount,
        chunks,
        aabb,
        centerOfMass,
        volume,
    };
}

function getVoxel(shape: VoxelShape, wx: number, wy: number, wz: number): boolean {
    // compute chunk indices
    const cx = Math.floor(wx / CHUNK_SIZE);
    const cy = Math.floor(wy / CHUNK_SIZE);
    const cz = Math.floor(wz / CHUNK_SIZE);

    // check bounds
    if (cx < shape.chunkBounds[0][0] || cx >= shape.chunkBounds[1][0]) return false;
    if (cy < shape.chunkBounds[0][1] || cy >= shape.chunkBounds[1][1]) return false;
    if (cz < shape.chunkBounds[0][2] || cz >= shape.chunkBounds[1][2]) return false;

    // get chunk
    const chunkIndex =
        (cz - shape.chunkBounds[0][2]) * shape.chunkCount[1] * shape.chunkCount[0] +
        (cy - shape.chunkBounds[0][1]) * shape.chunkCount[0] +
        (cx - shape.chunkBounds[0][0]);
    const chunk = shape.chunks[chunkIndex];

    if (!chunk) return false;

    // compute local voxel coords
    let lx = wx % CHUNK_SIZE;
    let ly = wy % CHUNK_SIZE;
    let lz = wz % CHUNK_SIZE;

    // handle negative coordinates
    if (lx < 0) lx += CHUNK_SIZE;
    if (ly < 0) ly += CHUNK_SIZE;
    if (lz < 0) lz += CHUNK_SIZE;

    // get voxel
    const voxelIndex = lz * CHUNK_SIZE * CHUNK_SIZE + ly * CHUNK_SIZE + lx;

    return chunk.voxels[voxelIndex] !== 0;
}

function getVoxelRelative(shape: VoxelShape, chunk: VoxelShapeChunk, lx: number, ly: number, lz: number): boolean {
    // if within chunk bounds, access directly
    if (lx >= 0 && lx < CHUNK_SIZE && ly >= 0 && ly < CHUNK_SIZE && lz >= 0 && lz < CHUNK_SIZE) {
        const voxelIndex = lz * CHUNK_SIZE * CHUNK_SIZE + ly * CHUNK_SIZE + lx;
        return chunk.voxels[voxelIndex] !== 0;
    }

    // outside chunk bounds, convert to world coords and use getVoxel
    const worldX = chunk.cx * CHUNK_SIZE + lx;
    const worldY = chunk.cy * CHUNK_SIZE + ly;
    const worldZ = chunk.cz * CHUNK_SIZE + lz;

    return getVoxel(shape, worldX, worldY, worldZ);
}

function setVoxel(shape: VoxelShape, voxelX: number, voxelY: number, voxelZ: number, filled: boolean): void {
    // compute chunk indices
    const cx = Math.floor(voxelX / CHUNK_SIZE);
    const cy = Math.floor(voxelY / CHUNK_SIZE);
    const cz = Math.floor(voxelZ / CHUNK_SIZE);

    // check bounds
    if (cx < shape.chunkBounds[0][0] || cx >= shape.chunkBounds[1][0]) return;
    if (cy < shape.chunkBounds[0][1] || cy >= shape.chunkBounds[1][1]) return;
    if (cz < shape.chunkBounds[0][2] || cz >= shape.chunkBounds[1][2]) return;

    // get or create chunk
    const chunkIndex =
        (cz - shape.chunkBounds[0][2]) * shape.chunkCount[1] * shape.chunkCount[0] +
        (cy - shape.chunkBounds[0][1]) * shape.chunkCount[0] +
        (cx - shape.chunkBounds[0][0]);

    let chunk = shape.chunks[chunkIndex];
    if (!chunk) {
        chunk = {
            voxels: new Uint8Array(CHUNK_VOXEL_COUNT),
            sum: 0,
            cx,
            cy,
            cz,
        };
        shape.chunks[chunkIndex] = chunk;
    }

    // compute local voxel coords
    let lx = voxelX % CHUNK_SIZE;
    let ly = voxelY % CHUNK_SIZE;
    let lz = voxelZ % CHUNK_SIZE;

    // handle negative coordinates
    if (lx < 0) lx += CHUNK_SIZE;
    if (ly < 0) ly += CHUNK_SIZE;
    if (lz < 0) lz += CHUNK_SIZE;

    // set voxel and update sum
    const voxelIndex = lz * CHUNK_SIZE * CHUNK_SIZE + ly * CHUNK_SIZE + lx;
    const wasFilled = chunk.voxels[voxelIndex] !== 0;
    const nowFilled = filled;

    chunk.voxels[voxelIndex] = filled ? 1 : 0;

    if (wasFilled && !nowFilled) {
        chunk.sum--;
    } else if (!wasFilled && nowFilled) {
        chunk.sum++;
    }
}

const voxelShapeDef = defineShape<VoxelShape>({
    type: ShapeType.USER_1,
    category: ShapeCategory.OTHER,
    computeMassProperties(out, _shape): void {
        // only static voxel shapes for now, no mass
        out.mass = 0;
    },
    castRay: castRayVsVoxels,
    collidePoint: collidePointVsVoxels,
    getSurfaceNormal(out, _shape, subShapeId) {
        // decode face from subShapeId (only 3 bits)
        subShape.popIndex(_subShapeIdPopResult, subShapeId, VOXEL_FACE_COUNT);
        const hitFace = _subShapeIdPopResult.value as VoxelFace;

        // get normal directly from face lookup table
        const faceNormal = VOXEL_FACE_NORMALS[hitFace];
        out.normal[0] = faceNormal[0];
        out.normal[1] = faceNormal[1];
        out.normal[2] = faceNormal[2];
    },
    getSupportingFace(ioResult, _direction, _shape, subShapeId) {
        // decode face from subShapeId (only 3 bits)
        subShape.popIndex(_subShapeIdPopResult, subShapeId, VOXEL_FACE_COUNT);
        const hitFace = _subShapeIdPopResult.value as VoxelFace;

        // derive voxel position from out.position (contact point) and face
        getVoxelFromHitPosition(
            _getSupportingFace_voxelPos,
            ioResult.position[0],
            ioResult.position[1],
            ioResult.position[2],
            hitFace,
        );

        const vx = _getSupportingFace_voxelPos[0];
        const vy = _getSupportingFace_voxelPos[1];
        const vz = _getSupportingFace_voxelPos[2];

        const face = ioResult.face;

        // build the face quad based on hitFace
        // vertices are counter-clockwise when viewed from outside
        switch (hitFace) {
            case VoxelFace.POSITIVE_Y: {
                // top face (y = vy + 1)
                const y = vy + 1;
                face.vertices[0] = vx;
                face.vertices[1] = y;
                face.vertices[2] = vz;
                face.vertices[3] = vx + 1;
                face.vertices[4] = y;
                face.vertices[5] = vz;
                face.vertices[6] = vx + 1;
                face.vertices[7] = y;
                face.vertices[8] = vz + 1;
                face.vertices[9] = vx;
                face.vertices[10] = y;
                face.vertices[11] = vz + 1;
                break;
            }
            case VoxelFace.NEGATIVE_Y: {
                // bottom face (y = vy)
                const y = vy;
                face.vertices[0] = vx;
                face.vertices[1] = y;
                face.vertices[2] = vz;
                face.vertices[3] = vx;
                face.vertices[4] = y;
                face.vertices[5] = vz + 1;
                face.vertices[6] = vx + 1;
                face.vertices[7] = y;
                face.vertices[8] = vz + 1;
                face.vertices[9] = vx + 1;
                face.vertices[10] = y;
                face.vertices[11] = vz;
                break;
            }
            case VoxelFace.POSITIVE_X: {
                // right face (x = vx + 1)
                const x = vx + 1;
                face.vertices[0] = x;
                face.vertices[1] = vy;
                face.vertices[2] = vz;
                face.vertices[3] = x;
                face.vertices[4] = vy;
                face.vertices[5] = vz + 1;
                face.vertices[6] = x;
                face.vertices[7] = vy + 1;
                face.vertices[8] = vz + 1;
                face.vertices[9] = x;
                face.vertices[10] = vy + 1;
                face.vertices[11] = vz;
                break;
            }
            case VoxelFace.NEGATIVE_X: {
                // left face (x = vx)
                const x = vx;
                face.vertices[0] = x;
                face.vertices[1] = vy;
                face.vertices[2] = vz;
                face.vertices[3] = x;
                face.vertices[4] = vy + 1;
                face.vertices[5] = vz;
                face.vertices[6] = x;
                face.vertices[7] = vy + 1;
                face.vertices[8] = vz + 1;
                face.vertices[9] = x;
                face.vertices[10] = vy;
                face.vertices[11] = vz + 1;
                break;
            }
            case VoxelFace.POSITIVE_Z: {
                // front face (z = vz + 1)
                const z = vz + 1;
                face.vertices[0] = vx;
                face.vertices[1] = vy;
                face.vertices[2] = z;
                face.vertices[3] = vx + 1;
                face.vertices[4] = vy;
                face.vertices[5] = z;
                face.vertices[6] = vx + 1;
                face.vertices[7] = vy + 1;
                face.vertices[8] = z;
                face.vertices[9] = vx;
                face.vertices[10] = vy + 1;
                face.vertices[11] = z;
                break;
            }
            case VoxelFace.NEGATIVE_Z: {
                // back face (z = vz)
                const z = vz;
                face.vertices[0] = vx;
                face.vertices[1] = vy;
                face.vertices[2] = z;
                face.vertices[3] = vx;
                face.vertices[4] = vy + 1;
                face.vertices[5] = z;
                face.vertices[6] = vx + 1;
                face.vertices[7] = vy + 1;
                face.vertices[8] = z;
                face.vertices[9] = vx + 1;
                face.vertices[10] = vy;
                face.vertices[11] = z;
                break;
            }
        }

        face.numVertices = 4;
        transformFace(face, ioResult.position, ioResult.quaternion, ioResult.scale);
    },
    register() {
        // voxels x convex shapes
        for (const shapeDef of Object.values(shapeDefs)) {
            if (shapeDef.category === ShapeCategory.CONVEX) {
                setCollideShapeFn(ShapeType.USER_1, shapeDef.type, collideVoxelsVsConvex);
                setCollideShapeFn(shapeDef.type, ShapeType.USER_1, reversedCollideShapeVsShape(collideVoxelsVsConvex));

                // cast not implemented for this example!
            }
        }
    },
});

const _castRayVsVoxels_quat = quat.create();
const _castRayVsVoxels_rayOriginLocal = vec3.create();
const _castRayVsVoxels_rayDirectionLocal = vec3.create();
const _castRayVsVoxels_hit = createCastRayHit();
const _castRayVsVoxels_subShapeIdBuilder = subShape.builder();

function castRayVsVoxels(
    collector: CastRayCollector,
    _settings: CastRaySettings,
    ray: Raycast3,
    shape: VoxelShape,
    subShapeId: number,
    subShapeIdBitsB: number,
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
    // transform ray to voxel local space
    const rayOrigin = ray.origin;
    const rayDir = ray.direction;

    // translate origin to local space
    _castRayVsVoxels_rayOriginLocal[0] = rayOrigin[0] - posX;
    _castRayVsVoxels_rayOriginLocal[1] = rayOrigin[1] - posY;
    _castRayVsVoxels_rayOriginLocal[2] = rayOrigin[2] - posZ;

    // rotate by inverse quaternion
    quat.set(_castRayVsVoxels_quat, quatX, quatY, quatZ, quatW);
    quat.conjugate(_castRayVsVoxels_quat, _castRayVsVoxels_quat);
    vec3.transformQuat(_castRayVsVoxels_rayOriginLocal, _castRayVsVoxels_rayOriginLocal, _castRayVsVoxels_quat);

    // rotate direction (no translation)
    vec3.transformQuat(_castRayVsVoxels_rayDirectionLocal, rayDir, _castRayVsVoxels_quat);

    // apply scale
    _castRayVsVoxels_rayOriginLocal[0] /= Math.abs(scaleX);
    _castRayVsVoxels_rayOriginLocal[1] /= Math.abs(scaleY);
    _castRayVsVoxels_rayOriginLocal[2] /= Math.abs(scaleZ);

    _castRayVsVoxels_rayDirectionLocal[0] /= Math.abs(scaleX);
    _castRayVsVoxels_rayDirectionLocal[1] /= Math.abs(scaleY);
    _castRayVsVoxels_rayDirectionLocal[2] /= Math.abs(scaleZ);

    // normalize direction after scaling
    const dirLength = Math.sqrt(
        _castRayVsVoxels_rayDirectionLocal[0] * _castRayVsVoxels_rayDirectionLocal[0] +
            _castRayVsVoxels_rayDirectionLocal[1] * _castRayVsVoxels_rayDirectionLocal[1] +
            _castRayVsVoxels_rayDirectionLocal[2] * _castRayVsVoxels_rayDirectionLocal[2],
    );

    if (dirLength < 1e-10) {
        return; // degenerate ray
    }

    const invDirLength = 1.0 / dirLength;
    _castRayVsVoxels_rayDirectionLocal[0] *= invDirLength;
    _castRayVsVoxels_rayDirectionLocal[1] *= invDirLength;
    _castRayVsVoxels_rayDirectionLocal[2] *= invDirLength;

    const maxDistance = ray.length * dirLength;

    // get voxel bounds in local space
    const boundsMinX = shape.chunkBounds[0][0] * CHUNK_SIZE;
    const boundsMinY = shape.chunkBounds[0][1] * CHUNK_SIZE;
    const boundsMinZ = shape.chunkBounds[0][2] * CHUNK_SIZE;
    const boundsMaxX = shape.chunkBounds[1][0] * CHUNK_SIZE;
    const boundsMaxY = shape.chunkBounds[1][1] * CHUNK_SIZE;
    const boundsMaxZ = shape.chunkBounds[1][2] * CHUNK_SIZE;

    // dda algorithm setup
    const ox = _castRayVsVoxels_rayOriginLocal[0];
    const oy = _castRayVsVoxels_rayOriginLocal[1];
    const oz = _castRayVsVoxels_rayOriginLocal[2];
    const dx = _castRayVsVoxels_rayDirectionLocal[0];
    const dy = _castRayVsVoxels_rayDirectionLocal[1];
    const dz = _castRayVsVoxels_rayDirectionLocal[2];

    // compute ray-AABB intersection to find starting point
    const invDx = Math.abs(dx) > 1e-10 ? 1.0 / dx : Infinity;
    const invDy = Math.abs(dy) > 1e-10 ? 1.0 / dy : Infinity;
    const invDz = Math.abs(dz) > 1e-10 ? 1.0 / dz : Infinity;

    const t1x = (boundsMinX - ox) * invDx;
    const t2x = (boundsMaxX - ox) * invDx;
    const t1y = (boundsMinY - oy) * invDy;
    const t2y = (boundsMaxY - oy) * invDy;
    const t1z = (boundsMinZ - oz) * invDz;
    const t2z = (boundsMaxZ - oz) * invDz;

    const tMinX = Math.min(t1x, t2x);
    const tMaxX_bounds = Math.max(t1x, t2x);
    const tMinY = Math.min(t1y, t2y);
    const tMaxY_bounds = Math.max(t1y, t2y);
    const tMinZ = Math.min(t1z, t2z);
    const tMaxZ_bounds = Math.max(t1z, t2z);

    const tEnter = Math.max(0, Math.max(tMinX, Math.max(tMinY, tMinZ)));
    const tExit = Math.min(maxDistance, Math.min(tMaxX_bounds, Math.min(tMaxY_bounds, tMaxZ_bounds)));

    // check if ray intersects the voxel volume
    if (tEnter >= tExit || tExit < 0) {
        return; // no intersection with voxel bounds
    }

    // start DDA from entry point (or origin if already inside)
    const startX = ox + dx * tEnter;
    const startY = oy + dy * tEnter;
    const startZ = oz + dz * tEnter;

    // step direction
    const stepX = dx > 0 ? 1 : dx < 0 ? -1 : 0;
    const stepY = dy > 0 ? 1 : dy < 0 ? -1 : 0;
    const stepZ = dz > 0 ? 1 : dz < 0 ? -1 : 0;

    // current voxel position
    // When ray enters from outside on a boundary (integer position), we need to handle
    // the case where floor() gives us the wrong voxel for negative-direction rays.
    // If entering at exactly an integer boundary with negative step, we want the voxel
    // on the inside (one less than floor).
    let vx = Math.floor(startX);
    let vy = Math.floor(startY);
    let vz = Math.floor(startZ);

    // Correct for boundary entry with negative step direction
    // If we entered from outside (tEnter > 0) and landed exactly on an integer boundary,
    // and we're moving in negative direction, adjust to the correct voxel
    const BOUNDARY_EPSILON = 1e-8;
    if (tEnter > 0) {
        // Check if startX is very close to an integer (on a boundary)
        const fracX = startX - vx;
        const fracY = startY - vy;
        const fracZ = startZ - vz;

        if (stepX < 0 && fracX < BOUNDARY_EPSILON) {
            vx -= 1;
        }
        if (stepY < 0 && fracY < BOUNDARY_EPSILON) {
            vy -= 1;
        }
        if (stepZ < 0 && fracZ < BOUNDARY_EPSILON) {
            vz -= 1;
        }
    }

    // tMax = distance to next voxel boundary along each axis
    let tMaxX = stepX > 0 ? (vx + 1 - ox) * invDx : stepX < 0 ? (vx - ox) * invDx : Infinity;

    let tMaxY = stepY > 0 ? (vy + 1 - oy) * invDy : stepY < 0 ? (vy - oy) * invDy : Infinity;

    let tMaxZ = stepZ > 0 ? (vz + 1 - oz) * invDz : stepZ < 0 ? (vz - oz) * invDz : Infinity;

    // tDelta = distance to traverse one voxel along each axis
    const tDeltaX = Math.abs(invDx);
    const tDeltaY = Math.abs(invDy);
    const tDeltaZ = Math.abs(invDz);

    // traverse voxel grid with DDA
    let currentDistance = tEnter; // start from entry point

    // track which axis we last stepped on (to determine hit face)
    // 0 = X axis, 1 = Y axis, 2 = Z axis, -1 = initial entry
    let lastStepAxis = -1;

    // for initial entry, determine which face we entered from based on tEnter
    if (tEnter > 0) {
        // we entered from outside, determine which axis boundary we crossed
        if (tMinX >= tMinY && tMinX >= tMinZ) {
            lastStepAxis = 0; // entered via X boundary
        } else if (tMinY >= tMinZ) {
            lastStepAxis = 1; // entered via Y boundary
        } else {
            lastStepAxis = 2; // entered via Z boundary
        }
    }

    // maximum steps: a ray can cross at most ceil(distance/minDelta) voxels per axis
    // for a diagonal ray, total voxels visited <= sum of crossings per axis + 1
    // use the ray's travel distance to compute worst case
    const rayTravel = maxDistance - tEnter;
    const maxStepsX = tDeltaX < Infinity ? Math.ceil(rayTravel / tDeltaX) : 0;
    const maxStepsY = tDeltaY < Infinity ? Math.ceil(rayTravel / tDeltaY) : 0;
    const maxStepsZ = tDeltaZ < Infinity ? Math.ceil(rayTravel / tDeltaZ) : 0;
    const maxSteps = maxStepsX + maxStepsY + maxStepsZ + 1;

    for (let step = 0; step < maxSteps; step++) {
        // compute current chunk coordinates
        const cx = Math.floor(vx / CHUNK_SIZE);
        const cy = Math.floor(vy / CHUNK_SIZE);
        const cz = Math.floor(vz / CHUNK_SIZE);

        // check if chunk is in bounds
        const inBounds =
            cx >= shape.chunkBounds[0][0] &&
            cx < shape.chunkBounds[1][0] &&
            cy >= shape.chunkBounds[0][1] &&
            cy < shape.chunkBounds[1][1] &&
            cz >= shape.chunkBounds[0][2] &&
            cz < shape.chunkBounds[1][2];

        // if chunk doesn't exist or is empty, skip to chunk exit boundary
        if (!inBounds) {
            return; // ray exited voxel shape bounds
        }

        const chunkIndex =
            (cz - shape.chunkBounds[0][2]) * shape.chunkCount[1] * shape.chunkCount[0] +
            (cy - shape.chunkBounds[0][1]) * shape.chunkCount[0] +
            (cx - shape.chunkBounds[0][0]);
        const chunk = shape.chunks[chunkIndex];

        if (!chunk || chunk.sum === 0) {
            // chunk is empty - skip to chunk boundary
            const chunkMinX = cx * CHUNK_SIZE;
            const chunkMinY = cy * CHUNK_SIZE;
            const chunkMinZ = cz * CHUNK_SIZE;
            const chunkMaxX = chunkMinX + CHUNK_SIZE;
            const chunkMaxY = chunkMinY + CHUNK_SIZE;
            const chunkMaxZ = chunkMinZ + CHUNK_SIZE;

            // find distance to exit this chunk
            let tExitX = Infinity;
            let tExitY = Infinity;
            let tExitZ = Infinity;

            if (dx > 0) {
                const t = (chunkMaxX - ox) * invDx;
                if (t > currentDistance) tExitX = t;
            } else if (dx < 0) {
                const t = (chunkMinX - ox) * invDx;
                if (t > currentDistance) tExitX = t;
            }

            if (dy > 0) {
                const t = (chunkMaxY - oy) * invDy;
                if (t > currentDistance) tExitY = t;
            } else if (dy < 0) {
                const t = (chunkMinY - oy) * invDy;
                if (t > currentDistance) tExitY = t;
            }

            if (dz > 0) {
                const t = (chunkMaxZ - oz) * invDz;
                if (t > currentDistance) tExitZ = t;
            } else if (dz < 0) {
                const t = (chunkMinZ - oz) * invDz;
                if (t > currentDistance) tExitZ = t;
            }

            // find the minimum valid exit distance
            const tExit = Math.min(tExitX, tExitY, tExitZ);

            if (tExit >= maxDistance || tExit === Infinity) {
                return; // ray doesn't reach next chunk
            }

            // update lastStepAxis based on which boundary we crossed
            if (tExit === tExitX) {
                lastStepAxis = 0;
            } else if (tExit === tExitY) {
                lastStepAxis = 1;
            } else {
                lastStepAxis = 2;
            }

            // jump to the chunk boundary voxel
            const epsilon = 0.0001;
            const exitX = ox + dx * (tExit + epsilon);
            const exitY = oy + dy * (tExit + epsilon);
            const exitZ = oz + dz * (tExit + epsilon);

            vx = Math.floor(exitX);
            vy = Math.floor(exitY);
            vz = Math.floor(exitZ);

            // recalculate tMax values for the new position
            if (Math.abs(dx) > 1e-10) {
                tMaxX = stepX > 0 ? (vx + 1 - ox) * invDx : stepX < 0 ? (vx - ox) * invDx : Infinity;
            }
            if (Math.abs(dy) > 1e-10) {
                tMaxY = stepY > 0 ? (vy + 1 - oy) * invDy : stepY < 0 ? (vy - oy) * invDy : Infinity;
            }
            if (Math.abs(dz) > 1e-10) {
                tMaxZ = stepZ > 0 ? (vz + 1 - oz) * invDz : stepZ < 0 ? (vz - oz) * invDz : Infinity;
            }

            currentDistance = tExit;
            continue;
        }

        // chunk exists and has voxels - check if current voxel is filled
        if (getVoxel(shape, vx, vy, vz)) {
            // currentDistance is when we entered this voxel:
            // - First iteration: currentDistance = tEnter (entry into voxel bounds)
            // - After stepping: currentDistance = the tMax value we crossed to enter this voxel
            const fraction = Math.max(0, currentDistance) / maxDistance;

            if (fraction <= collector.earlyOutFraction) {
                // determine which face was hit based on which axis we stepped on
                // the hit face is opposite to the ray direction on that axis
                let hitFace: VoxelFace;
                if (lastStepAxis === 0) {
                    hitFace = stepX > 0 ? VoxelFace.NEGATIVE_X : VoxelFace.POSITIVE_X;
                } else if (lastStepAxis === 1) {
                    hitFace = stepY > 0 ? VoxelFace.NEGATIVE_Y : VoxelFace.POSITIVE_Y;
                } else if (lastStepAxis === 2) {
                    hitFace = stepZ > 0 ? VoxelFace.NEGATIVE_Z : VoxelFace.POSITIVE_Z;
                } else {
                    // lastStepAxis === -1: ray started inside this voxel
                    // pick face based on dominant ray direction (face we would have entered from)
                    const absDx = Math.abs(dx);
                    const absDy = Math.abs(dy);
                    const absDz = Math.abs(dz);
                    if (absDx > absDy && absDx > absDz) {
                        hitFace = dx > 0 ? VoxelFace.NEGATIVE_X : VoxelFace.POSITIVE_X;
                    } else if (absDy > absDz) {
                        hitFace = dy > 0 ? VoxelFace.NEGATIVE_Y : VoxelFace.POSITIVE_Y;
                    } else {
                        hitFace = dz > 0 ? VoxelFace.NEGATIVE_Z : VoxelFace.POSITIVE_Z;
                    }
                }

                // encode only the face (3 bits) instead of chunk+voxel indices
                _castRayVsVoxels_subShapeIdBuilder.value = subShapeId;
                _castRayVsVoxels_subShapeIdBuilder.currentBit = subShapeIdBitsB;
                subShape.pushIndex(
                    _castRayVsVoxels_subShapeIdBuilder,
                    _castRayVsVoxels_subShapeIdBuilder,
                    hitFace,
                    VOXEL_FACE_COUNT,
                );

                _castRayVsVoxels_hit.status = CastRayStatus.COLLIDING;
                _castRayVsVoxels_hit.fraction = fraction;
                _castRayVsVoxels_hit.subShapeId = _castRayVsVoxels_subShapeIdBuilder.value;
                _castRayVsVoxels_hit.bodyIdB = collector.bodyIdB;
                collector.addHit(_castRayVsVoxels_hit);
            }
            return;
        }

        // advance to next voxel
        if (tMaxX < tMaxY) {
            if (tMaxX < tMaxZ) {
                if (tMaxX > maxDistance) return;
                vx += stepX;
                currentDistance = tMaxX;
                tMaxX += tDeltaX;
                lastStepAxis = 0;
            } else {
                if (tMaxZ > maxDistance) return;
                vz += stepZ;
                currentDistance = tMaxZ;
                tMaxZ += tDeltaZ;
                lastStepAxis = 2;
            }
        } else {
            if (tMaxY < tMaxZ) {
                if (tMaxY > maxDistance) return;
                vy += stepY;
                currentDistance = tMaxY;
                tMaxY += tDeltaY;
                lastStepAxis = 1;
            } else {
                if (tMaxZ > maxDistance) return;
                vz += stepZ;
                currentDistance = tMaxZ;
                tMaxZ += tDeltaZ;
                lastStepAxis = 2;
            }
        }
    }
}

const _subShapeIdPopResult = subShape.popResult();
const _getSupportingFace_voxelPos = vec3.create();

const _collidePointVsVoxels_hit = createCollidePointHit();
const _collidePointVsVoxels_quatB = quat.create();
const _collidePointVsVoxels_posB = vec3.create();
const _collidePointVsVoxels_subShapeIdBuilder = subShape.builder();

function collidePointVsVoxels(
    collector: CollidePointCollector,
    _settings: CollidePointSettings,
    pointX: number,
    pointY: number,
    pointZ: number,
    shapeB: VoxelShape,
    subShapeIdB: number,
    _subShapeIdBitsB: number,
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
    // transform point to voxel's local space
    const localX = pointX - posBX;
    const localY = pointY - posBY;
    const localZ = pointZ - posBZ;

    // rotate point by inverse of voxel rotation
    quat.set(_collidePointVsVoxels_quatB, quatBX, quatBY, quatBZ, quatBW);
    quat.conjugate(_collidePointVsVoxels_quatB, _collidePointVsVoxels_quatB);
    vec3.set(_collidePointVsVoxels_posB, localX, localY, localZ);
    vec3.transformQuat(_collidePointVsVoxels_posB, _collidePointVsVoxels_posB, _collidePointVsVoxels_quatB);

    // apply accumulated scale
    const scaledX = _collidePointVsVoxels_posB[0] / Math.abs(scaleBX);
    const scaledY = _collidePointVsVoxels_posB[1] / Math.abs(scaleBY);
    const scaledZ = _collidePointVsVoxels_posB[2] / Math.abs(scaleBZ);

    // convert to voxel coordinates (floor to get voxel indices)
    const vx = Math.floor(scaledX);
    const vy = Math.floor(scaledY);
    const vz = Math.floor(scaledZ);

    // check if this voxel is filled
    if (getVoxel(shapeB, vx, vy, vz)) {
        // compute point's position relative to voxel center
        const relX = scaledX - (vx + 0.5);
        const relY = scaledY - (vy + 0.5);
        const relZ = scaledZ - (vz + 0.5);

        // determine closest face for the point
        const hitFace = getClosestFaceForPoint(relX, relY, relZ);

        // encode only the face (3 bits)
        _collidePointVsVoxels_subShapeIdBuilder.value = subShapeIdB;
        _collidePointVsVoxels_subShapeIdBuilder.currentBit = _subShapeIdBitsB;
        subShape.pushIndex(
            _collidePointVsVoxels_subShapeIdBuilder,
            _collidePointVsVoxels_subShapeIdBuilder,
            hitFace,
            VOXEL_FACE_COUNT,
        );

        _collidePointVsVoxels_hit.subShapeIdB = _collidePointVsVoxels_subShapeIdBuilder.value;
        _collidePointVsVoxels_hit.bodyIdB = collector.bodyIdB;
        collector.addHit(_collidePointVsVoxels_hit);
    }
}

const _voxelBoxShape = box.create({ halfExtents: vec3.fromValues(0.5, 0.5, 0.5), convexRadius: 0.05 });
const _voxelBoxPosition = vec3.create();
const _voxelBoxQuaternion = quat.fromValues(0, 0, 0, 1);
const _voxelBoxScale = vec3.fromValues(1, 1, 1);

const _collideVoxelsVsConvex_quatAInv = quat.create();
const _collideVoxelsVsConvex_posB = vec3.create();
const _collideVoxelsVsConvex_quatB = quat.create();
const _collideVoxelsVsConvex_scaleB = vec3.create();
const _collideVoxelsVsConvex_posBRelative = vec3.create();
const _collideVoxelsVsConvex_posBInA = vec3.create();
const _collideVoxelsVsConvex_quatBInA = quat.create();
const _collideVoxelsVsConvex_scaleAInv = vec3.create();
const _collideVoxelsVsConvex_aabbMatrix = mat4.create();
const _collideVoxelsVsConvex_convexAABB = box3.create();
const _collideVoxelsVsConvex_subShapeIdBuilder = subShape.builder();
const _collideVoxelsVsConvex_posBRelativeToBox = vec3.create();

function collideVoxelsVsConvex(
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shapeA: Shape,
    subShapeIdA: number,
    _subShapeIdBitsA: number,
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
    _subShapeIdBitsB: number,
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
    const voxels = shapeA as VoxelShape;

    // transform convex (B) into voxel (A) local space
    const posAInvX = -posAX;
    const posAInvY = -posAY;
    const posAInvZ = -posAZ;

    quat.set(_collideVoxelsVsConvex_quatAInv, quatAX, quatAY, quatAZ, quatAW);
    quat.conjugate(_collideVoxelsVsConvex_quatAInv, _collideVoxelsVsConvex_quatAInv);

    vec3.set(_collideVoxelsVsConvex_posB, posBX, posBY, posBZ);
    quat.set(_collideVoxelsVsConvex_quatB, quatBX, quatBY, quatBZ, quatBW);
    vec3.set(_collideVoxelsVsConvex_scaleB, scaleBX, scaleBY, scaleBZ);

    // translate B relative to A
    vec3.set(_collideVoxelsVsConvex_posBRelative, posAInvX, posAInvY, posAInvZ);
    vec3.add(_collideVoxelsVsConvex_posBRelative, _collideVoxelsVsConvex_posB, _collideVoxelsVsConvex_posBRelative);

    // rotate B into A's local space
    vec3.transformQuat(_collideVoxelsVsConvex_posBInA, _collideVoxelsVsConvex_posBRelative, _collideVoxelsVsConvex_quatAInv);

    quat.multiply(_collideVoxelsVsConvex_quatBInA, _collideVoxelsVsConvex_quatAInv, _collideVoxelsVsConvex_quatB);

    // account for A's scale
    vec3.set(_collideVoxelsVsConvex_scaleAInv, 1.0 / Math.abs(scaleAX), 1.0 / Math.abs(scaleAY), 1.0 / Math.abs(scaleAZ));
    vec3.mul(_collideVoxelsVsConvex_posBInA, _collideVoxelsVsConvex_posBInA, _collideVoxelsVsConvex_scaleAInv);

    // compute convex shape B's AABB in voxel space A
    mat4.fromRotationTranslationScale(
        _collideVoxelsVsConvex_aabbMatrix,
        _collideVoxelsVsConvex_quatBInA,
        _collideVoxelsVsConvex_posBInA,
        _collideVoxelsVsConvex_scaleB,
    );

    box3.transformMat4(_collideVoxelsVsConvex_convexAABB, shapeB.aabb, _collideVoxelsVsConvex_aabbMatrix);

    // expand slightly for separation distance tolerance
    box3.expandByMargin(_collideVoxelsVsConvex_convexAABB, _collideVoxelsVsConvex_convexAABB, settings.maxSeparationDistance);

    // get integer voxel bounds
    const minVX = Math.floor(_collideVoxelsVsConvex_convexAABB[0][0]);
    const minVY = Math.floor(_collideVoxelsVsConvex_convexAABB[0][1]);
    const minVZ = Math.floor(_collideVoxelsVsConvex_convexAABB[0][2]);
    const maxVX = Math.ceil(_collideVoxelsVsConvex_convexAABB[1][0]);
    const maxVY = Math.ceil(_collideVoxelsVsConvex_convexAABB[1][1]);
    const maxVZ = Math.ceil(_collideVoxelsVsConvex_convexAABB[1][2]);

    for (let vz = minVZ; vz <= maxVZ; vz++) {
        for (let vy = minVY; vy <= maxVY; vy++) {
            for (let vx = minVX; vx <= maxVX; vx++) {
                if (!getVoxel(voxels, vx, vy, vz)) continue;

                // voxel center position in voxel space
                vec3.set(_voxelBoxPosition, vx + 0.5, vy + 0.5, vz + 0.5);

                // compute B's position relative to the voxel box (both in voxel space)
                vec3.sub(_collideVoxelsVsConvex_posBRelativeToBox, _collideVoxelsVsConvex_posBInA, _voxelBoxPosition);

                // determine face from direction of convex relative to voxel center
                // this is a heuristic - the actual contact normal may differ slightly
                const hitFace = getFaceFromNormal(
                    _collideVoxelsVsConvex_posBRelativeToBox[0],
                    _collideVoxelsVsConvex_posBRelativeToBox[1],
                    _collideVoxelsVsConvex_posBRelativeToBox[2],
                );

                // encode only the face (3 bits)
                _collideVoxelsVsConvex_subShapeIdBuilder.value = subShapeIdA;
                _collideVoxelsVsConvex_subShapeIdBuilder.currentBit = _subShapeIdBitsA;
                subShape.pushIndex(
                    _collideVoxelsVsConvex_subShapeIdBuilder,
                    _collideVoxelsVsConvex_subShapeIdBuilder,
                    hitFace,
                    VOXEL_FACE_COUNT,
                );

                // test voxel box vs convex shape
                // voxel box is at origin with identity rotation (in its own local space)
                // convex B is at posBRelativeToBox relative to the voxel box
                collideConvexVsConvexLocal(
                    collector,
                    settings,
                    _voxelBoxShape,
                    _collideVoxelsVsConvex_subShapeIdBuilder.value,
                    shapeB,
                    subShapeIdB,
                    _collideVoxelsVsConvex_posBRelativeToBox,
                    _collideVoxelsVsConvex_quatBInA,
                    _voxelBoxScale,
                    _collideVoxelsVsConvex_scaleB,
                    _voxelBoxPosition,
                    _voxelBoxQuaternion,
                );

                if (collector.shouldEarlyOut()) {
                    return;
                }
            }
        }
    }
}

/* voxel meshing */

type CulledMesherResult = {
    positions: Float32Array;
    indices: Uint32Array;
    normals: Float32Array;
};

const DIRECTION_VECTORS: number[][][] = new Array(3);
for (let i = 0; i < 3; ++i) {
    DIRECTION_VECTORS[i] = [
        [0, 0, 0],
        [0, 0, 0],
    ];
    DIRECTION_VECTORS[i][0][(i + 1) % 3] = 1;
    DIRECTION_VECTORS[i][1][(i + 2) % 3] = 1;
}

const AXIS = {
    X: 0,
    Y: 1,
    Z: 2,
};

const FACE = {
    NORTH: 0,
    EAST: 1,
    SOUTH: 2,
    WEST: 3,
    UP: 4,
    DOWN: 5,
};

const SIDE = {
    CURRENT: 0,
    NEXT: 1,
};

const FACES: { [axis: number]: { [side: number]: number } } = {
    [AXIS.X]: { [SIDE.CURRENT]: FACE.EAST, [SIDE.NEXT]: FACE.WEST },
    [AXIS.Y]: { [SIDE.CURRENT]: FACE.UP, [SIDE.NEXT]: FACE.DOWN },
    [AXIS.Z]: { [SIDE.CURRENT]: FACE.SOUTH, [SIDE.NEXT]: FACE.NORTH },
};

const FACE_NORMALS: { [face: number]: [number, number, number] } = {
    [FACE.NORTH]: [0, 0, -1],
    [FACE.SOUTH]: [0, 0, 1],
    [FACE.EAST]: [1, 0, 0],
    [FACE.WEST]: [-1, 0, 0],
    [FACE.UP]: [0, 1, 0],
    [FACE.DOWN]: [0, -1, 0],
};

function createCulledMesh(voxelShape: VoxelShape, chunk: VoxelShapeChunk): CulledMesherResult {
    const positions: number[] = [];
    const indices: number[] = [];
    const normals: number[] = [];

    // march over the chunk, comparing neighbouring blocks in px, py, pz directions
    for (let x = -1; x < CHUNK_SIZE; x++) {
        for (let z = -1; z < CHUNK_SIZE; z++) {
            for (let y = -1; y < CHUNK_SIZE; y++) {
                const marchBlockSolid = getVoxelRelative(voxelShape, chunk, x, y, z);

                for (let dir = 0; dir < 3; dir++) {
                    const marchNeighbourX = x + (dir === 0 ? 1 : 0);
                    const marchNeighbourY = y + (dir === 1 ? 1 : 0);
                    const marchNeighbourZ = z + (dir === 2 ? 1 : 0);

                    const marchNeighbourBlockSolid = getVoxelRelative(
                        voxelShape,
                        chunk,
                        marchNeighbourX,
                        marchNeighbourY,
                        marchNeighbourZ,
                    );

                    if (marchBlockSolid === marchNeighbourBlockSolid) continue;

                    const side = marchBlockSolid ? 0 : 1;

                    const blockPosX = x;
                    const blockPosY = y;
                    const blockPosZ = z;
                    const actualBlockX = blockPosX + (dir === 0 ? side : 0);
                    const actualBlockY = blockPosY + (dir === 1 ? side : 0);
                    const actualBlockZ = blockPosZ + (dir === 2 ? side : 0);

                    // don't create faces for blocks outside of the chunk
                    if (
                        actualBlockX < 0 ||
                        actualBlockX >= CHUNK_SIZE ||
                        actualBlockY < 0 ||
                        actualBlockY >= CHUNK_SIZE ||
                        actualBlockZ < 0 ||
                        actualBlockZ >= CHUNK_SIZE
                    ) {
                        continue;
                    }

                    const face = FACES[dir][side];
                    const [dx, dy, dz] = FACE_NORMALS[face];
                    const [ux, uy, uz] = DIRECTION_VECTORS[dir][side];
                    const [vx, vy, vz] = DIRECTION_VECTORS[dir][side ^ 1];

                    const nwx = chunk.cx * CHUNK_SIZE + marchNeighbourX;
                    const nwy = chunk.cy * CHUNK_SIZE + marchNeighbourY;
                    const nwz = chunk.cz * CHUNK_SIZE + marchNeighbourZ;

                    // positions - use world coordinates
                    positions.push(nwx, nwy, nwz);
                    positions.push(nwx + ux, nwy + uy, nwz + uz);
                    positions.push(nwx + ux + vx, nwy + uy + vy, nwz + uz + vz);
                    positions.push(nwx + vx, nwy + vy, nwz + vz);

                    // normals
                    normals.push(dx, dy, dz);
                    normals.push(dx, dy, dz);
                    normals.push(dx, dy, dz);
                    normals.push(dx, dy, dz);

                    // indices - two triangles for quad
                    const index = positions.length / 3 - 4;
                    const a = index;
                    const b = index + 1;
                    const c = index + 2;
                    const d = index + 3;

                    indices.push(a, b, d);
                    indices.push(b, c, d);
                }
            }
        }
    }

    return {
        positions: new Float32Array(positions),
        indices: new Uint32Array(indices),
        normals: new Float32Array(normals),
    };
}

// map of chunk index to THREE.Mesh
const chunkMeshes = new Map<number, THREE.Mesh>();

// shared material for all chunk meshes
const chunkMaterial = new THREE.MeshStandardMaterial({
    color: 0x999999,
    roughness: 0.8,
    metalness: 0.2,
});

// track dirty chunks that need rebuilding
const dirtyChunks = new Set<number>();

function markChunkDirty(chunkIndex: number): void {
    dirtyChunks.add(chunkIndex);
}

function updateChunkVisuals(voxelShape: VoxelShape, chunkIndex: number, scene: THREE.Scene): void {
    // remove existing mesh if it exists
    const existingMesh = chunkMeshes.get(chunkIndex);

    if (existingMesh) {
        scene.remove(existingMesh);
        existingMesh.geometry.dispose();
        chunkMeshes.delete(chunkIndex);
    }

    const chunk = voxelShape.chunks[chunkIndex];
    if (!chunk || chunk.sum === 0) return;

    const mesherResult = createCulledMesh(voxelShape, chunk);

    if (mesherResult.indices.length === 0) return;

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(mesherResult.positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(mesherResult.normals, 3));
    geometry.setIndex(new THREE.BufferAttribute(mesherResult.indices, 1));

    const mesh = new THREE.Mesh(geometry, chunkMaterial);
    scene.add(mesh);
    chunkMeshes.set(chunkIndex, mesh);
}

function processDirtyChunks(voxelShape: VoxelShape, scene: THREE.Scene, maxChunksPerFrame: number): void {
    let processed = 0;
    const iterator = dirtyChunks.values();

    while (processed < maxChunksPerFrame && dirtyChunks.size > 0) {
        const result = iterator.next();
        if (result.done) break;

        const chunkIndex = result.value;
        updateChunkVisuals(voxelShape, chunkIndex, scene);
        dirtyChunks.delete(chunkIndex);
        processed++;
    }
}

function meshAllChunks(voxelShape: VoxelShape, scene: THREE.Scene): void {
    // mesh all chunks
    for (let i = 0; i < voxelShape.chunks.length; i++) {
        updateChunkVisuals(voxelShape, i, scene);
    }
}

function markVoxelAndNeighborsDirty(voxelShape: VoxelShape, vx: number, vy: number, vz: number): void {
    // get chunk coordinates
    const cx = Math.floor(vx / CHUNK_SIZE);
    const cy = Math.floor(vy / CHUNK_SIZE);
    const cz = Math.floor(vz / CHUNK_SIZE);

    // get local voxel coordinates within chunk
    let lx = vx % CHUNK_SIZE;
    let ly = vy % CHUNK_SIZE;
    let lz = vz % CHUNK_SIZE;
    if (lx < 0) lx += CHUNK_SIZE;
    if (ly < 0) ly += CHUNK_SIZE;
    if (lz < 0) lz += CHUNK_SIZE;

    // mark current chunk dirty
    const chunkIndex =
        (cz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
        (cy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
        (cx - voxelShape.chunkBounds[0][0]);
    markChunkDirty(chunkIndex);

    // check if voxel is on chunk boundaries and mark neighboring chunks dirty
    // X boundary
    if (lx === 0) {
        const neighborCx = cx - 1;
        if (neighborCx >= voxelShape.chunkBounds[0][0] && neighborCx < voxelShape.chunkBounds[1][0]) {
            const neighborIndex =
                (cz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
                (cy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
                (neighborCx - voxelShape.chunkBounds[0][0]);
            markChunkDirty(neighborIndex);
        }
    } else if (lx === CHUNK_SIZE - 1) {
        const neighborCx = cx + 1;
        if (neighborCx >= voxelShape.chunkBounds[0][0] && neighborCx < voxelShape.chunkBounds[1][0]) {
            const neighborIndex =
                (cz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
                (cy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
                (neighborCx - voxelShape.chunkBounds[0][0]);
            markChunkDirty(neighborIndex);
        }
    }

    // Y boundary
    if (ly === 0) {
        const neighborCy = cy - 1;
        if (neighborCy >= voxelShape.chunkBounds[0][1] && neighborCy < voxelShape.chunkBounds[1][1]) {
            const neighborIndex =
                (cz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
                (neighborCy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
                (cx - voxelShape.chunkBounds[0][0]);
            markChunkDirty(neighborIndex);
        }
    } else if (ly === CHUNK_SIZE - 1) {
        const neighborCy = cy + 1;
        if (neighborCy >= voxelShape.chunkBounds[0][1] && neighborCy < voxelShape.chunkBounds[1][1]) {
            const neighborIndex =
                (cz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
                (neighborCy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
                (cx - voxelShape.chunkBounds[0][0]);
            markChunkDirty(neighborIndex);
        }
    }

    // Z boundary
    if (lz === 0) {
        const neighborCz = cz - 1;
        if (neighborCz >= voxelShape.chunkBounds[0][2] && neighborCz < voxelShape.chunkBounds[1][2]) {
            const neighborIndex =
                (neighborCz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
                (cy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
                (cx - voxelShape.chunkBounds[0][0]);
            markChunkDirty(neighborIndex);
        }
    } else if (lz === CHUNK_SIZE - 1) {
        const neighborCz = cz + 1;
        if (neighborCz >= voxelShape.chunkBounds[0][2] && neighborCz < voxelShape.chunkBounds[1][2]) {
            const neighborIndex =
                (neighborCz - voxelShape.chunkBounds[0][2]) * voxelShape.chunkCount[1] * voxelShape.chunkCount[0] +
                (cy - voxelShape.chunkBounds[0][1]) * voxelShape.chunkCount[0] +
                (cx - voxelShape.chunkBounds[0][0]);
            markChunkDirty(neighborIndex);
        }
    }
}

/* physics world */

declare module 'crashcat' {
    interface ShapeTypeRegistry {
        [ShapeType.USER_1]: VoxelShape;
    }
}

registerShapes([...ALL_SHAPE_DEFS, voxelShapeDef]);

const worldSettings = createWorldSettings();

const BROADPHASE_LAYER_MOVING = addBroadphaseLayer(worldSettings);
const BROADPHASE_LAYER_NOT_MOVING = addBroadphaseLayer(worldSettings);

const OBJECT_LAYER_MOVING = addObjectLayer(worldSettings, BROADPHASE_LAYER_MOVING);
const OBJECT_LAYER_VOXEL_TERRAIN = addObjectLayer(worldSettings, BROADPHASE_LAYER_NOT_MOVING);

enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_MOVING);
enableCollision(worldSettings, OBJECT_LAYER_MOVING, OBJECT_LAYER_VOXEL_TERRAIN);

const world = createWorld(worldSettings);

// Create filter for raycasting (only hit voxel terrain)
const voxelTerrainFilter = filter.create(worldSettings.layers);
filter.disableObjectLayer(voxelTerrainFilter, worldSettings.layers, OBJECT_LAYER_MOVING);

/* voxel terrain generation */

const TERRAIN_SIZE = 32;
const TERRAIN_HEIGHT = 16;
const BASE_HEIGHT = 4;

// Create voxel shape with chunk bounds to contain the terrain
// We need chunks from (-2, 0, -2) to (2, 2, 2) to cover -32..32 x 0..20 z -32..32
const CHUNKS_MIN_X = Math.floor(-TERRAIN_SIZE / CHUNK_SIZE);
const CHUNKS_MAX_X = Math.ceil(TERRAIN_SIZE / CHUNK_SIZE);
const CHUNKS_MIN_Y = 0;
const CHUNKS_MAX_Y = Math.ceil((BASE_HEIGHT + TERRAIN_HEIGHT) / CHUNK_SIZE);
const CHUNKS_MIN_Z = Math.floor(-TERRAIN_SIZE / CHUNK_SIZE);
const CHUNKS_MAX_Z = Math.ceil(TERRAIN_SIZE / CHUNK_SIZE);

const chunkBounds: Box3 = [
    [CHUNKS_MIN_X, CHUNKS_MIN_Y, CHUNKS_MIN_Z],
    [CHUNKS_MAX_X, CHUNKS_MAX_Y, CHUNKS_MAX_Z],
];

const voxelShape = createVoxelShape({ chunkBounds });

const simplexNoise = createSimplex2D(42);

for (let x = -TERRAIN_SIZE; x < TERRAIN_SIZE; x++) {
    for (let z = -TERRAIN_SIZE; z < TERRAIN_SIZE; z++) {
        // Layer 1: Large scale hills
        const noise1 = simplexNoise(x * 0.02, z * 0.02) * 0.5 + 0.5; // 0-1
        // Layer 2: Medium details
        const noise2 = simplexNoise(x * 0.08, z * 0.08) * 0.5 + 0.5; // 0-1
        // Layer 3: Small details
        const noise3 = simplexNoise(x * 0.2, z * 0.2) * 0.5 + 0.5; // 0-1

        // Combine layers with different weights
        const combinedNoise = noise1 * 0.6 + noise2 * 0.3 + noise3 * 0.1;
        const height = Math.floor(BASE_HEIGHT + combinedNoise * TERRAIN_HEIGHT);

        for (let y = 0; y < height; y++) {
            setVoxel(voxelShape, x, y, z, true);
        }
    }
}

// create static rigid body for the terrain
rigidBody.create(world, {
    shape: voxelShape,
    objectLayer: OBJECT_LAYER_VOXEL_TERRAIN,
    motionType: MotionType.STATIC,
    position: vec3.fromValues(0, 0, 0),
    quaternion: quat.create(),
});

/* rendering */

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a1a);

// mesh voxel terrain after creation
meshAllChunks(voxelShape, scene);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 10, 30);
camera.lookAt(0, 5, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const onResize = () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
};

window.addEventListener('resize', onResize);
onResize();

const orbitControls = new OrbitControls(camera, renderer.domElement);
orbitControls.enableDamping = true;
orbitControls.target.set(0, 5, 0);

const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
scene.add(ambientLight);

/* voxel editing with raycasting */

const rayCollector = createClosestCastRayCollector();
const raySettings = createDefaultCastRaySettings();

// Brush settings
let brushSize = 1;
const MIN_BRUSH_SIZE = 1;
const MAX_BRUSH_SIZE = 10;

// Brush box visualization (renders on top of everything)
const brushBoxGeometry = new THREE.BoxGeometry(1, 1, 1);
const brushBoxEdges = new THREE.EdgesGeometry(brushBoxGeometry);
const brushBoxMaterial = new THREE.LineBasicMaterial({
    color: 0xffffff,
    depthTest: false,
    transparent: true,
    opacity: 0.8,
});
const brushBox = new THREE.LineSegments(brushBoxEdges, brushBoxMaterial);
brushBox.visible = false;
scene.add(brushBox);

// Hit point visualization
const hitPointGeometry = new THREE.SphereGeometry(0.3, 16, 16);
const hitPointMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
const hitPoint = new THREE.Mesh(hitPointGeometry, hitPointMaterial);
hitPoint.visible = false;
scene.add(hitPoint);

// Hit normal visualization
const hitNormalArrow = new THREE.ArrowHelper(
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(0, 0, 0),
    2, // length
    0x00ff00, // green color
    0.4, // head length
    0.2, // head width
);
hitNormalArrow.visible = false;
scene.add(hitNormalArrow);

// Mouse tracking
const mouse = new THREE.Vector2();
const threeRaycaster = new THREE.Raycaster();

window.addEventListener('pointermove', (event) => {
    mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
    mouse.y = -(event.clientY / window.innerHeight) * 2 + 1;
});

// Keyboard controls for brush size
window.addEventListener('keydown', (event) => {
    if (event.key === '[') {
        brushSize = Math.max(MIN_BRUSH_SIZE, brushSize - 1);
        console.log('Brush size:', brushSize);
    } else if (event.key === ']') {
        brushSize = Math.min(MAX_BRUSH_SIZE, brushSize + 1);
        console.log('Brush size:', brushSize);
    }
});

// Click to break/build voxels
window.addEventListener('mousedown', (event) => {
    if (!rayCollector.hit || rayCollector.hit.status !== CastRayStatus.COLLIDING) {
        return;
    }

    // Recompute ray from camera (same as in animate loop)
    threeRaycaster.setFromCamera(mouse, camera);
    const rayOrigin = threeRaycaster.ray.origin;
    const rayDirection = threeRaycaster.ray.direction;

    // Compute hit position using same calculation as physics raycast
    const maxDistance = 100;
    const hitDistance = rayCollector.hit.fraction * maxDistance;
    const hitX = rayOrigin.x + rayDirection.x * hitDistance;
    const hitY = rayOrigin.y + rayDirection.y * hitDistance;
    const hitZ = rayOrigin.z + rayDirection.z * hitDistance;
    const hitPosition = vec3.fromValues(hitX, hitY, hitZ);

    // Get surface normal at hit point using the subShapeId
    const hitNormal = vec3.create();
    getShapeSurfaceNormal(hitNormal, voxelShape, hitPosition, rayCollector.hit.subShapeId);

    // Left click (button 0) = break, Right click (button 2) = build
    const isBreak = event.button === 0;

    if (isBreak) {
        // break: step slightly into surface (against normal) to find center voxel
        const epsilon = 0.01;
        const breakX = hitX - hitNormal[0] * epsilon;
        const breakY = hitY - hitNormal[1] * epsilon;
        const breakZ = hitZ - hitNormal[2] * epsilon;

        const centerVx = Math.floor(breakX);
        const centerVy = Math.floor(breakY);
        const centerVz = Math.floor(breakZ);

        // Apply brush size - remove voxels in a cube around center
        const halfSize = Math.floor(brushSize / 2);
        for (let dx = -halfSize; dx <= halfSize; dx++) {
            for (let dy = -halfSize; dy <= halfSize; dy++) {
                for (let dz = -halfSize; dz <= halfSize; dz++) {
                    const vx = centerVx + dx;
                    const vy = centerVy + dy;
                    const vz = centerVz + dz;

                    setVoxel(voxelShape, vx, vy, vz, false);
                    markVoxelAndNeighborsDirty(voxelShape, vx, vy, vz);
                }
            }
        }

        // wake bodies in area around the modified region
        const wakeAABB = box3.create();
        vec3.set(wakeAABB[0], centerVx - halfSize - 1, centerVy - halfSize - 1, centerVz - halfSize - 1);
        vec3.set(wakeAABB[1], centerVx + halfSize + 1, centerVy + halfSize + 1, centerVz + halfSize + 1);
        rigidBody.wakeInAABB(world, wakeAABB);
    } else {
        // build: step along normal (away from surface) to place center voxel
        const placeX = hitX + hitNormal[0] * 0.5;
        const placeY = hitY + hitNormal[1] * 0.5;
        const placeZ = hitZ + hitNormal[2] * 0.5;

        const centerVx = Math.floor(placeX);
        const centerVy = Math.floor(placeY);
        const centerVz = Math.floor(placeZ);

        // Apply brush size - place voxels in a cube around center
        const halfSize = Math.floor(brushSize / 2);
        for (let dx = -halfSize; dx <= halfSize; dx++) {
            for (let dy = -halfSize; dy <= halfSize; dy++) {
                for (let dz = -halfSize; dz <= halfSize; dz++) {
                    const vx = centerVx + dx;
                    const vy = centerVy + dy;
                    const vz = centerVz + dz;

                    setVoxel(voxelShape, vx, vy, vz, true);
                    markVoxelAndNeighborsDirty(voxelShape, vx, vy, vz);
                }
            }
        }

        // wake bodies in area around the modified region
        const wakeAABB = box3.create();
        vec3.set(wakeAABB[0], centerVx - halfSize - 1, centerVy - halfSize - 1, centerVz - halfSize - 1);
        vec3.set(wakeAABB[1], centerVx + halfSize + 1, centerVy + halfSize + 1, centerVz + halfSize + 1);
        rigidBody.wakeInAABB(world, wakeAABB);
    }
});

// Right click context menu disable
window.addEventListener('contextmenu', (event) => {
    event.preventDefault();
});

const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
directionalLight.position.set(5, 10, 7);
scene.add(directionalLight);

/* shape spawning system */

type ShapeConfig = {
    name: string;
    createShape: () => Shape;
    color: number;
};

const shapeConfigs: ShapeConfig[] = [
    {
        name: 'sphere',
        createShape: () => sphere.create({ radius: 0.5 }),
        color: 0xff6b6b,
    },
    {
        name: 'box',
        createShape: () => box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0.05 }),
        color: 0x4ecdc4,
    },
    {
        name: 'capsule',
        createShape: () => capsule.create({ halfHeightOfCylinder: 1.0, radius: 1 }),
        color: 0xdfe6e9,
    },
];

type DynamicShape = {
    body: RigidBody;
    config: ShapeConfig;
};

const dynamicShapes: DynamicShape[] = [];

const settings = {
    numberOfBodies: 50,
    respawnIntervalMs: 500,
};

const SPAWN_HEIGHT = 25;
const SPAWN_AREA = 10; // +/- units on x and z

function spawnShape(config: ShapeConfig): void {
    const shape = config.createShape();

    // Random position above the ground
    const x = randomFloat(-SPAWN_AREA, SPAWN_AREA, Math.random());
    const z = randomFloat(-SPAWN_AREA, SPAWN_AREA, Math.random());
    const position = vec3.fromValues(x, SPAWN_HEIGHT, z);

    // Random rotation
    const axis = vec3.fromValues(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
    vec3.normalize(axis, axis);
    const angle = Math.random() * Math.PI * 2;
    const rotation = quat.create();
    quat.setAxisAngle(rotation, axis, angle);

    const body = rigidBody.create(world, {
        shape,
        objectLayer: OBJECT_LAYER_MOVING,
        motionType: MotionType.DYNAMIC,
        position,
        quaternion: rotation,
        restitution: 0,
        friction: 0.5,
    });

    dynamicShapes.push({
        body,
        config,
    });
}

function updateBodyCount() {
    const targetCount = settings.numberOfBodies;
    const currentCount = dynamicShapes.length;

    if (currentCount < targetCount) {
        // Spawn more bodies
        const toSpawn = targetCount - currentCount;
        for (let i = 0; i < toSpawn; i++) {
            const config = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
            spawnShape(config);
        }
    } else if (currentCount > targetCount) {
        // Remove excess bodies
        const toRemove = currentCount - targetCount;
        for (let i = 0; i < toRemove; i++) {
            const shape = dynamicShapes.pop();
            if (shape) {
                rigidBody.remove(world, shape.body);
            }
        }
    }
}

// Initial spawn
updateBodyCount();

/* simulation loop */

const options = debugRenderer.createDefaultOptions();
const debugRendererState = debugRenderer.init(options);
const ui = debugUI.init(options);
scene.add(debugRendererState.object3d);

ui.gui
    .add(settings, 'numberOfBodies', 0, 1000, 1)
    .name('Number of Bodies')
    .onChange(() => {
        updateBodyCount();
    });
ui.gui.add(settings, 'respawnIntervalMs', 100, 5000, 100).name('Respawn Interval (ms)');

const maxDelta = 1 / 30;
let lastTime = performance.now();
let lastRespawnTime = performance.now();
let respawnIndex = 0;

function animate() {
    requestAnimationFrame(animate);

    const currentTime = performance.now();
    const delta = Math.min((currentTime - lastTime) / 1000, maxDelta);
    lastTime = currentTime;

    // Update physics with delta time
    debugUI.beginPerf(ui);
    updateWorld(world, undefined, delta);
    debugUI.endPerf(ui);
    debugUI.updateStats(ui, world);

    // Perform raycast for voxel editing
    threeRaycaster.setFromCamera(mouse, camera);
    const rayOrigin = threeRaycaster.ray.origin;
    const rayDirection = threeRaycaster.ray.direction;

    const origin = vec3.fromValues(rayOrigin.x, rayOrigin.y, rayOrigin.z);
    const direction = vec3.fromValues(rayDirection.x, rayDirection.y, rayDirection.z);
    const maxDistance = 100;

    rayCollector.reset();
    castRay(world, rayCollector, raySettings, origin, direction, maxDistance, voxelTerrainFilter);

    // Update hit point visualization and brush box
    if (rayCollector.hit.status === CastRayStatus.COLLIDING) {
        const hitDistance = rayCollector.hit.fraction * maxDistance;
        const hitX = rayOrigin.x + rayDirection.x * hitDistance;
        const hitY = rayOrigin.y + rayDirection.y * hitDistance;
        const hitZ = rayOrigin.z + rayDirection.z * hitDistance;
        hitPoint.position.set(hitX, hitY, hitZ);
        hitPoint.visible = true;

        // Update hit normal arrow
        const hitPosition = vec3.fromValues(hitX, hitY, hitZ);
        const hitNormal = vec3.create();
        getShapeSurfaceNormal(hitNormal, voxelShape, hitPosition, rayCollector.hit.subShapeId);
        hitNormalArrow.position.set(hitX, hitY, hitZ);
        hitNormalArrow.setDirection(new THREE.Vector3(hitNormal[0], hitNormal[1], hitNormal[2]));
        hitNormalArrow.visible = true;

        // Update brush box visualization
        // Position it at the voxel that would be affected
        const epsilon = 0.01;
        const breakX = hitX - hitNormal[0] * epsilon;
        const breakY = hitY - hitNormal[1] * epsilon;
        const breakZ = hitZ - hitNormal[2] * epsilon;

        const centerVx = Math.floor(breakX);
        const centerVy = Math.floor(breakY);
        const centerVz = Math.floor(breakZ);

        // Center the brush box on the affected voxels
        brushBox.position.set(centerVx + 0.5, centerVy + 0.5, centerVz + 0.5);
        brushBox.scale.set(brushSize, brushSize, brushSize);
        brushBox.visible = true;
    } else {
        hitPoint.visible = false;
        hitNormalArrow.visible = false;
        brushBox.visible = false;
    }

    // Process dirty chunks (rebuild 3 per frame)
    processDirtyChunks(voxelShape, scene, 3);

    // Round-robin respawning based on interval
    if (dynamicShapes.length > 0 && currentTime - lastRespawnTime >= settings.respawnIntervalMs) {
        lastRespawnTime = currentTime;

        // Get the next shape to respawn in round-robin fashion
        const shapeToRespawn = dynamicShapes[respawnIndex % dynamicShapes.length];

        // Remove old body properly
        rigidBody.remove(world, shapeToRespawn.body);

        // Create new body with random shape type
        const newConfig = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
        const newShape = newConfig.createShape();

        const x = randomFloat(-SPAWN_AREA, SPAWN_AREA);
        const z = randomFloat(-SPAWN_AREA, SPAWN_AREA);
        const position = vec3.fromValues(x, SPAWN_HEIGHT, z);

        const axis = vec3.fromValues(Math.random() - 0.5, Math.random() - 0.5, Math.random() - 0.5);
        vec3.normalize(axis, axis);
        const angle = Math.random() * Math.PI * 2;
        const rotation = quat.create();
        quat.setAxisAngle(rotation, axis, angle);

        const newBody = rigidBody.create(world, {
            shape: newShape,
            objectLayer: OBJECT_LAYER_MOVING,
            motionType: MotionType.DYNAMIC,
            position,
            quaternion: rotation,
            restitution: 0.3,
            friction: 0.5,
        });

        // Replace the body in the array
        shapeToRespawn.body = newBody;
        shapeToRespawn.config = newConfig;

        respawnIndex++;
    }

    // Maintain target body count
    while (dynamicShapes.length < settings.numberOfBodies) {
        const config = shapeConfigs[Math.floor(Math.random() * shapeConfigs.length)];
        spawnShape(config);
    }

    // Update debug renderer
    debugRenderer.update(debugRendererState, world);

    // Update orbit controls
    orbitControls.update();

    // Render
    renderer.render(scene, camera);
}

animate();
