import type { Box3, Vec3 } from 'mathcat';

// triangle buffer stride
export const TRIANGLE_STRIDE = 8;

// offsets within each triangle's stride
export const OFFSET_INDEX_A = 0;
export const OFFSET_INDEX_B = 1;
export const OFFSET_INDEX_C = 2;
export const OFFSET_NORMAL_X = 3;
export const OFFSET_NORMAL_Y = 4;
export const OFFSET_NORMAL_Z = 5;
export const OFFSET_ACTIVE_EDGES = 6;
export const OFFSET_MATERIAL_ID = 7;

/** triangle mesh data in interleaved format */
export type TriangleMeshData = {
	/** deduplicated vertex positions [x0, y0, z0, x1, y1, z1, ...] */
	positions: number[];

	/** interleaved triangle data (stride = TRIANGLE_STRIDE) */
	triangleBuffer: number[];

	/** total number of triangles */
	triangleCount: number;
};

/** get triangle vertex indices */
export function getTriangleIndices(out: [number, number, number], data: TriangleMeshData, triIdx: number): void {
	const offset = triIdx * TRIANGLE_STRIDE;
	out[0] = data.triangleBuffer[offset + OFFSET_INDEX_A];
	out[1] = data.triangleBuffer[offset + OFFSET_INDEX_B];
	out[2] = data.triangleBuffer[offset + OFFSET_INDEX_C];
}

/** get triangle vertex positions */
export function getTriangleVertices(outA: Vec3, outB: Vec3, outC: Vec3, data: TriangleMeshData, triIdx: number): void {
	const buffer = data.triangleBuffer;
	const positions = data.positions;
	const offset = triIdx * TRIANGLE_STRIDE;

	const ia = buffer[offset + OFFSET_INDEX_A];
	const ib = buffer[offset + OFFSET_INDEX_B];
	const ic = buffer[offset + OFFSET_INDEX_C];

	outA[0] = positions[ia * 3 + 0];
	outA[1] = positions[ia * 3 + 1];
	outA[2] = positions[ia * 3 + 2];

	outB[0] = positions[ib * 3 + 0];
	outB[1] = positions[ib * 3 + 1];
	outB[2] = positions[ib * 3 + 2];

	outC[0] = positions[ic * 3 + 0];
	outC[1] = positions[ic * 3 + 1];
	outC[2] = positions[ic * 3 + 2];
}

/** get triangle normal */
export function getTriangleNormal(out: Vec3, data: TriangleMeshData, triIdx: number): void {
	const buffer = data.triangleBuffer;
	const offset = triIdx * TRIANGLE_STRIDE;
	out[0] = buffer[offset + OFFSET_NORMAL_X];
	out[1] = buffer[offset + OFFSET_NORMAL_Y];
	out[2] = buffer[offset + OFFSET_NORMAL_Z];
}

/** set triangle normal */
export function setTriangleNormal(data: TriangleMeshData, triIdx: number, normal: Vec3): void {
	const offset = triIdx * TRIANGLE_STRIDE;
	data.triangleBuffer[offset + OFFSET_NORMAL_X] = normal[0];
	data.triangleBuffer[offset + OFFSET_NORMAL_Y] = normal[1];
	data.triangleBuffer[offset + OFFSET_NORMAL_Z] = normal[2];
}

/** get active edges bitmask (bit 0 = edge AB, bit 1 = edge BC, bit 2 = edge CA) */
export function getActiveEdges(data: TriangleMeshData, triIdx: number): number {
	return data.triangleBuffer[triIdx * TRIANGLE_STRIDE + OFFSET_ACTIVE_EDGES];
}

/** set active edges */
export function setActiveEdges(data: TriangleMeshData, triIdx: number, activeEdges: number): void {
	const offset = triIdx * TRIANGLE_STRIDE;
	data.triangleBuffer[offset + OFFSET_ACTIVE_EDGES] = activeEdges;
}

/** get material id */
export function getMaterialId(data: TriangleMeshData, triIdx: number): number {
	return data.triangleBuffer[triIdx * TRIANGLE_STRIDE + OFFSET_MATERIAL_ID];
}

/** calculate triangle aabb from vertices */
export function calculateTriangleAABB(out: Box3, data: TriangleMeshData, triIdx: number): void {
	const buffer = data.triangleBuffer;
	const positions = data.positions;
	const offset = triIdx * TRIANGLE_STRIDE;

	// get vertices
	const ia = buffer[offset + OFFSET_INDEX_A];
	const ib = buffer[offset + OFFSET_INDEX_B];
	const ic = buffer[offset + OFFSET_INDEX_C];
	const ax = positions[ia * 3 + 0];
	const ay = positions[ia * 3 + 1];
	const az = positions[ia * 3 + 2];
	const bx = positions[ib * 3 + 0];
	const by = positions[ib * 3 + 1];
	const bz = positions[ib * 3 + 2];
	const cx = positions[ic * 3 + 0];
	const cy = positions[ic * 3 + 1];
	const cz = positions[ic * 3 + 2];

	// compute aabb
	let minX = ax;
	let minY = ay;
	let minZ = az;
	let maxX = ax;
	let maxY = ay;
	let maxZ = az;

	if (bx < minX) minX = bx;
	if (by < minY) minY = by;
	if (bz < minZ) minZ = bz;
	if (bx > maxX) maxX = bx;
	if (by > maxY) maxY = by;
	if (bz > maxZ) maxZ = bz;

	if (cx < minX) minX = cx;
	if (cy < minY) minY = cy;
	if (cz < minZ) minZ = cz;
	if (cx > maxX) maxX = cx;
	if (cy > maxY) maxY = cy;
	if (cz > maxZ) maxZ = cz;

	out[0][0] = minX;
	out[0][1] = minY;
	out[0][2] = minZ;
	out[1][0] = maxX;
	out[1][1] = maxY;
	out[1][2] = maxZ;
}

/** calculate triangle centroid from vertices */
export function calculateTriangleCentroid(out: Vec3, data: TriangleMeshData, triIdx: number): void {
	const buffer = data.triangleBuffer;
	const positions = data.positions;
	const offset = triIdx * TRIANGLE_STRIDE;

	// get vertices
	const ia = buffer[offset + OFFSET_INDEX_A];
	const ib = buffer[offset + OFFSET_INDEX_B];
	const ic = buffer[offset + OFFSET_INDEX_C];

	const ax = positions[ia * 3 + 0];
	const ay = positions[ia * 3 + 1];
	const az = positions[ia * 3 + 2];

	const bx = positions[ib * 3 + 0];
	const by = positions[ib * 3 + 1];
	const bz = positions[ib * 3 + 2];

	const cx = positions[ic * 3 + 0];
	const cy = positions[ic * 3 + 1];
	const cz = positions[ic * 3 + 2];

	// compute centroid
	out[0] = (ax + bx + cx) / 3;
	out[1] = (ay + by + cy) / 3;
	out[2] = (az + bz + cz) / 3;
}

/** swap two triangles */
export function swapTriangles(data: TriangleMeshData, indexA: number, indexB: number): void {
	if (indexA === indexB) return;

	const buffer = data.triangleBuffer;
	const offsetA = indexA * TRIANGLE_STRIDE;
	const offsetB = indexB * TRIANGLE_STRIDE;

	// swap stride
	for (let i = 0; i < TRIANGLE_STRIDE; i++) {
		const tmp = buffer[offsetA + i];
		buffer[offsetA + i] = buffer[offsetB + i];
		buffer[offsetB + i] = tmp;
	}
}
