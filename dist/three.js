import { ConstraintType, MotionType, ShapeType, bvh, rigidBody } from "crashcat";
import * as THREE from "three";
//#region \0rolldown/runtime.js
var __defProp = Object.defineProperty;
var __exportAll = (all, no_symbols) => {
	let target = {};
	for (var name in all) __defProp(target, name, {
		get: all[name],
		enumerable: true
	});
	if (!no_symbols) __defProp(target, Symbol.toStringTag, { value: "Module" });
	return target;
};
//#endregion
//#region node_modules/.pnpm/mathcat@0.0.13/node_modules/mathcat/dist/vec3.js
/**
* Creates a new, empty vec3
*
* @returns a new 3D vector
*/
function create$1() {
	return [
		0,
		0,
		0
	];
}
/**
* Copy the values from one vec3 to another
*
* @param out the receiving vector
* @param a the source vector
* @returns out
*/
function copy(out, a) {
	out[0] = a[0];
	out[1] = a[1];
	out[2] = a[2];
	return out;
}
/**
* Set the components of a vec3 to the given values
*
* @param out the receiving vector
* @param x X component
* @param y Y component
* @param z Z component
* @returns out
*/
function set(out, x, y, z) {
	out[0] = x;
	out[1] = y;
	out[2] = z;
	return out;
}
/**
* Adds two vec3's
*
* @param out the receiving vector
* @param a the first operand
* @param b the second operand
* @returns out
*/
function add(out, a, b) {
	out[0] = a[0] + b[0];
	out[1] = a[1] + b[1];
	out[2] = a[2] + b[2];
	return out;
}
/**
* Transforms the vec3 with a quat
* Can also be used for dual quaternions. (Multiply it with the real part)
*
* @param out the receiving vector
* @param a the vector to transform
* @param q quaternion to transform with
* @returns out
*/
function transformQuat(out, a, q) {
	const qx = q[0];
	const qy = q[1];
	const qz = q[2];
	const qw = q[3];
	const x = a[0];
	const y = a[1];
	const z = a[2];
	let uvx = qy * z - qz * y;
	let uvy = qz * x - qx * z;
	let uvz = qx * y - qy * x;
	let uuvx = qy * uvz - qz * uvy;
	let uuvy = qz * uvx - qx * uvz;
	let uuvz = qx * uvy - qy * uvx;
	const w2 = qw * 2;
	uvx *= w2;
	uvy *= w2;
	uvz *= w2;
	uuvx *= 2;
	uuvy *= 2;
	uuvz *= 2;
	out[0] = x + uvx + uuvx;
	out[1] = y + uvy + uuvy;
	out[2] = z + uvz + uuvz;
	return out;
}
//#endregion
//#region node_modules/.pnpm/mathcat@0.0.13/node_modules/mathcat/dist/quat.js
/**
* Creates a new identity quat
*
* @returns a new quaternion
*/
function create() {
	return [
		0,
		0,
		0,
		1
	];
}
/**
* Sets a quat from the given angle and rotation axis,
* then returns it.
*
* @param out the receiving quaternion
* @param axis the axis around which to rotate
* @param rad the angle in radians
* @returns out
**/
function setAxisAngle(out, axis, rad) {
	rad *= .5;
	const s = Math.sin(rad);
	out[0] = s * axis[0];
	out[1] = s * axis[1];
	out[2] = s * axis[2];
	out[3] = Math.cos(rad);
	return out;
}
/**
* Multiplies two quat's
*
* @param out the receiving quaternion
* @param a the first operand
* @param b the second operand
* @returns out
*/
function multiply(out, a, b) {
	const ax = a[0];
	const ay = a[1];
	const az = a[2];
	const aw = a[3];
	const bx = b[0];
	const by = b[1];
	const bz = b[2];
	const bw = b[3];
	out[0] = ax * bw + aw * bx + ay * bz - az * by;
	out[1] = ay * bw + aw * by + az * bx - ax * bz;
	out[2] = az * bw + aw * bz + ax * by - ay * bx;
	out[3] = aw * bw - ax * bx - ay * by - az * bz;
	return out;
}
//#endregion
//#region three/debug-renderer.ts
var debug_renderer_exports = /* @__PURE__ */ __exportAll({
	BodyColorMode: () => BodyColorMode,
	clear: () => clear,
	createDefaultOptions: () => createDefaultOptions,
	dispose: () => dispose,
	init: () => init,
	invalidateBodyShape: () => invalidateBodyShape,
	update: () => update
});
let BodyColorMode = /* @__PURE__ */ function(BodyColorMode) {
	BodyColorMode[BodyColorMode["INSTANCE"] = 0] = "INSTANCE";
	BodyColorMode[BodyColorMode["MOTION_TYPE"] = 1] = "MOTION_TYPE";
	BodyColorMode[BodyColorMode["SLEEPING"] = 2] = "SLEEPING";
	BodyColorMode[BodyColorMode["ISLAND"] = 3] = "ISLAND";
	return BodyColorMode;
}({});
function createUnitSphereGeometry() {
	return new THREE.SphereGeometry(1, 32, 32);
}
function createUnitBoxGeometry() {
	return new THREE.BoxGeometry(1, 1, 1);
}
function createUnitPlaneGeometry() {
	return new THREE.PlaneGeometry(1, 1, 1, 1);
}
function createUnitCapsuleGeometry() {
	return new THREE.CapsuleGeometry(1, 2, 16, 32);
}
function createUnitConeGeometry() {
	return new THREE.ConeGeometry(.5, 1, 8);
}
function createUnitCylinderGeometry() {
	return new THREE.CylinderGeometry(.5, .5, 1, 16);
}
function createTriangleMeshGeometry(shape) {
	const geometry = new THREE.BufferGeometry();
	const positions = new Float32Array(shape.data.positions);
	geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
	const indexCount = shape.data.triangleCount * 3;
	const indices = new Uint32Array(indexCount);
	for (let i = 0; i < shape.data.triangleCount; i++) {
		const offset = i * 8;
		indices[i * 3 + 0] = shape.data.triangleBuffer[offset + 0];
		indices[i * 3 + 1] = shape.data.triangleBuffer[offset + 1];
		indices[i * 3 + 2] = shape.data.triangleBuffer[offset + 2];
	}
	geometry.setIndex(new THREE.BufferAttribute(indices, 1));
	const vertexCount = positions.length / 3;
	const uvs = new Float32Array(vertexCount * 2);
	for (let i = 0; i < vertexCount; i++) {
		uvs[i * 2 + 0] = 0;
		uvs[i * 2 + 1] = 0;
	}
	geometry.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));
	geometry.computeVertexNormals();
	return geometry;
}
function createConvexHullGeometry(shape) {
	const geometry = new THREE.BufferGeometry();
	const vertices = [];
	const indices = [];
	const uvs = [];
	for (const face of shape.faces) {
		const faceVertices = [];
		for (let i = 0; i < face.numVertices; i++) {
			const pb = shape.vertexIndices[face.firstVertex + i] * 3;
			faceVertices.push([
				shape.pointPositions[pb],
				shape.pointPositions[pb + 1],
				shape.pointPositions[pb + 2]
			]);
		}
		if (faceVertices.length >= 3) {
			const baseIdx = vertices.length / 3;
			for (const v of faceVertices) {
				vertices.push(v[0], v[1], v[2]);
				uvs.push(0, 0);
			}
			for (let i = 1; i < faceVertices.length - 1; i++) indices.push(baseIdx, baseIdx + i, baseIdx + i + 1);
		}
	}
	geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(vertices), 3));
	geometry.setAttribute("uv", new THREE.BufferAttribute(new Float32Array(uvs), 2));
	if (indices.length > 0) {
		geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(indices), 1));
		geometry.computeVertexNormals();
	}
	return geometry;
}
/** create default debug renderer options */
function createDefaultOptions() {
	return {
		bodies: {
			enabled: true,
			color: 0,
			wireframe: false,
			showLinearVelocity: false,
			showAngularVelocity: false
		},
		contacts: { enabled: false },
		contactConstraints: { enabled: false },
		constraints: {
			enabled: false,
			drawLimits: false,
			size: .5
		},
		broadphaseDbvt: {
			enabled: false,
			showLeafNodes: true,
			showNonLeafNodes: true
		},
		triangleMeshBvh: {
			enabled: false,
			showLeafNodes: true,
			showNonLeafNodes: true
		},
		staticCompoundBvh: {
			enabled: false,
			showLeafNodes: true,
			showNonLeafNodes: true
		}
	};
}
/**
* initialize the debug renderer
* @param options - optional debug renderer options. if not provided, default options will be used.
*/
function init(options) {
	const maxInstances = 1e4;
	const maxVertexCount = 1e5;
	const maxIndexCount = 1e5;
	const bodiesMaterial = new THREE.MeshPhongMaterial();
	const bodiesBatchedMesh = new THREE.BatchedMesh(maxInstances, maxVertexCount, maxIndexCount, bodiesMaterial);
	const velocitiesMaterial = new THREE.MeshBasicMaterial();
	velocitiesMaterial.depthTest = false;
	const velocitiesBatchedMesh = new THREE.BatchedMesh(maxInstances, maxVertexCount, maxIndexCount, velocitiesMaterial);
	velocitiesBatchedMesh.frustumCulled = false;
	velocitiesBatchedMesh.renderOrder = 1e3;
	const sphereGeometry = createUnitSphereGeometry();
	const cylinderGeometry = createUnitCylinderGeometry();
	const coneGeometry = createUnitConeGeometry();
	const contactsSpheresMaxCount = 500;
	const contactsCylindersMaxCount = 500;
	const contactsConesMaxCount = 500;
	const contactsMaterial = new THREE.MeshPhongMaterial();
	contactsMaterial.depthTest = false;
	const contactsSpheresMesh = new THREE.InstancedMesh(sphereGeometry, contactsMaterial, contactsSpheresMaxCount);
	contactsSpheresMesh.frustumCulled = false;
	contactsSpheresMesh.renderOrder = 1;
	contactsSpheresMesh.count = 0;
	const contactsCylindersMesh = new THREE.InstancedMesh(cylinderGeometry, contactsMaterial, contactsCylindersMaxCount);
	contactsCylindersMesh.frustumCulled = false;
	contactsCylindersMesh.renderOrder = 1;
	contactsCylindersMesh.count = 0;
	const contactsConesMesh = new THREE.InstancedMesh(coneGeometry, contactsMaterial, contactsConesMaxCount);
	contactsConesMesh.frustumCulled = false;
	contactsConesMesh.renderOrder = 1;
	contactsConesMesh.count = 0;
	const contactsContainer = new THREE.Object3D();
	contactsContainer.add(contactsSpheresMesh);
	contactsContainer.add(contactsCylindersMesh);
	contactsContainer.add(contactsConesMesh);
	const constraintContactsSpheresMaxCount = 1e3;
	const constraintContactsCylindersMaxCount = 1e3;
	const constraintContactsConesMaxCount = 500;
	const constraintContactsMaterial = new THREE.MeshPhongMaterial();
	constraintContactsMaterial.depthTest = false;
	const constraintContactsSpheresMesh = new THREE.InstancedMesh(sphereGeometry, constraintContactsMaterial, constraintContactsSpheresMaxCount);
	constraintContactsSpheresMesh.frustumCulled = false;
	constraintContactsSpheresMesh.renderOrder = 1;
	constraintContactsSpheresMesh.count = 0;
	const constraintContactsCylindersMesh = new THREE.InstancedMesh(cylinderGeometry, constraintContactsMaterial, constraintContactsCylindersMaxCount);
	constraintContactsCylindersMesh.frustumCulled = false;
	constraintContactsCylindersMesh.renderOrder = 1;
	constraintContactsCylindersMesh.count = 0;
	const constraintContactsConesMesh = new THREE.InstancedMesh(coneGeometry, constraintContactsMaterial, constraintContactsConesMaxCount);
	constraintContactsConesMesh.frustumCulled = false;
	constraintContactsConesMesh.renderOrder = 1;
	constraintContactsConesMesh.count = 0;
	const constraintContactsContainer = new THREE.Object3D();
	constraintContactsContainer.add(constraintContactsSpheresMesh);
	constraintContactsContainer.add(constraintContactsCylindersMesh);
	constraintContactsContainer.add(constraintContactsConesMesh);
	const broadphaseDbvtGeometry = new THREE.BufferGeometry();
	const broadphaseDbvtMaterial = new THREE.LineBasicMaterial({ vertexColors: true });
	const broadphaseDbvtLines = new THREE.LineSegments(broadphaseDbvtGeometry, broadphaseDbvtMaterial);
	broadphaseDbvtLines.frustumCulled = false;
	const triangleMeshBvhContainer = new THREE.Object3D();
	const triangleMeshBvhLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ vertexColors: true }));
	triangleMeshBvhLines.frustumCulled = false;
	triangleMeshBvhContainer.add(triangleMeshBvhLines);
	const staticCompoundBvhContainer = new THREE.Object3D();
	const staticCompoundBvhLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ vertexColors: true }));
	staticCompoundBvhLines.frustumCulled = false;
	staticCompoundBvhContainer.add(staticCompoundBvhLines);
	const constraintsGeometry = new THREE.BufferGeometry();
	const constraintsMaterial = new THREE.LineBasicMaterial({
		vertexColors: true,
		depthTest: false,
		depthWrite: false
	});
	const constraintsLineSegments = new THREE.LineSegments(constraintsGeometry, constraintsMaterial);
	constraintsLineSegments.frustumCulled = false;
	constraintsLineSegments.renderOrder = 999;
	const edgeWireframeContainer = new THREE.Object3D();
	const object3d = new THREE.Object3D();
	object3d.add(bodiesBatchedMesh);
	object3d.add(contactsContainer);
	object3d.add(constraintContactsContainer);
	object3d.add(velocitiesBatchedMesh);
	object3d.add(broadphaseDbvtLines);
	object3d.add(constraintsLineSegments);
	object3d.add(edgeWireframeContainer);
	object3d.add(triangleMeshBvhContainer);
	object3d.add(staticCompoundBvhContainer);
	return {
		object3d,
		options: options ?? createDefaultOptions(),
		nextTriangleMeshId: 0,
		bodies: {
			batchedMesh: bodiesBatchedMesh,
			velocitiesBatchedMesh,
			material: bodiesMaterial,
			velocitiesMaterial,
			maxVertexCount,
			maxIndexCount,
			currentVertexCount: 0,
			currentIndexCount: 0,
			geometryCache: /* @__PURE__ */ new Map(),
			velocityCylinderGeometryId: null,
			velocityConeGeometryId: null,
			instances: /* @__PURE__ */ new Map(),
			instanceColors: /* @__PURE__ */ new Map(),
			edgeWireframes: /* @__PURE__ */ new Map(),
			edgeWireframeContainer,
			previousWireframe: false,
			linearVelocityArrows: /* @__PURE__ */ new Map(),
			angularVelocityArrows: /* @__PURE__ */ new Map()
		},
		contacts: {
			container: contactsContainer,
			spheresMesh: contactsSpheresMesh,
			cylindersMesh: contactsCylindersMesh,
			conesMesh: contactsConesMesh,
			spheresMaxCount: contactsSpheresMaxCount,
			cylindersMaxCount: contactsCylindersMaxCount,
			conesMaxCount: contactsConesMaxCount
		},
		contactConstraints: {
			container: constraintContactsContainer,
			spheresMesh: constraintContactsSpheresMesh,
			cylindersMesh: constraintContactsCylindersMesh,
			conesMesh: constraintContactsConesMesh,
			spheresMaxCount: constraintContactsSpheresMaxCount,
			cylindersMaxCount: constraintContactsCylindersMaxCount,
			conesMaxCount: constraintContactsConesMaxCount
		},
		constraints: { lineSegments: constraintsLineSegments },
		broadphase: { dbvtLines: broadphaseDbvtLines },
		triangleMeshBvh: {
			lines: triangleMeshBvhLines,
			cache: /* @__PURE__ */ new Map(),
			container: triangleMeshBvhContainer,
			previousShowLeafNodes: true,
			previousShowNonLeafNodes: true
		},
		staticCompoundBvh: {
			lines: staticCompoundBvhLines,
			cache: /* @__PURE__ */ new Map(),
			container: staticCompoundBvhContainer,
			previousShowLeafNodes: true,
			previousShowNonLeafNodes: true
		}
	};
}
function ensureBatchedMeshCapacity(mesh, requiredInstances) {
	const currentMax = mesh.maxInstanceCount;
	if (mesh.instanceCount + requiredInstances <= currentMax) return;
	mesh.optimize();
	if (mesh.instanceCount + requiredInstances <= currentMax) return;
	const newMaxInstances = Math.max(currentMax * 2, mesh.instanceCount + requiredInstances);
	mesh.setInstanceCount(newMaxInstances);
}
function ensureBatchedMeshGeometryCapacity(mesh, state, geometry) {
	const positionAttr = geometry.attributes.position;
	const indexAttr = geometry.index;
	const requiredVertices = positionAttr ? positionAttr.count : 0;
	const requiredIndices = indexAttr ? indexAttr.count : 0;
	if (state.bodies.currentVertexCount + requiredVertices > state.bodies.maxVertexCount || state.bodies.currentIndexCount + requiredIndices > state.bodies.maxIndexCount) {
		const newMaxVertexCount = Math.max(state.bodies.maxVertexCount, state.bodies.currentVertexCount + requiredVertices + 5e3);
		const newMaxIndexCount = Math.max(state.bodies.maxIndexCount, state.bodies.currentIndexCount + requiredIndices + 5e3);
		state.bodies.maxVertexCount = newMaxVertexCount;
		state.bodies.maxIndexCount = newMaxIndexCount;
		mesh.setGeometrySize(newMaxVertexCount, newMaxIndexCount);
	}
	state.bodies.currentVertexCount += requiredVertices;
	state.bodies.currentIndexCount += requiredIndices;
}
function addInstanceToBatchedMesh(mesh, geometryId, matrix, color) {
	ensureBatchedMeshCapacity(mesh, 1);
	const instanceId = mesh.addInstance(geometryId);
	mesh.setMatrixAt(instanceId, matrix);
	mesh.setColorAt(instanceId, color);
	return instanceId;
}
function getOrCreateGeometry(mesh, cache, key, state) {
	const cached = cache.get(key);
	if (cached) {
		cached.refCount++;
		return cached.geometryId;
	}
	let geometry;
	if (key === "unit-sphere") geometry = createUnitSphereGeometry();
	else if (key === "unit-box") geometry = createUnitBoxGeometry();
	else if (key === "unit-capsule") geometry = createUnitCapsuleGeometry();
	else if (key === "unit-cone") geometry = createUnitConeGeometry();
	else if (key === "unit-cylinder") geometry = createUnitCylinderGeometry();
	else if (key === "unit-plane") geometry = createUnitPlaneGeometry();
	else throw new Error(`Unexpected shape key: ${key}`);
	ensureBatchedMeshGeometryCapacity(mesh, state, geometry);
	const geometryId = mesh.addGeometry(geometry);
	cache.set(key, {
		geometry,
		geometryId,
		refCount: 1
	});
	return geometryId;
}
const _matrix = new THREE.Matrix4();
const _position = new THREE.Vector3();
const _quaternion = new THREE.Quaternion();
const _scale = new THREE.Vector3();
const _shapeLocalMatrix = new THREE.Matrix4();
const _childLocalMatrix = new THREE.Matrix4();
const _combinedLocalMatrix = new THREE.Matrix4();
const _worldShapeMatrix = new THREE.Matrix4();
const _instanceColor = new THREE.Color();
const _planeNormalDefault = new THREE.Vector3();
const _planeNormalTarget = new THREE.Vector3();
const _identityMatrix = new THREE.Matrix4();
const _up = new THREE.Vector3();
const _shaftMidpoint = new THREE.Vector3();
const _tipPosition = new THREE.Vector3();
const _edgeDir = new THREE.Vector3();
const _edgeMidpoint = new THREE.Vector3();
const _currentPointA = new THREE.Vector3();
const _prevPointA = new THREE.Vector3();
const _firstPoint = new THREE.Vector3();
const _normalStart = new THREE.Vector3();
const _normalVec = new THREE.Vector3();
const _normalShaftMidpoint = new THREE.Vector3();
const _normalTipPosition = new THREE.Vector3();
const _tangent1Vec = new THREE.Vector3();
const _tangent1Midpoint = new THREE.Vector3();
const _tangent2Vec = new THREE.Vector3();
const _tangent2Midpoint = new THREE.Vector3();
function addShapeInstances(state, shape, bodyMatrix, localMatrix, color, instanceIds) {
	switch (shape.type) {
		case ShapeType.SPHERE: {
			const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, "unit-sphere", state);
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(shape.radius, shape.radius, shape.radius);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.BOX: {
			const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, "unit-box", state);
			const [hx, hy, hz] = shape.halfExtents;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(hx * 2, hy * 2, hz * 2);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.CAPSULE: {
			const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, "unit-capsule", state);
			const r = shape.radius;
			const h = shape.halfHeightOfCylinder;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(r, h, r);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.CYLINDER: {
			const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, "unit-cylinder", state);
			const r = shape.radius;
			const h = shape.halfHeight;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(r * 2, h * 2, r * 2);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.PLANE: {
			const geometryId = getOrCreateGeometry(state.bodies.batchedMesh, state.bodies.geometryCache, "unit-plane", state);
			const [nx, ny, nz] = shape.plane.normal;
			const distance = -shape.plane.constant;
			const size = shape.halfExtent * 2;
			_planeNormalDefault.set(0, 0, 1);
			_planeNormalTarget.set(nx, ny, nz);
			_quaternion.setFromUnitVectors(_planeNormalDefault, _planeNormalTarget);
			_position.set(nx * distance, ny * distance, nz * distance);
			_scale.set(size, size, 1);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.TRIANGLE_MESH: {
			const key = `triangle-mesh:${state.nextTriangleMeshId++}`;
			const geometry = createTriangleMeshGeometry(shape);
			ensureBatchedMeshGeometryCapacity(state.bodies.batchedMesh, state, geometry);
			const geometryId = state.bodies.batchedMesh.addGeometry(geometry);
			state.bodies.geometryCache.set(key, {
				geometry,
				geometryId,
				refCount: 1
			});
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.CONVEX_HULL: {
			const key = `convex-hull:${state.nextTriangleMeshId++}`;
			const geometry = createConvexHullGeometry(shape);
			ensureBatchedMeshGeometryCapacity(state.bodies.batchedMesh, state, geometry);
			const geometryId = state.bodies.batchedMesh.addGeometry(geometry);
			state.bodies.geometryCache.set(key, {
				geometry,
				geometryId,
				refCount: 1
			});
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
			const instanceId = addInstanceToBatchedMesh(state.bodies.batchedMesh, geometryId, _worldShapeMatrix, color);
			instanceIds.push(instanceId);
			break;
		}
		case ShapeType.COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
				addShapeInstances(state, child.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
			}
			break;
		case ShapeType.STATIC_COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
				addShapeInstances(state, child.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
			}
			break;
		case ShapeType.TRANSFORMED: {
			const [tx, ty, tz] = shape.position;
			const [qx, qy, qz, qw] = shape.quaternion;
			_position.set(tx, ty, tz);
			_quaternion.set(qx, qy, qz, qw);
			_scale.set(1, 1, 1);
			_matrix.compose(_position, _quaternion, _scale);
			_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
			addShapeInstances(state, shape.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
			break;
		}
		case ShapeType.SCALED: {
			const [sx, sy, sz] = shape.scale;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(sx, sy, sz);
			_matrix.compose(_position, _quaternion, _scale);
			_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
			addShapeInstances(state, shape.shape, bodyMatrix, _childLocalMatrix, color, instanceIds);
			break;
		}
		case ShapeType.OFFSET_CENTER_OF_MASS:
			addShapeInstances(state, shape.shape, bodyMatrix, localMatrix, color, instanceIds);
			break;
	}
}
function updateShapeInstances(mesh, shape, bodyMatrix, localMatrix, instanceIds, index, color) {
	switch (shape.type) {
		case ShapeType.SPHERE:
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(shape.radius, shape.radius, shape.radius);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		case ShapeType.BOX: {
			const [hx, hy, hz] = shape.halfExtents;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(hx * 2, hy * 2, hz * 2);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		}
		case ShapeType.CAPSULE: {
			const r = shape.radius;
			const h = shape.halfHeightOfCylinder;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(r, h, r);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		}
		case ShapeType.CYLINDER: {
			const r = shape.radius;
			const h = shape.halfHeight;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(r * 2, h * 2, r * 2);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		}
		case ShapeType.PLANE: {
			const [nx, ny, nz] = shape.plane.normal;
			const distance = -shape.plane.constant;
			const size = shape.halfExtent * 2;
			_planeNormalDefault.set(0, 0, 1);
			_planeNormalTarget.set(nx, ny, nz);
			_quaternion.setFromUnitVectors(_planeNormalDefault, _planeNormalTarget);
			_position.set(nx * distance, ny * distance, nz * distance);
			_scale.set(size, size, 1);
			_shapeLocalMatrix.compose(_position, _quaternion, _scale);
			_combinedLocalMatrix.multiplyMatrices(localMatrix, _shapeLocalMatrix);
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, _combinedLocalMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		}
		case ShapeType.TRIANGLE_MESH:
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		case ShapeType.CONVEX_HULL:
			_worldShapeMatrix.multiplyMatrices(bodyMatrix, localMatrix);
			mesh.setMatrixAt(instanceIds[index.value], _worldShapeMatrix);
			mesh.setColorAt(instanceIds[index.value], color);
			index.value++;
			break;
		case ShapeType.COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
				updateShapeInstances(mesh, child.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
			}
			break;
		case ShapeType.STATIC_COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
				updateShapeInstances(mesh, child.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
			}
			break;
		case ShapeType.TRANSFORMED: {
			const [tx, ty, tz] = shape.position;
			const [qx, qy, qz, qw] = shape.quaternion;
			_position.set(tx, ty, tz);
			_quaternion.set(qx, qy, qz, qw);
			_scale.set(1, 1, 1);
			_matrix.compose(_position, _quaternion, _scale);
			_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
			updateShapeInstances(mesh, shape.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
			break;
		}
		case ShapeType.SCALED: {
			const [sx, sy, sz] = shape.scale;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(sx, sy, sz);
			_matrix.compose(_position, _quaternion, _scale);
			_childLocalMatrix.multiplyMatrices(localMatrix, _matrix);
			updateShapeInstances(mesh, shape.shape, bodyMatrix, _childLocalMatrix, instanceIds, index, color);
			break;
		}
		case ShapeType.OFFSET_CENTER_OF_MASS:
			updateShapeInstances(mesh, shape.shape, bodyMatrix, localMatrix, instanceIds, index, color);
			break;
	}
}
const _colorWhite = new THREE.Color(16777215);
const _colorGreen = new THREE.Color(65280);
const _colorYellow = new THREE.Color(16776960);
const _colorBlue = new THREE.Color(255);
const _colorRed = new THREE.Color(16711680);
function getBodyColor(state, body) {
	switch (state.options.bodies.color) {
		case 0: {
			let color = state.bodies.instanceColors.get(body.id);
			if (!color) {
				color = new THREE.Color();
				const hash = body.id * 137.5;
				if (body.motionType === MotionType.STATIC) {
					const lightness = .22 + hash % 100 / 100 * .2;
					color.setHSL(0, 0, lightness);
				} else {
					const hue = hash % 360 / 360;
					color.setHSL(hue, .7, .6);
				}
				state.bodies.instanceColors.set(body.id, color);
			}
			return color;
		}
		case 1: switch (body.motionType) {
			case MotionType.DYNAMIC: return _colorGreen;
			case MotionType.KINEMATIC: return _colorYellow;
			case MotionType.STATIC: return _colorBlue;
			default: return _colorWhite;
		}
		case 2: return body.sleeping ? _colorBlue : _colorRed;
		case 3: {
			const islandIndex = body.islandIndex;
			if (islandIndex === -1) return _colorWhite;
			_instanceColor.setHSL(islandIndex * 137.5 % 360 / 360, .8, .6);
			return _instanceColor;
		}
		default: return _colorWhite;
	}
}
/**
* Invalidate a body's shape to force recreation of its visual instances.
* Call this when you mutate a body's shape properties in-place.
* @param state Debug renderer state
* @param bodyId The ID of the body whose shape was mutated
*/
function invalidateBodyShape(state, bodyId) {
	const instance = state.bodies.instances.get(bodyId);
	if (instance) {
		for (const instanceId of instance.instanceIds) state.bodies.batchedMesh.deleteInstance(instanceId);
		state.bodies.instances.delete(bodyId);
	}
	const wireframe = state.bodies.edgeWireframes.get(bodyId);
	if (wireframe) {
		state.bodies.edgeWireframeContainer.remove(wireframe.lineSegments);
		wireframe.lineSegments.geometry.dispose();
		for (const geometry of wireframe.geometries) geometry.dispose();
		state.bodies.edgeWireframes.delete(bodyId);
	}
}
const _updateMatrix = new THREE.Matrix4();
const _updatePosition = new THREE.Vector3();
const _updateQuaternion = new THREE.Quaternion();
const _updateScale = new THREE.Vector3();
const _contactPointColor = new THREE.Color(16711680);
const _contactNormalColor = new THREE.Color(65280);
const CONTACT_POINT_RADIUS = .05;
const CONTACT_NORMAL_LENGTH = .5;
const ARROW_SHAFT_RADIUS = .01;
const ARROW_TIP_HEIGHT = .1;
const linearVelocityColor = new THREE.Color(65280);
const angularVelocityColor = new THREE.Color(16711680);
const constraintManifoldPointColorA = new THREE.Color(65535);
const constraintManifoldPointColorB = new THREE.Color(16711935);
const constraintManifoldPointColorAActive = new THREE.Color(65535);
const constraintManifoldPointColorBActive = new THREE.Color(16711935);
const constraintManifoldEdgeColor = new THREE.Color(65535);
const constraintNormalColor = new THREE.Color(16711680);
const constraintTangent1Color = new THREE.Color(65280);
const constraintTangent2Color = new THREE.Color(255);
const CONSTRAINT_POINT_RADIUS_INACTIVE = .1;
const CONSTRAINT_POINT_RADIUS_ACTIVE = .2;
const CONSTRAINT_EDGE_RADIUS = .01;
const CONSTRAINT_TANGENT_LENGTH = .5;
const _edgeWireframeMaterial = new THREE.LineBasicMaterial({ vertexColors: true });
function clearBatchedMeshBodies(state) {
	for (const instance of state.bodies.instances.values()) for (const instanceId of instance.instanceIds) state.bodies.batchedMesh.deleteInstance(instanceId);
	state.bodies.instances.clear();
}
function clearEdgeWireframes(state) {
	for (const instance of state.bodies.edgeWireframes.values()) {
		state.bodies.edgeWireframeContainer.remove(instance.lineSegments);
		instance.lineSegments.geometry.dispose();
		for (const geometry of instance.geometries) geometry.dispose();
	}
	state.bodies.edgeWireframes.clear();
}
function createEdgeWireframeForShape(state, shape, parentMatrix, linePositions, lineColors, geometries, localTransforms, color) {
	switch (shape.type) {
		case ShapeType.SPHERE: {
			const geometry = new THREE.SphereGeometry(shape.radius, 16, 12);
			const edgesGeometry = new THREE.EdgesGeometry(geometry, 15);
			geometry.dispose();
			const positions = edgesGeometry.attributes.position.array;
			for (let i = 0; i < positions.length; i += 3) {
				_position.set(positions[i], positions[i + 1], positions[i + 2]);
				_position.applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			geometries.push(edgesGeometry);
			localTransforms.push(parentMatrix.clone());
			break;
		}
		case ShapeType.BOX: {
			const [hx, hy, hz] = shape.halfExtents;
			const geometry = new THREE.BoxGeometry(hx * 2, hy * 2, hz * 2);
			const edgesGeometry = new THREE.EdgesGeometry(geometry);
			geometry.dispose();
			const positions = edgesGeometry.attributes.position.array;
			for (let i = 0; i < positions.length; i += 3) {
				_position.set(positions[i], positions[i + 1], positions[i + 2]);
				_position.applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			geometries.push(edgesGeometry);
			localTransforms.push(parentMatrix.clone());
			break;
		}
		case ShapeType.CAPSULE: {
			const r = shape.radius;
			const h = shape.halfHeightOfCylinder;
			const geometry = new THREE.CapsuleGeometry(r, h * 2, 8, 16);
			const edgesGeometry = new THREE.EdgesGeometry(geometry, 15);
			geometry.dispose();
			const positions = edgesGeometry.attributes.position.array;
			for (let i = 0; i < positions.length; i += 3) {
				_position.set(positions[i], positions[i + 1], positions[i + 2]);
				_position.applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			geometries.push(edgesGeometry);
			localTransforms.push(parentMatrix.clone());
			break;
		}
		case ShapeType.CYLINDER: {
			const r = shape.radius;
			const h = shape.halfHeight;
			const geometry = new THREE.CylinderGeometry(r, r, h * 2, 32);
			const edgesGeometry = new THREE.EdgesGeometry(geometry);
			geometry.dispose();
			const positions = edgesGeometry.attributes.position.array;
			for (let i = 0; i < positions.length; i += 3) {
				_position.set(positions[i], positions[i + 1], positions[i + 2]);
				_position.applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			geometries.push(edgesGeometry);
			localTransforms.push(parentMatrix.clone());
			break;
		}
		case ShapeType.PLANE: {
			const [nx, ny, nz] = shape.plane.normal;
			const distance = -shape.plane.constant;
			const size = shape.halfExtent * 2;
			const geometry = new THREE.PlaneGeometry(size, size, 1, 1);
			const edgesGeometry = new THREE.EdgesGeometry(geometry);
			geometry.dispose();
			const defaultNormal = new THREE.Vector3(0, 0, 1);
			const planeNormal = new THREE.Vector3(nx, ny, nz);
			_quaternion.setFromUnitVectors(defaultNormal, planeNormal);
			_position.set(nx * distance, ny * distance, nz * distance);
			_scale.set(1, 1, 1);
			_matrix.compose(_position, _quaternion, _scale);
			const matrix = parentMatrix.clone().multiply(_matrix);
			const positions = edgesGeometry.attributes.position.array;
			for (let i = 0; i < positions.length; i += 3) {
				_position.set(positions[i], positions[i + 1], positions[i + 2]);
				_position.applyMatrix4(matrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			geometries.push(edgesGeometry);
			localTransforms.push(matrix.clone());
			break;
		}
		case ShapeType.TRIANGLE_MESH: {
			const positions = shape.data.positions;
			const triangleBuffer = shape.data.triangleBuffer;
			const triangleCount = shape.data.triangleCount;
			for (let i = 0; i < triangleCount; i++) {
				const offset = i * 8;
				const idxA = triangleBuffer[offset + 0];
				const idxB = triangleBuffer[offset + 1];
				const idxC = triangleBuffer[offset + 2];
				const ax = positions[idxA * 3 + 0];
				const ay = positions[idxA * 3 + 1];
				const az = positions[idxA * 3 + 2];
				const bx = positions[idxB * 3 + 0];
				const by = positions[idxB * 3 + 1];
				const bz = positions[idxB * 3 + 2];
				const cx = positions[idxC * 3 + 0];
				const cy = positions[idxC * 3 + 1];
				const cz = positions[idxC * 3 + 2];
				_position.set(ax, ay, az).applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
				_position.set(bx, by, bz).applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
				_position.set(bx, by, bz).applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
				_position.set(cx, cy, cz).applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
				_position.set(cx, cy, cz).applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
				_position.set(ax, ay, az).applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			localTransforms.push(parentMatrix.clone());
			break;
		}
		case ShapeType.CONVEX_HULL: {
			const geometry = createConvexHullGeometry(shape);
			const edgesGeometry = new THREE.EdgesGeometry(geometry, 30);
			geometry.dispose();
			const positions = edgesGeometry.attributes.position.array;
			for (let i = 0; i < positions.length; i += 3) {
				_position.set(positions[i], positions[i + 1], positions[i + 2]);
				_position.applyMatrix4(parentMatrix);
				linePositions.push(_position.x, _position.y, _position.z);
				lineColors.push(color.r, color.g, color.b);
			}
			geometries.push(edgesGeometry);
			localTransforms.push(parentMatrix.clone());
			break;
		}
		case ShapeType.COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				const childMatrix = parentMatrix.clone().multiply(_matrix);
				createEdgeWireframeForShape(state, child.shape, childMatrix, linePositions, lineColors, geometries, localTransforms, color);
			}
			break;
		case ShapeType.STATIC_COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				const childMatrix = parentMatrix.clone().multiply(_matrix);
				createEdgeWireframeForShape(state, child.shape, childMatrix, linePositions, lineColors, geometries, localTransforms, color);
			}
			break;
		case ShapeType.TRANSFORMED: {
			const [tx, ty, tz] = shape.position;
			const [qx, qy, qz, qw] = shape.quaternion;
			_position.set(tx, ty, tz);
			_quaternion.set(qx, qy, qz, qw);
			_scale.set(1, 1, 1);
			_matrix.compose(_position, _quaternion, _scale);
			const transformedMatrix = parentMatrix.clone().multiply(_matrix);
			createEdgeWireframeForShape(state, shape.shape, transformedMatrix, linePositions, lineColors, geometries, localTransforms, color);
			break;
		}
		case ShapeType.SCALED: {
			const [sx, sy, sz] = shape.scale;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(sx, sy, sz);
			_matrix.compose(_position, _quaternion, _scale);
			const scaledMatrix = parentMatrix.clone().multiply(_matrix);
			createEdgeWireframeForShape(state, shape.shape, scaledMatrix, linePositions, lineColors, geometries, localTransforms, color);
			break;
		}
		case ShapeType.OFFSET_CENTER_OF_MASS:
			createEdgeWireframeForShape(state, shape.shape, parentMatrix, linePositions, lineColors, geometries, localTransforms, color);
			break;
	}
}
function updateBodiesBatchedMesh(state, world) {
	state.bodies.batchedMesh.visible = true;
	state.bodies.edgeWireframeContainer.visible = false;
	const activeBodyIds = /* @__PURE__ */ new Set();
	for (const body of rigidBody.iterate(world)) activeBodyIds.add(body.id);
	for (const [bodyId, instance] of state.bodies.instances) if (!activeBodyIds.has(bodyId)) {
		for (const instanceId of instance.instanceIds) state.bodies.batchedMesh.deleteInstance(instanceId);
		state.bodies.instances.delete(bodyId);
	}
	for (const body of rigidBody.iterate(world)) {
		let instance = state.bodies.instances.get(body.id);
		if (instance && instance.shape !== body.shape) {
			for (const instanceId of instance.instanceIds) state.bodies.batchedMesh.deleteInstance(instanceId);
			instance = void 0;
		}
		_updatePosition.set(body.position[0], body.position[1], body.position[2]);
		_updateQuaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
		_updateScale.set(1, 1, 1);
		_updateMatrix.compose(_updatePosition, _updateQuaternion, _updateScale);
		const color = getBodyColor(state, body);
		if (!instance) {
			const instanceIds = [];
			_identityMatrix.identity();
			addShapeInstances(state, body.shape, _updateMatrix, _identityMatrix, color, instanceIds);
			const newInstance = {
				bodyId: body.id,
				shape: body.shape,
				instanceIds
			};
			state.bodies.instances.set(body.id, newInstance);
			continue;
		}
		_identityMatrix.identity();
		updateShapeInstances(state.bodies.batchedMesh, body.shape, _updateMatrix, _identityMatrix, instance.instanceIds, { value: 0 }, color);
	}
}
function updateBodiesEdgeWireframe(state, world) {
	state.bodies.batchedMesh.visible = false;
	state.bodies.edgeWireframeContainer.visible = true;
	const activeBodyIds = /* @__PURE__ */ new Set();
	for (const body of rigidBody.iterate(world)) activeBodyIds.add(body.id);
	for (const [bodyId, instance] of state.bodies.edgeWireframes) if (!activeBodyIds.has(bodyId)) {
		state.bodies.edgeWireframeContainer.remove(instance.lineSegments);
		instance.lineSegments.geometry.dispose();
		for (const geometry of instance.geometries) geometry.dispose();
		state.bodies.edgeWireframes.delete(bodyId);
	}
	for (const body of rigidBody.iterate(world)) {
		let instance = state.bodies.edgeWireframes.get(body.id);
		if (instance && instance.shape !== body.shape) {
			state.bodies.edgeWireframeContainer.remove(instance.lineSegments);
			instance.lineSegments.geometry.dispose();
			for (const geometry of instance.geometries) geometry.dispose();
			state.bodies.edgeWireframes.delete(body.id);
			instance = void 0;
		}
		if (!instance) {
			const linePositions = [];
			const lineColors = [];
			const geometries = [];
			const localTransforms = [];
			const color = getBodyColor(state, body);
			const identityMatrix = new THREE.Matrix4();
			createEdgeWireframeForShape(state, body.shape, identityMatrix, linePositions, lineColors, geometries, localTransforms, color);
			const geometry = new THREE.BufferGeometry();
			geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(linePositions), 3));
			geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(lineColors), 3));
			const lineSegments = new THREE.LineSegments(geometry, _edgeWireframeMaterial);
			lineSegments.frustumCulled = false;
			state.bodies.edgeWireframeContainer.add(lineSegments);
			const newInstance = {
				bodyId: body.id,
				shape: body.shape,
				lineSegments,
				localTransforms,
				geometries
			};
			state.bodies.edgeWireframes.set(body.id, newInstance);
			_updatePosition.set(body.position[0], body.position[1], body.position[2]);
			_updateQuaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
			_updateScale.set(1, 1, 1);
			_updateMatrix.compose(_updatePosition, _updateQuaternion, _updateScale);
			newInstance.lineSegments.position.set(_updatePosition.x, _updatePosition.y, _updatePosition.z);
			newInstance.lineSegments.quaternion.set(_updateQuaternion.x, _updateQuaternion.y, _updateQuaternion.z, _updateQuaternion.w);
			continue;
		}
		_updatePosition.set(body.position[0], body.position[1], body.position[2]);
		_updateQuaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
		_updateScale.set(1, 1, 1);
		_updateMatrix.compose(_updatePosition, _updateQuaternion, _updateScale);
		instance.lineSegments.position.set(_updatePosition.x, _updatePosition.y, _updatePosition.z);
		instance.lineSegments.quaternion.set(_updateQuaternion.x, _updateQuaternion.y, _updateQuaternion.z, _updateQuaternion.w);
	}
}
function updateBodies(state, world) {
	const wireframeChanged = state.bodies.previousWireframe !== state.options.bodies.wireframe;
	if (!state.options.bodies.enabled) {
		if (state.bodies.instances.size > 0) clearBatchedMeshBodies(state);
		if (state.bodies.edgeWireframes.size > 0) clearEdgeWireframes(state);
		state.bodies.batchedMesh.visible = false;
		state.bodies.edgeWireframeContainer.visible = false;
		state.bodies.previousWireframe = state.options.bodies.wireframe;
		return;
	}
	if (wireframeChanged) if (state.bodies.previousWireframe) clearEdgeWireframes(state);
	else clearBatchedMeshBodies(state);
	if (state.options.bodies.wireframe) updateBodiesEdgeWireframe(state, world);
	else updateBodiesBatchedMesh(state, world);
	state.bodies.previousWireframe = state.options.bodies.wireframe;
}
function ensureContactsSpheresCapacity(state, requiredCount) {
	if (requiredCount <= state.contacts.spheresMaxCount) return;
	const newMaxCount = Math.max(state.contacts.spheresMaxCount * 2, requiredCount);
	const oldMesh = state.contacts.spheresMesh;
	const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
	newMesh.frustumCulled = false;
	newMesh.renderOrder = oldMesh.renderOrder;
	newMesh.count = 0;
	state.contacts.container.remove(oldMesh);
	state.contacts.container.add(newMesh);
	state.contacts.spheresMesh = newMesh;
	state.contacts.spheresMaxCount = newMaxCount;
}
function ensureContactsCylindersCapacity(state, requiredCount) {
	if (requiredCount <= state.contacts.cylindersMaxCount) return;
	const newMaxCount = Math.max(state.contacts.cylindersMaxCount * 2, requiredCount);
	const oldMesh = state.contacts.cylindersMesh;
	const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
	newMesh.frustumCulled = false;
	newMesh.renderOrder = oldMesh.renderOrder;
	newMesh.count = 0;
	state.contacts.container.remove(oldMesh);
	state.contacts.container.add(newMesh);
	state.contacts.cylindersMesh = newMesh;
	state.contacts.cylindersMaxCount = newMaxCount;
}
function ensureContactsConesCapacity(state, requiredCount) {
	if (requiredCount <= state.contacts.conesMaxCount) return;
	const newMaxCount = Math.max(state.contacts.conesMaxCount * 2, requiredCount);
	const oldMesh = state.contacts.conesMesh;
	const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
	newMesh.frustumCulled = false;
	newMesh.renderOrder = oldMesh.renderOrder;
	newMesh.count = 0;
	state.contacts.container.remove(oldMesh);
	state.contacts.container.add(newMesh);
	state.contacts.conesMesh = newMesh;
	state.contacts.conesMaxCount = newMaxCount;
}
function ensureConstraintContactsSpheresCapacity(state, requiredCount) {
	if (requiredCount <= state.contactConstraints.spheresMaxCount) return;
	const newMaxCount = Math.max(state.contactConstraints.spheresMaxCount * 2, requiredCount);
	const oldMesh = state.contactConstraints.spheresMesh;
	const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
	newMesh.frustumCulled = false;
	newMesh.renderOrder = oldMesh.renderOrder;
	newMesh.count = 0;
	state.contactConstraints.container.remove(oldMesh);
	state.contactConstraints.container.add(newMesh);
	state.contactConstraints.spheresMesh = newMesh;
	state.contactConstraints.spheresMaxCount = newMaxCount;
}
function ensureConstraintContactsCylindersCapacity(state, requiredCount) {
	if (requiredCount <= state.contactConstraints.cylindersMaxCount) return;
	const newMaxCount = Math.max(state.contactConstraints.cylindersMaxCount * 2, requiredCount);
	const oldMesh = state.contactConstraints.cylindersMesh;
	const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
	newMesh.frustumCulled = false;
	newMesh.renderOrder = oldMesh.renderOrder;
	newMesh.count = 0;
	state.contactConstraints.container.remove(oldMesh);
	state.contactConstraints.container.add(newMesh);
	state.contactConstraints.cylindersMesh = newMesh;
	state.contactConstraints.cylindersMaxCount = newMaxCount;
}
function ensureConstraintContactsConesCapacity(state, requiredCount) {
	if (requiredCount <= state.contactConstraints.conesMaxCount) return;
	const newMaxCount = Math.max(state.contactConstraints.conesMaxCount * 2, requiredCount);
	const oldMesh = state.contactConstraints.conesMesh;
	const newMesh = new THREE.InstancedMesh(oldMesh.geometry, oldMesh.material, newMaxCount);
	newMesh.frustumCulled = false;
	newMesh.renderOrder = oldMesh.renderOrder;
	newMesh.count = 0;
	state.contactConstraints.container.remove(oldMesh);
	state.contactConstraints.container.add(newMesh);
	state.contactConstraints.conesMesh = newMesh;
	state.contactConstraints.conesMaxCount = newMaxCount;
}
function updateContacts(state, world) {
	if (!state.options.contacts.enabled) {
		state.contacts.spheresMesh.count = 0;
		state.contacts.cylindersMesh.count = 0;
		state.contacts.conesMesh.count = 0;
		return;
	}
	let totalContacts = 0;
	for (let i = 0; i < world.contactConstraints.count; i++) {
		const constraint = world.contactConstraints.pool[i];
		totalContacts += constraint.numContactPoints;
	}
	const sphereCount = totalContacts;
	const cylinderCount = totalContacts;
	const coneCount = totalContacts;
	ensureContactsSpheresCapacity(state, sphereCount);
	ensureContactsCylindersCapacity(state, cylinderCount);
	ensureContactsConesCapacity(state, coneCount);
	state.contacts.spheresMesh.count = sphereCount;
	state.contacts.cylindersMesh.count = cylinderCount;
	state.contacts.conesMesh.count = coneCount;
	let sphereIdx = 0;
	let cylinderIdx = 0;
	let coneIdx = 0;
	for (let constraintIdx = 0; constraintIdx < world.contactConstraints.count; constraintIdx++) {
		const constraint = world.contactConstraints.pool[constraintIdx];
		for (let i = 0; i < constraint.numContactPoints; i++) {
			const contactPoint = constraint.contactPoints[i];
			const normal = constraint.normal;
			_position.set(contactPoint.positionA[0], contactPoint.positionA[1], contactPoint.positionA[2]);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(CONTACT_POINT_RADIUS, CONTACT_POINT_RADIUS, CONTACT_POINT_RADIUS);
			_matrix.compose(_position, _quaternion, _scale);
			state.contacts.spheresMesh.setMatrixAt(sphereIdx, _matrix);
			state.contacts.spheresMesh.setColorAt(sphereIdx, _contactPointColor);
			sphereIdx++;
			const shaftLength = CONTACT_NORMAL_LENGTH - ARROW_TIP_HEIGHT;
			_up.set(0, 1, 0);
			_normalVec.set(normal[0], normal[1], normal[2]);
			_quaternion.setFromUnitVectors(_up, _normalVec);
			_shaftMidpoint.copy(_position).addScaledVector(_normalVec, shaftLength / 2);
			_scale.set(ARROW_SHAFT_RADIUS, shaftLength, ARROW_SHAFT_RADIUS);
			_matrix.compose(_shaftMidpoint, _quaternion, _scale);
			state.contacts.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
			state.contacts.cylindersMesh.setColorAt(cylinderIdx, _contactNormalColor);
			cylinderIdx++;
			_tipPosition.copy(_position).addScaledVector(_normalVec, .45);
			_scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);
			_matrix.compose(_tipPosition, _quaternion, _scale);
			state.contacts.conesMesh.setMatrixAt(coneIdx, _matrix);
			state.contacts.conesMesh.setColorAt(coneIdx, _contactNormalColor);
			coneIdx++;
		}
	}
	state.contacts.spheresMesh.instanceMatrix.needsUpdate = true;
	state.contacts.cylindersMesh.instanceMatrix.needsUpdate = true;
	state.contacts.conesMesh.instanceMatrix.needsUpdate = true;
	if (state.contacts.spheresMesh.instanceColor) state.contacts.spheresMesh.instanceColor.needsUpdate = true;
	if (state.contacts.cylindersMesh.instanceColor) state.contacts.cylindersMesh.instanceColor.needsUpdate = true;
	if (state.contacts.conesMesh.instanceColor) state.contacts.conesMesh.instanceColor.needsUpdate = true;
}
function updateContactConstraints(state, world) {
	if (!state.options.contactConstraints.enabled) {
		state.contactConstraints.spheresMesh.count = 0;
		state.contactConstraints.cylindersMesh.count = 0;
		state.contactConstraints.conesMesh.count = 0;
		return;
	}
	let sphereCount = 0;
	let cylinderCount = 0;
	let coneCount = 0;
	for (let i = 0; i < world.contactConstraints.count; i++) {
		const constraint = world.contactConstraints.pool[i];
		if (constraint.numContactPoints === 0) continue;
		sphereCount += constraint.numContactPoints * 2;
		const edgeCount = constraint.numContactPoints > 2 ? constraint.numContactPoints : constraint.numContactPoints - 1;
		cylinderCount += Math.max(0, edgeCount);
		cylinderCount += 3;
		coneCount += 1;
	}
	ensureConstraintContactsSpheresCapacity(state, sphereCount);
	ensureConstraintContactsCylindersCapacity(state, cylinderCount);
	ensureConstraintContactsConesCapacity(state, coneCount);
	state.contactConstraints.spheresMesh.count = sphereCount;
	state.contactConstraints.cylindersMesh.count = cylinderCount;
	state.contactConstraints.conesMesh.count = coneCount;
	let sphereIdx = 0;
	let cylinderIdx = 0;
	let coneIdx = 0;
	for (let constraintIdx = 0; constraintIdx < world.contactConstraints.count; constraintIdx++) {
		const constraint = world.contactConstraints.pool[constraintIdx];
		if (constraint.numContactPoints === 0) continue;
		for (let i = 0; i < constraint.numContactPoints; i++) {
			const contactPoint = constraint.contactPoints[i];
			const hasImpulse = contactPoint.normalConstraint.totalLambda !== 0 || constraint.frictionConstraint1.totalLambda !== 0 || constraint.frictionConstraint2.totalLambda !== 0 || constraint.angularFrictionConstraint.totalLambda !== 0;
			const radius = hasImpulse ? CONSTRAINT_POINT_RADIUS_ACTIVE : CONSTRAINT_POINT_RADIUS_INACTIVE;
			const colorA = hasImpulse ? constraintManifoldPointColorAActive : constraintManifoldPointColorA;
			const colorB = hasImpulse ? constraintManifoldPointColorBActive : constraintManifoldPointColorB;
			_position.set(contactPoint.positionA[0], contactPoint.positionA[1], contactPoint.positionA[2]);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(radius, radius, radius);
			_matrix.compose(_position, _quaternion, _scale);
			state.contactConstraints.spheresMesh.setMatrixAt(sphereIdx, _matrix);
			state.contactConstraints.spheresMesh.setColorAt(sphereIdx, colorA);
			sphereIdx++;
			_position.set(contactPoint.positionB[0], contactPoint.positionB[1], contactPoint.positionB[2]);
			_matrix.compose(_position, _quaternion, _scale);
			state.contactConstraints.spheresMesh.setMatrixAt(sphereIdx, _matrix);
			state.contactConstraints.spheresMesh.setColorAt(sphereIdx, colorB);
			sphereIdx++;
			_currentPointA.set(contactPoint.positionA[0], contactPoint.positionA[1], contactPoint.positionA[2]);
			if (i > 0) {
				_edgeDir.subVectors(_currentPointA, _prevPointA);
				const edgeLength = _edgeDir.length();
				if (edgeLength > .001) {
					_edgeDir.normalize();
					_edgeMidpoint.addVectors(_prevPointA, _currentPointA).multiplyScalar(.5);
					_up.set(0, 1, 0);
					_quaternion.setFromUnitVectors(_up, _edgeDir);
					_scale.set(CONSTRAINT_EDGE_RADIUS, edgeLength, CONSTRAINT_EDGE_RADIUS);
					_matrix.compose(_edgeMidpoint, _quaternion, _scale);
					state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
					state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintManifoldEdgeColor);
					cylinderIdx++;
				}
			}
			_prevPointA.copy(_currentPointA);
		}
		if (constraint.numContactPoints > 2) {
			_firstPoint.set(constraint.contactPoints[0].positionA[0], constraint.contactPoints[0].positionA[1], constraint.contactPoints[0].positionA[2]);
			_edgeDir.subVectors(_firstPoint, _prevPointA);
			const edgeLength = _edgeDir.length();
			if (edgeLength > .001) {
				_edgeDir.normalize();
				_edgeMidpoint.addVectors(_prevPointA, _firstPoint).multiplyScalar(.5);
				_up.set(0, 1, 0);
				_quaternion.setFromUnitVectors(_up, _edgeDir);
				_scale.set(CONSTRAINT_EDGE_RADIUS, edgeLength, CONSTRAINT_EDGE_RADIUS);
				_matrix.compose(_edgeMidpoint, _quaternion, _scale);
				state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
				state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintManifoldEdgeColor);
				cylinderIdx++;
			}
		}
		const firstContactPoint = constraint.contactPoints[0];
		_normalStart.set(firstContactPoint.positionA[0], firstContactPoint.positionA[1], firstContactPoint.positionA[2]);
		_normalVec.set(constraint.normal[0], constraint.normal[1], constraint.normal[2]);
		const normalShaftLength = CONTACT_NORMAL_LENGTH - ARROW_TIP_HEIGHT;
		_up.set(0, 1, 0);
		_quaternion.setFromUnitVectors(_up, _normalVec);
		_normalShaftMidpoint.copy(_normalStart).addScaledVector(_normalVec, normalShaftLength / 2);
		_scale.set(ARROW_SHAFT_RADIUS, normalShaftLength, ARROW_SHAFT_RADIUS);
		_matrix.compose(_normalShaftMidpoint, _quaternion, _scale);
		state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
		state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintNormalColor);
		cylinderIdx++;
		_normalTipPosition.copy(_normalStart).addScaledVector(_normalVec, .45);
		_scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);
		_matrix.compose(_normalTipPosition, _quaternion, _scale);
		state.contactConstraints.conesMesh.setMatrixAt(coneIdx, _matrix);
		state.contactConstraints.conesMesh.setColorAt(coneIdx, constraintNormalColor);
		coneIdx++;
		_tangent1Vec.set(constraint.tangent1[0], constraint.tangent1[1], constraint.tangent1[2]);
		_quaternion.setFromUnitVectors(_up, _tangent1Vec);
		_tangent1Midpoint.copy(_normalStart).addScaledVector(_tangent1Vec, CONSTRAINT_TANGENT_LENGTH / 2);
		_scale.set(ARROW_SHAFT_RADIUS, CONSTRAINT_TANGENT_LENGTH, ARROW_SHAFT_RADIUS);
		_matrix.compose(_tangent1Midpoint, _quaternion, _scale);
		state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
		state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintTangent1Color);
		cylinderIdx++;
		_tangent2Vec.set(constraint.tangent2[0], constraint.tangent2[1], constraint.tangent2[2]);
		_quaternion.setFromUnitVectors(_up, _tangent2Vec);
		_tangent2Midpoint.copy(_normalStart).addScaledVector(_tangent2Vec, CONSTRAINT_TANGENT_LENGTH / 2);
		_scale.set(ARROW_SHAFT_RADIUS, CONSTRAINT_TANGENT_LENGTH, ARROW_SHAFT_RADIUS);
		_matrix.compose(_tangent2Midpoint, _quaternion, _scale);
		state.contactConstraints.cylindersMesh.setMatrixAt(cylinderIdx, _matrix);
		state.contactConstraints.cylindersMesh.setColorAt(cylinderIdx, constraintTangent2Color);
		cylinderIdx++;
	}
	state.contactConstraints.spheresMesh.instanceMatrix.needsUpdate = true;
	state.contactConstraints.cylindersMesh.instanceMatrix.needsUpdate = true;
	state.contactConstraints.conesMesh.instanceMatrix.needsUpdate = true;
	if (state.contactConstraints.spheresMesh.instanceColor) state.contactConstraints.spheresMesh.instanceColor.needsUpdate = true;
	if (state.contactConstraints.cylindersMesh.instanceColor) state.contactConstraints.cylindersMesh.instanceColor.needsUpdate = true;
	if (state.contactConstraints.conesMesh.instanceColor) state.contactConstraints.conesMesh.instanceColor.needsUpdate = true;
}
function updateBroadphaseDbvt(state, world) {
	if (!state.options.broadphaseDbvt.enabled) {
		state.broadphase.dbvtLines.visible = false;
		return;
	}
	const positions = [];
	const colors = [];
	const layerColorsLeaf = [
		[
			0,
			1,
			0
		],
		[
			1,
			.5,
			0
		],
		[
			0,
			.5,
			1
		],
		[
			1,
			0,
			1
		],
		[
			1,
			1,
			0
		],
		[
			0,
			1,
			1
		]
	];
	const layerColorsNonLeaf = [
		[
			0,
			.3,
			0
		],
		[
			.3,
			.15,
			0
		],
		[
			0,
			.15,
			.3
		],
		[
			.3,
			0,
			.3
		],
		[
			.3,
			.3,
			0
		],
		[
			0,
			.3,
			.3
		]
	];
	for (let layerIndex = 0; layerIndex < world.broadphase.dbvts.length; layerIndex++) {
		const dbvt = world.broadphase.dbvts[layerIndex];
		if (dbvt.root === -1) continue;
		const stack = [dbvt.root];
		while (stack.length > 0) {
			const nodeIndex = stack.pop();
			const node = dbvt.nodes[nodeIndex];
			const isLeaf = node.left === -1 && node.right === -1;
			if (isLeaf && !state.options.broadphaseDbvt.showLeafNodes) {
				if (node.left !== -1) stack.push(node.left);
				if (node.right !== -1) stack.push(node.right);
				continue;
			}
			if (!isLeaf && !state.options.broadphaseDbvt.showNonLeafNodes) {
				if (node.left !== -1) stack.push(node.left);
				if (node.right !== -1) stack.push(node.right);
				continue;
			}
			const color = isLeaf ? layerColorsLeaf[layerIndex % layerColorsLeaf.length] : layerColorsNonLeaf[layerIndex % layerColorsNonLeaf.length];
			const minX = node.aabb[0], minY = node.aabb[1], minZ = node.aabb[2];
			const maxX = node.aabb[3], maxY = node.aabb[4], maxZ = node.aabb[5];
			positions.push(minX, minY, minZ, maxX, minY, minZ);
			colors.push(...color, ...color);
			positions.push(maxX, minY, minZ, maxX, minY, maxZ);
			colors.push(...color, ...color);
			positions.push(maxX, minY, maxZ, minX, minY, maxZ);
			colors.push(...color, ...color);
			positions.push(minX, minY, maxZ, minX, minY, minZ);
			colors.push(...color, ...color);
			positions.push(minX, maxY, minZ, maxX, maxY, minZ);
			colors.push(...color, ...color);
			positions.push(maxX, maxY, minZ, maxX, maxY, maxZ);
			colors.push(...color, ...color);
			positions.push(maxX, maxY, maxZ, minX, maxY, maxZ);
			colors.push(...color, ...color);
			positions.push(minX, maxY, maxZ, minX, maxY, minZ);
			colors.push(...color, ...color);
			positions.push(minX, minY, minZ, minX, maxY, minZ);
			colors.push(...color, ...color);
			positions.push(maxX, minY, minZ, maxX, maxY, minZ);
			colors.push(...color, ...color);
			positions.push(maxX, minY, maxZ, maxX, maxY, maxZ);
			colors.push(...color, ...color);
			positions.push(minX, minY, maxZ, minX, maxY, maxZ);
			colors.push(...color, ...color);
			if (node.left !== -1) stack.push(node.left);
			if (node.right !== -1) stack.push(node.right);
		}
	}
	const geometry = state.broadphase.dbvtLines.geometry;
	geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(positions), 3));
	geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(colors), 3));
	geometry.computeBoundingSphere();
	state.broadphase.dbvtLines.visible = true;
}
function updateVelocities(state, world) {
	if (state.bodies.linearVelocityArrows.size > 0) {
		for (const arrow of state.bodies.linearVelocityArrows.values()) {
			state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.shaftId);
			state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.tipId);
		}
		state.bodies.linearVelocityArrows.clear();
	}
	if (state.bodies.angularVelocityArrows.size > 0) {
		for (const arrow of state.bodies.angularVelocityArrows.values()) {
			state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.shaftId);
			state.bodies.velocitiesBatchedMesh.deleteInstance(arrow.tipId);
		}
		state.bodies.angularVelocityArrows.clear();
	}
	if (!state.options.bodies.showLinearVelocity && !state.options.bodies.showAngularVelocity) {
		state.bodies.velocitiesBatchedMesh.visible = false;
		return;
	}
	state.bodies.velocitiesBatchedMesh.visible = true;
	if (state.bodies.velocityCylinderGeometryId === null) state.bodies.velocityCylinderGeometryId = state.bodies.velocitiesBatchedMesh.addGeometry(createUnitCylinderGeometry());
	if (state.bodies.velocityConeGeometryId === null) state.bodies.velocityConeGeometryId = state.bodies.velocitiesBatchedMesh.addGeometry(createUnitConeGeometry());
	const cylinderGeomId = state.bodies.velocityCylinderGeometryId;
	const coneGeomId = state.bodies.velocityConeGeometryId;
	for (const body of world.bodies.pool) {
		if (body._pooled) continue;
		if (body.motionType === MotionType.STATIC) continue;
		const comX = body.centerOfMassPosition[0];
		const comY = body.centerOfMassPosition[1];
		const comZ = body.centerOfMassPosition[2];
		if (state.options.bodies.showLinearVelocity) {
			const velX = body.motionProperties.linearVelocity[0];
			const velY = body.motionProperties.linearVelocity[1];
			const velZ = body.motionProperties.linearVelocity[2];
			const velMag = Math.sqrt(velX * velX + velY * velY + velZ * velZ);
			if (velMag > .01) {
				const velNormX = velX / velMag;
				const velNormY = velY / velMag;
				const velNormZ = velZ / velMag;
				const shaftLength = Math.min(velMag, 5) - ARROW_TIP_HEIGHT;
				const shaftMidX = comX + velNormX * shaftLength * .5;
				const shaftMidY = comY + velNormY * shaftLength * .5;
				const shaftMidZ = comZ + velNormZ * shaftLength * .5;
				_position.set(shaftMidX, shaftMidY, shaftMidZ);
				_up.set(0, 1, 0);
				_normalVec.set(velNormX, velNormY, velNormZ);
				_quaternion.setFromUnitVectors(_up, _normalVec);
				_scale.set(ARROW_SHAFT_RADIUS, shaftLength, ARROW_SHAFT_RADIUS);
				const shaftMatrix = new THREE.Matrix4();
				shaftMatrix.compose(_position, _quaternion, _scale);
				const shaftId = addInstanceToBatchedMesh(state.bodies.velocitiesBatchedMesh, cylinderGeomId, shaftMatrix, linearVelocityColor);
				const tipX = comX + velNormX * (shaftLength + ARROW_TIP_HEIGHT * .5);
				const tipY = comY + velNormY * (shaftLength + ARROW_TIP_HEIGHT * .5);
				const tipZ = comZ + velNormZ * (shaftLength + ARROW_TIP_HEIGHT * .5);
				_position.set(tipX, tipY, tipZ);
				_scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);
				const tipMatrix = new THREE.Matrix4();
				tipMatrix.compose(_position, _quaternion, _scale);
				const tipId = addInstanceToBatchedMesh(state.bodies.velocitiesBatchedMesh, coneGeomId, tipMatrix, linearVelocityColor);
				state.bodies.linearVelocityArrows.set(body.id, {
					shaftId,
					tipId
				});
			}
		}
		if (state.options.bodies.showAngularVelocity) {
			const angVelX = body.motionProperties.angularVelocity[0];
			const angVelY = body.motionProperties.angularVelocity[1];
			const angVelZ = body.motionProperties.angularVelocity[2];
			const angVelMag = Math.sqrt(angVelX * angVelX + angVelY * angVelY + angVelZ * angVelZ);
			if (angVelMag > .01) {
				const angVelNormX = angVelX / angVelMag;
				const angVelNormY = angVelY / angVelMag;
				const angVelNormZ = angVelZ / angVelMag;
				const shaftLength = Math.min(angVelMag * .5, 3) - ARROW_TIP_HEIGHT;
				const shaftMidX = comX + angVelNormX * shaftLength * .5;
				const shaftMidY = comY + angVelNormY * shaftLength * .5;
				const shaftMidZ = comZ + angVelNormZ * shaftLength * .5;
				_position.set(shaftMidX, shaftMidY, shaftMidZ);
				_up.set(0, 1, 0);
				_normalVec.set(angVelNormX, angVelNormY, angVelNormZ);
				_quaternion.setFromUnitVectors(_up, _normalVec);
				_scale.set(ARROW_SHAFT_RADIUS, shaftLength, ARROW_SHAFT_RADIUS);
				const shaftMatrix = new THREE.Matrix4();
				shaftMatrix.compose(_position, _quaternion, _scale);
				const shaftId = addInstanceToBatchedMesh(state.bodies.velocitiesBatchedMesh, cylinderGeomId, shaftMatrix, angularVelocityColor);
				const tipX = comX + angVelNormX * (shaftLength + ARROW_TIP_HEIGHT * .5);
				const tipY = comY + angVelNormY * (shaftLength + ARROW_TIP_HEIGHT * .5);
				const tipZ = comZ + angVelNormZ * (shaftLength + ARROW_TIP_HEIGHT * .5);
				_position.set(tipX, tipY, tipZ);
				_scale.set(ARROW_SHAFT_RADIUS * 3, ARROW_TIP_HEIGHT, ARROW_SHAFT_RADIUS * 3);
				const tipMatrix = new THREE.Matrix4();
				tipMatrix.compose(_position, _quaternion, _scale);
				const tipId = addInstanceToBatchedMesh(state.bodies.velocitiesBatchedMesh, coneGeomId, tipMatrix, angularVelocityColor);
				state.bodies.angularVelocityArrows.set(body.id, {
					shaftId,
					tipId
				});
			}
		}
	}
}
/**
* Helper to collect triangle mesh shapes from a shape hierarchy.
* Returns array of { shape, worldMatrix } for each triangle mesh found.
*/
function collectTriangleMeshShapes(shape, parentMatrix, output) {
	switch (shape.type) {
		case ShapeType.TRIANGLE_MESH:
			output.push({
				shape,
				worldMatrix: parentMatrix.clone()
			});
			break;
		case ShapeType.COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				const childMatrix = parentMatrix.clone().multiply(_matrix);
				collectTriangleMeshShapes(child.shape, childMatrix, output);
			}
			break;
		case ShapeType.STATIC_COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				const childMatrix = parentMatrix.clone().multiply(_matrix);
				collectTriangleMeshShapes(child.shape, childMatrix, output);
			}
			break;
		case ShapeType.TRANSFORMED: {
			const [tx, ty, tz] = shape.position;
			const [qx, qy, qz, qw] = shape.quaternion;
			_position.set(tx, ty, tz);
			_quaternion.set(qx, qy, qz, qw);
			_scale.set(1, 1, 1);
			_matrix.compose(_position, _quaternion, _scale);
			const transformedMatrix = parentMatrix.clone().multiply(_matrix);
			collectTriangleMeshShapes(shape.shape, transformedMatrix, output);
			break;
		}
		case ShapeType.SCALED: {
			const [sx, sy, sz] = shape.scale;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(sx, sy, sz);
			_matrix.compose(_position, _quaternion, _scale);
			const scaledMatrix = parentMatrix.clone().multiply(_matrix);
			collectTriangleMeshShapes(shape.shape, scaledMatrix, output);
			break;
		}
		default: break;
	}
}
function collectStaticCompoundShapes(shape, parentMatrix, output) {
	switch (shape.type) {
		case ShapeType.STATIC_COMPOUND:
			output.push({
				shape,
				worldMatrix: parentMatrix.clone()
			});
			break;
		case ShapeType.COMPOUND:
			for (const child of shape.children) {
				const [tx, ty, tz] = child.position;
				const [qx, qy, qz, qw] = child.quaternion;
				_position.set(tx, ty, tz);
				_quaternion.set(qx, qy, qz, qw);
				_scale.set(1, 1, 1);
				_matrix.compose(_position, _quaternion, _scale);
				const childMatrix = parentMatrix.clone().multiply(_matrix);
				collectStaticCompoundShapes(child.shape, childMatrix, output);
			}
			break;
		case ShapeType.TRANSFORMED: {
			const [tx, ty, tz] = shape.position;
			const [qx, qy, qz, qw] = shape.quaternion;
			_position.set(tx, ty, tz);
			_quaternion.set(qx, qy, qz, qw);
			_scale.set(1, 1, 1);
			_matrix.compose(_position, _quaternion, _scale);
			const transformedMatrix = parentMatrix.clone().multiply(_matrix);
			collectStaticCompoundShapes(shape.shape, transformedMatrix, output);
			break;
		}
		case ShapeType.SCALED: {
			const [sx, sy, sz] = shape.scale;
			_position.set(0, 0, 0);
			_quaternion.set(0, 0, 0, 1);
			_scale.set(sx, sy, sz);
			_matrix.compose(_position, _quaternion, _scale);
			const scaledMatrix = parentMatrix.clone().multiply(_matrix);
			collectStaticCompoundShapes(shape.shape, scaledMatrix, output);
			break;
		}
		default: break;
	}
}
/**
* Add wireframe box edges to position/color arrays for a BVH AABB,
* transformed by a given world matrix.
*/
function addBvhBoxWireframe(positions, colors, aabbMin, aabbMax, worldMatrix, color) {
	const min = aabbMin;
	const max = aabbMax;
	const corners = [
		new THREE.Vector3(min[0], min[1], min[2]),
		new THREE.Vector3(max[0], min[1], min[2]),
		new THREE.Vector3(max[0], min[1], max[2]),
		new THREE.Vector3(min[0], min[1], max[2]),
		new THREE.Vector3(min[0], max[1], min[2]),
		new THREE.Vector3(max[0], max[1], min[2]),
		new THREE.Vector3(max[0], max[1], max[2]),
		new THREE.Vector3(min[0], max[1], max[2])
	];
	for (const corner of corners) corner.applyMatrix4(worldMatrix);
	for (const [a, b] of [
		[0, 1],
		[1, 2],
		[2, 3],
		[3, 0],
		[4, 5],
		[5, 6],
		[6, 7],
		[7, 4],
		[0, 4],
		[1, 5],
		[2, 6],
		[3, 7]
	]) {
		positions.push(corners[a].x, corners[a].y, corners[a].z);
		positions.push(corners[b].x, corners[b].y, corners[b].z);
		colors.push(color[0], color[1], color[2]);
		colors.push(color[0], color[1], color[2]);
	}
}
function updateTriangleMeshBvh(state, world) {
	if (!state.options.triangleMeshBvh.enabled) {
		state.triangleMeshBvh.container.visible = false;
		return;
	}
	state.triangleMeshBvh.container.visible = true;
	const showLeafNodes = state.options.triangleMeshBvh.showLeafNodes;
	const showNonLeafNodes = state.options.triangleMeshBvh.showNonLeafNodes;
	if (showLeafNodes !== state.triangleMeshBvh.previousShowLeafNodes || showNonLeafNodes !== state.triangleMeshBvh.previousShowNonLeafNodes) {
		for (const [_bodyId, lineSegments] of state.triangleMeshBvh.cache) {
			state.triangleMeshBvh.container.remove(lineSegments);
			lineSegments.geometry.dispose();
			if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
		}
		state.triangleMeshBvh.cache.clear();
		state.triangleMeshBvh.previousShowLeafNodes = showLeafNodes;
		state.triangleMeshBvh.previousShowNonLeafNodes = showNonLeafNodes;
	}
	const seenBodyIds = /* @__PURE__ */ new Set();
	for (const body of rigidBody.iterate(world)) {
		seenBodyIds.add(body.id);
		if (state.triangleMeshBvh.cache.has(body.id)) {
			const cachedLines = state.triangleMeshBvh.cache.get(body.id);
			_position.set(body.position[0], body.position[1], body.position[2]);
			_quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
			_scale.set(1, 1, 1);
			cachedLines.position.copy(_position);
			cachedLines.quaternion.copy(_quaternion);
			cachedLines.scale.copy(_scale);
			continue;
		}
		const identityMatrix = new THREE.Matrix4();
		const triangleMeshes = [];
		collectTriangleMeshShapes(body.shape, identityMatrix, triangleMeshes);
		if (triangleMeshes.length === 0) {
			const emptyLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
			emptyLines.visible = false;
			state.triangleMeshBvh.cache.set(body.id, emptyLines);
			state.triangleMeshBvh.container.add(emptyLines);
			continue;
		}
		const positions = [];
		const colors = [];
		for (const { shape: meshShape, worldMatrix } of triangleMeshes) {
			const buffer = meshShape.bvh.buffer;
			if (buffer.length === 0) continue;
			const leafColors = [
				[
					0,
					1,
					0
				],
				[
					0,
					1,
					.5
				],
				[
					0,
					.8,
					1
				],
				[
					0,
					.5,
					1
				],
				[
					.5,
					0,
					1
				],
				[
					1,
					0,
					1
				]
			];
			const nonLeafColors = [
				[
					0,
					.4,
					0
				],
				[
					0,
					.4,
					.2
				],
				[
					0,
					.3,
					.4
				],
				[
					0,
					.2,
					.4
				],
				[
					.2,
					0,
					.4
				],
				[
					.4,
					0,
					.4
				]
			];
			const stack = [{
				offset: 0,
				depth: 0
			}];
			while (stack.length > 0) {
				const { offset, depth } = stack.pop();
				const isLeaf = bvh.nodeIsLeaf(buffer, offset);
				if (isLeaf && !showLeafNodes) continue;
				if (!isLeaf && !showNonLeafNodes) {
					stack.push({
						offset: bvh.nodeLeft(offset),
						depth: depth + 1
					});
					stack.push({
						offset: bvh.nodeRight(buffer, offset),
						depth: depth + 1
					});
					continue;
				}
				const colorPalette = isLeaf ? leafColors : nonLeafColors;
				const color = colorPalette[depth % colorPalette.length];
				addBvhBoxWireframe(positions, colors, [
					buffer[offset + bvh.NODE_MIN_X],
					buffer[offset + bvh.NODE_MIN_Y],
					buffer[offset + bvh.NODE_MIN_Z]
				], [
					buffer[offset + bvh.NODE_MAX_X],
					buffer[offset + bvh.NODE_MAX_Y],
					buffer[offset + bvh.NODE_MAX_Z]
				], worldMatrix, color);
				if (!isLeaf) {
					stack.push({
						offset: bvh.nodeLeft(offset),
						depth: depth + 1
					});
					stack.push({
						offset: bvh.nodeRight(buffer, offset),
						depth: depth + 1
					});
				}
			}
		}
		const geometry = new THREE.BufferGeometry();
		geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(positions), 3));
		geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(colors), 3));
		const material = new THREE.LineBasicMaterial({ vertexColors: true });
		const lineSegments = new THREE.LineSegments(geometry, material);
		lineSegments.frustumCulled = false;
		_position.set(body.position[0], body.position[1], body.position[2]);
		_quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
		_scale.set(1, 1, 1);
		lineSegments.position.copy(_position);
		lineSegments.quaternion.copy(_quaternion);
		lineSegments.scale.copy(_scale);
		state.triangleMeshBvh.cache.set(body.id, lineSegments);
		state.triangleMeshBvh.container.add(lineSegments);
	}
	for (const [bodyId, lineSegments] of state.triangleMeshBvh.cache) if (!seenBodyIds.has(bodyId)) {
		state.triangleMeshBvh.container.remove(lineSegments);
		lineSegments.geometry.dispose();
		if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
		state.triangleMeshBvh.cache.delete(bodyId);
	}
}
function updateStaticCompoundBvh(state, world) {
	if (!state.options.staticCompoundBvh.enabled) {
		state.staticCompoundBvh.container.visible = false;
		return;
	}
	state.staticCompoundBvh.container.visible = true;
	const showLeafNodes = state.options.staticCompoundBvh.showLeafNodes;
	const showNonLeafNodes = state.options.staticCompoundBvh.showNonLeafNodes;
	if (showLeafNodes !== state.staticCompoundBvh.previousShowLeafNodes || showNonLeafNodes !== state.staticCompoundBvh.previousShowNonLeafNodes) {
		for (const [_bodyId, lineSegments] of state.staticCompoundBvh.cache) {
			state.staticCompoundBvh.container.remove(lineSegments);
			lineSegments.geometry.dispose();
			if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
		}
		state.staticCompoundBvh.cache.clear();
		state.staticCompoundBvh.previousShowLeafNodes = showLeafNodes;
		state.staticCompoundBvh.previousShowNonLeafNodes = showNonLeafNodes;
	}
	const seenBodyIds = /* @__PURE__ */ new Set();
	for (const body of rigidBody.iterate(world)) {
		seenBodyIds.add(body.id);
		if (state.staticCompoundBvh.cache.has(body.id)) {
			const cachedLines = state.staticCompoundBvh.cache.get(body.id);
			_position.set(body.position[0], body.position[1], body.position[2]);
			_quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
			_scale.set(1, 1, 1);
			cachedLines.position.copy(_position);
			cachedLines.quaternion.copy(_quaternion);
			cachedLines.scale.copy(_scale);
			continue;
		}
		const identityMatrix = new THREE.Matrix4();
		const staticCompounds = [];
		collectStaticCompoundShapes(body.shape, identityMatrix, staticCompounds);
		if (staticCompounds.length === 0) {
			const emptyLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
			emptyLines.visible = false;
			state.staticCompoundBvh.cache.set(body.id, emptyLines);
			state.staticCompoundBvh.container.add(emptyLines);
			continue;
		}
		const positions = [];
		const colors = [];
		for (const { shape: compoundShape, worldMatrix } of staticCompounds) {
			const buffer = compoundShape.bvh.buffer;
			if (buffer.length === 0) continue;
			const leafColors = [
				[
					1,
					.5,
					0
				],
				[
					1,
					.6,
					0
				],
				[
					1,
					.7,
					.2
				],
				[
					1,
					.8,
					.3
				],
				[
					1,
					.9,
					.4
				],
				[
					1,
					1,
					.5
				]
			];
			const nonLeafColors = [
				[
					.4,
					.2,
					0
				],
				[
					.4,
					.25,
					0
				],
				[
					.4,
					.3,
					.1
				],
				[
					.4,
					.35,
					.15
				],
				[
					.4,
					.4,
					.2
				],
				[
					.4,
					.45,
					.25
				]
			];
			const stack = [{
				offset: 0,
				depth: 0
			}];
			while (stack.length > 0) {
				const { offset, depth } = stack.pop();
				const isLeaf = bvh.nodeIsLeaf(buffer, offset);
				if (isLeaf && !showLeafNodes) continue;
				if (!isLeaf && !showNonLeafNodes) {
					stack.push({
						offset: bvh.nodeLeft(offset),
						depth: depth + 1
					});
					stack.push({
						offset: bvh.nodeRight(buffer, offset),
						depth: depth + 1
					});
					continue;
				}
				const colorPalette = isLeaf ? leafColors : nonLeafColors;
				const color = colorPalette[depth % colorPalette.length];
				addBvhBoxWireframe(positions, colors, [
					buffer[offset + bvh.NODE_MIN_X],
					buffer[offset + bvh.NODE_MIN_Y],
					buffer[offset + bvh.NODE_MIN_Z]
				], [
					buffer[offset + bvh.NODE_MAX_X],
					buffer[offset + bvh.NODE_MAX_Y],
					buffer[offset + bvh.NODE_MAX_Z]
				], worldMatrix, color);
				if (!isLeaf) {
					stack.push({
						offset: bvh.nodeLeft(offset),
						depth: depth + 1
					});
					stack.push({
						offset: bvh.nodeRight(buffer, offset),
						depth: depth + 1
					});
				}
			}
		}
		const geometry = new THREE.BufferGeometry();
		geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(positions), 3));
		geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(colors), 3));
		const material = new THREE.LineBasicMaterial({ vertexColors: true });
		const lineSegments = new THREE.LineSegments(geometry, material);
		lineSegments.frustumCulled = false;
		_position.set(body.position[0], body.position[1], body.position[2]);
		_quaternion.set(body.quaternion[0], body.quaternion[1], body.quaternion[2], body.quaternion[3]);
		_scale.set(1, 1, 1);
		lineSegments.position.copy(_position);
		lineSegments.quaternion.copy(_quaternion);
		lineSegments.scale.copy(_scale);
		state.staticCompoundBvh.cache.set(body.id, lineSegments);
		state.staticCompoundBvh.container.add(lineSegments);
	}
	for (const [bodyId, lineSegments] of state.staticCompoundBvh.cache) if (!seenBodyIds.has(bodyId)) {
		state.staticCompoundBvh.container.remove(lineSegments);
		lineSegments.geometry.dispose();
		if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
		state.staticCompoundBvh.cache.delete(bodyId);
	}
}
function update(state, world) {
	updateBodies(state, world);
	updateContacts(state, world);
	updateContactConstraints(state, world);
	updateBroadphaseDbvt(state, world);
	drawConstraints(state, world);
	updateVelocities(state, world);
	updateTriangleMeshBvh(state, world);
	updateStaticCompoundBvh(state, world);
}
function addConstraintLine(positions, colors, from, to, color) {
	positions.push(from[0], from[1], from[2], to[0], to[1], to[2]);
	colors.push(...color, ...color);
}
function addConstraintMarker(positions, colors, pos, size, color) {
	addConstraintLine(positions, colors, [
		pos[0] - size,
		pos[1],
		pos[2]
	], [
		pos[0] + size,
		pos[1],
		pos[2]
	], color);
	addConstraintLine(positions, colors, [
		pos[0],
		pos[1] - size,
		pos[2]
	], [
		pos[0],
		pos[1] + size,
		pos[2]
	], color);
	addConstraintLine(positions, colors, [
		pos[0],
		pos[1],
		pos[2] - size
	], [
		pos[0],
		pos[1],
		pos[2] + size
	], color);
}
const _transformPointOut = create$1();
const _transformDirOut = create$1();
const _transformPointOut2 = create$1();
const _transformPoint_rotated = create$1();
const _constraint_twistAxis = create$1();
const _constraint_planeAxis = create$1();
const _constraint_normalAxis = create$1();
const _constraint_xAxis = create$1();
const _constraint_yAxis = create$1();
const _constraint_zAxis = create$1();
const _constraint_quat = create();
const _constraint_normal1 = create$1();
function transformPointToWorld(out, localPos, bodyTransform) {
	if (!bodyTransform) {
		set(out, 0, 0, 0);
		return;
	}
	transformQuat(_transformPoint_rotated, localPos, bodyTransform.quaternion);
	add(out, _transformPoint_rotated, bodyTransform.centerOfMassPosition);
}
function transformDirectionToWorld(out, localDir, bodyTransform) {
	if (!bodyTransform) {
		set(out, 0, 0, 0);
		return;
	}
	transformQuat(out, localDir, bodyTransform.quaternion);
}
const _pieRotationQuat = create();
const _pieAxisVec = create$1();
const _pieRotatedAxis = create$1();
function rotateVectorAroundAxis(out, vector, axis, angle) {
	setAxisAngle(_pieRotationQuat, axis, angle);
	transformQuat(out, vector, _pieRotationQuat);
}
function drawPie(positions, colors, center, radius, normal, axis, minAngle, maxAngle, color) {
	if (minAngle >= maxAngle) return;
	const numSegments = 32;
	const angleStep = (maxAngle - minAngle) / numSegments;
	copy(_pieAxisVec, axis);
	let prevPoint = null;
	for (let i = 0; i <= numSegments; i++) {
		rotateVectorAroundAxis(_pieRotatedAxis, _pieAxisVec, normal, minAngle + i * angleStep);
		const point = [
			center[0] + _pieRotatedAxis[0] * radius,
			center[1] + _pieRotatedAxis[1] * radius,
			center[2] + _pieRotatedAxis[2] * radius
		];
		addConstraintLine(positions, colors, center, point, color);
		if (prevPoint !== null) addConstraintLine(positions, colors, prevPoint, point, color);
		prevPoint = point;
	}
}
function drawSwingConeLimits(positions, colors, center, twistAxis, planeAxis, normalAxis, planeHalfAngle, normalHalfAngle, radius, color) {
	const maxAngle = Math.max(planeHalfAngle, normalHalfAngle);
	if (maxAngle <= 0) return;
	const numSegments = 32;
	const angleStep = 2 * Math.PI / numSegments;
	let prevPoint = null;
	for (let i = 0; i <= numSegments; i++) {
		const theta = i * angleStep;
		const cosTheta = Math.cos(theta);
		const sinTheta = Math.sin(theta);
		const cosMaxAngle = Math.cos(maxAngle);
		const sinMaxAngle = Math.sin(maxAngle);
		const dirX = twistAxis[0] * cosMaxAngle + (planeAxis[0] * cosTheta + normalAxis[0] * sinTheta) * sinMaxAngle;
		const dirY = twistAxis[1] * cosMaxAngle + (planeAxis[1] * cosTheta + normalAxis[1] * sinTheta) * sinMaxAngle;
		const dirZ = twistAxis[2] * cosMaxAngle + (planeAxis[2] * cosTheta + normalAxis[2] * sinTheta) * sinMaxAngle;
		const point = [
			center[0] + dirX * radius,
			center[1] + dirY * radius,
			center[2] + dirZ * radius
		];
		if (i % 4 === 0) addConstraintLine(positions, colors, center, point, color);
		if (prevPoint !== null) addConstraintLine(positions, colors, prevPoint, point, color);
		prevPoint = point;
	}
}
function drawConstraints(state, world) {
	if (!state.options.constraints.enabled) {
		state.constraints.lineSegments.visible = false;
		return;
	}
	state.constraints.lineSegments.visible = true;
	const positions = [];
	const colors = [];
	const size = state.options.constraints.size;
	const drawLimits = state.options.constraints.drawLimits;
	const redColor = [
		1,
		0,
		0
	];
	const greenColor = [
		0,
		1,
		0
	];
	const blueColor = [
		0,
		0,
		1
	];
	const whiteColor = [
		1,
		1,
		1
	];
	const purpleColor = [
		.5,
		0,
		.5
	];
	const yellowColor = [
		1,
		1,
		0
	];
	const constraints = world.constraints;
	const bodies = world.bodies.pool;
	const getBodyTransform = (bodyIndex) => {
		const body = bodies[bodyIndex];
		if (!body || body._pooled) return null;
		return {
			position: body.position,
			quaternion: body.quaternion,
			centerOfMassPosition: body.centerOfMassPosition
		};
	};
	const hingePool = constraints.pools[ConstraintType.HINGE];
	if (hingePool) for (const constraint of hingePool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		transformDirectionToWorld(_transformDirOut, constraint.localSpaceHingeAxis1, bodyA);
		const pos1 = _transformPointOut;
		const pos2 = _transformPointOut2;
		const axis1 = _transformDirOut;
		addConstraintMarker(positions, colors, pos1, size * .1, redColor);
		addConstraintLine(positions, colors, pos1, [
			pos1[0] + axis1[0] * size,
			pos1[1] + axis1[1] * size,
			pos1[2] + axis1[2] * size
		], redColor);
		addConstraintMarker(positions, colors, pos2, size * .1, greenColor);
		if (drawLimits && constraint.hasLimits && constraint.limitsMax > constraint.limitsMin) {
			set(_constraint_normal1, constraint.localSpaceNormalAxis1[0], constraint.localSpaceNormalAxis1[1], constraint.localSpaceNormalAxis1[2]);
			transformDirectionToWorld(_constraint_normal1, _constraint_normal1, bodyA);
			addConstraintLine(positions, colors, pos2, [
				pos2[0] + _constraint_normal1[0] * size,
				pos2[1] + _constraint_normal1[1] * size,
				pos2[2] + _constraint_normal1[2] * size
			], whiteColor);
			drawPie(positions, colors, pos1, size, axis1, _constraint_normal1, constraint.limitsMin, constraint.limitsMax, purpleColor);
		} else {
			set(_constraint_normal1, constraint.localSpaceNormalAxis1[0], constraint.localSpaceNormalAxis1[1], constraint.localSpaceNormalAxis1[2]);
			transformDirectionToWorld(_constraint_normal1, _constraint_normal1, bodyA);
			addConstraintLine(positions, colors, pos2, [
				pos2[0] + _constraint_normal1[0] * size,
				pos2[1] + _constraint_normal1[1] * size,
				pos2[2] + _constraint_normal1[2] * size
			], whiteColor);
		}
	}
	const swingTwistPool = constraints.pools[ConstraintType.SWING_TWIST];
	if (swingTwistPool) for (const constraint of swingTwistPool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		const pos1 = _transformPointOut;
		const pos2 = _transformPointOut2;
		addConstraintMarker(positions, colors, pos1, size * .1, redColor);
		addConstraintMarker(positions, colors, pos2, size * .1, greenColor);
		addConstraintLine(positions, colors, pos1, pos2, whiteColor);
		if (drawLimits) {
			const c2b1 = constraint.constraintToBody1;
			const qA = bodyA.quaternion;
			multiply(_constraint_quat, qA, c2b1);
			const constraintTransform = {
				...bodyA,
				quaternion: _constraint_quat
			};
			set(_constraint_xAxis, 1, 0, 0);
			set(_constraint_yAxis, 0, 1, 0);
			set(_constraint_zAxis, 0, 0, 1);
			transformDirectionToWorld(_constraint_twistAxis, _constraint_xAxis, constraintTransform);
			transformDirectionToWorld(_constraint_planeAxis, _constraint_yAxis, constraintTransform);
			transformDirectionToWorld(_constraint_normalAxis, _constraint_zAxis, constraintTransform);
			if (constraint.planeHalfConeAngle > 0 || constraint.normalHalfConeAngle > 0) drawSwingConeLimits(positions, colors, pos1, _constraint_twistAxis, _constraint_planeAxis, _constraint_normalAxis, constraint.planeHalfConeAngle, constraint.normalHalfConeAngle, size, greenColor);
			if (constraint.twistMaxAngle > constraint.twistMinAngle) drawPie(positions, colors, pos1, size, _constraint_twistAxis, _constraint_planeAxis, constraint.twistMinAngle, constraint.twistMaxAngle, purpleColor);
		}
	}
	const distancePool = constraints.pools[ConstraintType.DISTANCE];
	if (distancePool) for (const constraint of distancePool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		const pos1 = _transformPointOut;
		const pos2 = _transformPointOut2;
		addConstraintLine(positions, colors, pos1, pos2, greenColor);
		if (drawLimits && constraint.minDistance !== constraint.maxDistance) {
			const direction = [
				pos2[0] - pos1[0],
				pos2[1] - pos1[1],
				pos2[2] - pos1[2]
			];
			const currentDistance = Math.sqrt(direction[0] * direction[0] + direction[1] * direction[1] + direction[2] * direction[2]);
			if (currentDistance > .001) {
				const invLen = 1 / currentDistance;
				direction[0] *= invLen;
				direction[1] *= invLen;
				direction[2] *= invLen;
				if (constraint.minDistance > 0) {
					const numSegments = 16;
					const angleStep = 2 * Math.PI / numSegments;
					let perpX, perpY, perpZ;
					if (Math.abs(direction[0]) < .9) {
						perpX = 0;
						perpY = -direction[2];
						perpZ = direction[1];
					} else {
						perpX = -direction[1];
						perpY = direction[0];
						perpZ = 0;
					}
					const perpLen = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
					perpX /= perpLen;
					perpY /= perpLen;
					perpZ /= perpLen;
					let prevPoint = null;
					for (let i = 0; i <= numSegments; i++) {
						const angle = i * angleStep;
						const cos = Math.cos(angle);
						const point = [
							pos1[0] + perpX * cos * constraint.minDistance,
							pos1[1] + perpY * cos * constraint.minDistance,
							pos1[2] + perpZ * cos * constraint.minDistance
						];
						if (prevPoint !== null) addConstraintLine(positions, colors, prevPoint, point, blueColor);
						prevPoint = point;
					}
				}
				if (constraint.maxDistance < Infinity) {
					const numSegments = 16;
					const angleStep = 2 * Math.PI / numSegments;
					let perpX, perpY, perpZ;
					if (Math.abs(direction[0]) < .9) {
						perpX = 0;
						perpY = -direction[2];
						perpZ = direction[1];
					} else {
						perpX = -direction[1];
						perpY = direction[0];
						perpZ = 0;
					}
					const perpLen = Math.sqrt(perpX * perpX + perpY * perpY + perpZ * perpZ);
					perpX /= perpLen;
					perpY /= perpLen;
					perpZ /= perpLen;
					let prevPoint = null;
					for (let i = 0; i <= numSegments; i++) {
						const angle = i * angleStep;
						const cos = Math.cos(angle);
						const point = [
							pos1[0] + perpX * cos * constraint.maxDistance,
							pos1[1] + perpY * cos * constraint.maxDistance,
							pos1[2] + perpZ * cos * constraint.maxDistance
						];
						if (prevPoint !== null) addConstraintLine(positions, colors, prevPoint, point, redColor);
						prevPoint = point;
					}
				}
			}
		}
	}
	const conePool = constraints.pools[ConstraintType.CONE];
	if (conePool) for (const constraint of conePool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		transformDirectionToWorld(_transformDirOut, constraint.localSpaceTwistAxis1, bodyA);
		const pos1 = _transformPointOut;
		const pos2 = _transformPointOut2;
		const axis1 = _transformDirOut;
		addConstraintMarker(positions, colors, pos1, size * .1, redColor);
		addConstraintLine(positions, colors, pos1, [
			pos1[0] + axis1[0] * size,
			pos1[1] + axis1[1] * size,
			pos1[2] + axis1[2] * size
		], redColor);
		const axis2 = [
			0,
			0,
			0
		];
		transformDirectionToWorld(axis2, constraint.localSpaceTwistAxis2, bodyB);
		addConstraintMarker(positions, colors, pos2, size * .1, greenColor);
		addConstraintLine(positions, colors, pos2, [
			pos2[0] + axis2[0] * size,
			pos2[1] + axis2[1] * size,
			pos2[2] + axis2[2] * size
		], greenColor);
		if (drawLimits && constraint.cosHalfConeAngle < 1) {
			const halfConeAngle = Math.acos(constraint.cosHalfConeAngle);
			const perpX = [
				0,
				0,
				0
			];
			const perpY = [
				0,
				0,
				0
			];
			if (Math.abs(axis1[0]) < .9) {
				perpX[0] = 0;
				perpX[1] = -axis1[2];
				perpX[2] = axis1[1];
			} else {
				perpX[0] = -axis1[1];
				perpX[1] = axis1[0];
				perpX[2] = 0;
			}
			const perpXLen = Math.sqrt(perpX[0] * perpX[0] + perpX[1] * perpX[1] + perpX[2] * perpX[2]);
			perpX[0] /= perpXLen;
			perpX[1] /= perpXLen;
			perpX[2] /= perpXLen;
			perpY[0] = axis1[1] * perpX[2] - axis1[2] * perpX[1];
			perpY[1] = axis1[2] * perpX[0] - axis1[0] * perpX[2];
			perpY[2] = axis1[0] * perpX[1] - axis1[1] * perpX[0];
			drawSwingConeLimits(positions, colors, pos1, axis1, perpX, perpY, halfConeAngle, halfConeAngle, size, yellowColor);
		}
	}
	const fixedPool = constraints.pools[ConstraintType.FIXED];
	if (fixedPool) for (const constraint of fixedPool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		addConstraintMarker(positions, colors, _transformPointOut, size * .1, blueColor);
		addConstraintMarker(positions, colors, _transformPointOut2, size * .1, blueColor);
	}
	const pointPool = constraints.pools[ConstraintType.POINT];
	if (pointPool) for (const constraint of pointPool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		addConstraintMarker(positions, colors, _transformPointOut, size * .1, whiteColor);
		addConstraintMarker(positions, colors, _transformPointOut2, size * .1, whiteColor);
		addConstraintLine(positions, colors, _transformPointOut, _transformPointOut2, whiteColor);
	}
	const sliderPool = constraints.pools[ConstraintType.SLIDER];
	if (sliderPool) for (const constraint of sliderPool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePositionA, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePositionB, bodyB);
		transformDirectionToWorld(_transformDirOut, constraint.localSpaceSliderAxisA, bodyA);
		const pos1 = _transformPointOut;
		const pos2 = _transformPointOut2;
		const axis1 = _transformDirOut;
		addConstraintMarker(positions, colors, pos1, size * .1, redColor);
		addConstraintLine(positions, colors, pos1, [
			pos1[0] + axis1[0] * size,
			pos1[1] + axis1[1] * size,
			pos1[2] + axis1[2] * size
		], redColor);
		addConstraintMarker(positions, colors, pos2, size * .1, greenColor);
		if (drawLimits && constraint.hasLimits) {
			if (constraint.limitsMin > -Infinity) addConstraintMarker(positions, colors, [
				pos1[0] + axis1[0] * constraint.limitsMin,
				pos1[1] + axis1[1] * constraint.limitsMin,
				pos1[2] + axis1[2] * constraint.limitsMin
			], size * .15, blueColor);
			if (constraint.limitsMax < Infinity) addConstraintMarker(positions, colors, [
				pos1[0] + axis1[0] * constraint.limitsMax,
				pos1[1] + axis1[1] * constraint.limitsMax,
				pos1[2] + axis1[2] * constraint.limitsMax
			], size * .15, redColor);
			if (constraint.limitsMin > -Infinity && constraint.limitsMax < Infinity) addConstraintLine(positions, colors, [
				pos1[0] + axis1[0] * constraint.limitsMin,
				pos1[1] + axis1[1] * constraint.limitsMin,
				pos1[2] + axis1[2] * constraint.limitsMin
			], [
				pos1[0] + axis1[0] * constraint.limitsMax,
				pos1[1] + axis1[1] * constraint.limitsMax,
				pos1[2] + axis1[2] * constraint.limitsMax
			], purpleColor);
		}
	}
	const sixDOFPool = constraints.pools[ConstraintType.SIX_DOF];
	if (sixDOFPool) for (const constraint of sixDOFPool.constraints) {
		if (constraint._pooled || !constraint.enabled) continue;
		const bodyA = getBodyTransform(constraint.bodyIndexA);
		const bodyB = getBodyTransform(constraint.bodyIndexB);
		if (!bodyA || !bodyB) continue;
		transformPointToWorld(_transformPointOut, constraint.localSpacePosition1, bodyA);
		transformPointToWorld(_transformPointOut2, constraint.localSpacePosition2, bodyB);
		const pos1 = _transformPointOut;
		const pos2 = _transformPointOut2;
		addConstraintMarker(positions, colors, pos1, size * .1, purpleColor);
		addConstraintMarker(positions, colors, pos2, size * .1, purpleColor);
		if (drawLimits) {
			const c2b1 = constraint.constraintToBody1;
			const qA = bodyA.quaternion;
			const constraintQuat = [
				qA[3] * c2b1[0] + qA[0] * c2b1[3] + qA[1] * c2b1[2] - qA[2] * c2b1[1],
				qA[3] * c2b1[1] + qA[1] * c2b1[3] + qA[2] * c2b1[0] - qA[0] * c2b1[2],
				qA[3] * c2b1[2] + qA[2] * c2b1[3] + qA[0] * c2b1[1] - qA[1] * c2b1[0],
				qA[3] * c2b1[3] - qA[0] * c2b1[0] - qA[1] * c2b1[1] - qA[2] * c2b1[2]
			];
			const constraintTransform = {
				...bodyA,
				quaternion: constraintQuat
			};
			const axisX = [
				0,
				0,
				0
			];
			const axisY = [
				0,
				0,
				0
			];
			const axisZ = [
				0,
				0,
				0
			];
			transformDirectionToWorld(axisX, [
				1,
				0,
				0
			], constraintTransform);
			transformDirectionToWorld(axisY, [
				0,
				1,
				0
			], constraintTransform);
			transformDirectionToWorld(axisZ, [
				0,
				0,
				1
			], constraintTransform);
			const axes = [
				axisX,
				axisY,
				axisZ
			];
			const axisColors = [
				redColor,
				greenColor,
				blueColor
			];
			for (let i = 0; i < 3; i++) {
				const minLimit = constraint.limitMin[i];
				const maxLimit = constraint.limitMax[i];
				const axis = axes[i];
				const color = axisColors[i];
				if (!(minLimit <= -34e37 && maxLimit >= 34e37) && !(minLimit >= 34e37 && maxLimit <= -34e37) && minLimit < maxLimit) {
					if (minLimit > -34e37) addConstraintMarker(positions, colors, [
						pos1[0] + axis[0] * minLimit,
						pos1[1] + axis[1] * minLimit,
						pos1[2] + axis[2] * minLimit
					], size * .08, color);
					if (maxLimit < 34e37) addConstraintMarker(positions, colors, [
						pos1[0] + axis[0] * maxLimit,
						pos1[1] + axis[1] * maxLimit,
						pos1[2] + axis[2] * maxLimit
					], size * .08, color);
					if (minLimit > -34e37 && maxLimit < 34e37) addConstraintLine(positions, colors, [
						pos1[0] + axis[0] * minLimit,
						pos1[1] + axis[1] * minLimit,
						pos1[2] + axis[2] * minLimit
					], [
						pos1[0] + axis[0] * maxLimit,
						pos1[1] + axis[1] * maxLimit,
						pos1[2] + axis[2] * maxLimit
					], color);
				}
			}
			for (let i = 0; i < 3; i++) {
				const minLimit = constraint.limitMin[3 + i];
				const maxLimit = constraint.limitMax[3 + i];
				const axis = axes[i];
				const color = axisColors[i];
				if (!(minLimit <= -34e37 && maxLimit >= 34e37) && !(minLimit >= 34e37 && maxLimit <= -34e37) && minLimit < maxLimit && Math.abs(maxLimit - minLimit) > .001) {
					const perpAxis = axes[(i + 1) % 3];
					drawPie(positions, colors, pos1, size * .5, axis, perpAxis, minLimit, maxLimit, color);
				}
			}
		}
	}
	const geometry = state.constraints.lineSegments.geometry;
	if (positions.length > 0) {
		geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
		geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
		geometry.computeBoundingSphere();
	} else {
		geometry.deleteAttribute("position");
		geometry.deleteAttribute("color");
	}
}
/** dispose all gpu resources and remove everything from the scene graph. the state should not be used after calling this. */
function dispose(state) {
	state.bodies.batchedMesh.dispose();
	state.bodies.material.dispose();
	state.bodies.velocitiesBatchedMesh.dispose();
	state.bodies.velocitiesMaterial.dispose();
	for (const entry of state.bodies.geometryCache.values()) entry.geometry.dispose();
	for (const instance of state.bodies.edgeWireframes.values()) {
		instance.lineSegments.geometry.dispose();
		for (const geometry of instance.geometries) geometry.dispose();
	}
	const disposedGeometries = /* @__PURE__ */ new Set();
	const disposedMaterials = /* @__PURE__ */ new Set();
	const allInstancedMeshes = [
		state.contacts.spheresMesh,
		state.contacts.cylindersMesh,
		state.contacts.conesMesh,
		state.contactConstraints.spheresMesh,
		state.contactConstraints.cylindersMesh,
		state.contactConstraints.conesMesh
	];
	for (const mesh of allInstancedMeshes) {
		mesh.dispose();
		if (!disposedGeometries.has(mesh.geometry)) {
			mesh.geometry.dispose();
			disposedGeometries.add(mesh.geometry);
		}
		const mat = mesh.material;
		if (mat instanceof THREE.Material && !disposedMaterials.has(mat)) {
			mat.dispose();
			disposedMaterials.add(mat);
		}
	}
	state.broadphase.dbvtLines.geometry.dispose();
	if (state.broadphase.dbvtLines.material instanceof THREE.Material) state.broadphase.dbvtLines.material.dispose();
	state.constraints.lineSegments.geometry.dispose();
	if (state.constraints.lineSegments.material instanceof THREE.Material) state.constraints.lineSegments.material.dispose();
	for (const lineSegments of state.triangleMeshBvh.cache.values()) {
		lineSegments.geometry.dispose();
		if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
	}
	state.triangleMeshBvh.lines.geometry.dispose();
	if (state.triangleMeshBvh.lines.material instanceof THREE.Material) state.triangleMeshBvh.lines.material.dispose();
	for (const lineSegments of state.staticCompoundBvh.cache.values()) {
		lineSegments.geometry.dispose();
		if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
	}
	state.staticCompoundBvh.lines.geometry.dispose();
	if (state.staticCompoundBvh.lines.material instanceof THREE.Material) state.staticCompoundBvh.lines.material.dispose();
	if (state.object3d.parent) state.object3d.parent.remove(state.object3d);
}
function clear(state) {
	clearBatchedMeshBodies(state);
	clearEdgeWireframes(state);
	state.contacts.spheresMesh.count = 0;
	state.contacts.cylindersMesh.count = 0;
	state.contacts.conesMesh.count = 0;
	state.bodies.instanceColors.clear();
	state.broadphase.dbvtLines.visible = false;
	if (state.broadphase.dbvtLines.geometry) {
		state.broadphase.dbvtLines.geometry.dispose();
		state.broadphase.dbvtLines.geometry = new THREE.BufferGeometry();
	}
	for (const [_bodyId, lineSegments] of state.triangleMeshBvh.cache) {
		state.triangleMeshBvh.container.remove(lineSegments);
		lineSegments.geometry.dispose();
		if (lineSegments.material instanceof THREE.Material) lineSegments.material.dispose();
	}
	state.triangleMeshBvh.cache.clear();
	state.triangleMeshBvh.container.visible = false;
}
//#endregion
//#region three/shape-helpers.ts
function createShapeHelper(shape, options) {
	switch (shape.type) {
		case ShapeType.SPHERE: return createSphereHelper(shape, options);
		case ShapeType.BOX: return createBoxHelper(shape, options);
		case ShapeType.CAPSULE: return createCapsuleHelper(shape, options);
		case ShapeType.CYLINDER: return createCylinderHelper(shape, options);
		case ShapeType.CONVEX_HULL: return createConvexHullHelper(shape, options);
		case ShapeType.TRIANGLE_MESH: return createTriangleMeshHelper(shape, options);
		case ShapeType.COMPOUND: return createCompoundHelper(shape, options);
		case ShapeType.STATIC_COMPOUND: return createStaticCompoundHelper(shape, options);
		case ShapeType.TRANSFORMED: return createTransformedHelper(shape, options);
		case ShapeType.SCALED: return createScaledHelper(shape, options);
		case ShapeType.PLANE: return createPlaneHelper(shape, options);
		default: throw new Error(`Unsupported shape type: ${shape.type}`);
	}
}
function createSphereHelper(shape, options) {
	const geometry = new THREE.SphereGeometry(shape.radius, 32, 32);
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false
	});
	return {
		object: new THREE.Mesh(geometry, material),
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
function createBoxHelper(shape, options) {
	const [he0, he1, he2] = shape.halfExtents;
	const geometry = new THREE.BoxGeometry(he0 * 2, he1 * 2, he2 * 2);
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false
	});
	return {
		object: new THREE.Mesh(geometry, material),
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
function createCapsuleHelper(shape, options) {
	const geometry = new THREE.CapsuleGeometry(shape.radius, shape.halfHeightOfCylinder * 2, 16, 32);
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false
	});
	return {
		object: new THREE.Mesh(geometry, material),
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
function createCylinderHelper(shape, options) {
	const geometry = new THREE.CylinderGeometry(shape.radius, shape.radius, shape.halfHeight * 2, 64);
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false
	});
	return {
		object: new THREE.Mesh(geometry, material),
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
function createConvexHullHelper(shape, options) {
	const geometry = new THREE.BufferGeometry();
	const vertices = [];
	const indices = [];
	for (const face of shape.faces) {
		const faceVertices = [];
		for (let i = 0; i < face.numVertices; i++) {
			const pb = shape.vertexIndices[face.firstVertex + i] * 3;
			faceVertices.push([
				shape.pointPositions[pb],
				shape.pointPositions[pb + 1],
				shape.pointPositions[pb + 2]
			]);
		}
		if (faceVertices.length >= 3) {
			const baseIdx = vertices.length / 3;
			for (const v of faceVertices) vertices.push(v[0], v[1], v[2]);
			for (let i = 1; i < faceVertices.length - 1; i++) indices.push(baseIdx, baseIdx + i, baseIdx + i + 1);
		}
	}
	geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(vertices), 3));
	if (indices.length > 0) {
		geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(indices), 1));
		geometry.computeVertexNormals();
	}
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false
	});
	return {
		object: new THREE.Mesh(geometry, material),
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
function createTriangleMeshHelper(shape, options) {
	const geometry = new THREE.BufferGeometry();
	const positions = new Float32Array(shape.data.positions);
	geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
	const indexCount = shape.data.triangleCount * 3;
	const indices = new Uint32Array(indexCount);
	for (let i = 0; i < shape.data.triangleCount; i++) {
		const offset = i * 8;
		indices[i * 3 + 0] = shape.data.triangleBuffer[offset + 0];
		indices[i * 3 + 1] = shape.data.triangleBuffer[offset + 1];
		indices[i * 3 + 2] = shape.data.triangleBuffer[offset + 2];
	}
	geometry.setIndex(new THREE.BufferAttribute(indices, 1));
	geometry.computeVertexNormals();
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false
	});
	return {
		object: new THREE.Mesh(geometry, material),
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
function createCompoundHelper(shape, options) {
	const group = new THREE.Group();
	const childHelpers = [];
	for (const child of shape.children) {
		const childHelper = createShapeHelper(child.shape, options);
		childHelper.object.position.set(child.position[0], child.position[1], child.position[2]);
		childHelper.object.quaternion.set(child.quaternion[0], child.quaternion[1], child.quaternion[2], child.quaternion[3]);
		childHelpers.push(childHelper);
		group.add(childHelper.object);
	}
	return {
		object: group,
		dispose: () => {
			for (const childHelper of childHelpers) childHelper.dispose();
		}
	};
}
function createStaticCompoundHelper(shape, options) {
	const group = new THREE.Group();
	const childHelpers = [];
	for (const child of shape.children) {
		const childHelper = createShapeHelper(child.shape, options);
		childHelper.object.position.set(child.position[0], child.position[1], child.position[2]);
		childHelper.object.quaternion.set(child.quaternion[0], child.quaternion[1], child.quaternion[2], child.quaternion[3]);
		childHelpers.push(childHelper);
		group.add(childHelper.object);
	}
	return {
		object: group,
		dispose: () => {
			for (const childHelper of childHelpers) childHelper.dispose();
		}
	};
}
function createTransformedHelper(shape, options) {
	const group = new THREE.Group();
	const innerHelper = createShapeHelper(shape.shape, options);
	group.position.set(shape.position[0], shape.position[1], shape.position[2]);
	group.quaternion.set(shape.quaternion[0], shape.quaternion[1], shape.quaternion[2], shape.quaternion[3]);
	group.add(innerHelper.object);
	return {
		object: group,
		dispose: () => {
			innerHelper.dispose();
		}
	};
}
function createScaledHelper(shape, options) {
	const group = new THREE.Group();
	const innerHelper = createShapeHelper(shape.shape, options);
	group.scale.set(shape.scale[0], shape.scale[1], shape.scale[2]);
	group.add(innerHelper.object);
	return {
		object: group,
		dispose: () => {
			innerHelper.dispose();
		}
	};
}
function createPlaneHelper(shape, options) {
	const size = shape.halfExtent * 2;
	const geometry = new THREE.PlaneGeometry(size, size);
	const material = options?.material || new THREE.MeshStandardMaterial({
		color: 8947848,
		wireframe: false,
		side: THREE.DoubleSide,
		transparent: true,
		opacity: .5
	});
	const mesh = new THREE.Mesh(geometry, material);
	const normal = shape.plane.normal;
	const up = new THREE.Vector3(0, 1, 0);
	const target = new THREE.Vector3(normal[0], normal[1], normal[2]);
	const quaternion = new THREE.Quaternion();
	quaternion.setFromUnitVectors(up, target);
	mesh.quaternion.copy(quaternion);
	const distance = -shape.plane.constant;
	mesh.position.set(normal[0] * distance, normal[1] * distance, normal[2] * distance);
	return {
		object: mesh,
		dispose: () => {
			geometry.dispose();
			if (!options?.material) material.dispose();
		}
	};
}
//#endregion
export { createShapeHelper, debug_renderer_exports as debugRenderer };

//# sourceMappingURL=three.js.map