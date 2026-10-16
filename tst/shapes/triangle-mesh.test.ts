import { type Vec3, vec3 } from 'math';
import { describe, expect, test } from 'vitest';
import {
    box,
    capsule,
    type CollideShapeCollector,
    type CollideShapeHit,
    type CollideShapeSettings,
    type CastShapeSettings,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    computeMassProperties,
    createAllCastShapeCollector,
    createAnyCollidePointCollector,
    createDefaultCastShapeSettings,
    createDefaultCollidePointSettings,
    createDefaultCollideShapeSettings,
    EMPTY_SUB_SHAPE_ID,
    massProperties,
    type Shape,
    ShapeType,
    sphere,
    type TriangleMeshShape,
    triangleMesh,
} from '../../src';

describe('Triangle mesh AABB caching', () => {
    test('triangle mesh should have correct AABB', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });
        expect(mesh.aabb).toBeDefined();
        // Triangle vertices: (0,0,0), (1,0,0), (0,1,0)
        // AABB should be [0,0,0] to [1,1,0]
        expect(mesh.aabb[0]).toBe(0);
        expect(mesh.aabb[1]).toBe(0);
        expect(mesh.aabb[2]).toBe(0);
        expect(mesh.aabb[3]).toBe(1);
        expect(mesh.aabb[4]).toBe(1);
        expect(mesh.aabb[5]).toBe(0);
    });
});

describe('Triangle mesh creation and mass properties', () => {
    test('should create a triangle mesh', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        expect(mesh.type).toBe(ShapeType.TRIANGLE_MESH);
        expect(mesh.data.triangleCount).toBe(1);
        expect(mesh.bvh).toBeDefined();
    });

    test('should compute triangle mesh as static (zero mass)', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        const props = massProperties.create();
        computeMassProperties(props, mesh);

        // Triangle meshes are static geometry by default
        expect(props.mass).toBe(0);
    });

    test('should handle triangle mesh with multiple triangles', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1],
            indices: [0, 1, 2, 1, 2, 3],
        });

        expect(mesh.data.triangleCount).toBe(2);
        expect(mesh.bvh).toBeDefined();
    });
});

describe('Triangle mesh center of mass', () => {
    test('triangle mesh should have center of mass at origin', () => {
        const mesh = triangleMesh.create({
            positions: [0, 0, 0, 1, 0, 0, 0, 1, 0],
            indices: [0, 1, 2],
        });

        // currently returns zero
        expect(mesh.centerOfMass[0]).toBe(0);
        expect(mesh.centerOfMass[1]).toBe(0);
        expect(mesh.centerOfMass[2]).toBe(0);
    });
});

/** an n by n grid of square cells in the xz plane at y = 0 from the origin, two triangles a cell, facing +y */
function grid(n: number, cell: number): TriangleMeshShape {
    const positions: number[] = [];
    const indices: number[] = [];
    for (let z = 0; z <= n; z++) {
        for (let x = 0; x <= n; x++) {
            positions.push(x * cell, 0, z * cell);
        }
    }
    for (let z = 0; z < n; z++) {
        for (let x = 0; x < n; x++) {
            const i0 = z * (n + 1) + x;
            const i1 = i0 + 1;
            const i2 = i0 + n + 1;
            const i3 = i2 + 1;
            indices.push(i0, i2, i1, i1, i2, i3);
        }
    }
    return triangleMesh.create({ positions, indices });
}

/** unit quad in the xz plane at y = 1, normal +y, spanning [0,1] x [0,1] */
function quad(): TriangleMeshShape {
    return triangleMesh.create({ positions: [0, 1, 0, 1, 1, 0, 1, 1, 1, 0, 1, 1], indices: [0, 2, 1, 0, 3, 2] });
}

function unit(v: Vec3): Vec3 {
    return vec3.normalize(vec3.create(), v);
}

/** the normal a face's winding gives, (v1 - v0) x (v2 - v0) */
function windingNormal(vertices: number[]): Vec3 {
    const e1 = vec3.fromValues(vertices[3] - vertices[0], vertices[4] - vertices[1], vertices[5] - vertices[2]);
    const e2 = vec3.fromValues(vertices[6] - vertices[0], vertices[7] - vertices[1], vertices[8] - vertices[2]);
    return vec3.cross(vec3.create(), e1, e2);
}

type Pose = { position: Vec3; quaternion: [number, number, number, number]; scale: Vec3 };

function pose(position: Vec3, quaternion: [number, number, number, number] = [0, 0, 0, 1], scale: Vec3 = [1, 1, 1]): Pose {
    return { position, quaternion, scale };
}

function cast(settings: CastShapeSettings, a: Shape, poseA: Pose, displacement: Vec3, b: Shape, poseB: Pose) {
    const collector = createAllCastShapeCollector();
    castShapeVsShape(
        collector,
        settings,
        a,
        EMPTY_SUB_SHAPE_ID,
        0,
        poseA.position[0],
        poseA.position[1],
        poseA.position[2],
        poseA.quaternion[0],
        poseA.quaternion[1],
        poseA.quaternion[2],
        poseA.quaternion[3],
        poseA.scale[0],
        poseA.scale[1],
        poseA.scale[2],
        displacement[0],
        displacement[1],
        displacement[2],
        b,
        EMPTY_SUB_SHAPE_ID,
        0,
        poseB.position[0],
        poseB.position[1],
        poseB.position[2],
        poseB.quaternion[0],
        poseB.quaternion[1],
        poseB.quaternion[2],
        poseB.quaternion[3],
        poseB.scale[0],
        poseB.scale[1],
        poseB.scale[2],
    );
    return collector.hits;
}

/** collides a against b, keeping each hit's axis and both faces */
function collide(settings: CollideShapeSettings, a: Shape, poseA: Pose, b: Shape, poseB: Pose) {
    const hits: { penetrationAxis: Vec3; faceB: number[] }[] = [];
    const collector: CollideShapeCollector = {
        bodyIdB: -1,
        earlyOutFraction: Number.MAX_VALUE,
        addHit(hit: CollideShapeHit) {
            hits.push({
                penetrationAxis: vec3.clone(hit.penetrationAxis),
                faceB: hit.faceB.vertices.slice(0, hit.faceB.numVertices * 3),
            });
        },
        addMiss() {},
        shouldEarlyOut: () => false,
    };
    collideShapeVsShape(
        collector,
        settings,
        a,
        EMPTY_SUB_SHAPE_ID,
        0,
        poseA.position[0],
        poseA.position[1],
        poseA.position[2],
        poseA.quaternion[0],
        poseA.quaternion[1],
        poseA.quaternion[2],
        poseA.quaternion[3],
        poseA.scale[0],
        poseA.scale[1],
        poseA.scale[2],
        b,
        EMPTY_SUB_SHAPE_ID,
        0,
        poseB.position[0],
        poseB.position[1],
        poseB.position[2],
        poseB.quaternion[0],
        poseB.quaternion[1],
        poseB.quaternion[2],
        poseB.quaternion[3],
        poseB.scale[0],
        poseB.scale[1],
        poseB.scale[2],
    );
    return hits;
}

describe('Triangle mesh convex cast', () => {
    test('the active edge movement hint is turned into the mesh space', () => {
        const floor = grid(6, 1);
        const crate = box.create({ halfExtents: [0.3, 0.3, 0.3], convexRadius: 0 });
        const slide: Vec3 = [1, 0, 0];
        const settings = createDefaultCastShapeSettings();
        vec3.copy(settings.activeEdgeMovementDirection, slide);

        // sliding along x from 0.1 above, dipping 0.4 over 3, so the leading edge meets the triangles ahead
        // from the side. with the slide as the hint nothing pushes against it
        const upright = cast(settings, crate, pose([1, 0.4, 2.43]), [3, -0.4, 0], floor, pose([0, 0, 0]));
        expect(upright.length).toBeGreaterThan(2);
        for (const hit of upright) expect(vec3.dot(unit(hit.penetrationAxis), slide)).toBeLessThanOrEqual(1e-4);

        // the grid turned half way round about y and moved back over [0, 6]^2: the hint is world space, so it
        // turns into the mesh's space with it and the slide is still unimpeded
        const turned = cast(settings, crate, pose([1, 0.4, 2.43]), [3, -0.4, 0], floor, pose([6, 0, 6], [0, 1, 0, 0]));
        expect(turned.length).toBeGreaterThan(2);
        for (const hit of turned) expect(vec3.dot(unit(hit.penetrationAxis), slide)).toBeLessThanOrEqual(1e-4);
    });

    test('an interior triangle with no movement hint gives the axis into the mesh', () => {
        const floor = grid(4, 1);
        const crate = box.create({ halfExtents: [0.3, 0.3, 0.3], convexRadius: 0 });
        const settings = createDefaultCastShapeSettings();

        // straight down onto the middle of the grid, where every edge is inactive: the axis is the face
        // normal pointing a to b, down into the grid, and the normal points back up out of it
        const hits = cast(settings, crate, pose([2.3, 1, 2.6]), [0, -2, 0], floor, pose([0, 0, 0]));
        expect(hits.length).toBeGreaterThan(0);
        for (const hit of hits) {
            expect(hit.fraction).toBeCloseTo(0.35, 3);
            expect(unit(hit.penetrationAxis)[1]).toBeLessThan(-0.9999);
            expect(hit.normal[1]).toBeGreaterThan(0.9999);
        }
    });

    test("the convex shape's scale is applied once", () => {
        const cube = box.create({ halfExtents: [0.5, 0.5, 0.5], convexRadius: 0 });
        const settings = createDefaultCastShapeSettings();
        settings.collectFaces = true;

        // scaled by 2 (half extent 1) from y = 5 by -5 onto the quad at y = 1: its bottom at 4 closes 3, fraction 0.6
        const hits = cast(settings, cube, pose([0.5, 5, 0.5], [0, 0, 0, 1], [2, 2, 2]), [0, -5, 0], quad(), pose([0, 0, 0]));
        expect(hits.length).toBeGreaterThan(0);
        for (const hit of hits) {
            expect(hit.fraction).toBeCloseTo(0.6, 3);

            // the box's bottom face at the contact: y = 1, x and z in [-0.5, 1.5]
            expect(hit.faceA.numVertices).toBe(4);
            for (let v = 0; v < 4; v++) {
                expect(hit.faceA.vertices[v * 3 + 1]).toBeCloseTo(1, 3);
                expect(Math.abs(hit.faceA.vertices[v * 3] - 0.5)).toBeCloseTo(1, 3);
                expect(Math.abs(hit.faceA.vertices[v * 3 + 2] - 0.5)).toBeCloseTo(1, 3);
            }
        }
    });
});

describe('Triangle mesh cast tolerance', () => {
    test('the default collision tolerance resolves a curved contact to 1e-4', () => {
        const big = triangleMesh.create({ positions: [0, 0, 0, 2, 0, 0, 0, 2, 0], indices: [0, 1, 2] });
        const pill = capsule.create({ halfHeightOfCylinder: 0.01, radius: 0.5 });

        // sideways past the triangle's corner at the origin: the upper cap's centre, (x, -0.29, 0.2), is first
        // within the radius of that vertex when x = -sqrt(0.25 - 0.29^2 - 0.2^2)
        const hits = cast(createDefaultCastShapeSettings(), pill, pose([-2, -0.3, 0.2]), [3, 0, 0], big, pose([0, 0, 0]));
        expect(hits.length).toBeGreaterThan(0);
        const expected = (2 - Math.sqrt(0.25 - 0.29 * 0.29 - 0.2 * 0.2)) / 3;
        expect(Math.abs(Math.min(...hits.map((hit) => hit.fraction)) - expected)).toBeLessThan(1e-4);
    });
});

describe('Triangle mesh sphere cast', () => {
    test('the axis points from the sphere to the triangle', () => {
        // one triangle at y = 1 facing +y
        const floor = triangleMesh.create({ positions: [0, 1, 0, 0, 1, 1, 1, 1, 0], indices: [0, 1, 2] });
        const ball = sphere.create({ radius: 0.5 });
        const settings = createDefaultCastShapeSettings();

        // down from y = 3 by 2 onto it: touches with the centre at 1.5, after 0.75
        const hits = cast(settings, ball, pose([0.25, 3, 0.25]), [0, -2, 0], floor, pose([0, 0, 0]));
        expect(hits.length).toBe(1);
        expect(hits[0].fraction).toBeCloseTo(0.75, 5);
        expect(unit(hits[0].penetrationAxis)[1]).toBeLessThan(-0.9999);
        expect(hits[0].normal[1]).toBeGreaterThan(0.9999);
    });

    test('an inactive edge with no movement hint gives the face normal', () => {
        const floor = grid(4, 1);
        const ball = sphere.create({ radius: 0.4 });
        const settings = createDefaultCastShapeSettings();

        // 0.3 above the grid beside the interior edge at x = 2: the triangles across it are touched only at
        // that edge, so their closest point is off to the side. every edge here is inactive, so each contact
        // takes the face normal, a to b down into the grid
        const hits = cast(settings, ball, pose([1.9, 0.3, 2.5]), [0, -0.1, 0], floor, pose([0, 0, 0]));
        expect(hits.length).toBeGreaterThan(1);
        for (const hit of hits) expect(unit(hit.penetrationAxis)[1]).toBeLessThan(-0.9999);
    });
});

describe('Triangle mesh faces under a mirrored scale', () => {
    // mirrored in x the quad spans x in [-1, 0] and still faces +y; its face's winding must too
    const mirrored = pose([0, 0, 0], [0, 0, 0, 1], [-1, 1, 1]);

    test('cast', () => {
        const settings = createDefaultCastShapeSettings();
        settings.collectFaces = true;
        const cube = box.create({ halfExtents: [0.25, 0.25, 0.25], convexRadius: 0 });
        const hits = cast(settings, cube, pose([-0.5, 3, 0.5]), [0, -3, 0], quad(), mirrored);
        expect(hits.length).toBeGreaterThan(0);
        for (const hit of hits) {
            expect(hit.faceB.numVertices).toBe(3);
            expect(unit(windingNormal(hit.faceB.vertices))[1]).toBeGreaterThan(0.9999);
        }
    });

    test('collide convex and sphere', () => {
        const settings = createDefaultCollideShapeSettings();
        settings.collectFaces = true;
        const cube = box.create({ halfExtents: [0.25, 0.25, 0.25], convexRadius: 0 });
        const ball = sphere.create({ radius: 0.25 });
        for (const shape of [cube, ball]) {
            const hits = collide(settings, shape, pose([-0.5, 1.2, 0.5]), quad(), mirrored);
            expect(hits.length).toBeGreaterThan(0);
            for (const hit of hits) {
                expect(hit.faceB.length).toBe(9);
                expect(unit(windingNormal(hit.faceB))[1]).toBeGreaterThan(0.9999);
            }
        }
    });
});

describe('Triangle mesh collide point', () => {
    test('a point on a mesh with no height is outside', () => {
        const collector = createAnyCollidePointCollector();
        collidePointVsShape(
            collector,
            createDefaultCollidePointSettings(),
            0.25,
            1,
            0.75,
            quad(),
            EMPTY_SUB_SHAPE_ID,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1,
            1,
        );
        expect(collector.hit).toBe(null);
    });
});
