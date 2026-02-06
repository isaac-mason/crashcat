import { box3, mat4, type Quat, quat, raycast3, type Vec3, vec3 } from 'mathcat';
import type { RigidBody } from './body/rigid-body';
import { EMPTY_SUB_SHAPE_ID } from './body/sub-shape';
import * as broadphase from './broadphase/broadphase';
import type { CastRayCollector, CastRaySettings, CollidePointCollector, CollidePointSettings } from './collision/narrowphase';
import {
    type CastShapeCollector,
    type CastShapeSettings,
    type CollideShapeCollector,
    type CollideShapeSettings,
    castRayVsShape,
    castShapeVsShape,
    collidePointVsShape,
    collideShapeVsShape,
    collideShapeVsShapeWithInternalEdgeRemoval,
} from './collision/narrowphase';
import type { Filter } from './filter';
import type { Shape } from './shapes/shapes';
import type { World } from './world';

const _castRay_ray = raycast3.create();

const CastRayBodyVisitor = {
    shouldExit: false,
    collector: null! as CastRayCollector,
    settings: null! as CastRaySettings,
    origin: vec3.create(),
    direction: vec3.create(),
    maxDistance: 0,
    visit(body: RigidBody) {
        if (this.shouldExit) return;

        const { collector, settings, origin, direction, maxDistance } = this;

        raycast3.set(_castRay_ray, origin, direction, maxDistance);

        collector.bodyIdB = body.id;

        castRayVsShape(
            collector,
            settings,
            _castRay_ray,
            body.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            body.position[0],
            body.position[1],
            body.position[2],
            body.quaternion[0],
            body.quaternion[1],
            body.quaternion[2],
            body.quaternion[3],
            1,
            1,
            1,
        );

        this.shouldExit = collector.shouldEarlyOut();
    },
    set(collector: CastRayCollector, settings: CastRaySettings, origin: Vec3, direction: Vec3, maxDistance: number) {
        this.collector = collector;
        this.settings = settings;
        vec3.copy(this.origin, origin);
        vec3.copy(this.direction, direction);
        this.maxDistance = maxDistance;
    },
    reset() {
        this.shouldExit = false;
        this.collector = null!;
        this.settings = null!;
    },
};

export function castRay(
    world: World,
    collector: CastRayCollector,
    settings: CastRaySettings,
    origin: Vec3,
    direction: Vec3,
    length: number,
    filter: Filter,
): void {
    CastRayBodyVisitor.set(collector, settings, origin, direction, length);

    broadphase.castRay(world, origin, direction, length, filter, CastRayBodyVisitor);

    CastRayBodyVisitor.reset();
}

const _castShape_aabb = box3.create();
const _castShape_mat4 = mat4.create();

const CastShapeBodyVisitor = {
    shouldExit: false,
    collector: null! as CastShapeCollector,
    settings: null! as CastShapeSettings,
    shape: null! as Shape,
    position: vec3.create(),
    quaternion: quat.create(),
    scale: vec3.create(),
    displacement: vec3.create(),
    visit(body: RigidBody) {
        if (this.shouldExit) return;

        const { collector, settings, shape, position, quaternion, scale, displacement } = this;

        collector.bodyIdB = body.id;

        castShapeVsShape(
            collector,
            settings,
            shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            position[0],
            position[1],
            position[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
            scale[0],
            scale[1],
            scale[2],
            displacement[0],
            displacement[1],
            displacement[2],
            body.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            body.position[0],
            body.position[1],
            body.position[2],
            body.quaternion[0],
            body.quaternion[1],
            body.quaternion[2],
            body.quaternion[3],
            1,
            1,
            1,
        );

        this.shouldExit = collector.shouldEarlyOut();
    },
    set(
        collector: CastShapeCollector,
        settings: CastShapeSettings,
        shape: Shape,
        position: Vec3,
        quaternion: Quat,
        scale: Vec3,
        displacement: Vec3,
    ) {
        this.collector = collector;
        this.settings = settings;
        this.shape = shape;
        vec3.copy(this.position, position);
        quat.copy(this.quaternion, quaternion);
        vec3.copy(this.scale, scale);
        vec3.copy(this.displacement, displacement);
    },
    reset() {
        this.shouldExit = false;
        this.collector = null!;
        this.settings = null!;
        this.shape = null!;
    },
};

export function castShape(
    world: World,
    collector: CastShapeCollector,
    settings: CastShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    displacement: Vec3,
    filter: Filter,
): void {
    const aabb = box3.copy(_castShape_aabb, shape.aabb);
    const mat = mat4.fromRotationTranslationScale(_castShape_mat4, quaternion, position, scale);
    box3.transformMat4(aabb, aabb, mat);

    CastShapeBodyVisitor.set(collector, settings, shape, position, quaternion, scale, displacement);

    broadphase.castAABB(world, aabb, displacement, filter, CastShapeBodyVisitor);

    CastShapeBodyVisitor.reset();
}

const _collideShape_aabb = box3.create();
const _collideShape_mat4 = mat4.create();

const CollideShapeBodyVisitor = {
    shouldExit: false,
    collector: null! as CollideShapeCollector,
    settings: null! as CollideShapeSettings,
    shape: null! as Shape,
    position: vec3.create(),
    quaternion: quat.create(),
    scale: vec3.create(),
    visit(body: RigidBody) {
        if (this.shouldExit) return;

        const { collector, settings, shape, position, quaternion, scale } = this;

        collector.bodyIdB = body.id;

        collideShapeVsShape(
            collector,
            settings,
            shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            position[0],
            position[1],
            position[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
            scale[0],
            scale[1],
            scale[2],
            body.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            body.position[0],
            body.position[1],
            body.position[2],
            body.quaternion[0],
            body.quaternion[1],
            body.quaternion[2],
            body.quaternion[3],
            1,
            1,
            1,
        );

        this.shouldExit = collector.shouldEarlyOut();
    },
    set(
        collector: CollideShapeCollector,
        settings: CollideShapeSettings,
        shape: Shape,
        position: Vec3,
        quaternion: Quat,
        scale: Vec3,
    ) {
        this.collector = collector;
        this.settings = settings;
        this.shape = shape;
        vec3.copy(this.position, position);
        quat.copy(this.quaternion, quaternion);
        vec3.copy(this.scale, scale);
    },
    reset() {
        this.shouldExit = false;
        this.collector = null!;
        this.settings = null!;
        this.shape = null!;
    },
};

export function collideShape(
    world: World,
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    filter: Filter,
): void {
    const mat = mat4.fromRotationTranslationScale(_collideShape_mat4, quaternion, position, scale);
    const aabb = box3.transformMat4(_collideShape_aabb, shape.aabb, mat);

    // expand aabb by max separation distance
    box3.expandByMargin(aabb, aabb, settings.maxSeparationDistance);

    CollideShapeBodyVisitor.set(collector, settings, shape, position, quaternion, scale);

    broadphase.intersectAABB(world, aabb, filter, CollideShapeBodyVisitor);

    CollideShapeBodyVisitor.reset();
}

const CollideShapeWithInternalEdgeRemovalBodyVisitor = {
    shouldExit: false,
    collector: null! as CollideShapeCollector,
    settings: null! as CollideShapeSettings,
    shape: null! as Shape,
    position: vec3.create(),
    quaternion: quat.create(),
    scale: vec3.create(),
    visit(body: RigidBody) {
        if (this.shouldExit) return;

        const { collector, settings, shape, position, quaternion, scale } = this;

        collector.bodyIdB = body.id;

        collideShapeVsShapeWithInternalEdgeRemoval(
            collector,
            settings,
            shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            position[0],
            position[1],
            position[2],
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3],
            scale[0],
            scale[1],
            scale[2],
            body.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            body.position[0],
            body.position[1],
            body.position[2],
            body.quaternion[0],
            body.quaternion[1],
            body.quaternion[2],
            body.quaternion[3],
            1,
            1,
            1,
        );

        this.shouldExit = collector.shouldEarlyOut();
    },
    set(
        collector: CollideShapeCollector,
        settings: CollideShapeSettings,
        shape: Shape,
        position: Vec3,
        quaternion: Quat,
        scale: Vec3,
    ) {
        this.collector = collector;
        this.settings = settings;
        this.shape = shape;
        vec3.copy(this.position, position);
        quat.copy(this.quaternion, quaternion);
        vec3.copy(this.scale, scale);
    },
    reset() {
        this.shouldExit = false;
        this.collector = null!;
        this.settings = null!;
        this.shape = null!;
    },
};

/**
 * collide a shape against the world with enhanced internal edge removal.
 *
 * this eliminates "ghost collisions" when sliding across internal edges of triangle meshes
 * or compound shapes, preventing stuttering and catches on smooth surfaces.
 *
 * automatically forces collectFaces=true and collideOnlyWithActiveEdges=false.
 *
 * use this for character controllers, rolling objects, and anything that needs smooth
 * sliding across multi-primitive surfaces.
 */
export function collideShapeWithInternalEdgeRemoval(
    world: World,
    collector: CollideShapeCollector,
    settings: CollideShapeSettings,
    shape: Shape,
    position: Vec3,
    quaternion: Quat,
    scale: Vec3,
    filter: Filter,
): void {
    const mat = mat4.fromRotationTranslationScale(_collideShape_mat4, quaternion, position, scale);
    const aabb = box3.transformMat4(_collideShape_aabb, shape.aabb, mat);

    // expand aabb by max separation distance
    box3.expandByMargin(aabb, aabb, settings.maxSeparationDistance);

    CollideShapeWithInternalEdgeRemovalBodyVisitor.set(collector, settings, shape, position, quaternion, scale);

    broadphase.intersectAABB(world, aabb, filter, CollideShapeWithInternalEdgeRemovalBodyVisitor);

    CollideShapeWithInternalEdgeRemovalBodyVisitor.reset();
}

const CollidePointBodyVisitor = {
    shouldExit: false,
    collector: null! as CollidePointCollector,
    settings: null! as CollidePointSettings,
    point: vec3.create(),
    visit(body: RigidBody) {
        if (this.shouldExit) return;

        const { collector, settings, point } = this;

        collector.bodyIdB = body.id;

        collidePointVsShape(
            collector,
            settings,
            point[0],
            point[1],
            point[2],
            body.shape,
            EMPTY_SUB_SHAPE_ID,
            0,
            body.position[0],
            body.position[1],
            body.position[2],
            body.quaternion[0],
            body.quaternion[1],
            body.quaternion[2],
            body.quaternion[3],
            1,
            1,
            1,
        );

        this.shouldExit = collector.shouldEarlyOut();
    },
    set(collector: CollidePointCollector, settings: CollidePointSettings, point: Vec3) {
        this.collector = collector;
        this.settings = settings;
        vec3.copy(this.point, point);
    },
    reset() {
        this.shouldExit = false;
        this.collector = null!;
        this.settings = null!;
    },
};

export function collidePoint(
    world: World,
    collector: CollidePointCollector,
    settings: CollidePointSettings,
    point: Vec3,
    filter: Filter,
): void {
    CollidePointBodyVisitor.set(collector, settings, point);

    broadphase.intersectPoint(world, point, filter, CollidePointBodyVisitor);

    CollidePointBodyVisitor.reset();
}
