import * as THREE from 'three';
import { type Shape } from 'crashcat';
export type ShapeHelperOptions = {
    material?: THREE.Material;
};
export type ShapeHelper = {
    object: THREE.Object3D;
    dispose: () => void;
};
export declare function createShapeHelper(shape: Shape, options?: ShapeHelperOptions): ShapeHelper;
