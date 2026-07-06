export declare const DOF_ALL: number;
export declare const DOF_TRANSLATION_ONLY: number;
export declare const DOF_ROTATION_ONLY: number;
/** encodes degrees of freedom into a bitmask */
export declare function dof(translationX: boolean, translationY: boolean, translationZ: boolean, rotationX: boolean, rotationY: boolean, rotationZ: boolean): number;
