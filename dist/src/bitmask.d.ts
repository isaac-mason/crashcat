export declare const NONE_FLAG = 0;
export declare const ALL_FLAG: number;
export declare function createFlags<T extends readonly string[]>(keys: T): {
    [K in T[number] | 'none' | 'all']: number;
};
export declare function addFlag(flag: number, value: number): number;
export declare function removeFlag(flag: number, value: number): number;
export declare function toggleFlag(flag: number, value: number): number;
export declare function hasFlag(flag: number, value: number): boolean;
export declare function doesNotHaveFlag(flag: number, value: number): boolean;
export declare function setFlags(...values: number[]): number;
