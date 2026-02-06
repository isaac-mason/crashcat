export const NONE_FLAG = 0;
export const ALL_FLAG = ~(~0 << 31);

export function createFlags<T extends readonly string[]>(
    keys: T
): {
        [K in T[number] | "none" | "all"]: number;
    } {
    const result: any = {};

    result.none = 0;
    result.all = ~0;

    let index = 0;
    for (const key of keys) {
        result[key] = 1 << index++;
    }

    return result;
}

export function addFlag(flag: number, value: number): number {
    return flag | value;
}

export function removeFlag(flag: number, value: number): number {
    return flag & ~value;
}

export function toggleFlag(flag: number, value: number): number {
    return flag ^ value;
}

export function hasFlag(flag: number, value: number): boolean {
    return (flag & value) !== 0;
}

export function doesNotHaveFlag(flag: number, value: number): boolean {
    return (flag & value) === 0;
}

export function setFlags(...values: number[]): number {
    return values.reduce((acc, value) => acc | value, 0);
}
