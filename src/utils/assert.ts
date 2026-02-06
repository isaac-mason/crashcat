export const assert = (condition: boolean, message: string): void => {
    if (!condition) {
        throw new Error(`Assertion failed: ${message}`);
    }
};

export const assertNever = <T extends never>(value: never, message: string): T => {
    throw new Error(`${message} - value: ${JSON.stringify(value)}`);
};
