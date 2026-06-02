export declare function pool<T>(create: () => T): {
    request: () => T;
    release: (item: T) => void;
    reset: () => void;
};
