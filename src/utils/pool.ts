export function pool<T>(create: () => T) {
    const used: T[] = [];
    const free: T[] = [];

    return {
        request: (): T => {
            if (free.length > 0) {
                const item = free.pop()!;
                used.push(item);
                return item;
            } else {
                const item = create();
                used.push(item);
                return item;
            }
        },
        release: (item: T): void => {
            const index = used.indexOf(item);
            if (index !== -1) {
                used.splice(index, 1);
                free.push(item);
            }
        },
        reset: (): void => {
            while (used.length > 0) {
                const item = used.pop()!;
                free.push(item);
            }
        },
    };
}
