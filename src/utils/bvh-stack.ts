/**
 * Pooled stack for BVH traversal to reduce GC pressure.
 *
 * Used by both triangle mesh BVH and DBVT (broadphase).
 * Supports distance-based sorting for ray/shape cast queries.
 * Distance field is unused (set to 0) for intersection queries.
 */

export type BvhStack = {
    entries: Array<{ nodeIndex: number; distance: number }>;
    size: number;
};

/**
 * Create a new pooled stack with pre-allocated entries.
 *
 * @param initialCapacity - Initial number of entries to allocate (default: 32)
 * @returns A new stack instance
 */
export function create(initialCapacity = 32): BvhStack {
    const entries = new Array(initialCapacity);
    for (let i = 0; i < initialCapacity; i++) {
        entries[i] = { nodeIndex: 0, distance: 0 };
    }
    return { entries, size: 0 };
}

/**
 * Push a node index and distance onto the stack.
 * Automatically grows the stack by 2x if capacity is exceeded.
 *
 * @param stack - The stack to push onto
 * @param nodeIndex - The BVH node index
 * @param distance - Distance to the node (0 for non-distance queries)
 */
export function push(stack: BvhStack, nodeIndex: number, distance: number): void {
    if (stack.size >= stack.entries.length) {
        const newCapacity = stack.entries.length * 2;
        for (let i = stack.entries.length; i < newCapacity; i++) {
            stack.entries[i] = { nodeIndex: 0, distance: 0 };
        }
    }
    stack.entries[stack.size].nodeIndex = nodeIndex;
    stack.entries[stack.size].distance = distance;
    stack.size++;
}

/**
 * Pop an entry from the stack.
 *
 * @param stack - The stack to pop from
 * @returns The popped entry, or undefined if stack is empty
 */
export function pop(stack: BvhStack): { nodeIndex: number; distance: number } | undefined {
    if (stack.size === 0) return undefined;
    stack.size--;
    return stack.entries[stack.size];
}

/**
 * Reset the stack to empty state without deallocating entries.
 *
 * @param stack - The stack to reset
 */
export function reset(stack: BvhStack): void {
    stack.size = 0;
}
