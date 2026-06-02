/**
 * Pooled stack for BVH traversal to reduce GC pressure.
 *
 * Used by both triangle mesh BVH and DBVT (broadphase).
 * Supports distance-based sorting for ray/shape cast queries.
 * Distance field is unused (set to 0) for intersection queries.
 */
export type BvhStack = {
    entries: Array<{
        nodeIndex: number;
        distance: number;
    }>;
    size: number;
};
/**
 * Create a new pooled stack with pre-allocated entries.
 *
 * @param initialCapacity - Initial number of entries to allocate (default: 32)
 * @returns A new stack instance
 */
export declare function create(initialCapacity?: number): BvhStack;
/**
 * Push a node index and distance onto the stack.
 * Automatically grows the stack by 2x if capacity is exceeded.
 *
 * @param stack - The stack to push onto
 * @param nodeIndex - The BVH node index
 * @param distance - Distance to the node (0 for non-distance queries)
 */
export declare function push(stack: BvhStack, nodeIndex: number, distance: number): void;
/**
 * Pop an entry from the stack.
 *
 * @param stack - The stack to pop from
 * @returns The popped entry, or undefined if stack is empty
 */
export declare function pop(stack: BvhStack): {
    nodeIndex: number;
    distance: number;
} | undefined;
/**
 * Reset the stack to empty state without deallocating entries.
 *
 * @param stack - The stack to reset
 */
export declare function reset(stack: BvhStack): void;
