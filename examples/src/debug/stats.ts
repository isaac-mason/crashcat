/**
 * Performance statistics monitor
 * Based on stats.js by mrdoob / http://mrdoob.com/
 */

// chrome performance api
declare global {
    interface Performance {
        memory?: {
            usedJSHeapSize: number;
            jsHeapSizeLimit: number;
            totalJSHeapSize: number;
        };
    }
}

type PanelConfig = {
    name: string;
    fg: string;
    bg: string;
};

type Panel = {
    dom: HTMLCanvasElement;
    update: (value: number, maxValue: number) => void;
    resize: () => void;
};

type Stats = {
    REVISION: number;
    dom: HTMLDivElement;
    addPanel: (panel: Panel) => Panel;
    showPanel: (id: number) => void;
    begin: () => void;
    end: () => number;
    update: () => void;
};

const createPanel = (config: PanelConfig): Panel => {
    const { name, fg, bg } = config;

    let min = Infinity;
    let max = 0;
    const round = Math.round;
    const values: number[] = []; // Track values shown in the graph

    const canvas = document.createElement('canvas');
    canvas.style.cssText = 'width:100%;height:48px;display:block';

    const context = canvas.getContext('2d');
    if (!context) {
        throw new Error('Failed to get 2d context');
    }

    // Initialize canvas size
    const resize = (): void => {
        const PR = round(window.devicePixelRatio || 1);
        const rect = canvas.getBoundingClientRect();
        const width = rect.width * PR;
        const height = 48 * PR;

        // Only resize if dimensions actually changed
        if (canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;

            const TEXT_X = 3 * PR;
            const TEXT_Y = 2 * PR;
            const GRAPH_X = 3 * PR;
            const GRAPH_Y = 15 * PR;
            const GRAPH_WIDTH = width - 6 * PR;
            const GRAPH_HEIGHT = 30 * PR;

            context.font = `bold ${9 * PR}px Helvetica,Arial,sans-serif`;
            context.textBaseline = 'top';

            context.fillStyle = bg;
            context.fillRect(0, 0, width, height);

            context.fillStyle = fg;
            context.fillText(name, TEXT_X, TEXT_Y);
            context.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT);

            context.fillStyle = bg;
            context.globalAlpha = 0.9;
            context.fillRect(GRAPH_X, GRAPH_Y, GRAPH_WIDTH, GRAPH_HEIGHT);
        }
    };

    // Set initial size
    setTimeout(resize, 0);

    const update = (value: number, maxValue: number): void => {
        const PR = round(window.devicePixelRatio || 1);
        const width = canvas.width;
        const height = canvas.height;

        // Skip update if canvas hasn't been sized yet
        if (width === 0 || height === 0) {
            return;
        }

        const TEXT_X = 3 * PR;
        const TEXT_Y = 2 * PR;
        const GRAPH_X = 3 * PR;
        const GRAPH_Y = 15 * PR;
        const GRAPH_WIDTH = width - 6 * PR;
        const GRAPH_HEIGHT = 30 * PR;

        // Track this value in our sliding window
        values.push(value);
        
        // Keep values array size matching the graph width (each pixel represents one value)
        const maxValuesCount = Math.floor(GRAPH_WIDTH / PR);
        if (values.length > maxValuesCount) {
            values.shift();
        }

        // Calculate min/max from visible values in the graph
        min = Math.min(...values);
        max = Math.max(...values);
        const avg = values.reduce((sum, v) => sum + v, 0) / values.length;

        context.fillStyle = bg;
        context.globalAlpha = 1;
        context.fillRect(0, 0, width, GRAPH_Y);
        context.fillStyle = fg;
        context.font = `bold ${9 * PR}px Helvetica,Arial,sans-serif`;
        context.textBaseline = 'top';
        context.fillText(
            `${value.toFixed(2)} ${name} (${min.toFixed(2)}-${avg.toFixed(2)}-${max.toFixed(2)})`,
            TEXT_X,
            TEXT_Y
        );

        context.drawImage(
            canvas,
            GRAPH_X + PR,
            GRAPH_Y,
            GRAPH_WIDTH - PR,
            GRAPH_HEIGHT,
            GRAPH_X,
            GRAPH_Y,
            GRAPH_WIDTH - PR,
            GRAPH_HEIGHT
        );

        context.fillRect(GRAPH_X + GRAPH_WIDTH - PR, GRAPH_Y, PR, GRAPH_HEIGHT);

        context.fillStyle = bg;
        context.globalAlpha = 0.9;
        context.fillRect(
            GRAPH_X + GRAPH_WIDTH - PR,
            GRAPH_Y,
            PR,
            round((1 - value / maxValue) * GRAPH_HEIGHT)
        );
    };

    return {
        dom: canvas,
        update,
        resize,
    };
};

export const createStats = (): Stats => {
    const panels: Panel[] = [];

    const container = document.createElement('div');
    container.style.cssText =
        'position:relative;opacity:0.9;width:100%;display:flex;flex-direction:column;gap:2px';

    const showPanel = (id: number): void => {
        for (let i = 0; i < container.children.length; i++) {
            const child = container.children[i] as HTMLElement;
            child.style.display = i === id ? 'block' : 'none';
        }
    };

    const addPanel = (panel: Panel): Panel => {
        container.appendChild(panel.dom);
        panels.push(panel);
        return panel;
    };

    // Add ResizeObserver to handle container resizing
    const resizeObserver = new ResizeObserver(() => {
        for (const panel of panels) {
            panel.resize();
        }
    });
    resizeObserver.observe(container);

    // Remove click-to-cycle behavior - show all panels at once
    // container.addEventListener(
    //     'click',
    //     (event) => {
    //         event.preventDefault();
    //         showPanel(++mode % container.children.length);
    //     },
    //     false
    // );

    let beginTime = (performance || Date).now();
    let prevTime = beginTime;
    let frames = 0;

    const fpsPanel = addPanel(createPanel({ name: 'FPS', fg: '#0ff', bg: '#002' }));
    const msPanel = addPanel(createPanel({ name: 'MS', fg: '#0f0', bg: '#020' }));

    let memPanel: Panel | undefined;
    if (self.performance?.memory) {
        memPanel = addPanel(createPanel({ name: 'MB', fg: '#f08', bg: '#201' }));
    }

    // Show all panels by default (don't hide any)
    // showPanel(1); // Show MS panel by default

    const begin = (): void => {
        beginTime = (performance || Date).now();
    };

    const end = (): number => {
        frames++;

        const time = (performance || Date).now();

        msPanel.update(time - beginTime, 200);

        if (time >= prevTime + 1000) {
            fpsPanel.update((frames * 1000) / (time - prevTime), 100);

            prevTime = time;
            frames = 0;

            if (memPanel && performance.memory) {
                const memory = performance.memory;
                memPanel.update(
                    memory.usedJSHeapSize / 1048576,
                    memory.jsHeapSizeLimit / 1048576
                );
            }
        }

        return time;
    };

    const update = (): void => {
        beginTime = end();
    };

    return {
        REVISION: 16,
        dom: container,
        addPanel,
        showPanel,
        begin,
        end,
        update,
    };
};

export default createStats;
