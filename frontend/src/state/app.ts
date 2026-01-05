// frontend/src/state/app.ts
/**
 * Application-level state management.
 * Contains mode flags and timeline state.
 */

export interface AppState {
    // Mode flags
    isBagMode: boolean;      // true => freeze live growth; use map_full_at
    liveMode: boolean;       // auto-follow end in live mode (ignored in bag mode)
    isPlaying: boolean;      // bag playback
    playRate: number;        // playback speed (1x)

    // Spoof state
    isGeneratingSpoof: boolean;

    // Current bag info
    currentBagName: string | null;
    spoofedBagPath: string | null;

    // Attack mode
    attackModeEnabled: boolean;
    attackConfirmed: boolean;
    attackHalfwayReached: boolean;
    attackHalfwayTime: number | null;
}

export function createInitialAppState(): AppState {
    return {
        isBagMode: false,
        liveMode: true,
        isPlaying: false,
        playRate: 1.0,
        isGeneratingSpoof: false,
        currentBagName: null,
        spoofedBagPath: null,
        attackModeEnabled: false,
        attackConfirmed: false,
        attackHalfwayReached: false,
        attackHalfwayTime: null,
    };
}

// Global mutable state (for easy access from render loops)
export const appState = createInitialAppState();

// State update functions
export function enterBagMode(bagName: string): void {
    appState.isBagMode = true;
    appState.liveMode = false;
    appState.currentBagName = bagName;
    appState.isPlaying = false;
}

export function exitBagMode(): void {
    appState.isBagMode = false;
    appState.liveMode = true;
    appState.currentBagName = null;
    appState.isPlaying = false;
}

export function setPlaying(playing: boolean): void {
    appState.isPlaying = playing;
}

export function setPlayRate(rate: number): void {
    appState.playRate = rate;
}

export function setGeneratingSpoof(generating: boolean): void {
    appState.isGeneratingSpoof = generating;
}
