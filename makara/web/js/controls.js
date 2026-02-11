/**
 * Controls.js - Keyboard and Input Controls Manager
 * 
 * This class manages keyboard shortcuts and input handling for the Marine Vessel Simulator.
 * It provides a unified interface for managing user interactions with the 3D scene.
 * 
 * Features:
 * - Keyboard shortcuts for common operations (transform, view controls)
 * - Integration with ThreeScene for viewport manipulations
 * - Event listeners for user input
 */

class Controls {
    /**
     * Initialize the Controls manager
     * @param {ThreeScene} threeScene - Reference to the main ThreeScene instance for callbacks
     */
    constructor(threeScene) {
        this.threeScene = threeScene;
        this.setupKeyboardShortcuts();
    }

    /**
     * Setup keyboard shortcuts for scene manipulation
     * Maps keys to specific actions in the 3D viewer:
     * - g: Toggle grid visibility
     * - w: Toggle wireframe mode
     * - r: Reset camera to default view
     * - Escape: Deselect current object/cancel transformation
     */
    setupKeyboardShortcuts() {
        document.addEventListener('keydown', (event) => {
            switch(event.key) {
                case 'g':
                    this.threeScene.toggleGrid();
                    break;
                case 'w':
                    this.threeScene.toggleWireframe();
                    break;
                case 'r':
                    this.threeScene.resetCamera();
                    break;
                case 'Escape':
                    if (this.threeScene.transformControls.object) {
                        this.threeScene.transformControls.detach();
                    }
                    break;
            }
        });
    }
}

// Export the class to the global scope for use in the application
window.Controls = Controls; 