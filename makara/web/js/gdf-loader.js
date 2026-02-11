/**
 * GDFLoader.js - Geometric Data File Loader for Marine Vessel Simulation
 * 
 * This class provides functionality to load and parse GDF (Geometric Data File) files,
 * which are commonly used in marine engineering to define vessel hull geometry.
 * GDF is a format used by hydrodynamic analysis tools like WAMIT and HYDRA.
 * 
 * The loader:
 * - Parses GDF file content containing vertex and panel data
 * - Converts the data into Three.js compatible geometry
 * - Optimizes the geometry by reusing vertices and calculating normals
 * - Creates Three.js BufferGeometry that can be used with meshes
 * 
 * GDF File Format:
 * - Header line with vessel name
 * - ULEN GRAV line (unit length and gravity)
 * - ISX ISY line (symmetry flags)
 * - Number of panels
 * - Vertex coordinates (x,y,z) for each panel (4 vertices per panel)
 */

class GDFLoader {
    /**
     * Initialize the GDF loader
     */
    constructor() {
        this.vertices = [];  // Array of unique vertices
        this.panels = [];    // Array of panel definitions referencing vertex indices
    }

    /**
     * Parse GDF file content and extract geometry data
     * @param {string} content - The text content of the GDF file
     * @returns {GDFLoader} - Returns this loader instance for chaining
     */
    parse(content) {
        try {
            // Clean up the content - remove empty lines and trim whitespace
            const lines = content.split('\n')
                .map(line => line.trim())
                .filter(line => line.length > 0);

            console.log('Processing GDF file with', lines.length, 'lines');
            console.log('First few lines:', lines.slice(0, 5));

            let currentLine = 0;

            // Line 1: Header (vessel name)
            const header = lines[currentLine++];
            console.log('GDF Header:', header);

            // Line 2: ULEN GRAV
            const ulenGrav = lines[currentLine++].split(/\s+/);
            const ulen = parseFloat(ulenGrav[0]);
            const grav = parseFloat(ulenGrav[1]);
            console.log('ULEN:', ulen, 'GRAV:', grav);

            // Line 3: ISX ISY (symmetry flags)
            const symmetry = lines[currentLine++].split(/\s+/);
            const isx = parseInt(symmetry[0]);
            const isy = parseInt(symmetry[1]);
            console.log('Symmetry flags - ISX:', isx, 'ISY:', isy);

            // Line 4: Number of panels
            const nPanels = parseInt(lines[currentLine++]);
            console.log('Number of panels:', nPanels);

            // Read all vertices first
            const vertexMap = new Map(); // Use a map to store unique vertices
            let vertexId = 0;

            // First pass: collect all unique vertices
            let linePointer = currentLine;
            for (let i = 0; i < nPanels; i++) {
                for (let j = 0; j < 4; j++) {
                    if (linePointer >= lines.length) {
                        throw new Error(`Unexpected end of file while reading panel ${i} vertex ${j}`);
                    }

                    const coords = lines[linePointer++].split(/\s+/).filter(v => v.length > 0).map(Number);
                    if (coords.length < 3) {
                        throw new Error(`Invalid vertex format at line ${linePointer}`);
                    }

                    const vertexKey = `${coords[0]},${coords[1]},${coords[2]}`;
                    if (!vertexMap.has(vertexKey)) {
                        vertexMap.set(vertexKey, vertexId++);
                        this.vertices.push(new THREE.Vector3(coords[0], coords[1], coords[2]));
                    }
                }
            }

            // Second pass: create panels with vertex indices
            for (let i = 0; i < nPanels; i++) {
                const panelVertices = [];
                for (let j = 0; j < 4; j++) {
                    const coords = lines[currentLine++].split(/\s+/).filter(v => v.length > 0).map(Number);
                    const vertexKey = `${coords[0]},${coords[1]},${coords[2]}`;
                    panelVertices.push(vertexMap.get(vertexKey));
                }
                this.panels.push(panelVertices);
            }

            console.log('Loaded unique vertices:', this.vertices.length);
            console.log('Loaded panels:', this.panels.length);

            return this;
        } catch (error) {
            console.error('Error parsing GDF file:', error);
            throw error;
        }
    }

    /**
     * Create a Three.js BufferGeometry from the parsed GDF data
     * Triangulates the quad panels and calculates normals
     * @returns {THREE.BufferGeometry} - Three.js geometry ready for rendering
     */
    createGeometry() {
        try {
            console.log('Creating geometry from GDF data');
            if (this.vertices.length === 0 || this.panels.length === 0) {
                throw new Error('No vertices or panels loaded');
            }

            const geometry = new THREE.BufferGeometry();
            const vertices = [];
            const normals = [];
            const indices = [];

            // Convert panels to triangles
            let indexOffset = 0;
            this.panels.forEach((panel, panelIndex) => {
                // Get the four vertices of the panel
                const v0 = this.vertices[panel[0]];
                const v1 = this.vertices[panel[1]];
                const v2 = this.vertices[panel[2]];
                const v3 = this.vertices[panel[3]];

                if (!v0 || !v1 || !v2 || !v3) {
                    console.error(`Invalid vertex reference in panel ${panelIndex}:`, panel);
                    return;
                }

                // Calculate panel normal using the first triangle
                const edge1 = new THREE.Vector3().subVectors(v1, v0);
                const edge2 = new THREE.Vector3().subVectors(v2, v0);
                const normal = new THREE.Vector3().crossVectors(edge1, edge2).normalize();

                // Add vertices for first triangle (v0, v1, v2)
                vertices.push(
                    v0.x, v0.y, v0.z,
                    v1.x, v1.y, v1.z,
                    v2.x, v2.y, v2.z
                );

                // Add vertices for second triangle (v0, v2, v3)
                vertices.push(
                    v0.x, v0.y, v0.z,
                    v2.x, v2.y, v2.z,
                    v3.x, v3.y, v3.z
                );

                // Add normals for all vertices
                for (let i = 0; i < 6; i++) {
                    normals.push(normal.x, normal.y, normal.z);
                }

                // Add indices for both triangles
                indices.push(
                    indexOffset, indexOffset + 1, indexOffset + 2,
                    indexOffset + 3, indexOffset + 4, indexOffset + 5
                );
                indexOffset += 6;
            });

            console.log('Created geometry with:',
                '\nVertices:', vertices.length / 3,
                '\nNormals:', normals.length / 3,
                '\nIndices:', indices.length);

            // Set attributes on the buffer geometry
            geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
            geometry.setAttribute('normal', new THREE.Float32BufferAttribute(normals, 3));
            geometry.setIndex(indices);
            geometry.computeBoundingSphere();

            return geometry;
        } catch (error) {
            console.error('Error creating geometry:', error);
            throw error;
        }
    }

    /**
     * Load a GDF file from a File object and create a Three.js geometry
     * @param {File} file - The GDF file to load
     * @returns {Promise<THREE.BufferGeometry>} - Promise resolving to the created geometry
     */
    load(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            
            reader.onload = (e) => {
                try {
                    console.log('File loaded, starting parse...');
                    console.log('File content preview:', e.target.result.substring(0, 200));
                    
                    this.parse(e.target.result);
                    const geometry = this.createGeometry();
                    
                    if (!geometry) {
                        throw new Error('Failed to create geometry from GDF data');
                    }
                    
                    resolve(geometry);
                } catch (error) {
                    console.error('Error processing GDF file:', error);
                    reject(error);
                }
            };
            
            reader.onerror = (error) => {
                console.error('Error reading file:', error);
                reject(error);
            };
            
            reader.readAsText(file);
        });
    }
}

// Export the class to the global scope for use in the application
window.GDFLoader = GDFLoader; 