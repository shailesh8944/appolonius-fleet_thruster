/**
 * ThreeScene.js - 3D Visualization Engine for Marine Vessel Simulator
 * 
 * This class provides the 3D visualization capabilities for the Marine Vessel Simulator.
 * It handles all Three.js setup and rendering, including:
 * - Scene, camera, and renderer initialization
 * - Lighting and environment setup
 * - Object creation and transformation
 * - Interactive controls (orbit controls, transform controls)
 * - Object selection and property editing
 * - Visual feedback and highlighting
 * - Component visualization (vessels, thrusters, control surfaces, sensors)
 * - Performance optimization for real-time 3D rendering
 * 
 * The class integrates with the VesselModel to synchronize the 3D representation
 * with the underlying data model, allowing bidirectional updates between the
 * visual components and their configuration parameters.
 */

class ThreeScene {
    constructor() {
        // Initialize properties
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.selectedObject = null;
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        this.needsRender = true;
        this.modelMeshes = new Map();
        
        // Track rendering during drag operations
        this._dragRenderInterval = null;
        
        // Initialize component maps
        this.vessel = null;
        this.controlSurfaces = new Map();
        this.thrusters = new Map();
        this.sensors = new Map();
        this.transformMode = 'translate';
        this.transformControls = null;
        
        // Ensure DOM is loaded before initializing
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => {
                this.init();
            });
        } else {
            try {
                this.init();
            } catch (error) {
                console.error('Error initializing ThreeScene:', error);
                // Attempt delayed initialization as a fallback
                setTimeout(() => {
                    try {
                        this.init();
                    } catch (retryError) {
                        console.error('Failed to initialize ThreeScene after retry:', retryError);
                    }
                }, 500);
            }
        }
    }

    init() {
        // Get container
        this.container = document.getElementById('viewer3D');
        
        // Initialize scene
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x1e293b);
        
        // Create renderer with hardware acceleration
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            powerPreference: 'high-performance',
            precision: 'highp'
        });
        this.renderer.setPixelRatio(window.devicePixelRatio); // Adapt to device
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        this.renderer.outputEncoding = THREE.sRGBEncoding;
        this.renderer.physicallyCorrectLights = true;
        this.container.appendChild(this.renderer.domElement);
        
        // Performance optimization - set fixed size buffer based on container size
        const containerRect = this.container.getBoundingClientRect();
        this.renderer.setSize(containerRect.width, containerRect.height);
        
        // Initialize camera
        this.camera = new THREE.PerspectiveCamera(45, containerRect.width / containerRect.height, 0.1, 1000);
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
        
        // Initialize orbit controls
        this.orbitControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.orbitControls.enableDamping = true;
        this.orbitControls.dampingFactor = 0.25;
        this.orbitControls.screenSpacePanning = false;
        this.orbitControls.maxPolarAngle = Math.PI / 1.5;
        this.orbitControls.update();
        
        // Initialize transform controls
        this.initTransformControls();
        
        // Setup raycaster for object selection
        this.raycaster = new THREE.Raycaster();
        this.mouse = new THREE.Vector2();
        
        // Add basic scene elements
        this.addGrid();
        this.addLights();
        
        // Add global axes helper
        this.addGlobalAxesHelper();
        
        // Initialize event listeners
        this.initializeEventListeners();
        
        // Add performance monitoring
        if (window.location.search.includes('stats=1')) {
            this.addPerformanceMonitor();
        }
        
        // Flag for controlling render frequency
        this.needsRender = true;
        
        // Start animation loop
        this.animate();
    }

    initTransformControls() {
        // Create transform controls
        this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement);
        this.scene.add(this.transformControls);
        
        // Set space to 'local' to rotate around object's center
        this.transformControls.setSpace('local');
        
        // Ensure rotation is around the center pivot point
        this.transformControls.setMode('translate'); // Start with translate mode
        
        // Event listeners for transform controls
        this.transformControls.addEventListener('dragging-changed', (event) => {
            // Disable orbit controls while transforming
            this.orbitControls.enabled = !event.value;
            
            // Force continuous rendering during dragging
            if (event.value) {
                // When dragging starts, set up an interval to force rendering
                this._dragRenderInterval = setInterval(() => {
                    this.forceRender();
                }, 16); // Approximately 60fps
            } else {
                // When dragging stops, clear the interval
                if (this._dragRenderInterval) {
                    clearInterval(this._dragRenderInterval);
                    this._dragRenderInterval = null;
                }
            }
        });
        
        // Add a change event listener for continuous updates during transformation
        this.transformControls.addEventListener('change', () => {
            this.forceRender();
        });
        
        this.transformControls.addEventListener('mouseDown', () => {
            // When starting transformation, store the object's initial position
            // This is important for rotation around local center
            if (this.transformControls.object) {
                const object = this.transformControls.object;
                // Store the object's world position for reference
                object.userData.initialWorldPosition = new THREE.Vector3();
                object.getWorldPosition(object.userData.initialWorldPosition);
            }
        });
        
        this.transformControls.addEventListener('objectChange', () => {
            // Object is being transformed
            if (this.transformControls.object) {
                this.onObjectTransformed(this.transformControls.object);
                // Force render on each object change for smooth updates
                this.forceRender();
            }
        });
        
        this.transformControls.addEventListener('mouseUp', () => {
            // Transform complete, update UI
            if (this.transformControls.object) {
                // Clean up temporary data
                delete this.transformControls.object.userData.initialWorldPosition;
                this.onObjectTransformed(this.transformControls.object);
            }
        });
    }

    addGrid() {
        // Remove existing grid if any
        if (this.grid) {
            this.scene.remove(this.grid);
        }
        
        // Create a grid helper
        const size = 20;
        const divisions = 20;
        this.grid = new THREE.GridHelper(size, divisions);
        this.grid.material.opacity = 0.2;
        this.grid.material.transparent = true;
        this.scene.add(this.grid);
    }

    addLights() {
        // Add ambient light
        const ambientLight = new THREE.AmbientLight(0x404040, 0.5);
        this.scene.add(ambientLight);

        // Add directional lights from different angles
        const createDirLight = (x, y, z) => {
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.5);
            dirLight.position.set(x, y, z);
            return dirLight;
        };

        this.scene.add(createDirLight(5, 5, 5));
        this.scene.add(createDirLight(-5, 5, -5));
        this.scene.add(createDirLight(0, -5, 0));
        
        // Add a hemisphere light
        const hemiLight = new THREE.HemisphereLight(0xffffff, 0x404040, 0.6);
        this.scene.add(hemiLight);
    }

    async loadModelGeometry(file) {
        try {
            console.log('Starting model file load...');
            
            // Remove old vessel if it exists
            if (this.vessel) {
                this.scene.remove(this.vessel);
            }

            // Create main vessel group
            this.vessel = new THREE.Group();
            this.vessel.userData.type = 'vessel';
            this.vessel.userData.isTransformable = true;

            // Load STL file
            const loader = new THREE.STLLoader();
            const geometry = await new Promise((resolve, reject) => {
                loader.load(
                    URL.createObjectURL(file),
                    resolve,
                    undefined,
                    reject
                );
            });

            console.log('STL loaded, vertex count:', geometry.attributes.position.count);

            // Compute vertex normals for proper lighting
            geometry.computeVertexNormals();

            // Calculate bounding box and center
            geometry.computeBoundingBox();
            const bbox = geometry.boundingBox;
            const size = new THREE.Vector3();
            bbox.getSize(size);
            const center = new THREE.Vector3();
            bbox.getCenter(center);

            // Calculate scale to normalize size
            const maxDim = Math.max(size.x, size.y, size.z);
            const scale = 2 / maxDim;

            // Create a container group for the STL and its bounding box
            const modelGroup = new THREE.Group();
            modelGroup.userData.type = 'modelGroup';
            modelGroup.userData.isTransformable = true;

            // Create STL mesh with a bright, solid material
            const stlMaterial = new THREE.MeshStandardMaterial({
                color: 0x3498db,
                metalness: 0.3,
                roughness: 0.4,
                side: THREE.DoubleSide
            });

            const stlMesh = new THREE.Mesh(geometry, stlMaterial);
            stlMesh.castShadow = true;
            stlMesh.receiveShadow = true;
            
            // Center the geometry at origin
            geometry.center();
            
            // Create bounding box mesh
            const boxGeometry = new THREE.BoxGeometry(
                size.x,
                size.y,
                size.z
            );
            const boxMaterial = new THREE.LineBasicMaterial({
                color: 0x00ff00,
                transparent: true,
                opacity: 0.5
            });
            const boxMesh = new THREE.LineSegments(
                new THREE.EdgesGeometry(boxGeometry),
                boxMaterial
            );

            // Add both meshes to model group
            modelGroup.add(stlMesh);
            modelGroup.add(boxMesh);

            // Scale the entire model group
            modelGroup.scale.multiplyScalar(scale);

            // Create vessel coordinate axes (marine convention)
            const axesLength = maxDim * scale * 0.5;
            const axesGroup = new THREE.Group();
            axesGroup.userData.isAxes = true;

            // X-axis (surge - forward) - Red
            const xAxis = new THREE.ArrowHelper(
                new THREE.Vector3(1, 0, 0),
                new THREE.Vector3(0, 0, 0),
                axesLength,
                0xff0000,
                axesLength * 0.2,
                axesLength * 0.1
            );
            axesGroup.add(xAxis);

            // Y-axis (sway - starboard) - Green
            const yAxis = new THREE.ArrowHelper(
                new THREE.Vector3(0, 1, 0),
                new THREE.Vector3(0, 0, 0),
                axesLength,
                0x00ff00,
                axesLength * 0.2,
                axesLength * 0.1
            );
            axesGroup.add(yAxis);

            // Z-axis (heave - down) - Blue
            const zAxis = new THREE.ArrowHelper(
                new THREE.Vector3(0, 0, 1),
                new THREE.Vector3(0, 0, 0),
                axesLength,
                0x0000ff,
                axesLength * 0.2,
                axesLength * 0.1
            );
            axesGroup.add(zAxis);

            // Add model group and axes to vessel group
            this.vessel.add(modelGroup);
            this.vessel.add(axesGroup);

            // Store references
            this.vessel.userData.stlMesh = stlMesh;
            this.vessel.userData.boxMesh = boxMesh;
            this.vessel.userData.axesGroup = axesGroup;
            this.vessel.userData.modelGroup = modelGroup;
            this.vessel.userData.isSTLModel = true;

            // Add vessel to scene
            this.scene.add(this.vessel);

            // Make the vessel selectable initially
            this.selectObject(this.vessel);

            // Reset camera and reattach components
            this.resetCamera();
            this.reattachComponents();

            console.log('Model loaded successfully:', {
                vertices: geometry.attributes.position.count,
                size: size.toArray(),
                scale: scale,
                center: center.toArray()
            });

            // Add more lighting to better see the model
            if (!this.spotLight) {
                this.spotLight = new THREE.SpotLight(0xffffff, 1);
                this.spotLight.position.set(5, 5, 5);
                this.spotLight.angle = Math.PI / 4;
                this.spotLight.penumbra = 0.1;
                this.spotLight.decay = 2;
                this.spotLight.distance = 200;
                this.scene.add(this.spotLight);
            }

            // Set initial visibility states
            const stlToggle = document.getElementById('toggleSTLVisibility');
            const boxToggle = document.getElementById('toggleBoxVisibility');
            if (stlToggle) stlToggle.checked = true;
            if (boxToggle) boxToggle.checked = true;

            this.render();
            this.updateSceneHierarchy();
            return {
                success: true,
                dimensions: {
                    length: size.x,
                    breadth: size.y,
                    depth: size.z
                }
            };
        } catch (error) {
            console.error('Error loading STL file:', error);
            return {
                success: false,
                error: error.message
            };
        }
    }

    createVessel(length, breadth, depth) {
        try {
            console.log(`Creating vessel with dimensions: ${length} x ${breadth} x ${depth}`);
            
            // Don't create default vessel if we have an STL model loaded
            if (this.vessel && this.vessel.userData.isSTLModel) {
                console.log('Skipping default vessel creation as STL model is loaded');
                return;
            }

            // Remove existing vessel if any
            if (this.vessel) {
                this.scene.remove(this.vessel);
            }

            // Create vessel container group
            this.vessel = new THREE.Group();
            this.vessel.userData.type = 'vessel';
            this.vessel.userData.isSTLModel = false;
            this.vessel.name = 'Vessel';

            // Create vessel box for visualization
            const geometry = new THREE.BoxGeometry(length, depth, breadth);
            const material = new THREE.MeshPhongMaterial({
                color: 0x3c78d8,
                transparent: true,
                opacity: 0.7,
                side: THREE.DoubleSide
            });
            const vesselMesh = new THREE.Mesh(geometry, material);
            vesselMesh.userData.type = 'vesselBody';
            vesselMesh.name = 'Hull';

            // Add wireframe
            const edgeGeometry = new THREE.EdgesGeometry(geometry);
            const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x000000 });
            const wireframe = new THREE.LineSegments(edgeGeometry, edgeMaterial);
            wireframe.userData.isWireframe = true;
            vesselMesh.add(wireframe);

            // Store original size
            this.vessel.userData.dimensions = { length, breadth, depth };

            // Add mesh to vessel container
            this.vessel.add(vesselMesh);

            // Add vessel to scene
            this.scene.add(this.vessel);

            // Add local coordinate axes
            const axes = this.createLocalAxes(Math.max(length, breadth, depth) * 0.5);
            this.vessel.add(axes);

            // Reattach components if any
            this.reattachComponents();

            // Update scene
            this.render();
            this.updateSceneHierarchy();
            
            return this.vessel;
        } catch (error) {
            console.error('Error creating vessel:', error);
            return null;
        }
    }

    updateVesselDimensions(length, breadth, depth) {
        if (!this.vessel || this.vessel.userData.isSTLModel) {
            return;
        }

        // Update stored dimensions
        this.vessel.userData.dimensions = {
            length: length || 1,
            breadth: breadth || 1,
            depth: depth || 1
        };

        // Update vessel mesh geometry
        const vesselMesh = this.vessel.userData.vesselMesh;
        if (vesselMesh) {
            const newGeometry = new THREE.BoxGeometry(length || 1, depth || 1, breadth || 1);
            vesselMesh.geometry.dispose();
            vesselMesh.geometry = newGeometry;
        }

        // Update axes size
        const axesGroup = this.vessel.userData.axesGroup;
        if (axesGroup) {
            const axesLength = Math.max(length, breadth, depth) * 0.5;
            axesGroup.children.forEach((axis, index) => {
                if (axis instanceof THREE.ArrowHelper) {
                    axis.setLength(axesLength, axesLength * 0.2, axesLength * 0.1);
                }
            });
        }

        // Reposition components based on new dimensions
        this.reattachComponents();

        // Update scene
        this.render();
        this.updateSceneHierarchy();
    }

    reattachComponents() {
        try {
            // Only proceed if vessel exists
            if (!this.vessel) {
                console.warn('Cannot reattach components: vessel does not exist');
                return;
            }
            
            // Reattach control surfaces
            if (this.controlSurfaces) {
                this.controlSurfaces.forEach((surface, id) => {
                    if (surface.parent) {
                        surface.parent.remove(surface);
                    }
                    this.vessel.add(surface);
                });
            }

            // Reattach thrusters
            if (this.thrusters) {
                this.thrusters.forEach((thruster, id) => {
                    if (thruster.parent) {
                        thruster.parent.remove(thruster);
                    }
                    this.vessel.add(thruster);
                });
            }

            // Reattach sensors
            if (this.sensors) {
                this.sensors.forEach((sensor, id) => {
                    if (sensor.parent) {
                        sensor.parent.remove(sensor);
                    }
                    this.vessel.add(sensor);
                });
            }
            
            // Mark for rendering
            this.render();
        } catch (error) {
            console.error('Error in reattachComponents:', error);
        }
    }

    createLocalAxes(size = 0.3) {
        const axes = new THREE.Group();
        axes.userData.isAxes = true;
        
        // Create a container for all axes elements
        const axesContainer = new THREE.Group();
        
        // X axis (red)
        const xGeometry = new THREE.CylinderGeometry(0.01, 0.01, size);
        const xMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const xAxis = new THREE.Mesh(xGeometry, xMaterial);
        xAxis.rotation.z = -Math.PI / 2;
        xAxis.position.x = size / 2;
        axesContainer.add(xAxis);

        // Y axis (green)
        const yGeometry = new THREE.CylinderGeometry(0.01, 0.01, size);
        const yMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        const yAxis = new THREE.Mesh(yGeometry, yMaterial);
        yAxis.position.y = size / 2;
        axesContainer.add(yAxis);

        // Z axis (blue)
        const zGeometry = new THREE.CylinderGeometry(0.01, 0.01, size);
        const zMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff });
        const zAxis = new THREE.Mesh(zGeometry, zMaterial);
        zAxis.rotation.x = Math.PI / 2;
        zAxis.position.z = size / 2;
        axesContainer.add(zAxis);

        // Add labels
        const createLabel = (text, position, color) => {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 64;
            canvas.height = 32;
            context.fillStyle = color;
            context.font = 'bold 16px Arial';
            context.fillText(text, 0, 24);
            
            const texture = new THREE.CanvasTexture(canvas);
            const spriteMaterial = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(spriteMaterial);
            sprite.position.copy(position);
            sprite.scale.set(0.5, 0.25, 1);
            return sprite;
        };

        axesContainer.add(createLabel('X', new THREE.Vector3(size * 1.2, 0, 0), '#ff0000'));
        axesContainer.add(createLabel('Y', new THREE.Vector3(0, size * 1.2, 0), '#00ff00'));
        axesContainer.add(createLabel('Z', new THREE.Vector3(0, 0, size * 1.2), '#0000ff'));

        axes.add(axesContainer);
        return axes;
    }

    addComponentAxes(component) {
        // Calculate component dimensions to determine appropriate axes size
        let componentSize = 0.3; // Default fallback size
        
        // Get component dimensions if available
        if (component.geometry) {
            // Use bounding box to determine size
            component.geometry.computeBoundingBox();
            const boundingBox = component.geometry.boundingBox;
            const width = boundingBox.max.x - boundingBox.min.x;
            const height = boundingBox.max.y - boundingBox.min.y;
            const depth = boundingBox.max.z - boundingBox.min.z;
            
            // Use the largest dimension to calculate a proportional axes size
            const maxDimension = Math.max(width, height, depth);
            componentSize = maxDimension * 0.5; // 50% of the largest dimension
            
            // Clamp the size to reasonable bounds
            componentSize = Math.max(0.05, Math.min(componentSize, 0.5));
        }
        
        const axes = this.createLocalAxes(componentSize);
        axes.userData.isAxes = true;
        axes.userData.isComponentAxes = true;
        axes.userData.parentId = component.userData.id;
        axes.userData.componentId = component.userData.componentId;
        axes.userData.componentType = component.userData.componentType;
        component.add(axes);  // Add axes directly to the component
        return axes;
    }

    addControlSurface(id, type, position, orientation, area = 0.1) {
        // Create a simple representation of a control surface
        const width = Math.sqrt(area);
        const height = width;
        const depth = width * 0.1;  // Thin control surface
        
        const geometry = new THREE.BoxGeometry(width, height, depth);
        // Center the geometry around origin
        geometry.translate(0, 0, 0);
        
        const material = new THREE.MeshStandardMaterial({ 
            color: 0x00aaff,
            transparent: true,
            opacity: 0.7,
            metalness: 0.2,
            roughness: 0.8
        });
        
        const controlSurface = new THREE.Mesh(geometry, material);
        controlSurface.name = `Control Surface ${id} (${type})`;
        controlSurface.userData.componentType = 'controlSurface';
        controlSurface.userData.componentId = id;
        
        // Position and rotate
        controlSurface.position.set(position[0], position[1], position[2]);
        controlSurface.rotation.set(orientation[0], orientation[1], orientation[2]);
        
        // Add to scene
        this.scene.add(controlSurface);
        
        // Add axes at component center
        this.addComponentAxes(controlSurface);
        
        // Update scene hierarchy
        this.updateSceneHierarchy();
        
        this.render();
        
        return controlSurface;
    }

    updateControlSurfaceDimensions(id, width, height) {
        const controlSurface = this.getObjectById(id);
        if (!controlSurface) return;
        
        // Update dimensions
        const geometry = new THREE.BoxGeometry(width, height, width * 0.1);
        controlSurface.geometry.dispose();
        controlSurface.geometry = geometry;
        
        this.render();
    }

    updateControlSurfaceParameters(id, parameters) {
        const controlSurface = this.getObjectById(id);
        if (!controlSurface) return;
        
        // Update properties based on parameters
        if (parameters.position) {
            controlSurface.position.set(
                parameters.position[0],
                parameters.position[1],
                parameters.position[2]
            );
        }
        
        if (parameters.orientation) {
            controlSurface.rotation.set(
                parameters.orientation[0],
                parameters.orientation[1],
                parameters.orientation[2]
            );
        }
        
        if (parameters.area) {
            this.updateControlSurfaceSize(id, parameters.area);
        }
        
        this.render();
    }

    addThruster(id, name, position, orientation, diameter = 0.2) {
        // Create a simple representation of a thruster
        const length = diameter * 2;
        const radiusTop = diameter / 2;
        const radiusBottom = diameter / 2 * 1.2;
        
        const geometry = new THREE.CylinderGeometry(radiusTop, radiusBottom, length, 16);
        // Rotate the cylinder to align with X-axis (forward)
        geometry.rotateZ(Math.PI / 2);
        // Center the geometry
        geometry.translate(0, 0, 0);
        
        const material = new THREE.MeshStandardMaterial({ 
            color: 0xff5500,
            transparent: true,
            opacity: 0.7,
            metalness: 0.3,
            roughness: 0.7
        });
        
        const thruster = new THREE.Mesh(geometry, material);
        thruster.name = name || `Thruster ${id}`;
        thruster.userData.componentType = 'thruster';
        thruster.userData.componentId = id;
        
        // Position and rotate
        thruster.position.set(position[0], position[1], position[2]);
        thruster.rotation.set(orientation[0], orientation[1], orientation[2]);
        
        // Add to scene
        this.scene.add(thruster);
        
        // Add axes at component center
        this.addComponentAxes(thruster);
        
        // Update scene hierarchy
        this.updateSceneHierarchy();
        
        this.render();
        
        return thruster;
    }

    addSensor(id, type, position, orientation) {
        if (!position) {
            console.warn(`Cannot add sensor with ID ${id} visually as position is None`);
            return null;
        }

        let geometry, material;
        let size = 0.1;  // Default size for sensors
        
        // Choose geometry based on sensor type
        switch (type) {
            case 'IMU':
                geometry = new THREE.BoxGeometry(size, size, size);
                material = new THREE.MeshStandardMaterial({ color: 0x00ff00 });
                break;
            case 'GPS':
                geometry = new THREE.SphereGeometry(size / 2, 8, 8);
                material = new THREE.MeshStandardMaterial({ color: 0xff0000 });
                break;
            case 'DVL':
                geometry = new THREE.ConeGeometry(size / 2, size, 8);
                material = new THREE.MeshStandardMaterial({ color: 0x0000ff });
                break;
            default:
                geometry = new THREE.BoxGeometry(size, size / 2, size / 2);
                material = new THREE.MeshStandardMaterial({ color: 0xffff00 });
        }
        
        // Center the geometry
        geometry.translate(0, 0, 0);
        
        const sensor = new THREE.Mesh(geometry, material);
        sensor.name = `${type} Sensor ${id}`;
        sensor.userData.componentType = 'sensor';
        sensor.userData.componentId = id;
        
        // Position
        sensor.position.set(position[0], position[1], position[2]);
        
        // Orientation - if quaternion, apply it, otherwise use Euler angles
        if (orientation && orientation.length === 4) {
            // Quaternion [w, x, y, z]
            sensor.quaternion.set(orientation[1], orientation[2], orientation[3], orientation[0]);
        } else if (orientation && orientation.length === 3) {
            // Euler angles [x, y, z]
            sensor.rotation.set(orientation[0], orientation[1], orientation[2]);
        }
        
        // Add to scene
        this.scene.add(sensor);
        
        // Add axes at component center
        this.addComponentAxes(sensor);
        
        // Update scene hierarchy
        this.updateSceneHierarchy();
        
        this.render();
        
        return sensor;
    }

    removeControlSurface(id) {
        const surface = this.controlSurfaces.get(id);
        if (surface) {
            this.vessel.remove(surface);
            this.controlSurfaces.delete(id);
            if (this.selectedObject === surface) {
                this.transformControls.detach();
                this.selectedObject = null;
            }
        }
    }

    removeThruster(id) {
        const thruster = this.thrusters.get(id);
        if (thruster) {
            this.vessel.remove(thruster);
            this.thrusters.delete(id);
            if (this.selectedObject === thruster) {
                this.transformControls.detach();
                this.selectedObject = null;
            }
        }
    }

    removeSensor(id) {
        const sensor = this.sensors.get(id);
        if (sensor) {
            this.vessel.remove(sensor);
            this.sensors.delete(id);
            if (this.selectedObject === sensor) {
                this.transformControls.detach();
                this.selectedObject = null;
            }
        }
    }

    setTransformMode(mode) {
        this.transformMode = mode;
        if (this.transformControls) {
            this.transformControls.setMode(mode);
            
            // When switching to rotate mode, ensure we're using the local space
            // and the pivot point is at the component's center
            if (mode === 'rotate' && this.transformControls.object) {
                const object = this.transformControls.object;
                
                // Always use local space for rotation
                this.transformControls.setSpace('local');
                
                // If this is a component axes, use the stored pivot point if available
                if (object.userData.isComponentAxes && object.userData.pivotPoint) {
                    console.log('Using pivot point for rotation:', object.userData.pivotPoint);
                    // Note: Three.js TransformControls will use the object's position as pivot
                    // The object's origin point is already at the component's center due to 
                    // our setup in addComponentAxes
                }
            }
        }
        
        // Update button states
        document.getElementById('translateMode')?.classList.toggle('active', mode === 'translate');
        document.getElementById('rotateMode')?.classList.toggle('active', mode === 'rotate');
        document.getElementById('scaleMode')?.classList.toggle('active', mode === 'scale');
        
        // Update button states using viewport buttons IDs too
        document.getElementById('btn-translate')?.classList.toggle('active', mode === 'translate');
        document.getElementById('btn-rotate')?.classList.toggle('active', mode === 'rotate');
        document.getElementById('btn-scale')?.classList.toggle('active', mode === 'scale');
    }

    updateTransformInfo() {
        const transformInfo = document.getElementById('transform-info');
        if (!transformInfo) return;
        
        const object = this.transformControls.object;
        if (!object) {
            transformInfo.innerHTML = '';
            return;
        }
        
        // Check if we're transforming component axes or a component
        let isAxes = object.userData && object.userData.isComponentAxes === true;
        let componentObject = isAxes ? object.parent : object;
        
        // Get object position in world space
        const position = new THREE.Vector3();
        object.getWorldPosition(position);
        
        // Get object rotation in world space
        const quaternion = new THREE.Quaternion();
        object.getWorldQuaternion(quaternion);
        const euler = new THREE.Euler().setFromQuaternion(quaternion);
        
        // Convert to degrees for display
        const rotDeg = {
            x: THREE.MathUtils.radToDeg(euler.x).toFixed(1),
            y: THREE.MathUtils.radToDeg(euler.y).toFixed(1),
            z: THREE.MathUtils.radToDeg(euler.z).toFixed(1)
        };
        
        // Get component type and name for display
        let componentName = componentObject ? (componentObject.name || 'Unknown Component') : (object.name || 'Unknown');
        let componentType = isAxes ? 
            object.userData.componentType : 
            (componentObject && componentObject.userData ? componentObject.userData.componentType : null);
            
        componentType = componentType || 
            (componentObject && componentObject.userData ? componentObject.userData.centerType : null) || 
            'Unknown';
        
        // Format the component type nicely
        let formattedType = componentType;
        if (componentType === 'controlSurface') formattedType = 'Control Surface';
        else if (componentType === 'thruster') formattedType = 'Thruster';
        else if (componentType === 'sensor') formattedType = 'Sensor';
        else if (componentType === 'cog') formattedType = 'Center of Gravity';
        else if (componentType === 'cob') formattedType = 'Center of Buoyancy';
        
        // If this is component axes, also update any input fields for the component
        if (isAxes && componentObject) {
            const componentId = object.userData.componentId;
            const componentType = object.userData.componentType;
            
            // Update input fields based on component type
            if (componentType === 'controlSurface') {
                const inputPrefix = 'cs';
                this.updatePositionInputFields(inputPrefix, position);
                this.updateRotationInputFields(inputPrefix, rotDeg);
            } else if (componentType === 'thruster') {
                const inputPrefix = 'thruster';
                this.updatePositionInputFields(inputPrefix, position);
                this.updateRotationInputFields(inputPrefix, rotDeg);
            } else if (componentType === 'sensor') {
                const inputPrefix = 'sensor';
                this.updatePositionInputFields(inputPrefix, position);
                
                // Sensors use quaternion for orientation
                const orW = document.getElementById(`${inputPrefix}OrientationW`);
                const orX = document.getElementById(`${inputPrefix}OrientationX`);
                const orY = document.getElementById(`${inputPrefix}OrientationY`);
                const orZ = document.getElementById(`${inputPrefix}OrientationZ`);
                
                if (orW) orW.value = quaternion.w.toFixed(3);
                if (orX) orX.value = quaternion.x.toFixed(3);
                if (orY) orY.value = quaternion.y.toFixed(3);
                if (orZ) orZ.value = quaternion.z.toFixed(3);
            }
        }
        
        // Create display text with component type and name
        transformInfo.innerHTML = `
            <div class="transform-info-header">
                <strong>${componentName}</strong>
                <div class="component-type">${formattedType}</div>
            </div>
            <div class="transform-info-item">
                <span class="label">Position:</span>
                <span class="value">X: ${position.x.toFixed(3)}, Y: ${position.y.toFixed(3)}, Z: ${position.z.toFixed(3)}</span>
            </div>
            <div class="transform-info-item">
                <span class="label">Rotation:</span>
                <span class="value">X: ${rotDeg.x}°, Y: ${rotDeg.y}°, Z: ${rotDeg.z}°</span>
            </div>
        `;
    }
    
    // Helper to update position input fields
    updatePositionInputFields(prefix, position) {
        const posX = document.getElementById(`${prefix}PositionX`);
        const posY = document.getElementById(`${prefix}PositionY`);
        const posZ = document.getElementById(`${prefix}PositionZ`);
        
        if (posX) posX.value = position.x.toFixed(3);
        if (posY) posY.value = position.y.toFixed(3);
        if (posZ) posZ.value = position.z.toFixed(3);
    }
    
    // Helper to update rotation input fields
    updateRotationInputFields(prefix, rotation) {
        const rotX = document.getElementById(`${prefix}OrientationX`);
        const rotY = document.getElementById(`${prefix}OrientationY`);
        const rotZ = document.getElementById(`${prefix}OrientationZ`);
        
        if (rotX) rotX.value = rotation.x;
        if (rotY) rotY.value = rotation.y;
        if (rotZ) rotZ.value = rotation.z;
    }

    onObjectTransformed(object) {
        if (!object) return;
        
        // If this is a component axes, update the component's data in the model
        if (object.userData && object.userData.isComponentAxes) {
            // Get the parent component
            const componentObject = object.parent;
            
            if (componentObject) {
                // Get component type and ID from the axes
                const componentType = object.userData.componentType;
                const componentId = object.userData.componentId;
                
                if (componentType && componentId) {
                    // Get component data from the model
                    const vesselModel = window.currentVesselModel;
                    if (vesselModel) {
                        // Get component world position and orientation
                        const worldPosition = new THREE.Vector3();
                        const worldQuaternion = new THREE.Quaternion();
                        
                        object.getWorldPosition(worldPosition);
                        object.getWorldQuaternion(worldQuaternion);
                        
                        const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
                        
                        // Ensure values are numbers, not strings
                        const position = [
                            parseFloat(worldPosition.x),
                            parseFloat(worldPosition.y),
                            parseFloat(worldPosition.z)
                        ];
                        
                        const orientation = [
                            parseFloat(worldEuler.x),
                            parseFloat(worldEuler.y),
                            parseFloat(worldEuler.z)
                        ];
                        
                        // Debug log
                        console.log(`Updating ${componentType} ${componentId} from axes transformation:`, {
                            position,
                            orientation
                        });
                        
                        // Update model data based on the component type
                        switch (componentType) {
                            case 'controlSurface':
                                vesselModel.updateControlSurface(componentId, {
                                    control_surface_location: position,
                                    control_surface_orientation: orientation
                                });
                                break;
                            case 'thruster':
                                vesselModel.updateThruster(componentId, {
                                    thruster_location: position,
                                    thruster_orientation: orientation
                                });
                                break;
                            case 'sensor':
                                // For sensors, use quaternion representation if needed
                                if (vesselModel.isSensorUsingQuaternion(componentId)) {
                                    const quatOrientation = [
                                        worldQuaternion.w,
                                        worldQuaternion.x,
                                        worldQuaternion.y,
                                        worldQuaternion.z
                                    ];
                                    vesselModel.updateSensor(componentId, {
                                        sensor_location: position,
                                        sensor_orientation: quatOrientation
                                    });
                                } else {
                                    vesselModel.updateSensor(componentId, {
                                        sensor_location: position,
                                        sensor_orientation: orientation
                                    });
                                }
                                break;
                        }
                        
                        console.log(`Updated ${componentType} ${componentId} from axes transformation`);
                    }
                }
            }
            
            // Update transform info in the side panel
            this.updateTransformInfo();
            return;
        }
        
        // Check if this is a center point
        if (object.userData && object.userData.centerType) {
            const type = object.userData.centerType;
            const position = [
                object.position.x,
                object.position.y,
                object.position.z
            ];
            
            // Extract rotation in degrees
            const orientation = [
                THREE.MathUtils.radToDeg(object.rotation.x),
                THREE.MathUtils.radToDeg(object.rotation.y),
                THREE.MathUtils.radToDeg(object.rotation.z)
            ];
            
            // Update the model data
            if (window.currentVesselModel && window.currentVesselModel.config) {
                // Update legacy format
                window.currentVesselModel.config.geometry[type] = position;
                
                // Update new format
                window.currentVesselModel.config.geometry[`${type}_position`] = position;
                window.currentVesselModel.config.geometry[`${type}_orientation`] = orientation;
                
                // Update UI if geometry modal is open
                this.updateCenterPointUI(type, position, orientation);
                
                console.log(`${type} position updated to:`, position);
                console.log(`${type} orientation updated to:`, orientation);
            }
            
            // Update transform info in the side panel
            this.updateTransformInfo();
            return;
        }
        
        // For other objects (vessel, etc.)
        this.updateTransformInfo();
    }

    onWindowResize() {
        this.updateRendererSize();
        this.render();
    }

    onMouseMove(event) {
        // Update normalized mouse position
        this.mouse.x = (event.offsetX / this.renderer.domElement.clientWidth) * 2 - 1;
        this.mouse.y = -(event.offsetY / this.renderer.domElement.clientHeight) * 2 + 1;
    }

    onClick(event) {
        // Update raycaster with camera and mouse position
        this.raycaster.setFromCamera(this.mouse, this.camera);
        
        // Find intersected objects (include all scene objects for FBX component selection)
        const intersects = this.raycaster.intersectObjects(this.scene.children, true);
        
        if (intersects.length > 0) {
            // Find the first selectable object
            const selectedIntersect = intersects.find(intersect => {
                // Check if the object is selectable
                const isSelectable = intersect.object.isMesh && 
                                     intersect.object.userData && 
                                     intersect.object.userData.selectable;
                
                if (!isSelectable) return false;
                
                // If vessel is hidden, don't allow selection of vessel parts but allow center points
                if (this.vessel && !this.vessel.visible) {
                    // Check if this object is part of the vessel
                    let isPartOfVessel = false;
                    this.vessel.traverse((child) => {
                        if (child === intersect.object) {
                            isPartOfVessel = true;
                        }
                    });
                    
                    // Allow selection of center points (which should have centerType property)
                    const isCenterPoint = intersect.object.userData && 
                                         intersect.object.userData.centerType !== undefined;
                    
                    // Only allow selection if it's NOT part of the vessel OR it's a center point
                    return !isPartOfVessel || isCenterPoint;
                }
                
                // Check if the object is part of the vessel model
                if (this.vessel) {
                    let isPartOfVessel = false;
                    this.vessel.traverse((child) => {
                        if (child === intersect.object) {
                            isPartOfVessel = true;
                        }
                    });

                    // If it's part of the vessel, check if model selectability is enabled
                    if (isPartOfVessel) {
                        // Check the state of the selectability toggle
                        const toggleModelSelectable = document.getElementById('toggleModelSelectable');
                        if (toggleModelSelectable && !toggleModelSelectable.checked) {
                            return false; // Not selectable when toggle is off
                        }
                    }
                }
                
                // If vessel is visible or doesn't exist, allow selection of any selectable object
                return true;
            });
            
            if (selectedIntersect) {
                console.log('Selected object:', selectedIntersect.object.name);
                this.selectObject(selectedIntersect.object);
            } else {
                // No selectable object found, deselect current
                this.selectObject(null);
            }
        } else {
            // No intersection, deselect current
            this.selectObject(null);
        }
    }

    selectObject(object) {
        // Deselect previously selected object
        if (this.selectedObject) {
            this.restoreOriginalMaterial(this.selectedObject);
            this.selectedObject = null;
        }
        
        this.transformControls.detach();
        
        // If a new object was selected
        if (object) {
            this.selectedObject = object;
            
            if (object.userData.selectable) {
                this.highlightObject(object);
                
                // Check if this object is a component (control surface, thruster, sensor)
                const isComponent = object.userData.componentType === 'controlSurface' || 
                                   object.userData.componentType === 'thruster' || 
                                   object.userData.componentType === 'sensor';
                
                if (isComponent) {
                    // Find the axes associated with this component
                    let axes = null;
                    object.traverse(child => {
                        if (child.userData && child.userData.isAxes) {
                            axes = child;
                        }
                    });
                    
                    // Store component data
                    const componentId = object.userData.componentId;
                    const componentType = object.userData.componentType;
                    
                    // If axes exist, attach transform controls to the axes
                    if (axes) {
                        // Don't recreate axes, just use the existing ones with their current orientation
                        // Store a reference to the component in axes.userData
                        axes.userData.componentType = componentType;
                        axes.userData.componentId = componentId;
                        axes.userData.isComponentAxes = true;
                        
                        // Attach transform controls to axes
                        this.transformControls.attach(axes);
                        
                        // Show component info
                        this.showComponentInfo(object);
                    } else {
                        // If no axes found, create them and attach
                        axes = this.addComponentAxes(object);
                        
                        // Store component info in axes.userData
                        axes.userData.componentType = componentType;
                        axes.userData.componentId = componentId;
                        axes.userData.isComponentAxes = true;
                        
                        this.transformControls.attach(axes);
                        
                        // Show component info
                        this.showComponentInfo(object);
                    }
                } else if (object.userData.centerType) {
                    // Center points can be transformed directly
                    this.transformControls.attach(object);
                    this.showCenterPointInfo(object);
                } else {
                    // For other objects, attach directly
                    this.transformControls.attach(object);
                    
                    // Show component info in the side panel
                    this.showComponentInfo(object);
                }
            }
        } else {
            // No object selected, clear info panel
            this.clearComponentInfo();
        }
        
        this.render();
    }

    // Helper method to restore the original material
    restoreOriginalMaterial(object) {
        if (object && object.isMesh) {
            if (Array.isArray(object.material)) {
                // For multi-material objects, restore each material
                if (object.userData.originalMaterials) {
                    object.material = object.userData.originalMaterials.map(m => m.clone());
                }
            } else {
                // For single material objects
                if (object.userData.originalMaterial) {
                    object.material = object.userData.originalMaterial.clone();
                }
            }
        }
    }

    // Helper method to highlight an object
    highlightObject(object) {
        if (object && object.isMesh) {
            if (Array.isArray(object.material)) {
                // For multi-material objects, highlight each material
                object.material.forEach(material => {
                    material.emissive = new THREE.Color(0x333333);
                    material.emissiveIntensity = 0.5;
                });
            } else {
                // For single material objects
                object.material.emissive = new THREE.Color(0x333333);
                object.material.emissiveIntensity = 0.5;
            }
        }
    }

    // Display component info in the UI
    showComponentInfo(object) {
        if (!object) return;
        
        // Get component data if it exists
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) {
            console.warn('No vessel model available');
            return;
        }
        
        // First get the component mapping
        const mappedData = vesselModel.getModelComponentData(object.uuid);
        
        // Then try to get the actual component data
        let componentData = null;
        let componentId = null;
        let componentType = null;
        
        // Get ID and type either from mapping or directly from object
        if (mappedData) {
            componentId = mappedData.id;
            componentType = mappedData.type;
        } else if (object.userData.componentId && object.userData.componentType) {
            componentId = object.userData.componentId;
            componentType = object.userData.componentType;
        } else if (object.parent && object.parent.userData.componentId && object.parent.userData.componentType) {
            componentId = object.parent.userData.componentId;
            componentType = object.parent.userData.componentType;
        }
        
        // Now get the actual component data from the model using ID and type
        if (componentId && componentType) {
            console.log(`Getting ${componentType} with ID ${componentId} for display`);
            
            switch (componentType) {
                case 'controlSurface':
                    componentData = vesselModel.getControlSurface(componentId);
                    break;
                case 'thruster':
                    componentData = vesselModel.getThruster(componentId);
                    break;
                case 'sensor':
                    componentData = vesselModel.getSensor(componentId);
                    break;
            }
            
            if (componentData) {
                console.log(`Retrieved ${componentType} data:`, componentData);
            } else {
                console.warn(`Could not find ${componentType} with ID ${componentId} in model`);
            }
        }
        
        // Create HTML for component info
        let html = `
            <div class="property-group">
                <div class="property-group-title">Component Properties</div>
                <div class="property-row">
                    <div class="property-label">Name</div>
                    <div class="property-value">${object.name || 'Unnamed'}</div>
                </div>`;
                
        // Add component type dropdown
        html += `
                <div class="property-row">
                    <div class="property-label">Type</div>
                    <div class="property-value">
                        <select id="componentTypeSelect" class="form-control">
                            <option value="none" ${!componentType ? 'selected' : ''}>Regular Component</option>
                            <option value="controlSurface" ${componentType === 'controlSurface' ? 'selected' : ''}>Control Surface</option>
                            <option value="thruster" ${componentType === 'thruster' ? 'selected' : ''}>Thruster</option>
                            <option value="sensor" ${componentType === 'sensor' ? 'selected' : ''}>Sensor</option>
                        </select>
                    </div>
                </div>
            </div>
            
            <div class="property-group">
                <div class="property-group-title">Transform</div>
                <div class="property-row">
                    <div class="property-label">Position</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="posX" value="${object.position.x.toFixed(3)}" step="0.1">
                            <input type="number" class="form-control" id="posY" value="${object.position.y.toFixed(3)}" step="0.1">
                            <input type="number" class="form-control" id="posZ" value="${object.position.z.toFixed(3)}" step="0.1">
                        </div>
                    </div>
                </div>
                <div class="property-row">
                    <div class="property-label">Rotation (deg)</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="rotX" value="${THREE.MathUtils.radToDeg(object.rotation.x).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotY" value="${THREE.MathUtils.radToDeg(object.rotation.y).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotZ" value="${THREE.MathUtils.radToDeg(object.rotation.z).toFixed(1)}" step="1">
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        // Add component-specific data if we have it
        if (componentData) {
            html += `
            <div class="property-group">
                <div class="property-group-title">Component Data</div>
                <div class="property-row">
                    <div class="property-label">ID</div>
                    <div class="property-value">${componentId}</div>
                </div>`;
            
            // Display position and orientation based on component type
            if (componentType === 'controlSurface') {
                // Position
                if (componentData.control_surface_location && Array.isArray(componentData.control_surface_location)) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Model Position</div>
                        <div class="property-value">
                            [${componentData.control_surface_location.map(v => parseFloat(v).toFixed(3)).join(', ')}]
                        </div>
                    </div>`;
                }
                
                // Orientation
                if (componentData.control_surface_orientation && Array.isArray(componentData.control_surface_orientation)) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Model Orientation (rad)</div>
                        <div class="property-value">
                            [${componentData.control_surface_orientation.map(v => parseFloat(v).toFixed(3)).join(', ')}]
                        </div>
                    </div>
                    <div class="property-row">
                        <div class="property-label">Model Orientation (deg)</div>
                        <div class="property-value">
                            [${componentData.control_surface_orientation.map(v => THREE.MathUtils.radToDeg(parseFloat(v)).toFixed(1)).join(', ')}]
                        </div>
                    </div>`;
                }
                
                // Area
                if (componentData.control_surface_area) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Area</div>
                        <div class="property-value">${parseFloat(componentData.control_surface_area).toFixed(3)} m²</div>
                    </div>`;
                }
            } else if (componentType === 'thruster') {
                // Position
                if (componentData.thruster_location && Array.isArray(componentData.thruster_location)) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Model Position</div>
                        <div class="property-value">
                            [${componentData.thruster_location.map(v => parseFloat(v).toFixed(3)).join(', ')}]
                        </div>
                    </div>`;
                }
                
                // Orientation
                if (componentData.thruster_orientation && Array.isArray(componentData.thruster_orientation)) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Model Orientation (rad)</div>
                        <div class="property-value">
                            [${componentData.thruster_orientation.map(v => parseFloat(v).toFixed(3)).join(', ')}]
                        </div>
                    </div>
                    <div class="property-row">
                        <div class="property-label">Model Orientation (deg)</div>
                        <div class="property-value">
                            [${componentData.thruster_orientation.map(v => THREE.MathUtils.radToDeg(parseFloat(v)).toFixed(1)).join(', ')}]
                        </div>
                    </div>`;
                }
                
                // Display thruster properties
                if (componentData.T_prop) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Thrust Coefficient</div>
                        <div class="property-value">${parseFloat(componentData.T_prop).toFixed(3)}</div>
                    </div>`;
                }
                
                if (componentData.D_prop) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Diameter</div>
                        <div class="property-value">${parseFloat(componentData.D_prop).toFixed(3)} m</div>
                    </div>`;
                }
                
                if (componentData.KT_at_J0 !== undefined) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">KT at J0</div>
                        <div class="property-value">${parseFloat(componentData.KT_at_J0).toFixed(3)}</div>
                    </div>`;
                }
                
                if (componentData.n_max !== undefined) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Max RPM</div>
                        <div class="property-value">${parseFloat(componentData.n_max).toFixed(0)}</div>
                    </div>`;
                }
            } else if (componentType === 'sensor') {
                // Position
                if (componentData.sensor_location && Array.isArray(componentData.sensor_location)) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Model Position</div>
                        <div class="property-value">
                            [${componentData.sensor_location.map(v => parseFloat(v).toFixed(3)).join(', ')}]
                        </div>
                    </div>`;
                }
                
                // Orientation
                if (componentData.sensor_orientation && Array.isArray(componentData.sensor_orientation)) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Model Orientation (quat)</div>
                        <div class="property-value">
                            [${componentData.sensor_orientation.map(v => parseFloat(v).toFixed(3)).join(', ')}]
                        </div>
                    </div>`;
                }
                
                // Publish rate
                if (componentData.publish_rate) {
                    html += `
                    <div class="property-row">
                        <div class="property-label">Publish Rate</div>
                        <div class="property-value">${parseFloat(componentData.publish_rate).toFixed(1)} Hz</div>
                    </div>`;
                }
            }
            
            html += `</div>`;
        }
        
        // Add configure button with dynamic text based on component status
        const isConfigured = object.children.some(child => child.userData.isComponentAxes);
        const buttonText = isConfigured ? 'Reconfigure Component' : 'Configure Component';
        
        html += `
            <div class="mt-3">
                <button id="btnConfigureComponent" class="btn btn-primary">${buttonText}</button>
            </div>
        `;
        
        // Update UI
        const propertiesPanel = document.getElementById('object-properties');
        if (propertiesPanel) {
            propertiesPanel.innerHTML = html;
            
            // Add event listener for type selection
            const typeSelect = document.getElementById('componentTypeSelect');
            if (typeSelect) {
                typeSelect.addEventListener('change', () => {
                    this.showComponentConfigModal(object, typeSelect.value);
                });
            }
            
            // Add event listener for configure button
            const configBtn = document.getElementById('btnConfigureComponent');
            if (configBtn) {
                configBtn.addEventListener('click', () => {
                    const typeValue = document.getElementById('componentTypeSelect').value;
                    this.showComponentConfigModal(object, typeValue);
                });
            }
            
            // Add event listeners for transform inputs
            ['posX', 'posY', 'posZ'].forEach(id => {
                document.getElementById(id)?.addEventListener('change', () => {
                    const x = parseFloat(document.getElementById('posX').value);
                    const y = parseFloat(document.getElementById('posY').value);
                    const z = parseFloat(document.getElementById('posZ').value);
                    object.position.set(x, y, z);
                    
                    // If this is a component with axes, update the axes position too
                    const axes = object.children.find(child => child.userData.isComponentAxes);
                    if (axes) {
                        // Get center offset of axes
                        const offset = axes.position.clone();
                        // Update component
                        this.updateComponentPositionFromObject(object);
                    } else {
                        // Direct update
                        this.updateComponentPositionFromObject(object);
                    }
                    
                    this.render();
                });
            });
            
            ['rotX', 'rotY', 'rotZ'].forEach(id => {
                document.getElementById(id)?.addEventListener('change', () => {
                    const x = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotX').value));
                    const y = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotY').value));
                    const z = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotZ').value));
                    object.rotation.set(x, y, z);
                    
                    // Update component orientation in model if needed
                    if (componentId && componentType) {
                        this.updateComponentOrientationFromObject(object);
                    }
                    
                    this.render();
                });
            });
        }
    }

    // Show modal to configure component based on selected type
    showComponentConfigModal(object, type) {
        // Check if this is a reconfiguration of an existing component
        const isReconfiguring = object.children.some(child => child.userData.isComponentAxes);
        
        // Get relevant modal based on component type
        let modalId;
        if (type === 'controlSurface') {
            modalId = 'fbxComponentModal';
            // Show control surface settings
            document.getElementById('fbxControlSurfaceSettings').style.display = 'block';
            document.getElementById('fbxThrusterSettings').style.display = 'none';
            document.getElementById('fbxSensorSettings').style.display = 'none';
        } else if (type === 'thruster') {
            modalId = 'fbxComponentModal';
            // Show thruster settings
            document.getElementById('fbxControlSurfaceSettings').style.display = 'none';
            document.getElementById('fbxThrusterSettings').style.display = 'block';
            document.getElementById('fbxSensorSettings').style.display = 'none';
        } else if (type === 'sensor') {
            modalId = 'fbxComponentModal';
            // Show control surface settings
            document.getElementById('fbxControlSurfaceSettings').style.display = 'none';
            document.getElementById('fbxThrusterSettings').style.display = 'none';
            document.getElementById('fbxSensorSettings').style.display = 'block';
        } else {
            // If none/regular, just remove any component mapping
            const vesselModel = window.currentVesselModel;
            if (vesselModel && vesselModel.isModelComponentMapped(object.uuid)) {
                vesselModel.unmapModelComponent(object.uuid);
                this.showComponentInfo(object); // Refresh UI
            }
            return;
        }
        
        // Set the component type in the modal
        document.getElementById('fbxComponentType').value = type;
        
        // Store the reference to the object in the modal for later use
        const modal = document.getElementById(modalId);
        if (modal) {
            modal.dataset.objectUuid = object.uuid;
            modal.dataset.isReconfiguring = isReconfiguring;
            
            // Show the modal
            const bsModal = new bootstrap.Modal(modal);
            bsModal.show();
        }
    }

    toggleGrid() {
        if (this.grid) {
        this.grid.visible = !this.grid.visible;
            this.render();
            return this.grid.visible;
        }
        return false;
    }

    toggleWireframe() {
        let changed = false;
        this.scene.traverse((object) => {
            if (object.isMesh && object.material) {
                if (Array.isArray(object.material)) {
                    object.material.forEach(material => {
                        material.wireframe = !material.wireframe;
                    });
            } else {
                    object.material.wireframe = !object.material.wireframe;
                }
                changed = true;
            }
        });
        
        if (changed) {
            this.render();
        }
    }

    resetCamera() {
        // Reset camera position and target
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
        this.orbitControls.target.set(0, 0, 0);
        this.orbitControls.update();
        this.render();
    }

    animate() {
        // Use requestAnimationFrame with the proper binding
        requestAnimationFrame(this.animate.bind(this));
        
        // Performance monitoring
        if (this.stats) this.stats.begin();
        
        // Only update controls if they exist and are enabled
        if (this.orbitControls && this.orbitControls.enabled) {
            this.orbitControls.update();
            // Orbit controls update should trigger a render
            this.needsRender = true;
        }
        
        // Only render when needed (needsRender flag is true)
        if (this.needsRender) {
            this.renderer.render(this.scene, this.camera);
            this.needsRender = false;
            
            // Render the global axes helper if it exists
            if (this.renderAxes) {
                this.renderAxes();
            }
        }
        
        // Performance monitoring
        if (this.stats) this.stats.end();
    }

    updateRendererSize() {
        if (!this.container) return;
        
        const rect = this.container.getBoundingClientRect();
        
        // Only update if the size actually changed
        if (this.lastWidth !== rect.width || this.lastHeight !== rect.height) {
            this.lastWidth = rect.width;
            this.lastHeight = rect.height;
            
            this.camera.aspect = rect.width / rect.height;
            this.camera.updateProjectionMatrix();
            this.renderer.setSize(rect.width, rect.height);
            
            // Force a render to update the viewport
            this.forceRender();
        }
    }

    render() {
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
            
            // Also render the axes helper if it exists
            if (this.renderAxes) {
                this.renderAxes();
            }
            
            this.needsRender = false;
        }
    }

    forceRender() {
        this.needsRender = true;
        if (this.renderer && this.scene && this.camera) {
            this.renderer.render(this.scene, this.camera);
            
            // Also render the axes helper if it exists
            if (this.renderAxes) {
                this.renderAxes();
            }
        }
    }

    getObjectById(id) {
        // Check control surfaces
        if (this.controlSurfaces.has(id)) {
            return this.controlSurfaces.get(id);
        }
        
        // Check thrusters
        if (this.thrusters.has(id)) {
            return this.thrusters.get(id);
        }
        
        // Check sensors
        if (this.sensors.has(id)) {
            return this.sensors.get(id);
        }
        
        return null;
    }

    updateControlSurfaceSize(id, area) {
        const surface = this.controlSurfaces.get(id);
        if (surface) {
            const scale = Math.sqrt(area);
            surface.scale.set(scale, scale, scale);
            this.render();
        }
    }

    updateThrusterSize(id, thrust) {
        const thruster = this.thrusters.get(id);
        if (thruster) {
            const scale = 0.2 + (thrust / 5000) * 0.8; // Scale between 0.2 and 1.0 based on thrust
            thruster.scale.set(scale, scale, scale);
            this.render();
        }
    }

    toggleBox(visible = null) {
        if (!this.vessel) return;
        
        const box = this.vessel.getObjectByName('boundingBox');
        if (box) {
            if (visible === null) {
                box.visible = !box.visible;
            } else {
                box.visible = visible;
            }
            
            // Update checkbox state
            const checkbox = document.getElementById('toggleBoxVisibility');
            if (checkbox) {
                checkbox.checked = box.visible;
            }
            
            this.render();
        }
    }

    toggleSTL(visible = null) {
        if (!this.vessel) return;
        
        const stlModel = this.vessel.getObjectByName('stlModel');
        if (stlModel) {
            if (visible === null) {
                stlModel.visible = !stlModel.visible;
            } else {
                stlModel.visible = visible;
            }
            
            // Update checkbox state
            const checkbox = document.getElementById('toggleSTLVisibility');
            if (checkbox) {
                checkbox.checked = stlModel.visible;
            }
            
            this.render();
        }
    }

    toggleGeometry(visible = null) {
        if (!this.vessel) return;
        
        // Toggle visibility of the entire vessel group or specific model parts
        if (visible === null) {
            // Toggle current state
            this.vessel.visible = !this.vessel.visible;
        } else {
            // Set to specific state
            this.vessel.visible = visible;
        }
        
        // Handle transform controls and selection when hiding
        if (!this.vessel.visible) {
            // If vessel is not visible and the current selected object is part of the vessel model,
            // deselect it but don't hide transform controls completely
            if (this.transformControls.object) {
                // Check if the selected object is part of the vessel model
                let isPartOfVessel = false;
                this.vessel.traverse((child) => {
                    if (child === this.transformControls.object) {
                        isPartOfVessel = true;
                    }
                });
                
                // Only deselect if it's part of the vessel model
                if (isPartOfVessel) {
                    this.restoreOriginalMaterial(this.transformControls.object);
                    this.transformControls.detach();
                    this.clearComponentInfo();
                }
            }
        }
        
        // Update checkbox state
        const checkbox = document.getElementById('toggleGeometryVisibility');
        if (checkbox) {
            checkbox.checked = this.vessel.visible;
        }
        
        // Update UI notification
        if (this.vessel.visible) {
            console.log('Geometry visibility: shown');
        } else {
            console.log('Geometry visibility: hidden');
        }
        
        this.render();
        
        // Return current visibility state
        return this.vessel.visible;
    }

    // Set transparency for the FBX model
    setModelTransparency(value) {
        if (!this.vessel) return;
        
        // Convert value (0-100) to opacity (1-0)
        const opacity = 1 - (value / 100);
        
        // Traverse all mesh objects and update their materials
        this.vessel.traverse((child) => {
            if (child.isMesh) {
                // Handle multiple materials
                if (Array.isArray(child.material)) {
                    child.material.forEach(material => {
                        material.transparent = opacity < 1;
                        material.opacity = opacity;
                    });
                } else {
                    // Handle single material
                    child.material.transparent = opacity < 1;
                    child.material.opacity = opacity;
                }
            }
        });
        
        // Force a render to update the appearance
        this.render();
    }

    // Set whether the model is selectable
    setModelSelectable(selectable) {
        if (!this.vessel) return;
        
        // Traverse all mesh objects and update their selectable property
        this.vessel.traverse((child) => {
            if (child.isMesh) {
                child.userData.selectable = selectable;
            }
        });
        
        // If the model is made non-selectable and a mesh is currently selected, deselect it
        if (!selectable && this.selectedObject) {
            // Check if the selected object is part of the vessel
            let isPartOfVessel = false;
            this.vessel.traverse((child) => {
                if (child === this.selectedObject) {
                    isPartOfVessel = true;
                }
            });
            
            // Only deselect if it's part of the vessel
            if (isPartOfVessel) {
                this.restoreOriginalMaterial(this.selectedObject);
                this.transformControls.detach();
                this.selectedObject = null;
                this.clearComponentInfo();
                
                // Update the properties panel to show no selection
                document.getElementById('object-properties').innerHTML = `
                    <div class="no-selection-message">
                        Select an object to view its properties
                    </div>
                `;
                
                // Force a render to update the appearance
                this.render();
            }
        }
    }

    // Add new method to create body center coordinate system
    addBodyCenterSystem(position = new THREE.Vector3(0, 0, 0)) {
        // Remove existing BCS if any
        if (this.bodyCenter) {
            this.scene.remove(this.bodyCenter);
        }

        const bodyCenterGroup = new THREE.Group();
        bodyCenterGroup.userData.isBodyCenter = true;
        bodyCenterGroup.userData.isTransformable = true;
        bodyCenterGroup.userData.type = 'bodyCenter';
        bodyCenterGroup.name = 'Body Center System';

        // Get reference size from vessel if it exists
        let axesLength = 1;
        if (this.vessel && this.vessel.userData.axesGroup) {
            const bbox = new THREE.Box3().setFromObject(this.vessel);
            const size = new THREE.Vector3();
            bbox.getSize(size);
            axesLength = Math.max(size.x, size.y, size.z) * 0.3;
        }

        // Create body center axes
        const bcAxesLength = axesLength;
        
        // Create a container for axes and sphere to keep them together
        const bcContainer = new THREE.Group();
        bcContainer.name = 'BC Container';
        
        // X-axis (surge)
        const bcXAxis = new THREE.ArrowHelper(
            new THREE.Vector3(1, 0, 0),
            new THREE.Vector3(0, 0, 0),
            bcAxesLength,
            0xff0000,
            bcAxesLength * 0.25,
            bcAxesLength * 0.15
        );
        bcContainer.add(bcXAxis);

        // Y-axis (sway)
        const bcYAxis = new THREE.ArrowHelper(
            new THREE.Vector3(0, 1, 0),
            new THREE.Vector3(0, 0, 0),
            bcAxesLength,
            0x00ff00,
            bcAxesLength * 0.25,
            bcAxesLength * 0.15
        );
        bcContainer.add(bcYAxis);

        // Z-axis (heave)
        const bcZAxis = new THREE.ArrowHelper(
            new THREE.Vector3(0, 0, 1),
            new THREE.Vector3(0, 0, 0),
            bcAxesLength,
            0x0000ff,
            bcAxesLength * 0.25,
            bcAxesLength * 0.15
        );
        bcContainer.add(bcZAxis);

        // Add center sphere (made much smaller)
        const bcSphereGeometry = new THREE.SphereGeometry(bcAxesLength * 0.05);
        const bcSphereMaterial = new THREE.MeshStandardMaterial({
            color: 0xffffff,
            metalness: 0.7,
            roughness: 0.3,
            emissive: 0x444444
        });
        const bcSphere = new THREE.Mesh(bcSphereGeometry, bcSphereMaterial);
        bcSphere.name = 'BC Sphere';
        bcContainer.add(bcSphere);

        // Add container to body center group
        bodyCenterGroup.add(bcContainer);
        
        // Set position
        bodyCenterGroup.position.copy(position);

        // Add to scene and store reference
        this.scene.add(bodyCenterGroup);
        this.bodyCenter = bodyCenterGroup;

        this.updateSceneHierarchy();

        return bodyCenterGroup;
    }

    removeBodyCenterSystem() {
        if (this.bodyCenterSystem) {
            this.scene.remove(this.bodyCenterSystem);
            this.bodyCenterSystem = null;
            this.render();
        }
    }

    // Add new method to update scene hierarchy
    updateSceneHierarchy() {
        // Placeholder for scene hierarchy update
        console.log("Scene hierarchy updated");
    }

    async loadFBXModel(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (event) => {
                const fileContents = event.target.result;
                
                const loader = new THREE.FBXLoader();
                try {
                    const object = loader.parse(fileContents);
                    
                    // Clean up any existing model
                    if (this.vessel) {
                        this.scene.remove(this.vessel);
                        this.vessel = null;
                        this.updateSceneHierarchy();
                    }
                    
                    // Set up the vessel container
                    this.vessel = new THREE.Group();
                    this.vessel.name = 'Vessel';
                    
                    // Create a mapping of original meshes
                    this.modelMeshes = new Map();
                    
                    // Process the loaded model to make every mesh selectable
                    let index = 0;
                    object.traverse(child => {
                        if (child.isMesh) {
                            // Store original materials for later reference
                            if (Array.isArray(child.material)) {
                                child.userData.originalMaterials = child.material.map(m => m.clone());
                            } else {
                                child.userData.originalMaterial = child.material.clone();
                            }
                            
                            // Make sure the mesh has a name for identification
                            if (!child.name || child.name === '') {
                                child.name = `Component_${index++}`;
                            }
                            
                            // Store in our mesh map
                            this.modelMeshes.set(child.uuid, child);
                            
                            // Make mesh selectable
                            child.userData.selectable = true;
                            
                            // Log the component for debugging
                            console.log(`FBX component found: ${child.name} (${child.uuid})`);
                        }
                    });
                    
                    // Add the model to the vessel container
                    this.vessel.add(object);
                    
                    // Calculate the bounding box
                    const boundingBox = new THREE.Box3().setFromObject(this.vessel);
                    const size = new THREE.Vector3();
                    boundingBox.getSize(size);
                    
                    // Extract dimensions before applying scale
                    const modelDimensions = this.getModelDimensions(this.vessel);
                    this.modelDimensions = modelDimensions; // Store for later reference
                    
                    // Determine scale to normalize the model
                    const maxDimension = Math.max(size.x, size.y, size.z);
                    const scale = 5 / maxDimension;
                    
                    // Center the model
                    const center = new THREE.Vector3();
                    boundingBox.getCenter(center);
                    this.vessel.position.sub(center.multiplyScalar(scale));
                    
                    // Scale the model
                    this.vessel.scale.set(scale, scale, scale);
                    
                    // Add vessel to scene
                    this.scene.add(this.vessel);
                    
                    // Add axes to help visualize orientation
                    const axes = this.createLocalAxes(2);
                    this.vessel.add(axes);
                    
                    // Setup center points visualization
                    this.setupCenterPoints();
                    
                    // Check the state of the geometry visibility toggle and apply
                    const toggleGeometryVisibility = document.getElementById('toggleGeometryVisibility');
                    if (toggleGeometryVisibility) {
                        this.vessel.visible = toggleGeometryVisibility.checked;
                        
                        // Update the toggle button in the viewport
                        const toggleButton = document.getElementById('btn-toggle-geometry');
                        if (toggleButton) {
                            if (this.vessel.visible) {
                                toggleButton.classList.add('active');
                            } else {
                                toggleButton.classList.remove('active');
                            }
                        }
                    }
                    
                    // Apply current transparency setting
                    const transparencySlider = document.getElementById('modelTransparency');
                    if (transparencySlider) {
                        const transparencyValue = parseInt(transparencySlider.value, 10);
                        this.setModelTransparency(transparencyValue);
                    }
                    
                    // Apply current selectability setting
                    const toggleModelSelectable = document.getElementById('toggleModelSelectable');
                    if (toggleModelSelectable) {
                        this.setModelSelectable(toggleModelSelectable.checked);
                    }
                    
                    // Update scene hierarchy
                    this.updateSceneHierarchy();
                    
                    // Update camera to focus on the model
                    this.resetCamera();
                    
                    // Force a render to show the model immediately
                    this.forceRender();
                    
                    // Return the loaded model along with dimensions
                    resolve({
                        vessel: this.vessel,
                        dimensions: modelDimensions
                    });
                } catch (error) {
                    console.error('Error loading FBX model:', error);
                    reject(error);
                }
            };
            reader.onerror = reject;
            reader.readAsArrayBuffer(file);
        });
    }

    selectModelComponent(object) {
        // Deselect previous object if any
        if (this.selectedObject) {
            // Restore original material
            if (this.selectedObject.isMesh) {
                this.selectedObject.material = this.selectedObject.userData.originalMaterial.clone();
            }
            
            // Remove transform controls
            this.transformControls.detach();
        }
        
        // Update selected object
        this.selectedObject = object;
        
        if (object) {
            // Highlight selected object
            if (object.isMesh) {
                // Clone the original material and modify it to show selection
                const highlightMaterial = object.userData.originalMaterial.clone();
                highlightMaterial.emissive = new THREE.Color(0x333333);
                highlightMaterial.emissiveIntensity = 0.5;
                object.material = highlightMaterial;
            }
            
            // Attach transform controls
            this.transformControls.attach(object);
            
            // Update UI with selection info
            this.showComponentSettings(object);
            } else {
            // Clear UI if no selection
            document.getElementById('object-properties').innerHTML = `
                <div class="no-selection-message">
                    Select an object to view its properties
                </div>
            `;
        }
        
        // Update scene
        this.render();
    }

    showComponentSettings(object) {
        // First check if this object is already mapped to a component
        const vesselModel = window.currentVesselModel; // This should be set in the main.js
        
        if (!vesselModel) return;
        
        const componentData = vesselModel.getModelComponentData(object.uuid);
        let componentType = componentData ? componentData.type : 'none';
        
        // Create HTML for properties panel
        let html = `
            <div class="property-group">
                <div class="property-group-title">Object Properties</div>
                <div class="property-row">
                    <div class="property-label">Name</div>
                    <div class="property-value">${object.name || 'Unnamed Object'}</div>
                </div>
                <div class="property-row">
                    <div class="property-label">Type</div>
                    <div class="property-value">
                        <select id="componentTypeSelect" class="form-control">
                            <option value="none" ${componentType === 'none' ? 'selected' : ''}>Regular Object</option>
                            <option value="controlSurface" ${componentType === 'controlSurface' ? 'selected' : ''}>Control Surface</option>
                            <option value="thruster" ${componentType === 'thruster' ? 'selected' : ''}>Thruster</option>
                            <option value="sensor" ${componentType === 'sensor' ? 'selected' : ''}>Sensor</option>
                        </select>
                    </div>
                </div>
            </div>
            
            <div class="property-group">
                <div class="property-group-title">Transform</div>
                <div class="property-row">
                    <div class="property-label">Position</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="posX" value="${object.position.x.toFixed(2)}" step="0.1">
                            <input type="number" class="form-control" id="posY" value="${object.position.y.toFixed(2)}" step="0.1">
                            <input type="number" class="form-control" id="posZ" value="${object.position.z.toFixed(2)}" step="0.1">
                        </div>
                    </div>
                </div>
                <div class="property-row">
                    <div class="property-label">Rotation (deg)</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="rotX" value="${THREE.MathUtils.radToDeg(object.rotation.x).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotY" value="${THREE.MathUtils.radToDeg(object.rotation.y).toFixed(1)}" step="1">
                            <input type="number" class="form-control" id="rotZ" value="${THREE.MathUtils.radToDeg(object.rotation.z).toFixed(1)}" step="1">
                        </div>
                    </div>
                </div>
                <div class="property-row">
                    <div class="property-label">Scale</div>
                    <div class="property-value">
                        <div class="input-group">
                            <input type="number" class="form-control" id="scaleX" value="${object.scale.x.toFixed(2)}" step="0.1" min="0.1">
                            <input type="number" class="form-control" id="scaleY" value="${object.scale.y.toFixed(2)}" step="0.1" min="0.1">
                            <input type="number" class="form-control" id="scaleZ" value="${object.scale.z.toFixed(2)}" step="0.1" min="0.1">
                        </div>
                    </div>
                </div>
            </div>
        `;
        
        // Add specific component settings if this is a component
        if (componentType !== 'none' && componentData) {
            // Add component-specific properties based on the component type
            html += this.createComponentSpecificProperties(componentType, componentData.id, vesselModel);
        }
        
        // Add button to apply changes
        html += `
            <div class="mt-3">
                <button id="btnApplyComponentChanges" class="btn btn-primary">Apply Changes</button>
                <button id="btnRemoveComponent" class="btn btn-danger" ${componentType === 'none' ? 'style="display:none"' : ''}>Remove Component</button>
            </div>
        `;
        
        // Update the properties panel
        document.getElementById('object-properties').innerHTML = html;
        
        // Set up event listeners for the form fields
        this.setupComponentPropertyListeners(object);
    }

    createComponentSpecificProperties(componentType, componentId, vesselModel) {
        let html = '';
        
        switch (componentType) {
            case 'controlSurface':
                const surface = vesselModel.getControlSurface(componentId);
                if (surface) {
                    html += `
                        <div class="property-group">
                            <div class="property-group-title">Control Surface Properties</div>
                            <div class="property-row">
                                <div class="property-label">Type</div>
                                <div class="property-value">
                                    <select id="csSurfaceType" class="form-control">
                                        <option value="Rudder" ${surface.control_surface_type === 'Rudder' ? 'selected' : ''}>Rudder</option>
                                        <option value="Fins" ${surface.control_surface_type === 'Fins' ? 'selected' : ''}>Fins</option>
                                        <option value="Elevator" ${surface.control_surface_type === 'Elevator' ? 'selected' : ''}>Elevator</option>
                                    </select>
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Area (m²)</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="csSurfaceArea" value="${surface.control_surface_area}" step="0.01" min="0.01">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Time Constant</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="csSurfaceTimeConstant" value="${surface.control_surface_T}" step="0.01" min="0.01">
                                </div>
                            </div>
                        </div>
                    `;
                }
                break;
                
            case 'thruster':
                const thruster = vesselModel.getThruster(componentId);
                if (thruster) {
                    html += `
                        <div class="property-group">
                            <div class="property-group-title">Thruster Properties</div>
                            <div class="property-row">
                                <div class="property-label">Name</div>
                                <div class="property-value">
                                    <input type="text" class="form-control" id="thrusterName" value="${thruster.thruster_name}">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Type</div>
                                <div class="property-value">
                                    <select id="thrusterType" class="form-control">
                                        <option value="Propeller" ${thruster.thruster_type === 'Propeller' ? 'selected' : ''}>Propeller</option>
                                        <option value="Jet" ${thruster.thruster_type === 'Jet' ? 'selected' : ''}>Jet</option>
                                        <option value="Tunnel" ${thruster.thruster_type === 'Tunnel' ? 'selected' : ''}>Tunnel</option>
                                    </select>
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Diameter (m)</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="thrusterDiameter" value="${thruster.D_prop}" step="0.01" min="0.01">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Thrust Coefficient</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="thrusterCoefficient" value="${thruster.T_prop}" step="0.1" min="0.1">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">KT at J0</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="thrusterKTatJ0" value="${thruster.KT_at_J0 || 0.0}" step="0.01" min="0.0">
                                </div>
                            </div>
                            <div class="property-row">
                                <div class="property-label">Max RPM</div>
                                <div class="property-value">
                                    <input type="number" class="form-control" id="thrusterNMax" value="${thruster.n_max || 2668}" step="1" min="1">
                                </div>
                            </div>
                        </div>
                    `;
                }
                break;
        }
        
        return html;
    }

    setupComponentPropertyListeners(object) {
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) return;
        
        // Component type selection change
        const componentTypeSelect = document.getElementById('componentTypeSelect');
        if (componentTypeSelect) {
            componentTypeSelect.addEventListener('change', (e) => {
                const newType = e.target.value;
                const oldType = vesselModel.getModelComponentData(object.uuid)?.type;
                
                // If changing from one type to another, remove the old mapping
                if (oldType && oldType !== 'none') {
                    const componentData = vesselModel.getModelComponentData(object.uuid);
                    if (componentData) {
                        const componentId = componentData.id;
                        
                        // Remove the old component
                        if (oldType === 'controlSurface') {
                            vesselModel.removeControlSurface(componentId);
                        } else if (oldType === 'thruster') {
                            vesselModel.removeThruster(componentId);
                        } else if (oldType === 'sensor') {
                            vesselModel.removeSensor(componentId);
                        }
                        
                        // Remove the mapping
                        vesselModel.unmapModelComponent(object.uuid);
                    }
                }
                
                // If changing to a component type, create a new component
                if (newType !== 'none') {
                    // Get the object's world position and orientation
                    const worldPosition = new THREE.Vector3();
                    object.getWorldPosition(worldPosition);
                    
                    // Convert to world rotation (Euler angles)
                    const worldQuaternion = new THREE.Quaternion();
                    object.getWorldQuaternion(worldQuaternion);
                    const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
                    
                    // Create the component based on type
                    let componentId;
                    if (newType === 'controlSurface') {
                        componentId = vesselModel.addControlSurface(
                            'Rudder',
                            [worldPosition.x, worldPosition.y, worldPosition.z],
                            [worldEuler.x, worldEuler.y, worldEuler.z],
                            0.1
                        );
                    } else if (newType === 'thruster') {
                        componentId = vesselModel.addThruster(
                            'Propeller',
                            [worldPosition.x, worldPosition.y, worldPosition.z],
                            [worldEuler.x, worldEuler.y, worldEuler.z],
                            0.2
                        );
                    } else if (newType === 'sensor') {
                        componentId = vesselModel.addSensor(
                            'IMU',
                            [worldPosition.x, worldPosition.y, worldPosition.z],
                            [worldEuler.x, worldEuler.y, worldEuler.z],
                            10
                        );
                    }
                    
                    // Map the object to the component
                    if (componentId) {
                        vesselModel.mapModelComponent(object.uuid, newType, componentId);
                    }
                }
                
                // Refresh the component settings display
                this.showComponentSettings(object);
                
                // Make the remove button visible if needed
                const removeBtn = document.getElementById('btnRemoveComponent');
                if (removeBtn) {
                    removeBtn.style.display = newType !== 'none' ? 'inline-block' : 'none';
                }
            });
        }
        
        // Transform properties change
        ['posX', 'posY', 'posZ'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('change', () => {
                    const x = parseFloat(document.getElementById('posX').value);
                    const y = parseFloat(document.getElementById('posY').value);
                    const z = parseFloat(document.getElementById('posZ').value);
                    
                    object.position.set(x, y, z);
                    this.updateComponentPositionFromObject(object);
                    this.render();
                });
            }
        });
        
        ['rotX', 'rotY', 'rotZ'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('change', () => {
                    const x = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotX').value));
                    const y = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotY').value));
                    const z = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotZ').value));
                    
                    object.rotation.set(x, y, z);
                    this.updateComponentOrientationFromObject(object);
                    this.render();
                });
            }
        });
        
        ['scaleX', 'scaleY', 'scaleZ'].forEach(id => {
            const input = document.getElementById(id);
            if (input) {
                input.addEventListener('change', () => {
                    const x = parseFloat(document.getElementById('scaleX').value);
                    const y = parseFloat(document.getElementById('scaleY').value);
                    const z = parseFloat(document.getElementById('scaleZ').value);
                    
                    object.scale.set(x, y, z);
                    this.render();
                });
            }
        });
        
        // Component-specific properties change
        const componentData = vesselModel.getModelComponentData(object.uuid);
        if (componentData) {
            const componentType = componentData.type;
            const componentId = componentData.id;
            
            // Control Surface properties
            if (componentType === 'controlSurface') {
                const surfaceTypeSelect = document.getElementById('csSurfaceType');
                const surfaceAreaInput = document.getElementById('csSurfaceArea');
                const timeConstantInput = document.getElementById('csSurfaceTimeConstant');
                
                if (surfaceTypeSelect) {
                    surfaceTypeSelect.addEventListener('change', () => {
                        vesselModel.updateControlSurface(componentId, {
                            control_surface_type: surfaceTypeSelect.value
                        });
                    });
                }
                
                if (surfaceAreaInput) {
                    surfaceAreaInput.addEventListener('change', () => {
                        vesselModel.updateControlSurface(componentId, {
                            control_surface_area: parseFloat(surfaceAreaInput.value)
                        });
                    });
                }
                
                if (timeConstantInput) {
                    timeConstantInput.addEventListener('change', () => {
                        vesselModel.updateControlSurface(componentId, {
                            control_surface_T: parseFloat(timeConstantInput.value)
                        });
                    });
                }
            }
            
            // Thruster properties
            else if (componentType === 'thruster') {
                const thrusterNameInput = document.getElementById('thrusterName');
                const thrusterTypeSelect = document.getElementById('thrusterType');
                const thrusterDiameterInput = document.getElementById('thrusterDiameter');
                const thrusterCoefficientInput = document.getElementById('thrusterCoefficient');
                
                if (thrusterNameInput) {
                    thrusterNameInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            thruster_name: thrusterNameInput.value
                        });
                    });
                }
                
                if (thrusterTypeSelect) {
                    thrusterTypeSelect.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            thruster_type: thrusterTypeSelect.value
                        });
                    });
                }
                
                if (thrusterDiameterInput) {
                    thrusterDiameterInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            D_prop: parseFloat(thrusterDiameterInput.value)
                        });
                    });
                }
                
                if (thrusterCoefficientInput) {
                    thrusterCoefficientInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            T_prop: parseFloat(thrusterCoefficientInput.value)
                        });
                    });
                }
                
                const thrusterKTatJ0Input = document.getElementById('thrusterKTatJ0');
                if (thrusterKTatJ0Input) {
                    thrusterKTatJ0Input.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            KT_at_J0: parseFloat(thrusterKTatJ0Input.value)
                        });
                    });
                }
                
                const thrusterNMaxInput = document.getElementById('thrusterNMax');
                if (thrusterNMaxInput) {
                    thrusterNMaxInput.addEventListener('change', () => {
                        vesselModel.updateThruster(componentId, {
                            n_max: parseFloat(thrusterNMaxInput.value)
                        });
                    });
                }
            }
            
            // Sensor properties
            else if (componentType === 'sensor') {
                const sensorTypeSelect = document.getElementById('sensorType');
                const sensorPublishRateInput = document.getElementById('sensorPublishRate');
                
                if (sensorTypeSelect) {
                    sensorTypeSelect.addEventListener('change', () => {
                        vesselModel.updateSensor(componentId, {
                            sensor_type: sensorTypeSelect.value
                        });
                    });
                }
                
                if (sensorPublishRateInput) {
                    sensorPublishRateInput.addEventListener('change', () => {
                        vesselModel.updateSensor(componentId, {
                            publish_rate: parseFloat(sensorPublishRateInput.value)
                        });
                    });
                }
            }
        }
        
        // Apply changes button
        const applyButton = document.getElementById('btnApplyComponentChanges');
        if (applyButton) {
            applyButton.addEventListener('click', () => {
                // Update transform
                const x = parseFloat(document.getElementById('posX').value);
                const y = parseFloat(document.getElementById('posY').value);
                const z = parseFloat(document.getElementById('posZ').value);
                object.position.set(x, y, z);
                
                const rotX = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotX').value));
                const rotY = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotY').value));
                const rotZ = THREE.MathUtils.degToRad(parseFloat(document.getElementById('rotZ').value));
                object.rotation.set(rotX, rotY, rotZ);
                
                const scaleX = parseFloat(document.getElementById('scaleX').value);
                const scaleY = parseFloat(document.getElementById('scaleY').value);
                const scaleZ = parseFloat(document.getElementById('scaleZ').value);
                object.scale.set(scaleX, scaleY, scaleZ);
                
                // Update component position & orientation if applicable
                this.updateComponentPositionFromObject(object);
                this.updateComponentOrientationFromObject(object);
                
                // Refresh view
                this.render();
            });
        }
        
        // Remove component button
        const removeButton = document.getElementById('btnRemoveComponent');
        if (removeButton) {
            removeButton.addEventListener('click', () => {
                const componentData = vesselModel.getModelComponentData(object.uuid);
                if (componentData) {
                    const componentType = componentData.type;
                    const componentId = componentData.id;
                    
                    // Remove the component
                    if (componentType === 'controlSurface') {
                        vesselModel.removeControlSurface(componentId);
                    } else if (componentType === 'thruster') {
                        vesselModel.removeThruster(componentId);
                    } else if (componentType === 'sensor') {
                        vesselModel.removeSensor(componentId);
                    }
                    
                    // Remove the mapping
                    vesselModel.unmapModelComponent(object.uuid);
                    
                    // Update UI
                    componentTypeSelect.value = 'none';
                    removeButton.style.display = 'none';
                    
                    // Refresh component settings
                    this.showComponentSettings(object);
                }
            });
        }
    }

    updateComponentPositionFromObject(object) {
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) return;
        
        // First determine the component ID and type
        let componentId, componentType;
        
        // Get from object's userData
        if (object.userData.componentId && object.userData.componentType) {
            componentId = object.userData.componentId;
            componentType = object.userData.componentType;
        } 
        // Or try to get from mapping
        else {
        const componentData = vesselModel.getModelComponentData(object.uuid);
            if (componentData) {
                componentId = componentData.id;
                componentType = componentData.type;
            }
        }
        
        if (!componentId || !componentType) {
            console.warn('Cannot update position: Missing component ID or type');
            return;
        }
        
        // FIXED: First look for axes to use their position instead of the object's
        const axes = object.children.find(child => child.userData.isComponentAxes);
        let worldPosition = new THREE.Vector3();
        
        // If axes exist, use them for position
        if (axes) {
            axes.getWorldPosition(worldPosition);
            console.log(`Using axes position for ${componentType} ${componentId}`);
        } else {
            // Fallback to component position if no axes found
            object.getWorldPosition(worldPosition);
            console.log(`No axes found, using object position for ${componentType} ${componentId}`);
        }
        
        const position = [worldPosition.x, worldPosition.y, worldPosition.z];
        
        console.log(`Updating ${componentType} ${componentId} position:`, position);
        
        // Create update data with the appropriate property name
        let updateData = {};
        let locationKey;
        
        switch (componentType) {
            case 'controlSurface':
                locationKey = 'control_surface_location';
                break;
            case 'thruster':
                locationKey = 'thruster_location';
                break;
            case 'sensor':
                locationKey = 'sensor_location';
                break;
            default:
                console.error('Unknown component type:', componentType);
                return;
        }
        
        updateData[locationKey] = position;
        
        // Update the component
        let success = false;
        try {
            switch (componentType) {
                case 'controlSurface':
                    success = vesselModel.updateControlSurface(componentId, updateData);
                    break;
                case 'thruster':
                    success = vesselModel.updateThruster(componentId, updateData);
                    break;
                case 'sensor':
                    success = vesselModel.updateSensor(componentId, updateData);
                    break;
            }
            
            if (success) {
                console.log(`Successfully updated ${componentType} ${componentId} position`);
            } else {
                console.error(`Failed to update ${componentType} ${componentId} position`);
            }
        } catch (error) {
            console.error(`Error updating ${componentType} ${componentId} position:`, error);
        }
    }

    updateComponentOrientationFromObject(object) {
        const vesselModel = window.currentVesselModel;
        if (!vesselModel) return;
        
        // First determine the component ID and type
        let componentId, componentType;
        
        // Get from object's userData
        if (object.userData.componentId && object.userData.componentType) {
            componentId = object.userData.componentId;
            componentType = object.userData.componentType;
        } 
        // Or try to get from mapping
        else {
        const componentData = vesselModel.getModelComponentData(object.uuid);
            if (componentData) {
                componentId = componentData.id;
                componentType = componentData.type;
            }
        }
        
        if (!componentId || !componentType) {
            console.warn('Cannot update orientation: Missing component ID or type');
            return;
        }
        
        // FIXED: First look for axes to use their orientation instead of the object's
        const axes = object.children.find(child => child.userData.isComponentAxes);
        let worldQuaternion = new THREE.Quaternion();
        
        // If axes exist, use them for orientation
        if (axes) {
            axes.getWorldQuaternion(worldQuaternion);
            console.log(`Using axes orientation for ${componentType} ${componentId}`);
        } else {
            // Fallback to component orientation if no axes found
            object.getWorldQuaternion(worldQuaternion);
            console.log(`No axes found, using object orientation for ${componentType} ${componentId}`);
        }
        
        // Convert to appropriate format based on component type
        let orientation;
        if (componentType === 'sensor') {
            // For sensors, use quaternion
            orientation = [
                worldQuaternion.w,
                worldQuaternion.x,
                worldQuaternion.y,
                worldQuaternion.z
            ];
        } else {
            // For other components, use Euler angles
            const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
            orientation = [worldEuler.x, worldEuler.y, worldEuler.z];
        }
        
        console.log(`Updating ${componentType} ${componentId} orientation:`, orientation);
        
        // Create update data with the appropriate property name
        let updateData = {};
        let orientationKey;
        
        switch (componentType) {
            case 'controlSurface':
                orientationKey = 'control_surface_orientation';
                break;
            case 'thruster':
                orientationKey = 'thruster_orientation';
                break;
            case 'sensor':
                orientationKey = 'sensor_orientation';
                break;
            default:
                console.error('Unknown component type:', componentType);
                return;
        }
        
        updateData[orientationKey] = orientation;
        
        // Update the component
        let success = false;
        try {
            switch (componentType) {
                case 'controlSurface':
                    success = vesselModel.updateControlSurface(componentId, updateData);
                    break;
                case 'thruster':
                    success = vesselModel.updateThruster(componentId, updateData);
                    break;
                case 'sensor':
                    success = vesselModel.updateSensor(componentId, updateData);
                    break;
            }
            
            if (success) {
                console.log(`Successfully updated ${componentType} ${componentId} orientation`);
            } else {
                console.error(`Failed to update ${componentType} ${componentId} orientation`);
            }
        } catch (error) {
            console.error(`Error updating ${componentType} ${componentId} orientation:`, error);
        }
    }

    // Selection methods
    setTransformMode(mode) {
        if (this.transformControls && mode) {
            this.transformControls.setMode(mode);
        }
    }

    // Add performance monitoring if needed
    addPerformanceMonitor() {
        // Import Stats.js dynamically
        const script = document.createElement('script');
        script.onload = () => {
            this.stats = new Stats();
            this.stats.showPanel(0); // 0: fps, 1: ms, 2: mb, 3+: custom
            document.body.appendChild(this.stats.dom);
            console.log('Performance monitor enabled');
        };
        script.src = 'https://cdnjs.cloudflare.com/ajax/libs/stats.js/r17/Stats.min.js';
        document.head.appendChild(script);
    }

    // Implement batched updates for transform inputs
    updateTransformFromInputs(object, position, rotation) {
        if (!object) return;
        
        // Update in a single batch to minimize renders
        object.position.set(position.x, position.y, position.z);
        object.rotation.set(
            THREE.MathUtils.degToRad(rotation.x),
            THREE.MathUtils.degToRad(rotation.y),
            THREE.MathUtils.degToRad(rotation.z)
        );
        
        // Trigger a single render
        this.render();
    }

    // Initialize event listeners
    initializeEventListeners() {
        // Window resize event - debounced for performance
        let resizeTimeout;
        window.addEventListener('resize', () => {
            if (resizeTimeout) clearTimeout(resizeTimeout);
            resizeTimeout = setTimeout(() => {
                this.updateRendererSize();
            }, 100);
        });
        
        // Mouse events for object selection
        this.renderer.domElement.addEventListener('mousemove', (event) => {
            // Calculate mouse position in normalized device coordinates
            const rect = this.renderer.domElement.getBoundingClientRect();
            this.mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
            this.mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;
        });
        
        this.renderer.domElement.addEventListener('click', this.onClick.bind(this));
        
        // Transform controls mode switching
        window.addEventListener('keydown', (event) => {
            // Only process if the 3D view is active (not typing in an input field)
            if (event.target.tagName === 'INPUT' || event.target.tagName === 'TEXTAREA') {
                return;
            }
            
            switch (event.key) {
                case 'g': // Translate
                    this.setTransformMode('translate');
                    break;
                case 'r': // Rotate
                    this.setTransformMode('rotate');
                    break;
                case 's': // Scale
                    this.setTransformMode('scale');
                    break;
                case 'Escape': // Deselect
                    this.selectObject(null);
                    break;
            }
        });
        
        // Optimize rendering during scroll events
        this.renderer.domElement.addEventListener('wheel', () => {
            this.needsRender = true;
        });
        
        // Handle low-level GL context loss
        this.renderer.domElement.addEventListener('webglcontextlost', (event) => {
            event.preventDefault();
            console.warn('WebGL context lost. Trying to restore...');
        });
        
        this.renderer.domElement.addEventListener('webglcontextrestored', () => {
            console.log('WebGL context restored');
            this.forceRender();
        });
    }

    // Set the enabled state of the controls
    setControlsEnabled(enabled) {
        if (this.orbitControls) {
            this.orbitControls.enabled = enabled;
        }
    }

    // Update the target position of orbit controls
    updateOrbitTarget(position) {
        if (this.orbitControls) {
            this.orbitControls.target.copy(position);
            this.orbitControls.update();
            this.render();
        }
    }

    // Method to get an object by property (e.g., uuid)
    getObjectByProperty(name, value) {
        if (!this.scene) {
            console.error('Scene not initialized');
            return null;
        }
        
        // Check if the scene itself has the property
        if (this.scene[name] === value) {
            return this.scene;
        }
        
        // Traverse all objects in the scene
        let result = null;
        this.scene.traverse(function(object) {
            if (object[name] === value && !result) {
                result = object;
            }
        });
        
        return result;
    }

    // Add coordinate axes to a component
    addComponentAxes(component, size = 0.15) {
        if (!component) {
            console.error('Cannot add axes to undefined component');
            return null;
        }
        
        // First, attempt to get the component ID and type
        let componentId = null;
        let componentType = null;
        
        // Try to get directly from component's userData
        if (component.userData.componentId && component.userData.componentType) {
            componentId = component.userData.componentId;
            componentType = component.userData.componentType;
            console.log(`Using component userData for axes: ID=${componentId}, Type=${componentType}`);
        } 
        // Try to get from vessel model mapping
        else if (window.currentVesselModel) {
            const mappedData = window.currentVesselModel.getModelComponentData(component.uuid);
            if (mappedData && mappedData.id && mappedData.type) {
                componentId = mappedData.id;
                componentType = mappedData.type;
                
                // Update the component userData too since it was missing
                component.userData.componentId = componentId;
                component.userData.componentType = componentType;
                console.log(`Updated component from model mapping: ID=${componentId}, Type=${componentType}`);
            }
        }
        
        // If still not found, try to determine from name
        if (!componentId || !componentType) {
            console.warn('Component missing ID or type when creating axes:', component.name);
            
            const name = component.name || '';
            // Parse from control surface name format
            if (name.includes('Control Surface')) {
                const idMatch = name.match(/Control Surface (\d+)/);
                if (idMatch && idMatch[1]) {
                    componentId = idMatch[1];
                    componentType = 'controlSurface';
                    console.log(`Recovered from name: ID=${componentId}, Type=${componentType}`);
                    
                    // Update component userData
                    component.userData.componentId = componentId;
                    component.userData.componentType = componentType;
                }
            } 
            // Parse from thruster name format
            else if (name.includes('Thruster')) {
                const idMatch = name.match(/Thruster (\d+)/);
                if (idMatch && idMatch[1]) {
                    componentId = idMatch[1];
                    componentType = 'thruster';
                    console.log(`Recovered from name: ID=${componentId}, Type=${componentType}`);
                    
                    // Update component userData
                    component.userData.componentId = componentId;
                    component.userData.componentType = componentType;
                }
            } 
            // Parse from sensor name format
            else if (name.includes('Sensor')) {
                const idMatch = name.match(/Sensor (\d+)/);
                if (idMatch && idMatch[1]) {
                    componentId = idMatch[1];
                    componentType = 'sensor';
                    console.log(`Recovered from name: ID=${componentId}, Type=${componentType}`);
                    
                    // Update component userData
                    component.userData.componentId = componentId;
                    component.userData.componentType = componentType;
                }
            }
        }
        
        // Check for existing axes
        const existingAxes = component.children.find(child => child.userData.isComponentAxes);
        if (existingAxes) {
            console.log('Component already has axes, returning existing axes');
            return existingAxes;
        }
        
        // Create axes group
        const axesGroup = new THREE.Group();
        axesGroup.name = 'componentAxes';
        
        // Set properties to indicate this is a component axes object
        axesGroup.userData.isComponentAxes = true;
        
        // Set component ID and type if we have them
        if (componentId && componentType) {
            axesGroup.userData.componentId = componentId;
            axesGroup.userData.componentType = componentType;
            console.log(`Set axes userData: ID=${componentId}, Type=${componentType}`);
        } else {
            console.error(`Failed to determine component ID and type for: ${component.name}`);
        }
        
        // Calculate component dimensions to determine appropriate axes size
        let componentSize = 0.15; // Default fallback size
        
        // Get component dimensions
        if (component.geometry) {
            // Use bounding box to determine size
            component.geometry.computeBoundingBox();
            const boundingBox = component.geometry.boundingBox;
            const width = boundingBox.max.x - boundingBox.min.x;
            const height = boundingBox.max.y - boundingBox.min.y;
            const depth = boundingBox.max.z - boundingBox.min.z;
            
            // Use the largest dimension to calculate a proportional axes size
            const maxDimension = Math.max(width, height, depth);
            componentSize = maxDimension * 0.5; // 50% of the largest dimension
            
            // Clamp the size to reasonable bounds
            componentSize = Math.max(0.05, Math.min(componentSize, 0.5));
        }
        
        // Create axes
        const axisLength = componentSize;
        const axisRadius = componentSize * 0.05;
        
        // Add a center sphere to visually indicate the pivot point
        const centerSphereGeometry = new THREE.SphereGeometry(axisRadius * 1.5, 8, 8);
        const centerSphereMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
        const centerSphere = new THREE.Mesh(centerSphereGeometry, centerSphereMaterial);
        
        // X axis (red) - forward
        const xAxisGeometry = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 8);
        xAxisGeometry.rotateZ(-Math.PI / 2); // Rotate to align with X axis
        const xAxisMaterial = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const xAxis = new THREE.Mesh(xAxisGeometry, xAxisMaterial);
        xAxis.position.set(axisLength / 2, 0, 0);
        
        // X axis cone
        const xConeGeometry = new THREE.ConeGeometry(axisRadius * 2, axisLength * 0.2, 8);
        xConeGeometry.rotateZ(-Math.PI / 2); // Rotate to align with X axis
        const xCone = new THREE.Mesh(xConeGeometry, xAxisMaterial);
        xCone.position.set(axisLength, 0, 0);
        
        // Y axis (green) - up
        const yAxisGeometry = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 8);
        const yAxisMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        const yAxis = new THREE.Mesh(yAxisGeometry, yAxisMaterial);
        yAxis.position.set(0, axisLength / 2, 0);
        
        // Y axis cone
        const yConeGeometry = new THREE.ConeGeometry(axisRadius * 2, axisLength * 0.2, 8);
        const yCone = new THREE.Mesh(yConeGeometry, yAxisMaterial);
        yCone.position.set(0, axisLength, 0);
        
        // Z axis (blue) - right
        const zAxisGeometry = new THREE.CylinderGeometry(axisRadius, axisRadius, axisLength, 8);
        zAxisGeometry.rotateX(Math.PI / 2); // Rotate to align with Z axis
        const zAxisMaterial = new THREE.MeshBasicMaterial({ color: 0x0000ff });
        const zAxis = new THREE.Mesh(zAxisGeometry, zAxisMaterial);
        zAxis.position.set(0, 0, axisLength / 2);
        
        // Z axis cone
        const zConeGeometry = new THREE.ConeGeometry(axisRadius * 2, axisLength * 0.2, 8);
        zConeGeometry.rotateX(Math.PI / 2); // Rotate to align with Z axis
        const zCone = new THREE.Mesh(zConeGeometry, zAxisMaterial);
        zCone.position.set(0, 0, axisLength);
        
        // Add axis labels
        const labelSize = componentSize * 0.3;
        const labelOffset = axisLength * 1.2;
        
        // X label
        const xSprite = this.createTextSprite('X', labelSize, 0xff0000);
        xSprite.position.set(labelOffset, 0, 0);
        
        // Y label
        const ySprite = this.createTextSprite('Y', labelSize, 0x00ff00);
        ySprite.position.set(0, labelOffset, 0);
        
        // Z label
        const zSprite = this.createTextSprite('Z', labelSize, 0x0000ff);
        zSprite.position.set(0, 0, labelOffset);
        
        // Add all elements to group
        axesGroup.add(centerSphere, xAxis, xCone, yAxis, yCone, zAxis, zCone, xSprite, ySprite, zSprite);
        
        // Calculate the center of the component
        let center = new THREE.Vector3();
        
        if (component.geometry) {
            // If component has geometry, use its bounding box center
            component.geometry.computeBoundingBox();
            component.geometry.boundingBox.getCenter(center);
            // This gives us the center in local coordinates, which is what we want
        }
        
        // Position the axes at the component's local center
        axesGroup.position.copy(center);
        
        // Make axes selectable and transformable
        axesGroup.userData.selectable = true;
        axesGroup.userData.transformable = true;
        
        // IMPORTANT: Store the component's center as the pivot point for rotations
        axesGroup.userData.pivotPoint = center.clone();
        
        // Add axes group to component
        component.add(axesGroup);
        
        // Force a render to show the change
        this.render();
        
        return axesGroup;
    }
    
    // Create a text sprite for labels
    createTextSprite(text, size = 0.1, color = 0xffffff) {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 128;
        canvas.height = 128;
        
        // Background - transparent
        context.fillStyle = 'rgba(0, 0, 0, 0)';
        context.fillRect(0, 0, canvas.width, canvas.height);
        
        // Text
        context.font = 'Bold 80px Arial';
        context.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(text, canvas.width / 2, canvas.height / 2);
        
        // Create sprite
        const texture = new THREE.CanvasTexture(canvas);
        const material = new THREE.SpriteMaterial({ map: texture });
        const sprite = new THREE.Sprite(material);
        sprite.scale.set(size, size, 1);
        
        return sprite;
    }

    // Add method to extract dimensions from FBX model
    getModelDimensions(object) {
        if (!object) {
            console.error('Cannot get dimensions: No object provided');
            return null;
        }
        
        try {
            // Calculate the bounding box
            const boundingBox = new THREE.Box3().setFromObject(object);
            const size = new THREE.Vector3();
            boundingBox.getSize(size);
            
            // Get min and max points - Box3 has min and max properties, not getMin/getMax methods
            const min = boundingBox.min;
            const max = boundingBox.max;
            
            console.log(`FBX Bounding Box: Min(${min.x.toFixed(2)}, ${min.y.toFixed(2)}, ${min.z.toFixed(2)}), Max(${max.x.toFixed(2)}, ${max.y.toFixed(2)}, ${max.z.toFixed(2)})`);
            console.log(`FBX Model dimensions: X: ${size.x.toFixed(2)}m, Y: ${size.y.toFixed(2)}m, Z: ${size.z.toFixed(2)}m`);
            
            // Determine which dimension is which (based on relative sizes)
            const dimensions = [size.x, size.y, size.z];
            const axes = ['x', 'y', 'z'];
            
            // Sort dimensions from largest to smallest
            const sorted = dimensions.map((val, idx) => ({val, axis: axes[idx]}))
                                 .sort((a, b) => b.val - a.val);
            
            // Assign dimensions based on the sorted order
            const length = sorted[0].val;
            const lengthAxis = sorted[0].axis;
            
            const breadth = sorted[1].val;
            const breadthAxis = sorted[1].axis;
            
            const depth = sorted[2].val;
            const depthAxis = sorted[2].axis;
            
            console.log(`Assigned FBX dimensions: Length: ${length.toFixed(2)}m (${lengthAxis}-axis), Breadth: ${breadth.toFixed(2)}m (${breadthAxis}-axis), Depth: ${depth.toFixed(2)}m (${depthAxis}-axis)`);
            
            // Get the center of the bounding box
            const center = new THREE.Vector3();
            boundingBox.getCenter(center);
            
            return {
                length,
                breadth, 
                depth,
                axes: {
                    length: lengthAxis,
                    breadth: breadthAxis,
                    depth: depthAxis
                },
                boundingBox: {
                    min: [min.x, min.y, min.z],
                    max: [max.x, max.y, max.z],
                    center: [center.x, center.y, center.z]
                }
            };
        } catch (error) {
            console.error('Error calculating model dimensions:', error);
            return null;
        }
    }

    // Add center points visualization and manipulation methods
    createCenterPoint(type, position, color) {
        // Create a visual representation of a center point
        const geometry = new THREE.SphereGeometry(0.1, 16, 16);
        const material = new THREE.MeshStandardMaterial({
            color: color,
            emissive: color,
            emissiveIntensity: 0.3,
            transparent: true,
            opacity: 0.8
        });
        
        const centerPoint = new THREE.Mesh(geometry, material);
        centerPoint.position.set(position[0], position[1], position[2]);
        centerPoint.userData.type = type;
        centerPoint.userData.centerType = type;
        centerPoint.userData.selectable = true;
        centerPoint.name = `${type}_Point`;
        
        // Add small axes to the center point for orientation
        const axes = this.createLocalAxes(0.2);
        centerPoint.add(axes);
        
        // Add text label
        const typeLabel = this.createTextSprite(type, 0.2, color);
        typeLabel.position.set(0, 0.2, 0);
        centerPoint.add(typeLabel);
        
        // Store reference
        this[`${type.toLowerCase()}Point`] = centerPoint;
        
        return centerPoint;
    }

    updateCenterPointsVisibility(visible) {
        // Update visibility of all center points
        const centerPoints = ['CO', 'CG', 'CB'];
        
        for (const type of centerPoints) {
            const pointKey = `${type.toLowerCase()}Point`;
            if (this[pointKey]) {
                this[pointKey].visible = visible;
            }
        }
        
        this.render();
    }

    updateCenterPoint(type, position, orientation) {
        // Update the position of a center point
        const pointKey = `${type.toLowerCase()}Point`;
        if (this[pointKey]) {
            this[pointKey].position.set(position[0], position[1], position[2]);
            
            // Update vessel model
            if (window.currentVesselModel && window.currentVesselModel.config) {
                // Update legacy format
                window.currentVesselModel.config.geometry[type] = [
                    position[0],
                    position[1],
                    position[2]
                ];
                
                // Update new format with position and orientation
                window.currentVesselModel.config.geometry[`${type}_position`] = [
                    position[0],
                    position[1],
                    position[2]
                ];
                
                // Update orientation if provided
                if (orientation && Array.isArray(orientation) && orientation.length === 3) {
                    // Convert orientation to radians for storage in the YAML file
                    const orientationInRadians = [
                        THREE.MathUtils.degToRad(orientation[0]),
                        THREE.MathUtils.degToRad(orientation[1]),
                        THREE.MathUtils.degToRad(orientation[2])
                    ];
                    
                    // Store radians in the model for YAML export
                    window.currentVesselModel.config.geometry[`${type}_orientation`] = orientationInRadians;
                    
                    // Apply rotation to the 3D object (Three.js uses radians)
                    const rotX = THREE.MathUtils.degToRad(orientation[0]);
                    const rotY = THREE.MathUtils.degToRad(orientation[1]);
                    const rotZ = THREE.MathUtils.degToRad(orientation[2]);
                    this[pointKey].rotation.set(rotX, rotY, rotZ);
                }
                
                // Update UI if geometry modal is open
                this.updateCenterPointUI(type, position, orientation);
            }
            
            this.render();
        }
    }

    updateCenterPointUI(type, position, orientation) {
        // Update the UI form fields if geometry modal is open
        const modalElement = document.getElementById('geometryModal');
        if (modalElement && modalElement.classList.contains('show')) {
            // Update position fields
            const xField = document.getElementById(`${type.toLowerCase()}X`);
            const yField = document.getElementById(`${type.toLowerCase()}Y`);
            const zField = document.getElementById(`${type.toLowerCase()}Z`);
            
            if (xField && yField && zField) {
                xField.value = position[0].toFixed(2);
                yField.value = position[1].toFixed(2);
                zField.value = position[2].toFixed(2);
            }
            
            // Update orientation fields if provided
            if (orientation && Array.isArray(orientation)) {
                const rollField = document.getElementById(`${type.toLowerCase()}Roll`);
                const pitchField = document.getElementById(`${type.toLowerCase()}Pitch`);
                const yawField = document.getElementById(`${type.toLowerCase()}Yaw`);
                
                if (rollField && pitchField && yawField) {
                    rollField.value = orientation[0].toFixed(2);
                    pitchField.value = orientation[1].toFixed(2);
                    yawField.value = orientation[2].toFixed(2);
                }
            }
        }
    }

    setupCenterPoints() {
        // Set up center points (CO, CG, CB) visualization and controls
        if (!this.vessel) return;
        
        // Remove existing center points
        if (this.coPoint) this.scene.remove(this.coPoint);
        if (this.cgPoint) this.scene.remove(this.cgPoint);
        if (this.cbPoint) this.scene.remove(this.cbPoint);
        
        // Get geometry data from the model
        const geometry = window.currentVesselModel.config.geometry;
        
        // Create CO point (Center of Origin) - Red
        const coPosition = geometry.CO_position || geometry.CO || [0, 0, 0];
        const coOrientation = geometry.CO_orientation || [0, 0, 0];
        this.coPoint = this.createCenterPoint('CO', coPosition, 0xff4444);
        if (coOrientation && coOrientation.every(val => val !== 0)) {
            // Apply rotation directly if values are already in radians
            this.coPoint.rotation.set(coOrientation[0], coOrientation[1], coOrientation[2]);
        }
        this.scene.add(this.coPoint);
        
        // Create CG point (Center of Gravity) - Green
        const cgPosition = geometry.CG_position || geometry.CG || [0, 0, 0];
        const cgOrientation = geometry.CG_orientation || [0, 0, 0];
        this.cgPoint = this.createCenterPoint('CG', cgPosition, 0x44ff44);
        if (cgOrientation && cgOrientation.every(val => val !== 0)) {
            // Apply rotation directly if values are already in radians
            this.cgPoint.rotation.set(cgOrientation[0], cgOrientation[1], cgOrientation[2]);
        }
        this.scene.add(this.cgPoint);
        
        // Create CB point (Center of Buoyancy) - Blue
        const cbPosition = geometry.CB_position || geometry.CB || [0, 0, 0];
        const cbOrientation = geometry.CB_orientation || [0, 0, 0];
        this.cbPoint = this.createCenterPoint('CB', cbPosition, 0x4444ff);
        if (cbOrientation && cbOrientation.every(val => val !== 0)) {
            // Apply rotation directly if values are already in radians
            this.cbPoint.rotation.set(cbOrientation[0], cbOrientation[1], cbOrientation[2]);
        }
        this.scene.add(this.cbPoint);
        
        // Initial visibility setting
        this.updateCenterPointsVisibility(true);
        
        this.render();
    }

    // When a center point is transformed, update the model
    onCenterPointTransformed(object) {
        if (object && object.userData && object.userData.centerType) {
            const type = object.userData.centerType;
            const position = [
                object.position.x,
                object.position.y,
                object.position.z
            ];
            
            // Extract rotation in degrees for UI display
            const orientationDegrees = [
                THREE.MathUtils.radToDeg(object.rotation.x),
                THREE.MathUtils.radToDeg(object.rotation.y),
                THREE.MathUtils.radToDeg(object.rotation.z)
            ];
            
            // Store orientation in radians for YAML export
            const orientationRadians = [
                object.rotation.x,
                object.rotation.y,
                object.rotation.z
            ];
            
            // Update the model data
            if (window.currentVesselModel && window.currentVesselModel.config) {
                // Update legacy format
                window.currentVesselModel.config.geometry[type] = position;
                
                // Update new format
                window.currentVesselModel.config.geometry[`${type}_position`] = position;
                window.currentVesselModel.config.geometry[`${type}_orientation`] = orientationRadians;
                
                // Update UI (using degrees for display)
                this.updateCenterPointUI(type, position, orientationDegrees);
                
                console.log(`${type} position updated to:`, position);
                console.log(`${type} orientation updated to (radians):`, orientationRadians);
            }
        }
    }
    
    // Add method to show center point info
    showCenterPointInfo(object) {
        if (!object || !object.userData || !object.userData.centerType) return;
        
        const type = object.userData.centerType;
        const position = [
            object.position.x.toFixed(2),
            object.position.y.toFixed(2),
            object.position.z.toFixed(2)
        ];
        
        // Get orientation in radians
        const orientationRad = [
            object.rotation.x.toFixed(4),
            object.rotation.y.toFixed(4),
            object.rotation.z.toFixed(4)
        ];
        
        // Get orientation in degrees for display
        const orientationDeg = [
            THREE.MathUtils.radToDeg(object.rotation.x).toFixed(2),
            THREE.MathUtils.radToDeg(object.rotation.y).toFixed(2),
            THREE.MathUtils.radToDeg(object.rotation.z).toFixed(2)
        ];
        
        // Update the component info panel in the right sidebar
        const componentInfoPanel = document.getElementById('component-info');
        if (componentInfoPanel) {
            componentInfoPanel.innerHTML = `
                <div class="panel-header">
                    <h5>${type} Point</h5>
                </div>
                <div class="panel-body">
                    <div class="info-item">
                        <strong>Type:</strong> ${type === 'CO' ? 'Center of Origin' : 
                                         type === 'CG' ? 'Center of Gravity' : 
                                         'Center of Buoyancy'}
                    </div>
                    <div class="info-item">
                        <strong>Position:</strong> [${position.join(', ')}]
                    </div>
                    <div class="info-item">
                        <strong>Orientation (rad):</strong> [${orientationRad.join(', ')}]
                    </div>
                    <div class="info-item">
                        <strong>Orientation (deg):</strong> [${orientationDeg.join(', ')}]
                    </div>
                </div>
            `;
        }
    }

    // Add method to clear component info
    clearComponentInfo() {
        const componentInfoPanel = document.getElementById('component-info');
        if (componentInfoPanel) {
            componentInfoPanel.innerHTML = '<div class="no-selection-message">Select a component to view its properties</div>';
        }
    }

    addGlobalAxesHelper() {
        // Create a div to hold the axes helper
        const axesContainer = document.createElement('div');
        axesContainer.id = 'global-axes-container';
        axesContainer.style.position = 'absolute';
        axesContainer.style.bottom = '20px';
        axesContainer.style.right = '20px';
        axesContainer.style.width = '80px';
        axesContainer.style.height = '80px';
        axesContainer.style.pointerEvents = 'none'; // Don't interfere with mouse events
        axesContainer.style.zIndex = '10'; // Make sure it's above the main renderer
        this.container.appendChild(axesContainer);
        
        // Create a separate renderer for the axes
        this.axesRenderer = new THREE.WebGLRenderer({ 
            alpha: true,
            antialias: true 
        });
        this.axesRenderer.setClearColor(0x000000, 0);
        this.axesRenderer.setSize(80, 80);
        axesContainer.appendChild(this.axesRenderer.domElement);
        
        // Create a separate scene for the axes
        this.axesScene = new THREE.Scene();
        
        // Create a separate camera for the axes
        this.axesCamera = new THREE.PerspectiveCamera(50, 1, 0.1, 200);
        this.axesCamera.position.set(0, 0, 30);
        this.axesCamera.lookAt(0, 0, 0);
        
        // Create the axes helper
        const axesHelper = new THREE.AxesHelper(20);
        this.axesScene.add(axesHelper);
        
        // Add labels for axes
        const addLabel = (text, position, color) => {
            const canvas = document.createElement('canvas');
            const context = canvas.getContext('2d');
            canvas.width = 64;
            canvas.height = 64;
            
            context.fillStyle = color;
            context.font = 'bold 48px Arial';
            context.fillText(text, 10, 48);
            
            const texture = new THREE.CanvasTexture(canvas);
            const material = new THREE.SpriteMaterial({ map: texture });
            const sprite = new THREE.Sprite(material);
            sprite.scale.set(5, 5, 5);
            sprite.position.copy(position);
            
            this.axesScene.add(sprite);
        };
        
        // Add labels at the end of each axis
        addLabel('X', new THREE.Vector3(22, 0, 0), '#ff0000');
        addLabel('Y', new THREE.Vector3(0, 22, 0), '#00ff00');
        addLabel('Z', new THREE.Vector3(0, 0, 22), '#0000ff');
        
        // Override the renderAxes method for the animation loop
        this.renderAxes = () => {
            // Sync the axes camera rotation with the main camera
            this.axesCamera.position.copy(this.camera.position);
            this.axesCamera.position.sub(this.orbitControls.target);
            this.axesCamera.position.setLength(30);
            this.axesCamera.lookAt(0, 0, 0);
            
            // Render the axes
            this.axesRenderer.render(this.axesScene, this.axesCamera);
        };
    }

    // Apply styling to components based on their type
    applyComponentStyling(object, componentType) {
        if (!object || !componentType) return;
        
        // Store original material if not already saved
        if (!object.userData.originalMaterial) {
            if (Array.isArray(object.material)) {
                object.userData.originalMaterials = object.material.map(m => m.clone());
            } else {
                object.userData.originalMaterial = object.material.clone();
            }
        }
        
        // Create a new material for the component based on its type
        let color;
        let opacity = 0.9;
        let transparent = true;
        
        // Assign colors based on component type
        switch (componentType) {
            case 'controlSurface':
                color = 0x3498db; // Blue for control surfaces
                break;
            case 'thruster':
                color = 0xe74c3c; // Red for thrusters
                break;
            case 'sensor':
                color = 0x2ecc71; // Green for sensors
                break;
            default:
                color = 0xf39c12; // Orange for unknown types
        }
        
        // Apply new material
        if (Array.isArray(object.material)) {
            // For multi-material objects, apply color to all materials
            object.material = object.material.map(() => {
                return new THREE.MeshPhongMaterial({
                    color: color,
                    transparent: transparent,
                    opacity: opacity,
                    shininess: 80
                });
            });
        } else {
            // For single-material objects
            object.material = new THREE.MeshPhongMaterial({
                color: color,
                transparent: transparent,
                opacity: opacity,
                shininess: 80
            });
        }
        
        // Mark the object as a component of this type
        object.userData.componentType = componentType;
        object.userData.isComponentMapped = true;
        
        // Force a render to update the appearance
        this.render();
    }
    
    // Restore original material for an object
    restoreOriginalMaterial(object) {
        if (!object) return;
        
        if (Array.isArray(object.material) && object.userData.originalMaterials) {
            object.material = object.userData.originalMaterials.map(m => m.clone());
        } else if (object.userData.originalMaterial) {
            object.material = object.userData.originalMaterial.clone();
        }
        
        // Force a render to update the appearance
        this.render();
    }

    // Get an object's index in the loaded model (for reconnecting components after loading)
    getObjectIndex(object) {
        if (!object || !this.vessel) return -1;
        
        // Get all mesh objects in the vessel
        const meshes = [];
        this.vessel.traverse(child => {
            if (child.isMesh) {
                meshes.push(child);
            }
        });
        
        // Find the index of this object in the meshes array
        return meshes.findIndex(mesh => mesh.uuid === object.uuid);
    }
}

// Export the class
window.ThreeScene = ThreeScene; 