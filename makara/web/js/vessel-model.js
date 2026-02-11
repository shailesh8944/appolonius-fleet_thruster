/**
 * VesselModel.js - Core Data Model for Marine Vessel Simulation
 * 
 * This class represents the central data model for a marine vessel in the simulation.
 * It maintains all configuration parameters, physical properties, components, and 
 * simulation settings needed to define a complete vessel for hydrodynamic simulation.
 * 
 * The model supports various vessel types (AUV, USV, Ship, ROV) and manages:
 * - Vessel geometry and dimensions
 * - Inertia and mass properties
 * - Hydrodynamic coefficients
 * - Control surfaces (fins, rudders, etc.)
 * - Thrusters and propulsion
 * - Sensors and instrumentation
 * - Initial conditions and simulation parameters
 * - 3D model associations
 * 
 * The class includes methods for adding, updating, and removing components, 
 * as well as importing/exporting the configuration to JSON and generating
 * YAML files for simulation.
 */

class VesselModel {
    /**
     * Create a new vessel model with default parameters
     * @param {string} name - Vessel name
     * @param {string} type - Vessel type (auv, usv, ship, rov)
     */
    constructor(name = 'New Vessel', type = 'auv') {
        // Main configuration object containing all vessel parameters
        this.config = {
            name: name,
            type: type,
            geometry: {
                length: 1.0,                // Hull length (m)
                breadth: 0.5,               // Hull breadth/width (m)
                depth: 0.5,                 // Hull depth/height (m)
                CO: [0, 0, 0],              // Center of origin (x, y, z) - Legacy format
                CG: [0, 0, 0],              // Center of gravity (x, y, z) - Legacy format
                CB: [0, 0, 0],              // Center of buoyancy (x, y, z) - Legacy format
                CO_position: [0, 0, 0],     // Center of origin position (x, y, z)
                CO_orientation: [0, 0, 0],  // Center of origin orientation (roll, pitch, yaw)
                CG_position: [0, 0, 0],     // Center of gravity position (x, y, z)
                CG_orientation: [0, 0, 0],  // Center of gravity orientation (roll, pitch, yaw)
                CB_position: [0, 0, 0],     // Center of buoyancy position (x, y, z)
                CB_orientation: [0, 0, 0],  // Center of buoyancy orientation (roll, pitch, yaw)
                gyration: [0.5, 0.8, 0.8],  // Radii of gyration [kxx, kyy, kzz]
                scale: [1, 1, 1],           // Scale factors for visualization
                geometry_file: ''           // Path to GDF file
            },
            inertia: {
                mass: 50.0,                 // Mass in kg
                buoyancy_mass: 50.0,        // Buoyancy in kg (= mass for neutral buoyancy)
                inertia_matrix: [           // Inertia matrix (kg*m^2)
                    [5, 0, 0],
                    [0, 10, 0],
                    [0, 0, 10]
                ],
                added_mass_matrix: null     // Added mass matrix (calculated from hydrodynamics)
            },
            hydrodynamics: {
                hydra_file: '',             // Path to hydrodynamic analysis output
                dim_flag: false,            // Dimensional flag
                cross_flow_drag: false,     // Use cross-flow drag calculation
                X_u_u: -0.01,               // Surge quadratic drag coefficient
                Y_v_v: -0.01,               // Sway quadratic drag coefficient
                Y_r_r: 0.001,               // Yaw drag coefficient
                N_v_v: -0.001,              // Sway-yaw coupling coefficient
                N_r_r: -0.001,              // Yaw rate damping coefficient
                Z_w_w: -0.01                // Heave quadratic drag coefficient
            },
            control_surfaces: {
                naca_number: '0015',        // Default NACA airfoil profile
                naca_file: '',              // Path to NACA data file
                delta_max: 35.0,            // Maximum deflection angle (degrees)
                deltad_max: 1.0,            // Maximum deflection rate (degrees/s)
                area: 0.1,                  // Default control surface area (m²)
                control_surfaces: []        // Array of control surface objects
            },
            thrusters: {
                thrusters: []               // Array of thruster objects
            },
            initial_conditions: {
                start_velocity: [0, 0, 0, 0, 0, 0], // Initial [u, v, w, p, q, r]
                start_location: [0, 0, 0],          // Initial position [x, y, z]
                start_orientation: [0, 0, 0],       // Initial orientation [roll, pitch, yaw]
                use_quaternion: false               // Whether to use quaternions for orientation
            },
            sensors: {
                sensors: []                 // Array of sensor objects
            },
            guidance: {
                guidance_type: 'waypoint',  // Guidance system type
                waypoints: []               // Array of waypoint objects
            },
            control: {
                control_surface_control_type: 'fixed_rudder', // Control surface controller type
                thruster_control_type: 'fixed_rpm'            // Thruster controller type
            },
            simulation: {
                sim_time: 100,              // Simulation time (s)
                time_step: 0.01,            // Simulation time step (s)
                density: 1025,              // Water density (kg/m³)
                gravity: 9.80665,           // Gravity acceleration (m/s²)
                world_size: [1000, 1000, 100], // World dimensions [x, y, z] (m)
                gps_datum: [12.99300425860631, 80.23913114094384, -87.0] // Reference GPS [lat, lon, alt]
            }
        };

        // Track component IDs for auto-incrementing IDs
        this.nextControlSurfaceId = 1;
        this.nextThrusterId = 1;
        this.nextSensorId = 1;
        this.nextWaypointId = 1;
        
        // Track 3D model data for visualization
        this.modelData = {
            name: name,
            type: type,
            length: 1.0,
            width: 0.5,
            height: 0.5,
            mass: 50.0,
            controlSurfaces: [],
            thrusters: [],
            sensors: [],
            stlModel: null,
            componentMap: new Map()  // Maps 3D object UUIDs to component data
        };
    }
    
    /**
     * Set basic vessel information
     * @param {string} name - Vessel name
     * @param {string} type - Vessel type (auv, usv, ship, rov)
     */
    setBasicInfo(name, type) {
        if (!name || name.trim() === '') {
            throw new Error('Vessel name cannot be empty');
        }
        
        this.config.name = name.trim();
        
        if (type && ['auv', 'usv', 'ship', 'rov'].includes(type)) {
            this.config.type = type;
        }
        
    }

    /**
     * Update vessel dimensions and recalculate dependent values
     * @param {number} length - Hull length (m)
     * @param {number} breadth - Hull breadth/width (m)
     * @param {number} depth - Hull depth/height (m)
     * @returns {VesselModel} - Returns this model instance for chaining
     */
    updateDimensions(length, breadth, depth) {
        if (!this.config || !this.config.geometry) {
            console.error("Vessel model configuration is not properly initialized");
            return this;
        }
        
        // Validate dimensions
        if (parseFloat(length) <= 0 || parseFloat(breadth) <= 0 || parseFloat(depth) <= 0) {
            console.warn('Dimensions must be positive values');
            return this;
        }
        
        this.config.geometry.length = parseFloat(length);
        this.config.geometry.breadth = parseFloat(breadth);
        this.config.geometry.depth = parseFloat(depth);
        
        console.log(`Updated vessel dimensions: L=${length}m, B=${breadth}m, D=${depth}m`);
        
        // Update gyration radii based on dimensions
        if (typeof this.updateGyrationRadii === 'function') {
            this.updateGyrationRadii();
        }
        
        // Update inertia matrix
        if (typeof this.updateInertiaMatrix === 'function') {
            this.updateInertiaMatrix();
        }
        
        // Update any UI elements if needed
        if (typeof this.updateUIElements === 'function') {
            this.updateUIElements();
        }
        
    
        return this;
    }
    
    // Update gyration radii based on vessel type and dimensions
    updateGyrationRadii() {
        const { length, breadth, depth } = this.config.geometry;
        const type = this.config.type;
        
        if (type === 'auv' || type === 'usv') {
            // For AUVs and USVs, use more streamlined values
            this.config.geometry.gyration = [
                breadth * 0.35, // Roll gyration radius (kxx)
                length * 0.25,  // Pitch gyration radius (kyy)
                length * 0.25   // Yaw gyration radius (kzz)
            ];
        } else if (type === 'ship') {
            // For ships, use standard naval architecture approximations
            this.config.geometry.gyration = [
                breadth * 0.4,  // Roll gyration radius (kxx)
                length * 0.25,  // Pitch gyration radius (kyy)
                length * 0.25   // Yaw gyration radius (kzz)
            ];
        } else {
            // Default values for other vessel types
            this.config.geometry.gyration = [
                breadth * 0.35, // Roll gyration radius (kxx)
                length * 0.25,  // Pitch gyration radius (kyy)
                length * 0.25   // Yaw gyration radius (kzz)
            ];
        }
    }

    // Update inertia matrix based on mass and gyration radii
    updateInertiaMatrix() {
        const mass = this.config.inertia.mass;
        const [kxx, kyy, kzz] = this.config.geometry.gyration;
        
        // Calculate moments of inertia (I = m * k²)
        const Ixx = mass * kxx * kxx;
        const Iyy = mass * kyy * kyy;
        const Izz = mass * kzz * kzz;
        
        // Update inertia matrix (assuming no products of inertia)
        this.config.inertia.inertia_matrix = [
            [Ixx, 0, 0],
            [0, Iyy, 0],
            [0, 0, Izz]
        ];
    }

    // Update physical properties
    updatePhysicalProperties(mass, buoyancy_mass, CG, CB, CG_orientation, CB_orientation) {
        if (mass <= 0) {
            throw new Error('Mass must be positive');
        }
        
        this.config.inertia.mass = parseFloat(mass);
        
        if (buoyancy_mass !== undefined && buoyancy_mass > 0) {
            this.config.inertia.buoyancy_mass = parseFloat(buoyancy_mass);
        } else {
            // By default, make buoyancy mass equal to mass (neutrally buoyant)
            this.config.inertia.buoyancy_mass = this.config.inertia.mass;
        }
        
        // Update both legacy and new position formats
        if (CG && Array.isArray(CG) && CG.length === 3) {
            this.config.geometry.CG = CG.map(val => parseFloat(val));
            this.config.geometry.CG_position = CG.map(val => parseFloat(val));
        }
        
        if (CB && Array.isArray(CB) && CB.length === 3) {
            this.config.geometry.CB = CB.map(val => parseFloat(val));
            this.config.geometry.CB_position = CB.map(val => parseFloat(val));
        }
        
        // Update orientations if provided
        if (CG_orientation && Array.isArray(CG_orientation) && CG_orientation.length === 3) {
            this.config.geometry.CG_orientation = CG_orientation.map(val => parseFloat(val));
        }
        
        if (CB_orientation && Array.isArray(CB_orientation) && CB_orientation.length === 3) {
            this.config.geometry.CB_orientation = CB_orientation.map(val => parseFloat(val));
        }
        
        // Update inertia matrix with new mass
        this.updateInertiaMatrix();
    }

    // Update hydrodynamic coefficients
    updateHydrodynamics(coefficients) {
        console.log("Updating hydrodynamics with:", coefficients);
        
        // Preserve the hydra_file value
        const hydraFile = this.config.hydrodynamics.hydra_file || '';
        
        // Start with a fresh hydrodynamics object to remove any old coefficients
        this.config.hydrodynamics = {
            hydra_file: hydraFile
        };
        
        // Add all the new coefficients
        for (const [key, value] of Object.entries(coefficients)) {
            // Skip hydra_file as it's already handled
            if (key === 'hydra_file') continue;
            
            if (key === 'dim_flag' || key === 'cross_flow_drag') {
                this.config.hydrodynamics[key] = Boolean(value);
            } else {
                // Any other key is treated as a numeric coefficient
                this.config.hydrodynamics[key] = parseFloat(value);
            }
        }
        
        console.log("Updated hydrodynamics:", this.config.hydrodynamics);
        
    }

    // Set Hydra file path
    setHydraFile(filePath) {
        this.config.hydrodynamics.hydra_file = filePath;
        
    }

    // Update simulation parameters
    updateSimulationParams(params) {
        console.log("Updating simulation parameters with:", params);
        
        // For complete parameter objects, replace the entire simulation config
        if (params.fullReplace) {
            // Remove the fullReplace flag and assign the rest
            const { fullReplace, ...newParams } = params;
            this.config.simulation = newParams;
        } else {
            // For individual updates, only update the specified keys
            for (const [key, value] of Object.entries(params)) {
                this.config.simulation[key] = value;
            }
        }
        
        console.log("Updated simulation config:", this.config.simulation);
        
    }

    // Add a new control surface
    addControlSurface(type, location, orientation, area = 0.1, id = null, naca = '0015', timeConstant = 0.1, deltaMax = 35.0, deltadMax = 1.0) {
        if (!type || !location || !orientation) {
            throw new Error('Missing required parameters for control surface');
        }
        
        const surfaceId = id || this.nextControlSurfaceId++;
        
        const surface = {
            control_surface_type: type,
            control_surface_id: surfaceId,
            control_surface_location: location.map(val => parseFloat(val)),
            control_surface_orientation: orientation.map(val => parseFloat(val)),
            control_surface_area: parseFloat(area),
            control_surface_NACA: naca,
            control_surface_T: parseFloat(timeConstant),
            control_surface_delta_max: parseFloat(deltaMax),
            control_surface_deltad_max: parseFloat(deltadMax)
        };

        // Ensure control_surfaces structure exists
        if (!this.config.control_surfaces) {
            this.config.control_surfaces = {
                naca_number: naca,
                naca_file: '',
                delta_max: 35.0,
                deltad_max: 1.0,
                area: 0.1,
                control_surfaces: []
            };
        }

        this.config.control_surfaces.control_surfaces.push(surface);
        
        return surface.control_surface_id;
    }

    // Add a new thruster
    addThruster(name, location, orientation, id = null, diameter = 0.1, tProp = 1.0, tp = 0.1, jVsKtFile = 'J_vs_KT.csv', ktAtJ0 = 0.0, nMax = 2668) {
        if (!location || !orientation) {
            throw new Error('Missing required parameters for thruster');
        }
        
        const thrusterId = id || this.nextThrusterId++;
        const thrusterName = name || `thruster_${thrusterId}`;
        
        const thruster = {
            thruster_name: thrusterName,
            thruster_id: thrusterId,
            thruster_location: location.map(val => parseFloat(val)),
            thruster_orientation: orientation.map(val => parseFloat(val)),
            T_prop: parseFloat(tProp),
            D_prop: parseFloat(diameter),
            tp: parseFloat(tp),
            J_vs_KT: `${this.config.name}/J_vs_KT.csv`, // This will be updated with proper path in YAML generation
            KT_at_J0: parseFloat(ktAtJ0),  // Add KT_at_J0 parameter
            n_max: parseFloat(nMax)  // Add n_max parameter
        };

        // Ensure thrusters structure exists
        if (!this.config.thrusters) {
            this.config.thrusters = { thrusters: [] };
        }
        
        this.config.thrusters.thrusters.push(thruster);
        
        return thruster.thruster_id;
    }

    // Add a new sensor
    addSensor(type, location, orientation, publishRate = 10, useNoneLocation = false, useNoneOrientation = false) {
        if (!type) {
            throw new Error('Missing sensor type');
        }
        
        const sensor = {
            sensor_type: type,
            sensor_id: this.nextSensorId++,
            sensor_location: useNoneLocation ? 'None' : location.map(val => parseFloat(val)),
            sensor_orientation: useNoneOrientation ? 'None' : orientation.map(val => parseFloat(val)),
            publish_rate: parseFloat(publishRate)
        };

        // Ensure sensors structure exists
        if (!this.config.sensors) {
            this.config.sensors = { sensors: [] };
        }

        this.config.sensors.sensors.push(sensor);
        
        return sensor.sensor_id;
    }
    
    // Add a new waypoint
    addWaypoint(x, y, z, psi = 0) {
        const waypoint = {
            id: this.nextWaypointId++,
            x: parseFloat(x),
            y: parseFloat(y),
            z: parseFloat(z),
            psi: parseFloat(psi)
        };
        
        if (!this.config.guidance.waypoints) {
            this.config.guidance.waypoints = [];
        }
        
        this.config.guidance.waypoints.push(waypoint);
        
        return waypoint.id;
    }

    // Update initial conditions
    updateInitialConditions(startVelocity, startLocation, startOrientation, useQuaternion = false) {
        // Validate inputs
        if (startVelocity && Array.isArray(startVelocity) && startVelocity.length === 6) {
            this.config.initial_conditions.start_velocity = startVelocity.map(val => parseFloat(val));
        }
        
        if (startLocation && Array.isArray(startLocation) && startLocation.length === 3) {
            this.config.initial_conditions.start_location = startLocation.map(val => parseFloat(val));
        }
        
        if (startOrientation && Array.isArray(startOrientation) && startOrientation.length === 3) {
            this.config.initial_conditions.start_orientation = startOrientation.map(val => parseFloat(val));
        }
        
        this.config.initial_conditions.use_quaternion = !!useQuaternion;
        
        
    }

    // Update control types
    updateControlTypes(surfaceControlType, thrusterControlType) {
        if (surfaceControlType) {
            this.config.control.control_surface_control_type = surfaceControlType;
        }
        
        if (thrusterControlType) {
            this.config.control.thruster_control_type = thrusterControlType;
        }
        
        
    }

    // Get control surface by ID
    getControlSurface(id) {
        return this.config.control_surfaces.control_surfaces.find(
            surface => surface.control_surface_id === id
        );
    }

    // Get thruster by ID
    getThruster(id) {
        if (!this.config.thrusters || !this.config.thrusters.thrusters) return null;
        return this.config.thrusters.thrusters.find(
            thruster => thruster.thruster_id === id
        );
    }

    // Get sensor by ID
    getSensor(id) {
        return this.config.sensors.sensors.find(
            sensor => sensor.sensor_id === id
        );
    }
    
    // Get waypoint by ID
    getWaypoint(id) {
        if (!this.config.guidance.waypoints) return null;
        return this.config.guidance.waypoints.find(
            waypoint => waypoint.id === id
        );
    }

    // Update control surface properties
    updateControlSurface(id, properties) {
        const surface = this.getControlSurface(id);
        if (!surface) {
            console.error(`Control surface with ID ${id} not found`);
            return false;
        }
        
        const validKeys = [
            'control_surface_type', 'control_surface_location', 'control_surface_orientation',
            'control_surface_area', 'control_surface_T', 'control_surface_delta_max', 
            'control_surface_deltad_max', 'control_surface_NACA'
        ];
        
        console.log(`Updating control surface ${id} with:`, properties);
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'control_surface_location' || key === 'control_surface_orientation') {
                    if (!Array.isArray(value)) {
                        console.error(`Invalid ${key} value:`, value);
                        continue;
                    }
                    
                    const parsedValues = value.map(val => parseFloat(val));
                    console.log(`Setting ${key} to:`, parsedValues);
                    surface[key] = parsedValues;
                    
                    // Log for debugging
                    if (key === 'control_surface_orientation') {
                        console.log(`Updated orientation for control surface ${id}:`, parsedValues);
                    }
                } else if (key === 'control_surface_area' || key === 'control_surface_T') {
                    surface[key] = parseFloat(value);
                } else {
                    surface[key] = value;
                }
            }
        }
        
        console.log(`Control surface ${id} after update:`, surface);
        
        return true;
    }

    // Update thruster properties
    updateThruster(id, properties) {
        const thruster = this.getThruster(id);
        if (!thruster) {
            console.error(`Thruster with ID ${id} not found`);
            return false;
        }
        
        const validKeys = [
            'thruster_name', 'thruster_type', 'thruster_location', 'thruster_orientation',
            'D_prop', 'T_prop', 'tp', 'KT_at_J0', 'n_max'
        ];
        
        console.log(`Updating thruster ${id} with:`, properties);
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'thruster_location' || key === 'thruster_orientation') {
                    if (!Array.isArray(value)) {
                        console.error(`Invalid ${key} value:`, value);
                        continue;
                    }
                    
                    const parsedValues = value.map(val => parseFloat(val));
                    console.log(`Setting ${key} to:`, parsedValues);
                    thruster[key] = parsedValues;
                } else if (key === 'D_prop' || key === 'T_prop' || key === 'tp' || key === 'KT_at_J0' || key === 'n_max') {
                    thruster[key] = parseFloat(value);
                } else {
                    thruster[key] = value;
                }
            }
        }
        
        console.log(`Thruster ${id} after update:`, thruster);
        
        return true;
    }

    // Determine if a sensor uses quaternion orientation
    isSensorUsingQuaternion(id) {
        const sensor = this.getSensor(id);
        if (!sensor) return false;
        
        // If the orientation is 'None', it's not using any orientation
        if (sensor.sensor_orientation === 'None') return false;
        
        // Check if orientation is an array with 4 elements (quaternion)
        return Array.isArray(sensor.sensor_orientation) && sensor.sensor_orientation.length === 4;
    }
    
    // Update sensor properties
    updateSensor(id, properties) {
        const sensor = this.getSensor(id);
        if (!sensor) {
            console.error(`Sensor with ID ${id} not found`);
            return false;
        }
        
        const validKeys = [
            'sensor_type', 'sensor_location', 'sensor_orientation', 'publish_rate'
        ];
        
        console.log(`Updating sensor ${id} with:`, properties);
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                if (key === 'sensor_location' || key === 'sensor_orientation') {
                    if (!Array.isArray(value)) {
                        console.error(`Invalid ${key} value:`, value);
                        continue;
                    }
                    
                    const parsedValues = value.map(val => parseFloat(val));
                    console.log(`Setting ${key} to:`, parsedValues);
                    sensor[key] = parsedValues;
                } else if (key === 'publish_rate') {
                    sensor[key] = parseFloat(value);
                } else {
                    sensor[key] = value;
                }
            }
        }
        
        console.log(`Sensor ${id} after update:`, sensor);
        
        return true;
    }
    
    // Update waypoint properties
    updateWaypoint(id, properties) {
        const waypoint = this.getWaypoint(id);
        if (!waypoint) return false;
        
        const validKeys = ['x', 'y', 'z', 'psi'];
        
        for (const [key, value] of Object.entries(properties)) {
            if (validKeys.includes(key)) {
                waypoint[key] = parseFloat(value);
            }
        }
        
        
        return true;
    }

    // Remove a control surface
    removeControlSurface(id) {
        const index = this.config.control_surfaces.control_surfaces.findIndex(
            surface => surface.control_surface_id === id
        );
        
        if (index !== -1) {
            this.config.control_surfaces.control_surfaces.splice(index, 1);
            
            return true;
        }
        
        return false;
    }

    // Remove a thruster
    removeThruster(id) {
        if (!this.config.thrusters || !this.config.thrusters.thrusters) return false;
        
        const index = this.config.thrusters.thrusters.findIndex(
            thruster => thruster.thruster_id === id
        );
        
        if (index !== -1) {
            this.config.thrusters.thrusters.splice(index, 1);
            
            return true;
        }
        
        return false;
    }

    // Remove a sensor
    removeSensor(id) {
        const index = this.config.sensors.sensors.findIndex(
            sensor => sensor.sensor_id === id
        );
        
        if (index !== -1) {
            this.config.sensors.sensors.splice(index, 1);
            
            return true;
        }
        
        return false;
    }
    
    // Remove a waypoint
    removeWaypoint(id) {
        if (!this.config.guidance.waypoints) return false;
        
        const index = this.config.guidance.waypoints.findIndex(
            waypoint => waypoint.id === id
        );
        
        if (index !== -1) {
            this.config.guidance.waypoints.splice(index, 1);
            
            return true;
        }
        
        return false;
    }
    
    // Map 3D model component to vessel component
    mapModelComponent(uuid, component) {
        if (!component.type) {
            console.error('Component type must be specified');
            return false;
        }
        
        this.modelData.componentMap.set(uuid, component);
        console.log(`Mapped component ${component.name} as ${component.type}`);
        return true;
    }
    
    // Remove mapping for a 3D model component
    unmapModelComponent(uuid) {
        if (this.modelData.componentMap.has(uuid)) {
            const component = this.modelData.componentMap.get(uuid);
            console.log(`Unmapped component ${component.name}`);
            this.modelData.componentMap.delete(uuid);
            return true;
        }
        return false;
    }
    
    // Get component data for a 3D model object
    getModelComponentData(uuid) {
        return this.modelData.componentMap.get(uuid);
    }
    
    // Check if a 3D model object is mapped to a component
    isModelComponentMapped(uuid) {
        return this.modelData.componentMap.has(uuid);
    }
    
    // Set model file (3D, Hydra, NACA)
    setModelFile(fileType, file) {
        if (!file) return;
        
        console.log(`Setting ${fileType} file:`, file.name);
        
        if (fileType === 'fbx') {
            // Store the FBX file
            this.modelData.fbxFile = file;
        } else if (fileType === 'hydra') {
            this.modelData.hydraZipFile = file;
        } else if (fileType === 'naca') {
            // Validate file type (should be CSV)
            if (!file.name.toLowerCase().endsWith('.csv')) {
                console.warn(`NACA file should be a CSV file. Received: ${file.type}`);
                // We'll still accept it, but warn
            }
            
            // Store the NACA file
            this.modelData.nacaFile = file;
            
            // Try to parse file name to extract NACA profile number
            const fileNameMatch = file.name.match(/NACA(\d{4})/i);
            if (fileNameMatch && fileNameMatch[1]) {
                // Extract NACA number from file name
                const nacaNumber = fileNameMatch[1];
                this.setNacaProfile(nacaNumber);
            }
            
            console.log(`NACA file set: ${file.name}`);
        }
    }
    
    // Set NACA profile number
    setNacaProfile(nacaNumber) {
        if (!nacaNumber) return;
        
        this.config.control_surfaces.naca_number = nacaNumber;
        console.log(`NACA profile set to: ${nacaNumber}`);
    }
    
    // Get NACA file content (uploaded file or default data)
    getDefaultNacaFile() {
        // If we have a custom NACA file, return its content
        if (this.modelData.nacaFile) {
            console.log(`Reading uploaded NACA file: ${this.modelData.nacaFile.name}`);
            
            // Return a promise that resolves to the file content
            return new Promise((resolve, reject) => {
                const reader = new FileReader();
                
                reader.onload = (e) => {
                    try {
                        const content = e.target.result;
                        
                        // Basic validation to ensure it looks like CSV
                        if (!content.includes(',')) {
                            console.warn('NACA file does not appear to be a valid CSV.');
                            // We'll still use it but warn
                        }
                        
                        console.log(`Successfully read NACA file (${content.length} bytes)`);
                        resolve(content);
                    } catch (err) {
                        console.error('Error processing NACA file content:', err);
                        reject(err);
                    }
                };
                
                reader.onerror = (e) => {
                    console.error('Failed to read NACA file:', e);
                    reject(new Error('Failed to read NACA file'));
                };
                
                reader.readAsText(this.modelData.nacaFile);
            });
        }
        
        // Default NACA 0015 airfoil data (Alpha, CL, CD format)
        console.log('Using default NACA airfoil data');
        // If window.showNotification exists, use it to inform the user
        if (window.showNotification && !this.hasShownDefaultNacaNotification) {
            window.showNotification('Using default NACA 0015 airfoil data. Upload a custom CSV file if needed.', 'info');
            this.hasShownDefaultNacaNotification = true; // Only show once per session
        }
        
        return `Alpha,CL,CD
-15.0,-1.45,0.0528
-12.5,-1.30,0.0347
-10.0,-1.10,0.0233
-7.5,-0.85,0.0141
-5.0,-0.55,0.0086
-2.5,-0.25,0.0060
0.0,0.00,0.0052
2.5,0.25,0.0060
5.0,0.55,0.0086
7.5,0.85,0.0141
10.0,1.10,0.0233
12.5,1.30,0.0347
15.0,1.45,0.0528`;
    }

    // Export the vessel configuration as JSON
    exportAsJson() {
        // Create a deep copy of the config to modify
        const exportData = {
            config: JSON.parse(JSON.stringify(this.config)),
            modelData: {
                name: this.modelData.name,
                type: this.modelData.type,
                length: this.modelData.length,
                width: this.modelData.width,
                height: this.modelData.height,
                mass: this.modelData.mass,
                componentMap: []
            }
        };

        // Convert componentMap to array for serialization
        this.modelData.componentMap.forEach((component, uuid) => {
            exportData.modelData.componentMap.push({
                uuid,
                component
            });
        });

        // Promise to handle async file reading
        return new Promise((resolve, reject) => {
            const pendingTasks = [];

            // Add FBX file if exists
            if (this.modelData.fbxFile) {
                const fbxTask = new Promise((resolveFbx) => {
                    const reader = new FileReader();
                    reader.onload = (e) => {
                        const base64Data = e.target.result.split(',')[1]; // Remove data URL prefix
                        exportData.modelData.fbxFile = {
                            name: this.modelData.fbxFile.name,
                            type: this.modelData.fbxFile.type,
                            size: this.modelData.fbxFile.size,
                            data: base64Data
                        };
                        resolveFbx();
                    };
                    reader.onerror = reject;
                    reader.readAsDataURL(this.modelData.fbxFile);
                });
                pendingTasks.push(fbxTask);
            }

            // Add NACA file if exists
            if (this.modelData.nacaFile) {
                const nacaTask = new Promise((resolveNaca) => {
                    const reader = new FileReader();
                    reader.onload = (e) => {
                        const base64Data = e.target.result.split(',')[1]; // Remove data URL prefix
                        exportData.modelData.nacaFile = {
                            name: this.modelData.nacaFile.name,
                            type: this.modelData.nacaFile.type,
                            size: this.modelData.nacaFile.size,
                            data: base64Data
                        };
                        resolveNaca();
                    };
                    reader.onerror = reject;
                    reader.readAsDataURL(this.modelData.nacaFile);
                });
                pendingTasks.push(nacaTask);
            }

            // Add hydra file if exists
            if (this.modelData.hydraZipFile) {
                const hydraTask = new Promise((resolveHydra) => {
                    const reader = new FileReader();
                    reader.onload = (e) => {
                        const base64Data = e.target.result.split(',')[1]; // Remove data URL prefix
                        exportData.modelData.hydraZipFile = {
                            name: this.modelData.hydraZipFile.name,
                            type: this.modelData.hydraZipFile.type,
                            size: this.modelData.hydraZipFile.size,
                            data: base64Data
                        };
                        resolveHydra();
                    };
                    reader.onerror = reject;
                    reader.readAsDataURL(this.modelData.hydraZipFile);
                });
                pendingTasks.push(hydraTask);
            }

            // When all file reading is done, resolve with the complete JSON
            Promise.all(pendingTasks)
                .then(() => resolve(JSON.stringify(exportData, null, 2)))
                .catch(reject);
        });
    }

    // Import vessel configuration from JSON
    importFromJson(jsonString) {
        try {
            const importedData = JSON.parse(jsonString);
            
            // Handle both formats - old format is just the config object
            // New format has config and modelData
            if (importedData.config && importedData.modelData) {
                // New format with modelData
                const config = importedData.config;
                
                // Validate essential structure
                if (!config.name || !config.geometry || !config.inertia) {
                    throw new Error('Invalid vessel configuration format');
                }
                
                this.config = config;
                
                // Handle modelData import
                this.importModelData(importedData.modelData);
            } else {
                // Old format - just the config object
                // Validate essential structure
                if (!importedData.name || !importedData.geometry || !importedData.inertia) {
                    throw new Error('Invalid vessel configuration format');
                }
                
                this.config = importedData;
            }
            
            // Reset component IDs
            this.resetComponentIds();
            
            return true;
        } catch (error) {
            console.error('Error importing vessel configuration:', error);
            return false;
        }
    }
    
    // Import model data from saved configuration
    async importModelData(modelData) {
        // Set basic modelData properties
        this.modelData.name = modelData.name;
        this.modelData.type = modelData.type;
        this.modelData.length = modelData.length;
        this.modelData.width = modelData.width;
        this.modelData.height = modelData.height;
        this.modelData.mass = modelData.mass;
        
        // Clear current component map
        this.modelData.componentMap.clear();
        
        // Import component map
        if (Array.isArray(modelData.componentMap)) {
            modelData.componentMap.forEach(item => {
                if (item.uuid && item.component) {
                    this.modelData.componentMap.set(item.uuid, item.component);
                }
            });
        }
        
        // Import FBX file if exists
        if (modelData.fbxFile && modelData.fbxFile.data) {
            const fbxFile = this.base64ToFile(
                modelData.fbxFile.data,
                modelData.fbxFile.name,
                modelData.fbxFile.type
            );
            this.modelData.fbxFile = fbxFile;
        }
        
        // Import NACA file if exists
        if (modelData.nacaFile && modelData.nacaFile.data) {
            const nacaFile = this.base64ToFile(
                modelData.nacaFile.data,
                modelData.nacaFile.name,
                modelData.nacaFile.type
            );
            this.modelData.nacaFile = nacaFile;
        }
        
        // Import hydra file if exists
        if (modelData.hydraZipFile && modelData.hydraZipFile.data) {
            const hydraFile = this.base64ToFile(
                modelData.hydraZipFile.data,
                modelData.hydraZipFile.name,
                modelData.hydraZipFile.type
            );
            this.modelData.hydraZipFile = hydraFile;
        }
    }
    
    // Helper to convert base64 to File object
    base64ToFile(base64Data, filename, mimeType) {
        const byteCharacters = atob(base64Data);
        const byteArrays = [];
        
        for (let offset = 0; offset < byteCharacters.length; offset += 512) {
            const slice = byteCharacters.slice(offset, offset + 512);
            
            const byteNumbers = new Array(slice.length);
            for (let i = 0; i < slice.length; i++) {
                byteNumbers[i] = slice.charCodeAt(i);
            }
            
            const byteArray = new Uint8Array(byteNumbers);
            byteArrays.push(byteArray);
        }
        
        const blob = new Blob(byteArrays, {type: mimeType});
        return new File([blob], filename, {type: mimeType});
    }
    
    // Reset component IDs to ensure they're sequential after import
    resetComponentIds() {
        // Reset control surface IDs
        if (this.config.control_surfaces && this.config.control_surfaces.control_surfaces) {
            let maxId = 0;
            
            this.config.control_surfaces.control_surfaces.forEach(surface => {
                if (surface.control_surface_id > maxId) {
                    maxId = surface.control_surface_id;
                }
            });
            
            this.nextControlSurfaceId = maxId + 1;
        } else {
            this.nextControlSurfaceId = 1;
        }
        
        // Reset thruster IDs
        if (this.config.thrusters && this.config.thrusters.thrusters) {
            let maxId = 0;
            
            this.config.thrusters.thrusters.forEach(thruster => {
                if (thruster.thruster_id > maxId) {
                    maxId = thruster.thruster_id;
                }
            });
            
            this.nextThrusterId = maxId + 1;
        } else {
            this.nextThrusterId = 1;
        }
        
        // Reset sensor IDs
        if (this.config.sensors && this.config.sensors.sensors) {
            let maxId = 0;
            
            this.config.sensors.sensors.forEach(sensor => {
                if (sensor.sensor_id > maxId) {
                    maxId = sensor.sensor_id;
                }
            });
            
            this.nextSensorId = maxId + 1;
        } else {
            this.nextSensorId = 1;
        }
        
        // Reset waypoint IDs
        if (this.config.guidance && this.config.guidance.waypoints) {
            let maxId = 0;
            
            this.config.guidance.waypoints.forEach(waypoint => {
                if (waypoint.id > maxId) {
                    maxId = waypoint.id;
                }
            });
            
            this.nextWaypointId = maxId + 1;
        } else {
            this.nextWaypointId = 1;
        }
    }
    
    // Generate YAML files
    generateYAMLFiles() {
        const yamlGenerator = new YAMLGenerator();
        return yamlGenerator.generateAllYAMLFiles(this);
    }
    
    // Create a ZIP file with YAML files
    async createYAMLZip() {
        const yamlGenerator = new YAMLGenerator();
        return await yamlGenerator.createZipFile(this);
    }

    // Methods for handling FBX model components
    getAllComponentsByType(type) {
        const components = [];
        this.modelData.componentMap.forEach((component, uuid) => {
            if (component.type === type) {
                components.push({uuid, ...component});
            }
        });
        return components;
    }
}

// Export the class to global scope
window.VesselModel = VesselModel; 