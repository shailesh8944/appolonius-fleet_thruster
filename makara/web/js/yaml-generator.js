/**
 * YAMLGenerator.js - Configuration Generator for Marine Vessel Simulation
 * 
 * This class generates YAML configuration files required for running marine vessel simulations.
 * It takes the vessel model data and transforms it into properly formatted YAML files
 * following the required schema for the simulation engine.
 * 
 * The generator creates separate YAML files for:
 * - Vessel geometry and physical properties
 * - Hydrodynamic coefficients
 * - Control surfaces configuration
 * - Thruster configuration
 * - Initial conditions
 * - Guidance and navigation settings
 * - Sensor configuration
 * - Simulation parameters
 * 
 * It handles proper formatting of numbers, arrays, and specialized comments
 * required by the simulation engine, and packages all files into a zip archive
 * for easy deployment.
 */

class YAMLGenerator {
    /**
     * Initialize the YAML generator with default simulation parameters
     */
    constructor() {
        // Default simulation parameters used as fallbacks
        this.defaultSimParams = {
            sim_time: 100,  // seconds
            time_step: 0.01,  // seconds
            density: 1025,  // kg/m³ (seawater)
            gravity: 9.80665,  // m/s²
            world_size: [1000, 1000, 100],  // meters [x, y, z]
            gps_datum: [12.99300425860631, 80.23913114094384, -87.0],  // [lat, lon, alt]
            nagents: 1
        };
    }

    /**
     * Format an array as an inline string with specified precision
     * Example: [1.0000, 2.0000, 3.0000]
     * 
     * @param {Array} arr - Array of numbers to format
     * @param {number} precision - Number of decimal places
     * @returns {string} - Formatted array string
     */
    formatInlineArray(arr, precision = 4) {
        if (!Array.isArray(arr)) {
            console.warn('Invalid array input:', arr);
            return '[0.0, 0.0, 0.0]';
        }
        return '[' + arr.map(val => Number(parseFloat(val).toFixed(precision))).join(', ') + ']';
    }

    /**
     * Format a 2D array as an inline string with specified precision
     * Example: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
     * 
     * @param {Array} arr - 2D array of numbers
     * @param {number} precision - Number of decimal places
     * @returns {string} - Formatted 2D array string
     */
    format2DInlineArray(arr, precision = 6) {
        if (!arr || !Array.isArray(arr)) return '[[0, 0, 0], [0, 0, 0], [0, 0, 0]]';
        
        let result = '[';
        for (let i = 0; i < arr.length; i++) {
            result += '[' + arr[i].map(val => Number(parseFloat(val)).toFixed(precision)).join(', ') + ']';
            if (i < arr.length - 1) result += ', ';
        }
        result += ']';
        return result;
    }

    /**
     * Create a Docker-compatible file path for simulation inputs
     * 
     * @param {string} vesselName - Name of the vessel
     * @param {string} fileName - Name of the file
     * @returns {string} - Docker-compatible path
     */
    formatFilePath(vesselName, fileName) {
        return `/workspaces/mavlab/inputs/${vesselName}/${fileName}`;
    }

    /**
     * Format a numerical value with a comment
     * Example: 1025 # kg/m³
     * 
     * @param {number|string} value - The value to format
     * @param {string} comment - Comment to add after the value
     * @returns {string} - Formatted value with comment
     */
    formatWithComment(value, comment) {
        return `${value} # ${comment}`;
    }

    /**
     * Generate the hydrodynamics configuration YAML
     * Contains drag coefficients and other hydrodynamic parameters
     * 
     * @param {Object} hydrodynamics - Hydrodynamic parameters object
     * @param {string} vesselName - Name of the vessel
     * @returns {string} - Formatted YAML content
     */
    generateHydrodynamicsYAML(hydrodynamics, vesselName) {
        console.log("Generating hydrodynamics YAML with data:", hydrodynamics);
        
        // Format the hydra file path
        const hydraFile = this.formatFilePath(vesselName, `HydRA/${vesselName.toUpperCase()}_hydra.json`);
        
        // Group coefficients by type based on their prefix
        const coefficientGroups = {
            X: { title: "Surge hydrodynamic derivatives", coeffs: {} },
            Y: { title: "Sway hydrodynamic derivatives", coeffs: {} },
            Z: { title: "Heave hydrodynamic derivatives", coeffs: {} },
            K: { title: "Roll hydrodynamic derivatives", coeffs: {} },
            M: { title: "Pitch hydrodynamic derivatives", coeffs: {} },
            N: { title: "Yaw hydrodynamic derivatives", coeffs: {} }
        };
        
        // Start with the standard header
        let yamlContent = `hydra_file: ${hydraFile}
dim_flag: ${hydrodynamics.dim_flag || false}
cross_flow_drag: ${hydrodynamics.cross_flow_drag || false}
`;

        // Process all hydrodynamic coefficients and group them
        Object.entries(hydrodynamics).forEach(([key, value]) => {
            // Skip the non-coefficient properties
            if (key === 'hydra_file' || key === 'dim_flag' || key === 'cross_flow_drag') {
                return;
            }
            
            // Group by first letter (the motion direction)
            const prefix = key.charAt(0).toUpperCase();
            if (coefficientGroups[prefix]) {
                coefficientGroups[prefix].coeffs[key] = value;
            } else {
                // For any coefficients that don't match standard groups
                console.log(`Unrecognized coefficient prefix in: ${key}`);
            }
        });
        
        // Add each group of coefficients to the YAML
        Object.values(coefficientGroups).forEach(group => {
            const coeffEntries = Object.entries(group.coeffs);
            if (coeffEntries.length > 0) {
                yamlContent += `\n# ${group.title}\n`;
                coeffEntries.sort().forEach(([key, value]) => {
                    yamlContent += `${key}: ${value}\n`;
                });
            }
        });
        
        // Add the footer note
        yamlContent += `\n## Consider underscore in the terms eg: X_u_u instead of Xuu\n\n\n\n`;

        console.log("Generated hydrodynamics YAML:", yamlContent);
        return yamlContent;
    }

    /**
     * Generate the vessel geometry configuration YAML
     * Contains dimensions, centers, and geometry file references
     * 
     * @param {Object} geometry - Geometry parameters object
     * @param {string} vesselName - Name of the vessel
     * @returns {Object} - Object with formatted geometry parameters
     */
    generateGeometryYAML(geometry, vesselName) {
        // Apply scale to dimensions if present
        const scale = geometry.scale || [1, 1, 1];
        const scaledLength = Number((geometry.length * scale[0]).toFixed(4));
        const scaledBreadth = Number((geometry.breadth * scale[1]).toFixed(4));
        const scaledDepth = Number((geometry.depth * scale[2]).toFixed(4));

        // Calculate default gyration values if not provided
        let gyration = geometry.gyration;
        if (!gyration || gyration.every(val => val === 0)) {
            // Default gyration calculation based on vessel dimensions
            gyration = [
                scaledBreadth * 0.35, // Roll gyration radius
                scaledLength * 0.25,  // Pitch gyration radius
                scaledLength * 0.25   // Yaw gyration radius
            ];
        }

        // Create the structure with position and orientation for each center
        return {
            length: scaledLength,
            breadth: scaledBreadth,
            depth: scaledDepth,
            gyration: this.formatInlineArray(gyration, 4),
            CO: {
                position: this.formatInlineArray(geometry.CO_position || geometry.CO || [0.0, 0.0, 0.0], 4),
                orientation: this.formatInlineArray(geometry.CO_orientation || [0.0, 0.0, 0.0], 4)
            },
            CG: {
                position: this.formatInlineArray(geometry.CG_position || geometry.CG || [0.0, 0.0, 0.0], 4),
                orientation: this.formatInlineArray(geometry.CG_orientation || [0.0, 0.0, 0.0], 4)
            },
            CB: {
                position: this.formatInlineArray(geometry.CB_position || geometry.CB || [0.0, 0.0, 0.0], 4),
                orientation: this.formatInlineArray(geometry.CB_orientation || [0.0, 0.0, 0.0], 4)
            },
            geometry_file: geometry.geometry_file || this.formatFilePath(vesselName, 'HydRA/input/vessel.gdf')
        };
    }

    /**
     * Generate the inertia configuration YAML
     * Contains mass, buoyancy, and inertia matrices
     * 
     * @param {Object} inertia - Inertia parameters object
     * @returns {Object} - Object with formatted inertia parameters
     */
    generateInertiaYAML(inertia) {
        const mass = Number(parseFloat(inertia.mass || 1).toFixed(4));
        const buoyancy_mass = Number(parseFloat(inertia.buoyancy_mass || mass).toFixed(4));
        
        return {
            mass: this.formatWithComment(mass, 'in kg'),
            buoyancy_mass: this.formatWithComment(buoyancy_mass, 'in kg'),
            inertia_matrix: inertia.inertia_matrix ? this.format2DInlineArray(inertia.inertia_matrix, 6) + ' # in kg*m^2' : 'None # in kg*m^2',
            added_mass_matrix: inertia.added_mass_matrix ? this.format2DInlineArray(inertia.added_mass_matrix, 6) + ' # in kg*m^2/s' : 'None # in kg*m^2/s'
        };
    }

    /**
     * Generate the control surfaces configuration YAML
     * Contains rudders, fins, and other control surfaces
     * 
     * @param {Object} controlSurfaces - Control surfaces parameters object
     * @param {string} vesselName - Name of the vessel
     * @returns {Object} - Object with formatted control surfaces parameters
     */
    generateControlSurfacesYAML(controlSurfaces, vesselName) {
        // First, log what we received to help debug
        console.log('Generating control surfaces YAML with:', controlSurfaces);
        
        // Default values if controlSurfaces is not properly initialized
        if (!controlSurfaces || typeof controlSurfaces !== 'object') {
            console.warn('controlSurfaces is not properly initialized, using defaults');
            controlSurfaces = { 
                naca_number: '0015',
                control_surfaces: []
            };
        }
        
        const nacaNumber = controlSurfaces.naca_number || '0015';
        const nacaFile = this.formatFilePath(vesselName, `NACA${nacaNumber}.csv`);
        
        // Use YAML anchors (&) for common values to enable reuse
        const yaml = {
            naca_number: `&naca ${nacaNumber}`,
            naca_file: nacaFile,
            delta_max: '&dmax 35.0',
            deltad_max: '&ddmax 1.0',
            area: '&area 0.1',
            control_surfaces: []
        };
        
        // Add control surfaces if available
        if (controlSurfaces.control_surfaces && Array.isArray(controlSurfaces.control_surfaces)) {
            // Log the count for debugging
            console.log(`Processing ${controlSurfaces.control_surfaces.length} control surfaces`);
            
            // Check for duplicate IDs before processing
            const uniqueIds = new Set();
            const validSurfaces = [];
            
            // Filter out duplicates based on control_surface_id
            controlSurfaces.control_surfaces.forEach(surface => {
                const id = surface.control_surface_id;
                if (!uniqueIds.has(id)) {
                    uniqueIds.add(id);
                    validSurfaces.push(surface);
                } else {
                    console.warn(`Duplicate control surface ID found: ${id}, skipping duplicate`);
                }
            });
            
            console.log(`After filtering, processing ${validSurfaces.length} unique control surfaces`);
            
            // Map each control surface to the YAML format
            yaml.control_surfaces = validSurfaces.map(surface => {
                // Log each control surface for debugging
                console.log('Processing control surface:', surface);
                
                // Ensure orientation values are converted to numbers to avoid string formatting issues
                const orientation = Array.isArray(surface.control_surface_orientation) ? 
                    surface.control_surface_orientation.map(val => parseFloat(val)) : 
                    [0, 0, 0];
                    
                const location = Array.isArray(surface.control_surface_location) ?
                    surface.control_surface_location.map(val => parseFloat(val)) :
                    [0, 0, 0];
                    
                return {
                    control_surface_type: surface.control_surface_type || 'Rudder',
                    control_surface_id: surface.control_surface_id,
                    control_surface_location: this.formatInlineArray(location, 4) + '    # With respect to BODY frame',
                    control_surface_orientation: this.formatInlineArray(orientation, 4) + '  # With respect to BODY frame',
                    control_surface_area: '*area',  // Use YAML anchor reference
                    control_surface_NACA: '*naca',  // Use YAML anchor reference
                    control_surface_T: surface.control_surface_T || 0.1,
                    control_surface_delta_max: '*dmax',  // Use YAML anchor reference
                    control_surface_deltad_max: '*ddmax'  // Use YAML anchor reference
                };
            });
        }
        
        return yaml;
    }

    generateThrustersYAML(thrusters, vesselName) {
        const yaml = {
            thrusters: []
        };
        
        if (thrusters?.thrusters && thrusters.thrusters.length > 0) {
            yaml.thrusters = thrusters.thrusters.map(thruster => ({
                thruster_name: thruster.thruster_name || `thruster_${thruster.thruster_id}`,
                thruster_id: thruster.thruster_id,
                thruster_location: this.formatInlineArray(thruster.thruster_location, 4) + '    # With respect to BODY frame',
                thruster_orientation: this.formatInlineArray(thruster.thruster_orientation, 4) + '  # With respect to BODY frame',
                T_prop: thruster.T_prop || 1.0,
                D_prop: Number(parseFloat(thruster.D_prop || 0.1).toFixed(4)),
                tp: Number(parseFloat(thruster.tp || 0.1).toFixed(4)),
                J_vs_KT: this.formatFilePath(vesselName, 'J_vs_KT.csv')
            }));
        }
        
        return yaml;
    }

    generateInitialConditionsYAML(initialConditions) {
        return {
            start_location: this.formatInlineArray(initialConditions.start_location || [0, 0, 0], 4),
            start_orientation: this.formatInlineArray(initialConditions.start_orientation || [0, 0, 0], 4),
            start_velocity: this.formatInlineArray(initialConditions.start_velocity || [0, 0, 0, 0, 0, 0], 4),
            use_quaternion: initialConditions.use_quaternion || false
        };
    }

    generateGuidanceYAML(guidance) {
        const yaml = {
            guidance_type: guidance.guidance_type || 'waypoint'
        };
        
        if (guidance.waypoints && guidance.waypoints.length > 0) {
            yaml.waypoints = guidance.waypoints.map(wp => ({
                x: Number(parseFloat(wp.x || 0).toFixed(4)),
                y: Number(parseFloat(wp.y || 0).toFixed(4)),
                z: Number(parseFloat(wp.z || 0).toFixed(4)),
                psi: Number(parseFloat(wp.psi || 0).toFixed(4))
            }));
        }
        
        return yaml;
    }

    generateControlYAML(control) {
        return {
            control_surface_control_type: control.control_surface_control_type || 'fixed_rudder',
            thruster_control_type: control.thruster_control_type || 'fixed_rpm'
        };
    }

    generateSensorsYAML(sensors) {
        const yaml = {
            sensors: []
        };
        
        if (sensors.sensors && sensors.sensors.length > 0) {
            yaml.sensors = sensors.sensors.map(sensor => ({
                sensor_type: sensor.sensor_type || 'IMU',
                sensor_id: sensor.sensor_id,
                sensor_location: sensor.sensor_location ? this.formatInlineArray(sensor.sensor_location, 4) + '          # With respect to BODY frame' : 'None',
                sensor_orientation: sensor.sensor_orientation ? this.formatInlineArray(sensor.sensor_orientation, 4) + '    # With respect to BODY frame' : 'None',
                publish_rate: Number(parseFloat(sensor.publish_rate || 10).toFixed(1))
            }));
        }
        
        return yaml;
    }

    generateSimulationInputYAML(vesselConfig) {
        console.log("Generating simulation input YAML with:", vesselConfig);
        
        const vesselName = vesselConfig.name || 'mavymini';
        const simConfig = vesselConfig.simulation || this.defaultSimParams;
        
        // Add vessel name and type to the agents if needed
        if (!simConfig.agents || !Array.isArray(simConfig.agents) || simConfig.agents.length === 0) {
            simConfig.agents = [{ 
                name: vesselName, 
                type: vesselConfig.type || 'auv',
                geometry: `/workspaces/mavlab/inputs/${vesselName}/geometry.yml`,
                inertia: `/workspaces/mavlab/inputs/${vesselName}/inertia.yml`,
                hydrodynamics: `/workspaces/mavlab/inputs/${vesselName}/hydrodynamics.yml`,
                control_surfaces: `/workspaces/mavlab/inputs/${vesselName}/control_surfaces.yml`,
                initial_conditions: `/workspaces/mavlab/inputs/${vesselName}/initial_conditions.yml`,
                sensors: `/workspaces/mavlab/inputs/${vesselName}/sensors.yml`,
                guidance: `/workspaces/mavlab/inputs/${vesselName}/guidance.yml`,
                control: `/workspaces/mavlab/inputs/${vesselName}/control.yml`,
                thrusters: `/workspaces/mavlab/inputs/${vesselName}/thrusters.yml`
            }];
        } else {
            // Make sure all paths have the correct vessel name instead of {name} placeholder
            simConfig.agents = simConfig.agents.map(agent => {
                const agentName = agent.name || vesselName;
                
                // Clone the agent object to avoid modifying the original
                const updatedAgent = { ...agent };
                
                // Replace {name} with the agent name in all file paths
                for (const [key, value] of Object.entries(updatedAgent)) {
                    if (typeof value === 'string' && value.includes('{name}')) {
                        updatedAgent[key] = value.replace(/{name}/g, agentName);
                    }
                }
                
                return updatedAgent;
            });
        }

        // Use the formatted YAML generator for consistent output
        return this.generateFormattedSimulationYAML(simConfig);
    }
    
    // Convert JavaScript object to YAML string with proper formatting
    toYAML(obj, indent = 0) {
        // If input is already a string, return it directly
        if (typeof obj === 'string') {
            return obj;
        }
        
        let yaml = '';
        const spaces = ' '.repeat(indent);
        
        for (const [key, value] of Object.entries(obj)) {
            // Skip empty arrays and objects
            if ((Array.isArray(value) && value.length === 0) || 
                (typeof value === 'object' && value !== null && !Array.isArray(value) && Object.keys(value).length === 0)) {
                continue;
            }
            
            if (typeof value === 'object' && value !== null) {
                if (Array.isArray(value)) {
                    // Handle arrays
                    yaml += `${spaces}${key}:\n`;
                    
                    // Check if array contains objects
                    if (value.length > 0 && typeof value[0] === 'object' && value[0] !== null) {
                        // Array of objects
                        for (const item of value) {
                            yaml += `${spaces}  - \n`;
                            // Process each property of the object with proper indentation
                            for (const [itemKey, itemValue] of Object.entries(item)) {
                                if (typeof itemValue === 'object' && itemValue !== null && !Array.isArray(itemValue)) {
                                    // Nested object
                                    yaml += `${spaces}    ${itemKey}:\n`;
                                    yaml += this.toYAML(itemValue, indent + 6);
                                } else if (Array.isArray(itemValue)) {
                                    // Nested array
                                    yaml += `${spaces}    ${itemKey}:\n`;
                                    for (const nestedItem of itemValue) {
                                        yaml += `${spaces}      - ${nestedItem}\n`;
                                    }
                                } else {
                                    // Simple value
                                    yaml += `${spaces}    ${itemKey}: ${itemValue}\n`;
                                }
                            }
                        }
                    } else {
                        // Array of primitives
                        for (const item of value) {
                            yaml += `${spaces}  - ${item}\n`;
                        }
                    }
                } else {
                    // Handle objects
                    yaml += `${spaces}${key}:\n${this.toYAML(value, indent + 2)}`;
                }
            } else if (typeof value === 'string' && (value.startsWith('*') || value.includes('#'))) {
                // Handle anchor references and comments
                yaml += `${spaces}${key}: ${value}\n`;
            } else if (typeof value === 'string' && value.startsWith('/')) {
                // File paths
                yaml += `${spaces}${key}: ${value}\n`;
            } else {
                // Default formatting
                yaml += `${spaces}${key}: ${value}\n`;
            }
        }
        
        return yaml;
    }

    // Generate all YAML files and return them as an object
    generateAllYAMLFiles(vesselModel) {
        const vessel = vesselModel.config;
        
        // Generate simulation_input.yml and hydrodynamics.yml directly as formatted strings
        const simulationInput = this.generateSimulationInputYAML(vessel);
        const hydrodynamicsYaml = this.generateHydrodynamicsYAML(vessel.hydrodynamics, vessel.name);
        
        const files = {
            'simulation_input.yml': simulationInput,
            'geometry.yml': this.toYAML(this.generateGeometryYAML(vessel.geometry, vessel.name)),
            'inertia.yml': this.toYAML(this.generateInertiaYAML(vessel.inertia)),
            'hydrodynamics.yml': hydrodynamicsYaml,
            'control_surfaces.yml': this.toYAML(this.generateControlSurfacesYAML(vessel.control_surfaces, vessel.name)),
            'initial_conditions.yml': this.toYAML(this.generateInitialConditionsYAML(vessel.initial_conditions)),
            'sensors.yml': this.toYAML(this.generateSensorsYAML(vessel.sensors)),
            'guidance.yml': this.toYAML(this.generateGuidanceYAML(vessel.guidance)),
            'control.yml': this.toYAML(this.generateControlYAML(vessel.control))
        };
        
        if (vessel.thrusters && vessel.thrusters.thrusters && vessel.thrusters.thrusters.length > 0) {
            files['thrusters.yml'] = this.toYAML(this.generateThrustersYAML(vessel.thrusters, vessel.name));
        }
        
        return files;
    }

    // Creates a .zip file with all the YAML files
    async createZipFile(vesselModel) {
        const files = this.generateAllYAMLFiles(vesselModel);
        const vesselName = vesselModel.config.name;
        
        const zip = new JSZip();
        // Create an 'inputs' folder at the root level
        const inputsFolder = zip.folder('inputs');
        // Create vessel-specific subfolder
        const vesselFolder = inputsFolder.folder(vesselName);
        
        // Add simulation_input.yml to the root of inputs folder
        if (files['simulation_input.yml']) {
            inputsFolder.file('simulation_input.yml', files['simulation_input.yml']);
        }
        
        // Add all other YAML files to the vessel folder
        for (const [filename, content] of Object.entries(files)) {
            if (filename !== 'simulation_input.yml') {
                vesselFolder.file(filename, content);
            }
        }
        
        // Handle HydRA files
        if (vesselModel.modelData && vesselModel.modelData.hydraZipFile) {
            try {
                console.log("Processing HydRA zip file...");
                
                // Load the HydRA zip file
                const hydraZip = await JSZip.loadAsync(vesselModel.modelData.hydraZipFile);
                const hydraFolder = vesselFolder.folder('HydRA');
                
                // Process all files in the HydRA zip, preserving internal structure
                const promises = [];
                hydraZip.forEach((relativePath, zipEntry) => {
                    // Skip directories, we'll create them as needed
                    if (!zipEntry.dir) {
                        // Preserve the internal directory structure but under the HydRA folder
                        // Remove any potential top-level folder from the path
                        let targetPath = relativePath;
                        const pathParts = relativePath.split('/');
                        
                        // If the file is nested under directories, preserve that structure
                        if (pathParts.length > 1) {
                            // Remove first part if it's a root folder name
                            // This keeps the structure but removes any source root folder name
                            const fileNameWithPath = pathParts.slice(1).join('/');
                            targetPath = fileNameWithPath;
                        }
                        
                        console.log(`Extracting HydRA file: ${relativePath} → ${targetPath}`);
                        
                        // Add file to the HydRA folder, preserving internal paths
                        const promise = zipEntry.async('blob').then(content => {
                            hydraFolder.file(targetPath, content);
                        });
                        
                        promises.push(promise);
                    }
                });
                
                // Wait for all files to be processed
                await Promise.all(promises);
                console.log("HydRA zip processing complete");
            } catch (error) {
                console.error("Error processing HydRA zip:", error);
                // Create an empty HydRA folder as fallback
                vesselFolder.folder('HydRA');
            }
        } else {
            // No HydRA zip, just create an empty folder
            vesselFolder.folder('HydRA');
        }
        
        // Include NACA file - always try to add it if available
        try {
            const nacaNumber = vesselModel.config.control_surfaces?.naca_number || '0015';
            
            // Check if we have a getDefaultNacaFile method that might return the uploaded file
            if (vesselModel.getDefaultNacaFile) {
                const nacaContent = vesselModel.getDefaultNacaFile();
                
                // Check if the result is a Promise
                if (nacaContent instanceof Promise) {
                    // Handle promise-based content
                    const resolvedContent = await nacaContent;
                    vesselFolder.file(`NACA${nacaNumber}.csv`, resolvedContent);
                    console.log(`Added NACA${nacaNumber}.csv file to zip (from uploaded file)`);
                } else {
                    // Handle direct string content
                    vesselFolder.file(`NACA${nacaNumber}.csv`, nacaContent);
                    console.log(`Added NACA${nacaNumber}.csv file to zip (from default data)`);
                }
            }
        } catch (error) {
            console.error("Error adding NACA file:", error);
            // Use a default NACA file as fallback
            const nacaNumber = vesselModel.config.control_surfaces?.naca_number || '0015';
            vesselFolder.file(`NACA${nacaNumber}.csv`, 
                `Alpha,CL,CD
-15.0,-1.45,0.0528
-10.0,-1.10,0.0233
-5.0,-0.55,0.0086
0.0,0.00,0.0052
5.0,0.55,0.0086
10.0,1.10,0.0233
15.0,1.45,0.0528`);
            console.log(`Added default NACA${nacaNumber}.csv file to zip due to error`);
        }
        
        // Generate zip content
        return await zip.generateAsync({ type: 'blob' });
    }

    // Format simulation parameters into a standardized simulation_input.yml format
    generateFormattedSimulationYAML(simParams) {
        if (!simParams || typeof simParams !== 'object') {
            return '';
        }
        
        // Extract basic parameters with defaults
        const simTime = simParams.sim_time || this.defaultSimParams.sim_time;
        const timeStep = simParams.time_step || this.defaultSimParams.time_step;
        const density = simParams.density || this.defaultSimParams.density;
        const gravity = simParams.gravity || this.defaultSimParams.gravity;
        
        // Format world_size array
        const worldSize = [...(simParams.world_size || this.defaultSimParams.world_size)];
        if (worldSize[2] === 100 || worldSize[2] === '100') {
            worldSize[2] = 'Inf'; // Use 'Inf' for z dimension by default
        }
        
        // Format GPS datum
        const gpsDatum = simParams.gps_datum || this.defaultSimParams.gps_datum;
        
        // Get geofence
        const geofence = simParams.geofence || [
            [12.993496, 80.239007],
            [12.993500, 80.238903],
            [12.993485, 80.238829]
        ];
        
        // Format the geofence
        let geofenceYaml = '';
        geofence.forEach(coord => {
            if (Array.isArray(coord) && coord.length >= 2) {
                geofenceYaml += `  - [${coord[0]}, ${coord[1]}]\n`;
            }
        });
        
        // Get agents information
        const nagents = simParams.nagents || 1;
        const agents = simParams.agents || [{ name: 'vessel', type: 'auv' }];
        
        // Format agents
        let agentsYaml = '';
        agents.forEach(agent => {
            const agentName = agent.name || 'vessel';
            agentsYaml += `  -\n    name: ${agentName}\n    type: ${agent.type || 'auv'}\n`;
            
            // Add file paths that include the vessel name directly (not using {name} placeholder)
            agentsYaml += `    geometry: /workspaces/mavlab/inputs/${agentName}/geometry.yml\n`;
            agentsYaml += `    inertia: /workspaces/mavlab/inputs/${agentName}/inertia.yml\n`;
            agentsYaml += `    hydrodynamics: /workspaces/mavlab/inputs/${agentName}/hydrodynamics.yml\n`;
            agentsYaml += `    control_surfaces: /workspaces/mavlab/inputs/${agentName}/control_surfaces.yml\n`;
            agentsYaml += `    initial_conditions: /workspaces/mavlab/inputs/${agentName}/initial_conditions.yml\n`;
            agentsYaml += `    sensors: /workspaces/mavlab/inputs/${agentName}/sensors.yml\n`;
            agentsYaml += `    guidance: /workspaces/mavlab/inputs/${agentName}/guidance.yml\n`;
            agentsYaml += `    control: /workspaces/mavlab/inputs/${agentName}/control.yml\n`;
            agentsYaml += `    thrusters: /workspaces/mavlab/inputs/${agentName}/thrusters.yml\n`;
        });
        
        // Build the YAML content directly as a string to match the exact format
        let yamlContent = `#Simulation Specific Inputs
sim_time: ${simTime}
time_step: ${timeStep}
density: ${density}
gravity: ${gravity}
world_size: [${worldSize[0]}, ${worldSize[1]}, ${worldSize[2]}]
gps_datum: [${gpsDatum[0]}, ${gpsDatum[1]}, ${gpsDatum[2]}]

geofence:
${geofenceYaml}
nagents: ${nagents}

agents:
${agentsYaml}`;

        return yamlContent;
    }
}

// Export the class to global scope
window.YAMLGenerator = YAMLGenerator; 