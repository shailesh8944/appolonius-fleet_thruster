// /**
//  * Main.js - Core Application Logic for Marine Vessel CAD Configurator
//  * 
//  * This file serves as the main entry point and orchestrator for the Marine Vessel CAD Configurator.
//  * It initializes the application, manages UI interactions, handles events, and coordinates
//  * between the data model (VesselModel) and visualization (ThreeScene).
//  * 
//  * Key responsibilities:
//  * - Application initialization and component wiring
//  * - UI event handling and user interaction processing
//  * - Management of toolbars, panels, and sidebars
//  * - File import/export and YAML generation
//  * - Modal dialogs and component configuration
//  * - Theme management and responsive adjustments
//  * - Notification system
//  */

// // Main application logic for the Marine Vessel CAD Configurator
// document.addEventListener('DOMContentLoaded', function() {
//     console.log("Initializing Marine Vessel CAD Configurator...");
    
//     // Initialize core objects
//     try {
//         initializeApplication();
//     } catch (error) {
//         console.error("Error initializing application:", error);
//         // Try again with a delay in case DOM elements aren't fully ready
//         setTimeout(initializeApplication, 100);
//     }
    
//     // Main initialization function
//     function initializeApplication() {
//         try {
//             console.log("Creating ThreeScene...");
//             const threeScene = new ThreeScene();
//             window.threeScene = threeScene; // Make it globally accessible
            
//             console.log("Creating VesselModel...");
//             const vesselModel = new VesselModel('New Vessel', 'auv');
//             window.currentVesselModel = vesselModel;
            
//             // Create initial vessel with default dimensions
//             if (typeof threeScene.createVessel === 'function') {
//                 console.log("Creating default vessel...");
//                 const length = 1.0;
//                 const breadth = 0.5;
//                 const depth = 0.5;
//                 const vessel = threeScene.createVessel(length, breadth, depth);
//                 if (vessel) {
//                     console.log("Default vessel created successfully");
//                 } else {
//                     console.warn("Failed to create default vessel");
//                 }
//             } else {
//                 console.error("createVessel method not found on threeScene");
//             }
            
//             // Populate form fields with initial values
//             const nameInput = document.getElementById('vesselName');
//             const typeSelect = document.getElementById('vesselType');
            
//             if (nameInput) nameInput.value = vesselModel.config.name || 'New Vessel';
//             if (typeSelect) typeSelect.value = vesselModel.config.type || 'auv';
            
//             console.log("Starting animation loop...");
//             threeScene.animate();
            
//             // Initialize UI components
//             initializeTheme();
//             initializeSidebars();
//             initializeToolbar();
//             initializeViewportControls(threeScene);
//             initializeModals(threeScene);
//             initializeFileHandlers(threeScene);
//             initializeYAMLGeneration();
            
//             // Initialize center points toggle
//             initializeCenterPointsControls(threeScene);
            
//             // Show initial notification
//             showNotification('Vessel simulator initialized', 'info');
            
//             console.log("Application initialized successfully");
//         } catch (error) {
//             console.error("Error in initializeApplication:", error);
//             // Display a user-friendly error message
//             const errorDiv = document.createElement('div');
//             errorDiv.style.position = 'fixed';
//             errorDiv.style.top = '50%';
//             errorDiv.style.left = '50%';
//             errorDiv.style.transform = 'translate(-50%, -50%)';
//             errorDiv.style.padding = '20px';
//             errorDiv.style.backgroundColor = '#f8d7da';
//             errorDiv.style.color = '#721c24';
//             errorDiv.style.borderRadius = '5px';
//             errorDiv.style.zIndex = '9999';
//             errorDiv.style.maxWidth = '80%';
//             errorDiv.innerHTML = `
//                 <h4>Application Error</h4>
//                 <p>There was an error initializing the application: ${error.message}</p>
//                 <p>Please check the console for more details.</p>
//                 <button style="padding: 5px 10px; background: #dc3545; color: white; border: none; border-radius: 3px; cursor: pointer;">
//                     Retry
//                 </button>
//             `;
//             document.body.appendChild(errorDiv);
            
//             // Add retry functionality
//             const retryButton = errorDiv.querySelector('button');
//             if (retryButton) {
//                 retryButton.addEventListener('click', () => {
//                     errorDiv.remove();
//                     setTimeout(initializeApplication, 100);
//                 });
//             }
//         }
//     }
    
//     // --------- UI Initialization Functions ---------
    
//     // Initialize sidebars
//     function initializeSidebars() {
//         // Tab switching in sidebars
//         document.querySelectorAll('.sidebar-tab').forEach(tab => {
//             tab.addEventListener('click', function() {
//                 const tabId = this.getAttribute('data-tab');
//                 const sidebar = this.closest('.left-sidebar, .right-sidebar');
                
//                 // Remove active class from all tabs and contents in this sidebar
//                 sidebar.querySelectorAll('.sidebar-tab').forEach(t => t.classList.remove('active'));
//                 sidebar.querySelectorAll('.sidebar-content').forEach(c => c.classList.remove('active'));
                
//                 // Add active class to selected tab and content
//                 this.classList.add('active');
//                 sidebar.querySelector(`#${tabId}`).classList.add('active');
//             });
//         });
        
//         // Hierarchy expand/collapse
//         document.getElementById('expandAll')?.addEventListener('click', function() {
//             document.querySelectorAll('.hierarchy-children').forEach(el => {
//                 el.style.display = 'block';
//                 el.previousElementSibling.querySelector('.hierarchy-toggle').innerHTML = '<i class="bi bi-caret-down-fill"></i>';
//             });
//         });
        
//         document.getElementById('collapseAll')?.addEventListener('click', function() {
//             document.querySelectorAll('.hierarchy-children').forEach(el => {
//                 el.style.display = 'none';
//                 el.previousElementSibling.querySelector('.hierarchy-toggle').innerHTML = '<i class="bi bi-caret-right-fill"></i>';
//             });
//         });
//     }
    
//     // Initialize toolbar
//     function initializeToolbar() {
//         // New vessel
//         document.getElementById('btn-new-vessel')?.addEventListener('click', function() {
//             if (confirm('Create a new vessel? Any unsaved changes will be lost.')) {
//                 window.currentVesselModel = new VesselModel();
                
//                 // Clear any loaded model
//                 if (threeScene.vessel) {
//                     threeScene.scene.remove(threeScene.vessel);
//                     threeScene.vessel = null;
//                 }
                
//                 // Update scene hierarchy
//                 threeScene.updateSceneHierarchy();
//                 threeScene.render();
                
//                 // Clear form fields
//                 document.getElementById('vesselName').value = '';
//                 document.getElementById('vesselType').value = 'auv';
                
//                 // Show notification
//                 showNotification('New vessel created', 'success');
//             }
//         });
        
//         // Save vessel configuration
//         document.getElementById('btn-save-vessel')?.addEventListener('click', async function() {
//             const vesselName = window.currentVesselModel.config.name;
//             if (!vesselName) {
//                 showNotification('Please enter a vessel name before saving', 'error');
//                 return;
//             }
            
//             try {
//                 // Show loading notification when saving with potentially large files
//                 showNotification('Preparing vessel configuration for saving...', 'info');
                
//                 // exportAsJson now returns a promise
//                 const configJson = await window.currentVesselModel.exportAsJson();
                
//                 const blob = new Blob([configJson], { type: 'application/json' });
//                 const url = URL.createObjectURL(blob);
                
//                 const a = document.createElement('a');
//                 a.href = url;
//                 a.download = `${vesselName}_config.json`;
//                 a.click();
                
//                 URL.revokeObjectURL(url);
//                 showNotification('Vessel configuration saved with FBX model and configured components', 'success');
//             } catch (error) {
//                 console.error('Error saving vessel configuration:', error);
//                 showNotification('Failed to save vessel configuration', 'error');
//             }
//         });
        
//         // Load vessel configuration
//         document.getElementById('btn-load-vessel')?.addEventListener('click', function() {
//             const input = document.createElement('input');
//             input.type = 'file';
//             input.accept = '.json';
            
//             input.addEventListener('change', async function() {
//                 if (this.files && this.files[0]) {
//                     const file = this.files[0];
//                     const reader = new FileReader();
                    
//                     reader.onload = async function(e) {
//                         try {
//                             // Show loading notification
//                             showNotification('Loading vessel configuration...', 'info');
                            
//                             const result = window.currentVesselModel.importFromJson(e.target.result);
                            
//                             if (result) {
//                                 // Update UI with loaded values
//                                 document.getElementById('vesselName').value = window.currentVesselModel.config.name;
//                                 document.getElementById('vesselType').value = window.currentVesselModel.config.type;
                                
//                                 // Clear any loaded model
//                                 if (threeScene.vessel) {
//                                     threeScene.scene.remove(threeScene.vessel);
//                                     threeScene.vessel = null;
//                                 }
                                
//                                 // Load FBX model if it exists in the imported data
//                                 if (window.currentVesselModel.modelData.fbxFile) {
//                                     try {
//                                         showNotification('Loading 3D model from saved configuration...', 'info');
//                                         const result = await threeScene.loadFBXModel(window.currentVesselModel.modelData.fbxFile);
                                        
//                                         // Restore component mappings
//                                         if (result.vessel) {
//                                             // Create a new component map with remapped UUIDs
//                                             const oldComponentMap = new Map();
//                                             window.currentVesselModel.modelData.componentMap.forEach((component, oldUuid) => {
//                                                 oldComponentMap.set(oldUuid, component);
//                                             });
                                            
//                                             // Clear the current component map as we'll rebuild it
//                                             window.currentVesselModel.modelData.componentMap.clear();
                                            
//                                             // First collect all components by name
//                                             const meshByName = new Map();
//                                             const meshByIndex = new Map();
//                                             let componentIndex = 0;
                                            
//                                             result.vessel.traverse(child => {
//                                                 if (child.isMesh) {
//                                                     // Store by name for named components
//                                                     if (child.name && child.name !== '') {
//                                                         meshByName.set(child.name, child);
//                                                     }
//                                                     // Also store by index for unnamed components
//                                                     meshByIndex.set(componentIndex++, child);
//                                                 }
//                                             });
                                            
//                                             // Now try to match old components with new meshes
//                                             oldComponentMap.forEach((component, oldUuid) => {
//                                                 // Try to find the component by name first
//                                                 let mesh = null;
                                                
//                                                 if (component.name && component.name !== '') {
//                                                     mesh = meshByName.get(component.name);
//                                                 }
                                                
//                                                 // If not found by name and we have index, try by index
//                                                 if (!mesh && component.index !== undefined) {
//                                                     mesh = meshByIndex.get(component.index);
//                                                 }
                                                
//                                                 // If we found a matching mesh, restore the component
//                                                 if (mesh) {
//                                                     // Save the component to the new UUID
//                                                     window.currentVesselModel.modelData.componentMap.set(mesh.uuid, component);
                                                    
//                                                     // Update mesh properties
//                                                     mesh.userData.isComponentMapped = true;
//                                                     mesh.userData.componentType = component.type;
//                                                     mesh.userData.componentId = component.id;
                                                    
//                                                     // Create visual indicators
//                                                     threeScene.addComponentAxes(mesh);
                                                    
//                                                     // Apply styling based on component type
//                                                     threeScene.applyComponentStyling(mesh, component.type);
                                                    
//                                                     console.log(`Restored component: ${component.name || 'unnamed'} (${component.type})`);
//                                                 } else {
//                                                     console.warn(`Could not find mesh for component: ${component.name || 'unnamed'} (${component.type})`);
//                                                 }
//                                             });
                                            
//                                             showNotification('Components restored successfully', 'success');
//                                         }
                                        
//                                         showNotification('3D model loaded successfully', 'success');
//                                     } catch (error) {
//                                         console.error('Error loading 3D model:', error);
//                                         showNotification('Failed to load 3D model from saved configuration', 'warning');
//                                     }
//                                 }
                                
//                                 // Update scene hierarchy
//                                 threeScene.updateSceneHierarchy();
//                                 threeScene.render();
                                
//                                 showNotification('Vessel configuration loaded successfully', 'success');
//                             } else {
//                                 showNotification('Failed to load configuration', 'error');
//                             }
//                         } catch (error) {
//                             console.error('Error loading vessel configuration:', error);
//                             showNotification('Invalid configuration file', 'error');
//                         }
//                     };
                    
//                     reader.readAsText(file);
//                 }
//             });
            
//             input.click();
//         });
        
//         // Generate YAML
//         document.getElementById('btn-generate-yaml')?.addEventListener('click', function() {
//             // Create bootstrap modal and show it
//             const yamlModal = new bootstrap.Modal(document.getElementById('yamlModal'));
            
//             // Generate YAML previews before showing the modal
//             generateYAMLPreviews();
            
//             yamlModal.show();
//         });
//     }
    
//     // Initialize viewport controls
//     function initializeViewportControls(threeScene) {
//         // Transform mode buttons
//         document.getElementById('btn-translate')?.addEventListener('click', function() {
//             threeScene.setTransformMode('translate');
//             setActiveViewportButton(this);
//         });
        
//         document.getElementById('btn-rotate')?.addEventListener('click', function() {
//             threeScene.setTransformMode('rotate');
//             setActiveViewportButton(this);
//         });
        
//         document.getElementById('btn-scale')?.addEventListener('click', function() {
//             threeScene.setTransformMode('scale');
//             setActiveViewportButton(this);
//         });
        
//         // Toggle grid
//         document.getElementById('btn-toggle-grid')?.addEventListener('click', function() {
//             threeScene.toggleGrid();
//             this.classList.toggle('active');
//         });
        
//         // Toggle wireframe
//         document.getElementById('btn-toggle-wireframe')?.addEventListener('click', function() {
//             threeScene.toggleWireframe();
//             this.classList.toggle('active');
//         });
        
//         // Toggle geometry
//         const geometryToggleBtn = document.getElementById('btn-toggle-geometry');
//         if (geometryToggleBtn) {
//             // Set initial state if vessel exists
//             if (threeScene.vessel && threeScene.vessel.visible) {
//                 geometryToggleBtn.classList.add('active');
//             }
            
//             geometryToggleBtn.addEventListener('click', function() {
//                 if (threeScene.vessel) {
//                     const isVisible = threeScene.toggleGeometry();
                    
//                     // Update active state based on visibility
//                     if (isVisible) {
//                         this.classList.add('active');
//                     } else {
//                         this.classList.remove('active');
//                     }
                    
//                     // Also update checkbox state in the sidebar
//                     const toggleGeometryVisibility = document.getElementById('toggleGeometryVisibility');
//                     if (toggleGeometryVisibility) {
//                         toggleGeometryVisibility.checked = isVisible;
//                     }
                    
//                     showNotification(`3D model ${isVisible ? 'shown' : 'hidden'}`, 'info');
//                 } else {
//                     showNotification('No 3D model loaded', 'warning');
//                     // Reset the toggle to active state if no model is loaded
//                     this.classList.add('active');
//                 }
//             });
//         }
        
//         // Center view
//         document.getElementById('btn-center-view')?.addEventListener('click', function() {
//             threeScene.resetCamera();
//         });
        
//         // Set active button
//         function setActiveViewportButton(button) {
//             document.querySelectorAll('.viewport-btn').forEach(btn => {
//                 btn.classList.remove('active');
//             });
//             button.classList.add('active');
//         }
        
//         // Set translate as default
//         document.getElementById('btn-translate')?.classList.add('active');
        
//         // Add keyboard shortcuts
//         window.addEventListener('keydown', function(event) {
//             if (document.activeElement.tagName === 'INPUT' || 
//                 document.activeElement.tagName === 'TEXTAREA' || 
//                 document.activeElement.tagName === 'SELECT') {
//                 return; // Don't trigger shortcuts when editing form fields
//             }
            
//             switch(event.key.toLowerCase()) {
//                 case 'g':
//                     threeScene.setTransformMode('translate');
//                     setActiveViewportButton(document.getElementById('btn-translate'));
//                     break;
//                 case 'r':
//                     threeScene.setTransformMode('rotate');
//                     setActiveViewportButton(document.getElementById('btn-rotate'));
//                     break;
//                 case 's':
//                     threeScene.setTransformMode('scale');
//                     setActiveViewportButton(document.getElementById('btn-scale'));
//                     break;
//                 case 'escape':
//                     if (threeScene.selectedObject) {
//                         threeScene.selectObject(null);
//                     }
//                     break;
//             }
//         });
//     }
    
//     // Initialize modals
//     function initializeModals(threeScene) {
//         // Control Surface Modal
//         const controlSurfaceModal = document.getElementById('controlSurfaceModal');
//         document.getElementById('btn-add-control-surface')?.addEventListener('click', function() {
//             const modal = new bootstrap.Modal(controlSurfaceModal);
//             modal.show();
//         });
        
//         document.getElementById('btnAddControlSurface')?.addEventListener('click', function() {
//             // Get values from form
//             const type = document.getElementById('controlSurfaceType').value;
//             const surfaceId = parseInt(document.getElementById('controlSurfaceId').value);
//             const position = [
//                 parseFloat(document.getElementById('csPositionX').value),
//                 parseFloat(document.getElementById('csPositionY').value),
//                 parseFloat(document.getElementById('csPositionZ').value)
//             ];
//             const orientation = [
//                 THREE.MathUtils.degToRad(parseFloat(document.getElementById('csOrientationX').value)),
//                 THREE.MathUtils.degToRad(parseFloat(document.getElementById('csOrientationY').value)),
//                 THREE.MathUtils.degToRad(parseFloat(document.getElementById('csOrientationZ').value))
//             ];
//             const area = parseFloat(document.getElementById('csArea').value);
//             const naca = document.getElementById('csNACA').value;
//             const timeConstant = parseFloat(document.getElementById('csTimeConstant').value);
//             const deltaMax = parseFloat(document.getElementById('csDeltaMax').value);
//             const deltadMax = parseFloat(document.getElementById('csDeltadMax').value);
            
//             console.log('Adding control surface with params:', { type, surfaceId, position, orientation, area });
            
//             // Debug before adding
//             console.log('Before adding control surface:');
//             logComponentCounts();
            
//             // Add control surface
//             const id = window.currentVesselModel.addControlSurface(
//                 type, position, orientation, area, surfaceId, naca, timeConstant, deltaMax, deltadMax
//             );
            
//             // Debug after adding
//             console.log('After adding control surface:');
//             logComponentCounts();
            
//             // Add visual representation if threeScene has a method for it
//             let controlSurfaceObj = null;
//             if (typeof threeScene.addControlSurface === 'function') {
//                 controlSurfaceObj = threeScene.addControlSurface(id, type, position, orientation, area);
                
//                 // Add component axes visualization
//                 if (controlSurfaceObj) {
//                     threeScene.addComponentAxes(controlSurfaceObj);
//                 }
//             }
            
//             // Close modal
//             bootstrap.Modal.getInstance(controlSurfaceModal).hide();
            
//             // Generate YAML previews to verify control surfaces are included
//             generateYAMLPreviews();
            
//             // Show notification
//             showNotification(`Added ${type} control surface (ID: ${id})`, 'success');
//         });
        
//         // Thruster Modal
//         const thrusterModal = document.getElementById('thrusterModal');
//         document.getElementById('btn-add-thruster')?.addEventListener('click', function() {
//             const modal = new bootstrap.Modal(thrusterModal);
//             modal.show();
//         });
        
//         document.getElementById('btnAddThruster')?.addEventListener('click', function() {
//             // Get values from form
//             const name = document.getElementById('thrusterName').value;
//             const thrusterId = parseInt(document.getElementById('thrusterId').value);
//             const position = [
//                 parseFloat(document.getElementById('thrusterPositionX').value),
//                 parseFloat(document.getElementById('thrusterPositionY').value),
//                 parseFloat(document.getElementById('thrusterPositionZ').value)
//             ];
//             const orientation = [
//                 THREE.MathUtils.degToRad(parseFloat(document.getElementById('thrusterOrientationX').value)),
//                 THREE.MathUtils.degToRad(parseFloat(document.getElementById('thrusterOrientationY').value)),
//                 THREE.MathUtils.degToRad(parseFloat(document.getElementById('thrusterOrientationZ').value))
//             ];
//             const tProp = parseFloat(document.getElementById('thrusterTProp').value);
//             const diameter = parseFloat(document.getElementById('thrusterDiameter').value);
//             const tp = parseFloat(document.getElementById('thrusterTp').value);
//             const jvsKTFile = document.getElementById('thrusterJvsKTFile').value;
//             const ktAtJ0 = parseFloat(document.getElementById('thrusterKTatJ0').value);
//             const nMax = parseFloat(document.getElementById('thrusterNMax').value);
            
//             // Add thruster
//             const id = window.currentVesselModel.addThruster(
//                 name, position, orientation, thrusterId, diameter, tProp, tp, jvsKTFile, ktAtJ0, nMax
//             );
            
//             // Add visual representation if threeScene has a method for it
//             let thrusterObj = null;
//             if (typeof threeScene.addThruster === 'function') {
//                 thrusterObj = threeScene.addThruster(id, name, position, orientation, diameter);
                
//                 // Add component axes visualization
//                 if (thrusterObj) {
//                     threeScene.addComponentAxes(thrusterObj);
//                 }
//             }
            
//             // Close modal
//             bootstrap.Modal.getInstance(thrusterModal).hide();
            
//             // Show notification
//             showNotification(`Added ${name} thruster (ID: ${id})`, 'success');
//         });
        
//         // Sensor Modal
//         const sensorModal = document.getElementById('sensorModal');
//         document.getElementById('btn-add-sensor')?.addEventListener('click', function() {
//             const modal = new bootstrap.Modal(sensorModal);
//             modal.show();
//         });
        
//         document.getElementById('btnAddSensor')?.addEventListener('click', function() {
//             // Get values from form
//             const type = document.getElementById('sensorType').value;
//             const useNoneLocation = document.getElementById('sensorNoneLocation').checked;
//             const useNoneOrientation = document.getElementById('sensorNoneOrientation').checked;
            
//             let position = [0, 0, 0];
//             if (!useNoneLocation) {
//                 position = [
//                     parseFloat(document.getElementById('sensorPositionX').value),
//                     parseFloat(document.getElementById('sensorPositionY').value),
//                     parseFloat(document.getElementById('sensorPositionZ').value)
//                 ];
//             }
            
//             let orientation = [1.0, 0.0, 0.0, 0.0]; // Quaternion [w, x, y, z]
//             if (!useNoneOrientation) {
//                 orientation = [
//                     parseFloat(document.getElementById('sensorOrientationW').value),
//                     parseFloat(document.getElementById('sensorOrientationX').value),
//                     parseFloat(document.getElementById('sensorOrientationY').value),
//                     parseFloat(document.getElementById('sensorOrientationZ').value)
//                 ];
//             }
            
//             const publishRate = parseFloat(document.getElementById('sensorPublishRate').value);
            
//             // Add sensor
//             const id = window.currentVesselModel.addSensor(
//                 type, position, orientation, publishRate, useNoneLocation, useNoneOrientation
//             );
            
//             // Add visual representation if threeScene has a method for it
//             let sensorObj = null;
//             if (typeof threeScene.addSensor === 'function') {
//                 sensorObj = threeScene.addSensor(id, type, useNoneLocation ? null : position, useNoneOrientation ? null : orientation);
                
//                 // Add component axes visualization if location is not None
//                 if (sensorObj && !useNoneLocation) {
//                     threeScene.addComponentAxes(sensorObj);
//                 }
//             }
            
//             // Close modal
//             bootstrap.Modal.getInstance(sensorModal).hide();
            
//             // Show notification
//             showNotification(`Added ${type} sensor (ID: ${id})`, 'success');
//         });
        
//         // FBX Component Modal
//         const fbxComponentModal = document.getElementById('fbxComponentModal');
//         document.getElementById('btnSaveFbxComponent')?.addEventListener('click', function() {
//             // Implementation to be added when handling FBX component selection
//             bootstrap.Modal.getInstance(fbxComponentModal).hide();
//         });
        
//         // FBX Component modal event handler
//         document.getElementById('btnApplyFbxComponent')?.addEventListener('click', () => {
//             const modal = document.getElementById('fbxComponentModal');
//             const objectUuid = modal?.dataset.objectUuid;
//             const object = threeScene.getObjectByProperty('uuid', objectUuid);
            
//             // Verify IDs are valid before proceeding, but skip validation if reconfiguring the same component
//             if (document.getElementById('fbxControlSurfaceSettings').style.display === 'block') {
//                 const surfaceIdInput = document.getElementById('fbxSurfaceId');
//                 const id = parseInt(surfaceIdInput.value);
//                 const isEditingSameComponent = object && 
//                                           object.userData.componentId === id && 
//                                           object.userData.componentType === 'controlSurface';
                
//                 // Only show error if invalid and not editing the same component
//                 if (surfaceIdInput.classList.contains('is-invalid') && !isEditingSameComponent) {
//                     showNotification('Control surface ID is already in use. Please use a different ID.', 'error');
//                     return;
//                 }
//             }
            
//             if (document.getElementById('fbxThrusterSettings').style.display === 'block') {
//                 const thrusterIdInput = document.getElementById('fbxThrusterId');
//                 const id = parseInt(thrusterIdInput.value);
//                 const isEditingSameComponent = object && 
//                                           object.userData.componentId === id && 
//                                           object.userData.componentType === 'thruster';
                
//                 // Only show error if invalid and not editing the same component
//                 if (thrusterIdInput.classList.contains('is-invalid') && !isEditingSameComponent) {
//                     showNotification('Thruster ID is already in use. Please use a different ID.', 'error');
//                     return;
//                 }
//             }
            
//             if (document.getElementById('fbxSensorSettings').style.display === 'block') {
//                 const sensorIdInput = document.getElementById('fbxSensorId');
//                 const id = parseInt(sensorIdInput.value);
//                 const isEditingSameComponent = object && 
//                                           object.userData.componentId === id && 
//                                           object.userData.componentType === 'sensor';
                
//                 // Only show error if invalid and not editing the same component
//                 if (sensorIdInput.classList.contains('is-invalid') && !isEditingSameComponent) {
//                     showNotification('Sensor ID is already in use. Please use a different ID.', 'error');
//                     return;
//                 }
//             }
            
//             if (!objectUuid) {
//                 console.error('No object UUID found in modal');
//                 return;
//             }
            
//             if (!object) {
//                 console.error('Object not found with UUID:', objectUuid);
//                 return;
//             }
            
//             const componentType = document.getElementById('fbxComponentType').value;
//             let componentData = {
//                 type: componentType,
//                 uuid: objectUuid,
//                 name: object.name,
//                 index: threeScene.getObjectIndex(object)
//             };
            
//             // Get component center position in world coordinates
//             const center = new THREE.Vector3();
//             // Calculate the center of the object's bounding box
//             if (object.geometry) {
//                 // For objects with geometry, calculate from geometry bounds
//                 object.geometry.computeBoundingBox();
//                 object.geometry.boundingBox.getCenter(center);
//                 object.localToWorld(center);
//             } else {
//                 // For groups or objects without geometry, use position
//                 center.copy(object.position);
//             }
            
//             // Use center as component's position
//             const position = [
//                 center.x,
//                 center.y,
//                 center.z
//             ];

//             // Get specific data based on component type
//             if (componentType === 'controlSurface') {
//                 const surfaceType = document.getElementById('fbxSurfaceType').value;
//                 const surfaceId = parseInt(document.getElementById('fbxSurfaceId').value);
//                 const surfaceName = document.getElementById('fbxSurfaceName').value || `${surfaceType} Surface`;
//                 const area = parseFloat(document.getElementById('fbxSurfaceArea').value);
//                 const naca = document.getElementById('fbxSurfaceNACA').value;
//                 const timeConstant = parseFloat(document.getElementById('fbxSurfaceTimeConstant').value);
//                 const deltaMax = parseFloat(document.getElementById('fbxSurfaceDeltaMax').value);
//                 const deltadMax = parseFloat(document.getElementById('fbxSurfaceDeltadMax').value);
                
//                 // Get orientation in radians
//                 const orientation = [0, 0, 0]; // Default orientation, should be updated based on object
                
//                 // Get the actual world position and orientation of the object
//                 const worldPosition = new THREE.Vector3();
//                 const worldQuaternion = new THREE.Quaternion();
                
//                 // FIXED: Always prioritize using axes for position and orientation
//                 // First, try to get existing axes
//                 let axes = object.children.find(child => child.userData.isComponentAxes);
                
//                 // If axes don't exist yet, create them first
//                 if (!axes) {
//                     axes = threeScene.addComponentAxes(object);
//                     console.log("Created new axes for component position/orientation");
//                 }
                
//                 // Now use the axes for position and orientation
//                 if (axes) {
//                     axes.getWorldPosition(worldPosition);
//                     axes.getWorldQuaternion(worldQuaternion);
//                     console.log("Using axes for position and orientation");
//                 } else {
//                     // Fallback only if axes creation failed for some reason
//                     object.getWorldPosition(worldPosition);
//                     object.getWorldQuaternion(worldQuaternion);
//                     console.log("Fallback: Using object position and orientation");
//                 }
                
//                 // Convert to Euler angles for orientation
//                 const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
                
//                 // Use the transformed position and orientation
//                 position[0] = worldPosition.x;
//                 position[1] = worldPosition.y;
//                 position[2] = worldPosition.z;
                
//                 orientation[0] = worldEuler.x;
//                 orientation[1] = worldEuler.y;
//                 orientation[2] = worldEuler.z;
                
//                 console.log("Using transformed position:", position);
//                 console.log("Using transformed orientation:", orientation);
                
//                 let id = surfaceId;
//                 // Check if this control surface already exists
//                 const existingSurface = window.currentVesselModel.getControlSurface(surfaceId);
                
//                 // Get the original component ID from the userData if it exists
//                 const originalComponentId = object.userData.componentId;
                
//                 // If the component existed but with a different ID, remove the old one first
//                 if (originalComponentId && originalComponentId !== surfaceId) {
//                     console.log(`ID changed from ${originalComponentId} to ${surfaceId}, removing old component`);
//                     window.currentVesselModel.removeControlSurface(originalComponentId);
//                 }
                
//                 if (existingSurface) {
//                     // Update existing control surface
//                     window.currentVesselModel.updateControlSurface(surfaceId, {
//                         control_surface_type: surfaceType,
//                         control_surface_location: position,
//                         control_surface_orientation: orientation,
//                         control_surface_area: area,
//                         control_surface_T: timeConstant,
//                         control_surface_delta_max: deltaMax,
//                         control_surface_deltad_max: deltadMax,
//                         control_surface_NACA: naca
//                     });
//                     console.log(`Updated existing control surface with ID ${surfaceId}`);
//                 } else {
//                     // Add new control surface
//                     id = window.currentVesselModel.addControlSurface(
//                         surfaceType, 
//                         position, 
//                         orientation, 
//                         area, 
//                         surfaceId, 
//                         naca, 
//                         timeConstant, 
//                         deltaMax, 
//                         deltadMax
//                     );
//                     console.log(`Added new control surface with ID ${id}`);
//                 }
                
//                 componentData = {
//                     ...componentData,
//                     surfaceType,
//                     id,
//                     name: surfaceName,
//                     area,
//                     naca,
//                     timeConstant,
//                     deltaMax,
//                     deltadMax,
//                     position
//                 };
                
//                 object.name = surfaceName;
                
//                 // Add axes to the component
//                 threeScene.addComponentAxes(object);
                
//             } else if (componentType === 'thruster') {
//                 const thrusterName = document.getElementById('fbxThrusterName').value || 'Thruster';
//                 const thrusterId = parseInt(document.getElementById('fbxThrusterId').value);
//                 const tProp = parseFloat(document.getElementById('fbxThrusterTProp').value);
//                 const diameter = parseFloat(document.getElementById('fbxThrusterDiameter').value);
//                 const tp = parseFloat(document.getElementById('fbxThrusterTp').value);
//                 const jvsKTFile = document.getElementById('fbxThrusterJvsKTFile').value;
//                 const ktAtJ0 = parseFloat(document.getElementById('fbxThrusterKTatJ0').value);
//                 const nMax = parseFloat(document.getElementById('fbxThrusterNMax').value);
                
//                 // Get orientation in radians
//                 const orientation = [0, 0, 0]; // Default orientation, should be updated based on object
                
//                 // Get the actual world position and orientation of the object
//                 const worldPosition = new THREE.Vector3();
//                 const worldQuaternion = new THREE.Quaternion();
                
//                 // FIXED: Always prioritize using axes for position and orientation
//                 // First, try to get existing axes
//                 let axes = object.children.find(child => child.userData.isComponentAxes);
                
//                 // If axes don't exist yet, create them first
//                 if (!axes) {
//                     axes = threeScene.addComponentAxes(object);
//                     console.log("Created new axes for component position/orientation");
//                 }
                
//                 // Now use the axes for position and orientation
//                 if (axes) {
//                     axes.getWorldPosition(worldPosition);
//                     axes.getWorldQuaternion(worldQuaternion);
//                     console.log("Using axes for position and orientation");
//                 } else {
//                     // Fallback only if axes creation failed for some reason
//                     object.getWorldPosition(worldPosition);
//                     object.getWorldQuaternion(worldQuaternion);
//                     console.log("Fallback: Using object position and orientation");
//                 }
                
//                 // Convert to Euler angles for orientation
//                 const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
                
//                 // Use the transformed position and orientation
//                 position[0] = worldPosition.x;
//                 position[1] = worldPosition.y;
//                 position[2] = worldPosition.z;
                
//                 orientation[0] = worldEuler.x;
//                 orientation[1] = worldEuler.y;
//                 orientation[2] = worldEuler.z;
                
//                 console.log("Using transformed position:", position);
//                 console.log("Using transformed orientation:", orientation);
                
//                 let id = thrusterId;
//                 // Check if this thruster already exists
//                 const existingThruster = window.currentVesselModel.getThruster(thrusterId);
                
//                 // Get the original component ID from the userData if it exists
//                 const originalThrusterId = object.userData.componentId;
                
//                 // If the component existed but with a different ID, remove the old one first
//                 if (originalThrusterId && originalThrusterId !== thrusterId) {
//                     console.log(`Thruster ID changed from ${originalThrusterId} to ${thrusterId}, removing old component`);
//                     window.currentVesselModel.removeThruster(originalThrusterId);
//                 }
                
//                 if (existingThruster) {
//                     // Update existing thruster
//                     window.currentVesselModel.updateThruster(thrusterId, {
//                         thruster_name: thrusterName,
//                         thruster_location: position,
//                         thruster_orientation: orientation,
//                         D_prop: diameter,
//                         T_prop: tProp,
//                         tp: tp,
//                         KT_at_J0: ktAtJ0,
//                         n_max: nMax
//                     });
//                     console.log(`Updated existing thruster with ID ${thrusterId}`);
//                 } else {
//                     // Add new thruster
//                     id = window.currentVesselModel.addThruster(
//                         thrusterName, 
//                         position, 
//                         orientation, 
//                         thrusterId, 
//                         diameter, 
//                         tProp, 
//                         tp, 
//                         jvsKTFile,
//                         ktAtJ0,
//                         nMax
//                     );
//                     console.log(`Added new thruster with ID ${id}`);
//                 }
                
//                 componentData = {
//                     ...componentData,
//                     id,
//                     name: thrusterName,
//                     tProp,
//                     diameter,
//                     tp,
//                     jvsKTFile,
//                     ktAtJ0,
//                     nMax,
//                     position
//                 };
                
//                 object.name = thrusterName;
                
//                 // Add axes to the component
//                 threeScene.addComponentAxes(object);
                
//             } else if (componentType === 'sensor') {
//                 const sensorType = document.getElementById('fbxSensorType').value;
//                 const sensorName = document.getElementById('fbxSensorName').value || `${sensorType} Sensor`;
//                 const sensorRate = parseFloat(document.getElementById('fbxSensorRate').value);
//                 const useNoneLocation = document.getElementById('fbxSensorNoneLocation').checked;
//                 const useNoneOrientation = document.getElementById('fbxSensorNoneOrientation').checked;
                
//                 // Get the sensor ID from the input field
//                 const sensorId = parseInt(document.getElementById('fbxSensorId').value);
                
//                 // Get orientation in radians
//                 const orientation = [0, 0, 0]; // Default orientation, should be updated based on object
                
//                 // Get the actual world position and orientation of the object
//                 const worldPosition = new THREE.Vector3();
//                 const worldQuaternion = new THREE.Quaternion();
                
//                 // FIXED: Always prioritize using axes for position and orientation
//                 // First, try to get existing axes
//                 let axes = object.children.find(child => child.userData.isComponentAxes);
                
//                 // If axes don't exist yet, create them first
//                 if (!axes) {
//                     axes = threeScene.addComponentAxes(object);
//                     console.log("Created new axes for component position/orientation");
//                 }
                
//                 // Now use the axes for position and orientation
//                 if (axes) {
//                     axes.getWorldPosition(worldPosition);
//                     axes.getWorldQuaternion(worldQuaternion);
//                     console.log("Using axes for position and orientation");
//                 } else {
//                     // Fallback only if axes creation failed for some reason
//                     object.getWorldPosition(worldPosition);
//                     object.getWorldQuaternion(worldQuaternion);
//                     console.log("Fallback: Using object position and orientation");
//                 }
                
//                 // Convert to Euler angles for orientation
//                 const worldEuler = new THREE.Euler().setFromQuaternion(worldQuaternion);
                
//                 // Use the transformed position and orientation
//                 position[0] = worldPosition.x;
//                 position[1] = worldPosition.y;
//                 position[2] = worldPosition.z;
                
//                 orientation[0] = worldEuler.x;
//                 orientation[1] = worldEuler.y;
//                 orientation[2] = worldEuler.z;
                
//                 console.log("Using transformed position:", position);
//                 console.log("Using transformed orientation:", orientation);
                
//                 let id = sensorId;
//                 // Check if this sensor already exists
//                 const existingSensor = sensorId ? window.currentVesselModel.getSensor(sensorId) : null;
                
//                 // Get the original component ID from the userData if it exists
//                 const originalSensorId = object.userData.componentId;
                
//                 // If the component existed but with a different ID, remove the old one first
//                 if (originalSensorId && originalSensorId !== sensorId) {
//                     console.log(`Sensor ID changed from ${originalSensorId} to ${sensorId}, removing old component`);
//                     window.currentVesselModel.removeSensor(originalSensorId);
//                 }
                
//                 if (existingSensor) {
//                     // Update existing sensor
//                     window.currentVesselModel.updateSensor(sensorId, {
//                         sensor_type: sensorType,
//                         sensor_location: useNoneLocation ? null : position,
//                         sensor_orientation: useNoneOrientation ? null : orientation,
//                         publish_rate: sensorRate
//                     });
//                     console.log(`Updated existing sensor with ID ${sensorId}`);
//                 } else {
//                     // Add new sensor
//                     id = window.currentVesselModel.addSensor(
//                         sensorType,
//                         useNoneLocation ? null : position,
//                         useNoneOrientation ? null : orientation,
//                         sensorRate,
//                         useNoneLocation,
//                         useNoneOrientation
//                     );
//                     console.log(`Added new sensor with ID ${id}`);
//                 }
                
//                 componentData = {
//                     ...componentData,
//                     sensorType,
//                     id,
//                     name: sensorName,
//                     publishRate: sensorRate,
//                     useNoneLocation,
//                     useNoneOrientation,
//                     position: useNoneLocation ? null : position,
//                     orientation: useNoneOrientation ? null : orientation
//                 };
                
//                 object.name = sensorName;
                
//                 // Add axes to the component if location is not None
//                 if (!useNoneLocation) {
//                     threeScene.addComponentAxes(object);
//                 }
//             }
            
//             // Map the component in the vessel model
//             if (window.currentVesselModel) {
//                 window.currentVesselModel.mapModelComponent(objectUuid, componentData);
//                 console.log(`Component ${componentData.name || object.name} configured as ${componentType}`);
                
//                 // CRITICAL FIX: Set component ID and type directly on the object's userData
//                 // This ensures the object itself always has the correct identification
//                 object.userData.componentId = componentData.id;
//                 object.userData.componentType = componentType;
//                 console.log(`Set component ID ${componentData.id} and type ${componentType} directly on object:`, object.name);
                
//                 // If there are existing axes, make sure they have the correct component ID and type
//                 const axes = object.children.find(child => child.userData.isComponentAxes);
//                 if (axes) {
//                     axes.userData.componentId = componentData.id;
//                     axes.userData.componentType = componentType;
//                     console.log(`Updated existing axes with component ID ${componentData.id} and type ${componentType}`);
//                 } else {
//                     // Add axes if they don't exist yet
//                     const newAxes = threeScene.addComponentAxes(object);
//                     console.log(`Added new axes to component:`, newAxes);
//                 }
                
//                 // Update the scene hierarchy
//                 threeScene.updateSceneHierarchy();
                
//                 // Re-select the object to update UI
//                 threeScene.selectObject(object);
                
//                 // Hide the modal
//                 const bsModal = bootstrap.Modal.getInstance(modal);
//                 bsModal.hide();
                
//                 // Show success notification
//                 showNotification(`Component ${componentData.name || object.name} configured successfully`, 'success');
//             } else {
//                 console.error('No vessel model available');
//                 showNotification('Failed to configure component: No vessel model available', 'error');
//             }
//         });
        
//         // Handle showing the modal and prepopulating fields
//         document.getElementById('fbxComponentModal')?.addEventListener('show.bs.modal', (event) => {
//             const modal = event.target;
//             const objectUuid = modal.dataset.objectUuid;
            
//             if (!objectUuid) return;
            
//             const object = threeScene.getObjectByProperty('uuid', objectUuid);
//             if (!object) return;
            
//             const componentData = window.currentVesselModel?.getModelComponentData(objectUuid);
//             const componentType = document.getElementById('fbxComponentType').value;
            
//             // Hide all settings panels first
//             document.getElementById('fbxControlSurfaceSettings').style.display = 'none';
//             document.getElementById('fbxThrusterSettings').style.display = 'none';
//             document.getElementById('fbxSensorSettings').style.display = 'none';
            
//             // Show the appropriate panel and set component name in the appropriate field based on type
//             if (componentType === 'controlSurface') {
//                 document.getElementById('fbxControlSurfaceSettings').style.display = 'block';
//                 document.getElementById('fbxSurfaceName').value = object.name;
                
//                 // Set default values or restore saved values
//                 if (componentData) {
//                     document.getElementById('fbxSurfaceType').value = componentData.surfaceType || 'Rudder';
//                     document.getElementById('fbxSurfaceId').value = componentData.id || 1;
//                     document.getElementById('fbxSurfaceArea').value = componentData.area || 0.1;
//                     document.getElementById('fbxSurfaceNACA').value = componentData.naca || '0015';
//                     document.getElementById('fbxSurfaceTimeConstant').value = componentData.timeConstant || 0.1;
//                     document.getElementById('fbxSurfaceDeltaMax').value = componentData.deltaMax || 35.0;
//                     document.getElementById('fbxSurfaceDeltadMax').value = componentData.deltadMax || 1.0;
//                 }
//             } else if (componentType === 'thruster') {
//                 document.getElementById('fbxThrusterSettings').style.display = 'block';
//                 document.getElementById('fbxThrusterName').value = object.name;
                
//                 if (componentData) {
//                     document.getElementById('fbxThrusterId').value = componentData.id || 1;
//                     document.getElementById('fbxThrusterTProp').value = componentData.tProp || 1.0;
//                     document.getElementById('fbxThrusterDiameter').value = componentData.diameter || 0.1;
//                     document.getElementById('fbxThrusterTp').value = componentData.tp || 0.1;
//                     document.getElementById('fbxThrusterJvsKTFile').value = componentData.jvsKTFile || 'J_vs_KT.csv';
//                 }
//             } else if (componentType === 'sensor') {
//                 document.getElementById('fbxSensorSettings').style.display = 'block';
//                 document.getElementById('fbxSensorName').value = object.name;
                
//                 if (componentData) {
//                     document.getElementById('fbxSensorType').value = componentData.sensorType || 'IMU';
//                     document.getElementById('fbxSensorId').value = componentData.id || 1;
//                     document.getElementById('fbxSensorRate').value = componentData.publishRate || 10;
                    
//                     // Handle None values for location and orientation
//                     const useNoneLocation = componentData.useNoneLocation || false;
//                     const useNoneOrientation = componentData.useNoneOrientation || false;
                    
//                     document.getElementById('fbxSensorNoneLocation').checked = useNoneLocation;
//                     document.getElementById('fbxSensorNoneOrientation').checked = useNoneOrientation;
                    
//                     // Update UI visibility based on None settings
//                     document.getElementById('fbxSensorOrientationGroup').style.display = 
//                         useNoneOrientation ? 'none' : 'block';
                    
//                     // Set orientation values if available
//                     if (componentData.orientation && !useNoneOrientation) {
//                         document.getElementById('fbxSensorOrientationW').value = componentData.orientation[0] || 1.0;
//                         document.getElementById('fbxSensorOrientationX').value = componentData.orientation[1] || 0.0;
//                         document.getElementById('fbxSensorOrientationY').value = componentData.orientation[2] || 0.0;
//                         document.getElementById('fbxSensorOrientationZ').value = componentData.orientation[3] || 0.0;
//                     }
//                 }
                
//                 // Trigger change event to handle visibility of location/orientation fields
//                 const event = new Event('change');
//                 document.getElementById('fbxSensorType').dispatchEvent(event);
//             }
//         });
        
//         // Add handler for Edit Hydrodynamics button
//         document.getElementById('btnEditHydrodynamics')?.addEventListener('click', function() {
//             // Get current hydrodynamics values from vessel model
//             const hydrodynamics = window.currentVesselModel.config.hydrodynamics;
//             console.log("Opening hydrodynamics modal with current data:", hydrodynamics);
            
//             // Populate the checkboxes
//             document.getElementById('dimFlag').checked = hydrodynamics.dim_flag || false;
//             document.getElementById('crossFlowDrag').checked = hydrodynamics.cross_flow_drag || false;
            
//             // Generate text content for the textarea based on current hydrodynamics
//             const textContent = Object.entries(hydrodynamics)
//                 .filter(([key]) => key !== 'dim_flag' && key !== 'cross_flow_drag' && key !== 'hydra_file')
//                 .map(([key, value]) => `${key}: ${value}`)
//                 .join('\n');
            
//             document.getElementById('hydroCoefficients').value = textContent;
            
//             // Show the modal
//             const hydrodynamicsModal = new bootstrap.Modal(document.getElementById('hydrodynamicsModal'));
//             hydrodynamicsModal.show();
//         });
        
//         // Add handler for file upload
//         document.getElementById('hydroFileUpload')?.addEventListener('change', function(event) {
//             const file = event.target.files[0];
//             if (file) {
//                 const reader = new FileReader();
//                 reader.onload = function(e) {
//                     document.getElementById('hydroCoefficients').value = e.target.result;
//                 };
//                 reader.readAsText(file);
//             }
//         });
        
//         // Add handler for Save Hydrodynamics button
//         document.getElementById('btnSaveHydrodynamics')?.addEventListener('click', function() {
//             // Get values from the form
//             const coefficientText = document.getElementById('hydroCoefficients').value;
            
//             // Parse the text into key-value pairs
//             const updatedHydrodynamics = {
//                 dim_flag: document.getElementById('dimFlag').checked,
//                 cross_flow_drag: document.getElementById('crossFlowDrag').checked,
//                 // Keep the existing hydra_file value
//                 hydra_file: window.currentVesselModel.config.hydrodynamics.hydra_file || ''
//             };
            
//             // Track parsing issues for user feedback
//             const parseIssues = [];
            
//             // Process each line of the text area
//             const lines = coefficientText.split('\n');
//             for (let i = 0; i < lines.length; i++) {
//                 const line = lines[i].trim();
                
//                 // Skip empty lines and comment lines
//                 if (!line || line.startsWith('#') || line.startsWith('//')) continue;
                
//                 // Parse key-value pairs (allow for different separators)
//                 const match = line.match(/([^:=,\s]+)\s*[:=,]\s*(-?\d+\.?\d*)/);
//                 if (match) {
//                     const [, key, valueStr] = match;
//                     const cleanKey = key.trim();
//                     const numValue = parseFloat(valueStr);
                    
//                     if (isNaN(numValue)) {
//                         parseIssues.push(`Line ${i+1}: "${valueStr}" is not a valid number`);
//                     } else {
//                         updatedHydrodynamics[cleanKey] = numValue;
//                     }
//                 } else if (line.includes(':') || line.includes('=')) {
//                     // Line has a separator but didn't parse correctly
//                     parseIssues.push(`Line ${i+1}: Format error in "${line}"`);
//                 }
//             }
            
//             console.log("Parsed hydrodynamics to save:", updatedHydrodynamics);
            
//             // Show warning if there were parse issues
//             if (parseIssues.length > 0) {
//                 const message = `Warning: Some lines couldn't be parsed:\n${parseIssues.slice(0, 3).join('\n')}${parseIssues.length > 3 ? `\n...and ${parseIssues.length - 3} more issues` : ''}`;
//                 showNotification(message, 'warning');
//             }
            
//             // Update the vessel model
//             window.currentVesselModel.updateHydrodynamics(updatedHydrodynamics);
            
//             // Verify the update worked
//             console.log("Vessel hydrodynamics after update:", window.currentVesselModel.config.hydrodynamics);
            
//             // Generate the hydrodynamics YAML directly and update the preview
//             const yamlGenerator = new YAMLGenerator();
//             const hydroYaml = yamlGenerator.generateHydrodynamicsYAML(
//                 window.currentVesselModel.config.hydrodynamics, 
//                 window.currentVesselModel.config.name
//             );
//             document.getElementById('hydrodynamics-yaml-content').textContent = hydroYaml;
            
//             // Also update all other YAML previews
//             generateYAMLPreviews();
            
//             // Close the modal
//             const hydrodynamicsModal = bootstrap.Modal.getInstance(document.getElementById('hydrodynamicsModal'));
//             hydrodynamicsModal.hide();
            
//             // Show success notification
//             const coeffCount = Object.keys(updatedHydrodynamics).length - 3; // Subtract dim_flag, cross_flow_drag, hydra_file
//             showNotification(`Hydrodynamics updated with ${coeffCount} coefficients`, 'success');
//         });
        
//         // Add handler for Edit Initial Conditions button
//         document.getElementById('btnEditInitialConditions')?.addEventListener('click', function() {
//             try {
//                 // Get current initial conditions from the vessel model
//                 const initialConditions = window.currentVesselModel.config.initial_conditions || {
//                     start_location: [0, 0, 0],
//                     start_orientation: [0, 0, 0],
//                     start_velocity: [0, 0, 0, 0, 0, 0],
//                     use_quaternion: false
//                 };
                
//                 // Populate the form fields with current values
//                 const startLocation = initialConditions.start_location || [0, 0, 0];
//                 document.getElementById('startLocationX').value = startLocation[0];
//                 document.getElementById('startLocationY').value = startLocation[1];
//                 document.getElementById('startLocationZ').value = startLocation[2];
                
//                 const startOrientation = initialConditions.start_orientation || [0, 0, 0];
//                 document.getElementById('startOrientationRoll').value = startOrientation[0];
//                 document.getElementById('startOrientationPitch').value = startOrientation[1];
//                 document.getElementById('startOrientationYaw').value = startOrientation[2];
                
//                 const startVelocity = initialConditions.start_velocity || [0, 0, 0, 0, 0, 0];
//                 document.getElementById('startVelocityU').value = startVelocity[0];
//                 document.getElementById('startVelocityV').value = startVelocity[1];
//                 document.getElementById('startVelocityW').value = startVelocity[2];
//                 document.getElementById('startVelocityP').value = startVelocity[3];
//                 document.getElementById('startVelocityQ').value = startVelocity[4];
//                 document.getElementById('startVelocityR').value = startVelocity[5];
                
//                 document.getElementById('useQuaternion').checked = initialConditions.use_quaternion || false;
                
//                 console.log("Opening initial conditions modal with:", initialConditions);
                
//                 // Show the modal
//                 const initialConditionsModal = new bootstrap.Modal(document.getElementById('initialConditionsModal'));
//                 initialConditionsModal.show();
//             } catch (error) {
//                 console.error("Error opening initial conditions modal:", error);
//                 showNotification("Error loading initial conditions", "error");
//             }
//         });
        
//         // Add handler for initial conditions file upload
//         document.getElementById('initialConditionsFileUpload')?.addEventListener('change', function(event) {
//             const file = event.target.files[0];
//             if (file) {
//                 const reader = new FileReader();
//                 reader.onload = function(e) {
//                     try {
//                         // Parse the YAML and populate form fields
//                         const initialConditions = jsyaml.load(e.target.result);
                        
//                         if (initialConditions && initialConditions.start_location) {
//                             document.getElementById('startLocationX').value = initialConditions.start_location[0];
//                             document.getElementById('startLocationY').value = initialConditions.start_location[1];
//                             document.getElementById('startLocationZ').value = initialConditions.start_location[2];
//                         }
                        
//                         if (initialConditions && initialConditions.start_orientation) {
//                             document.getElementById('startOrientationRoll').value = initialConditions.start_orientation[0];
//                             document.getElementById('startOrientationPitch').value = initialConditions.start_orientation[1];
//                             document.getElementById('startOrientationYaw').value = initialConditions.start_orientation[2];
//                         }
                        
//                         if (initialConditions && initialConditions.start_velocity) {
//                             document.getElementById('startVelocityU').value = initialConditions.start_velocity[0];
//                             document.getElementById('startVelocityV').value = initialConditions.start_velocity[1];
//                             document.getElementById('startVelocityW').value = initialConditions.start_velocity[2];
//                             document.getElementById('startVelocityP').value = initialConditions.start_velocity[3];
//                             document.getElementById('startVelocityQ').value = initialConditions.start_velocity[4];
//                             document.getElementById('startVelocityR').value = initialConditions.start_velocity[5];
//                         }
                        
//                         if (initialConditions && initialConditions.hasOwnProperty('use_quaternion')) {
//                             document.getElementById('useQuaternion').checked = initialConditions.use_quaternion;
//                         }
                        
//                         showNotification("Initial conditions file loaded successfully", "success");
//                     } catch (error) {
//                         console.error("Error parsing initial conditions file:", error);
//                         showNotification("Error parsing file: " + error.message, "error");
//                     }
//                 };
//                 reader.readAsText(file);
//             }
//         });
        
//         // Add handler for Save Initial Conditions button
//         document.getElementById('btnSaveInitialConditions')?.addEventListener('click', function() {
//             try {
//                 // Get values from the form fields
//                 const startLocation = [
//                     parseFloat(document.getElementById('startLocationX').value),
//                     parseFloat(document.getElementById('startLocationY').value),
//                     parseFloat(document.getElementById('startLocationZ').value)
//                 ];
                
//                 const startOrientation = [
//                     parseFloat(document.getElementById('startOrientationRoll').value),
//                     parseFloat(document.getElementById('startOrientationPitch').value),
//                     parseFloat(document.getElementById('startOrientationYaw').value)
//                 ];
                
//                 const startVelocity = [
//                     parseFloat(document.getElementById('startVelocityU').value),
//                     parseFloat(document.getElementById('startVelocityV').value),
//                     parseFloat(document.getElementById('startVelocityW').value),
//                     parseFloat(document.getElementById('startVelocityP').value),
//                     parseFloat(document.getElementById('startVelocityQ').value),
//                     parseFloat(document.getElementById('startVelocityR').value)
//                 ];
                
//                 const useQuaternion = document.getElementById('useQuaternion').checked;
                
//                 // Check for invalid values
//                 const hasInvalidValues = [...startLocation, ...startOrientation, ...startVelocity].some(val => isNaN(val));
//                 if (hasInvalidValues) {
//                     throw new Error("Some values are not valid numbers");
//                 }
                
//                 // Update the vessel model with the new initial conditions
//                 window.currentVesselModel.updateInitialConditions(
//                     startVelocity,
//                     startLocation,
//                     startOrientation,
//                     useQuaternion
//                 );
                
//                 // Generate the initial_conditions.yml directly and update the preview
//                 const yamlGenerator = new YAMLGenerator();
//                 const initialConditionsYaml = yamlGenerator.toYAML(
//                     yamlGenerator.generateInitialConditionsYAML(window.currentVesselModel.config.initial_conditions)
//                 );
//                 document.getElementById('initial-conditions-yaml-content').textContent = initialConditionsYaml;
                
//                 // Also update all other YAML previews
//                 generateYAMLPreviews();
                
//                 // Close the modal
//                 const initialConditionsModal = bootstrap.Modal.getInstance(document.getElementById('initialConditionsModal'));
//                 initialConditionsModal.hide();
                
//                 // Show success notification
//                 showNotification("Initial conditions updated successfully", "success");
//             } catch (error) {
//                 console.error("Error saving initial conditions:", error);
//                 showNotification("Error: " + error.message, "error");
//             }
//         });
        
//         // Add handler for Edit Inertia Parameters button
//         document.getElementById('btnEditInertia')?.addEventListener('click', function() {
//             try {
//                 // Get current inertia parameters from the vessel model
//                 const inertia = window.currentVesselModel.config.inertia || {
//                     mass: 1,
//                     buoyancy_mass: 1,
//                     inertia_matrix: null,
//                     added_mass_matrix: null
//                 };
                
//                 // Populate form fields with current values
//                 document.getElementById('vesselMass').value = inertia.mass || 1;
//                 document.getElementById('vesselBuoyancyMass').value = inertia.buoyancy_mass || 1;
                
//                 // Handle inertia matrix
//                 const useDefaultInertia = !inertia.inertia_matrix;
//                 document.getElementById('useDefaultInertia').checked = useDefaultInertia;
//                 document.getElementById('inertiaMatrixSection').style.display = useDefaultInertia ? 'none' : 'block';
                
//                 // Set inertia matrix values if available
//                 if (!useDefaultInertia && inertia.inertia_matrix) {
//                     document.getElementById('inertia11').value = inertia.inertia_matrix[0][0] || 5;
//                     document.getElementById('inertia12').value = inertia.inertia_matrix[0][1] || 0;
//                     document.getElementById('inertia13').value = inertia.inertia_matrix[0][2] || 0;
//                     document.getElementById('inertia21').value = inertia.inertia_matrix[1][0] || 0;
//                     document.getElementById('inertia22').value = inertia.inertia_matrix[1][1] || 10;
//                     document.getElementById('inertia23').value = inertia.inertia_matrix[1][2] || 0;
//                     document.getElementById('inertia31').value = inertia.inertia_matrix[2][0] || 0;
//                     document.getElementById('inertia32').value = inertia.inertia_matrix[2][1] || 0;
//                     document.getElementById('inertia33').value = inertia.inertia_matrix[2][2] || 10;
//                 }
                
//                 // Handle added mass matrix
//                 const useDefaultAddedMass = !inertia.added_mass_matrix;
//                 document.getElementById('useDefaultAddedMass').checked = useDefaultAddedMass;
//                 document.getElementById('addedMassSection').style.display = useDefaultAddedMass ? 'none' : 'block';
                
//                 if (!useDefaultAddedMass && inertia.added_mass_matrix) {
//                     document.getElementById('addedMass11').value = inertia.added_mass_matrix[0][0] || 0;
//                     document.getElementById('addedMass12').value = inertia.added_mass_matrix[0][1] || 0;
//                     document.getElementById('addedMass13').value = inertia.added_mass_matrix[0][2] || 0;
//                     document.getElementById('addedMass21').value = inertia.added_mass_matrix[1][0] || 0;
//                     document.getElementById('addedMass22').value = inertia.added_mass_matrix[1][1] || 0;
//                     document.getElementById('addedMass23').value = inertia.added_mass_matrix[1][2] || 0;
//                     document.getElementById('addedMass31').value = inertia.added_mass_matrix[2][0] || 0;
//                     document.getElementById('addedMass32').value = inertia.added_mass_matrix[2][1] || 0;
//                     document.getElementById('addedMass33').value = inertia.added_mass_matrix[2][2] || 0;
//                 }
                
//                 console.log("Opening inertia modal with:", inertia);
                
//                 // Show the modal
//                 const inertiaModal = new bootstrap.Modal(document.getElementById('inertiaModal'));
//                 inertiaModal.show();
//             } catch (error) {
//                 console.error("Error opening inertia modal:", error);
//                 showNotification("Error loading inertia parameters", "error");
//             }
//         });
        
//         // Add handler to toggle inertia matrix section visibility
//         document.getElementById('useDefaultInertia')?.addEventListener('change', function() {
//             document.getElementById('inertiaMatrixSection').style.display = this.checked ? 'none' : 'block';
//         });
        
//         // Add handler to toggle added mass section visibility
//         document.getElementById('useDefaultAddedMass')?.addEventListener('change', function() {
//             document.getElementById('addedMassSection').style.display = this.checked ? 'none' : 'block';
//         });
        
//         // Add handler for inertia file upload
//         document.getElementById('inertiaFileUpload')?.addEventListener('change', function(event) {
//             const file = event.target.files[0];
//             if (file) {
//                 const reader = new FileReader();
//                 reader.onload = function(e) {
//                     try {
//                         // Parse the YAML and populate form fields
//                         const inertiaParams = jsyaml.load(e.target.result);
                        
//                         if (inertiaParams) {
//                             if (inertiaParams.mass) {
//                                 document.getElementById('vesselMass').value = inertiaParams.mass;
//                             }
                            
//                             if (inertiaParams.buoyancy_mass) {
//                                 document.getElementById('vesselBuoyancyMass').value = inertiaParams.buoyancy_mass;
//                             }
                            
//                             // Handle inertia matrix
//                             const hasInertiaMatrix = inertiaParams.inertia_matrix && inertiaParams.inertia_matrix !== 'None';
//                             document.getElementById('useDefaultInertia').checked = !hasInertiaMatrix;
//                             document.getElementById('inertiaMatrixSection').style.display = hasInertiaMatrix ? 'block' : 'none';
                            
//                             // Set inertia matrix values if available
//                             if (hasInertiaMatrix) {
//                                 document.getElementById('inertia11').value = inertiaParams.inertia_matrix[0][0] || 5;
//                                 document.getElementById('inertia12').value = inertiaParams.inertia_matrix[0][1] || 0;
//                                 document.getElementById('inertia13').value = inertiaParams.inertia_matrix[0][2] || 0;
//                                 document.getElementById('inertia21').value = inertiaParams.inertia_matrix[1][0] || 0;
//                                 document.getElementById('inertia22').value = inertiaParams.inertia_matrix[1][1] || 10;
//                                 document.getElementById('inertia23').value = inertiaParams.inertia_matrix[1][2] || 0;
//                                 document.getElementById('inertia31').value = inertiaParams.inertia_matrix[2][0] || 0;
//                                 document.getElementById('inertia32').value = inertiaParams.inertia_matrix[2][1] || 0;
//                                 document.getElementById('inertia33').value = inertiaParams.inertia_matrix[2][2] || 10;
//                             }
                            
//                             // Handle added mass matrix
//                             const hasAddedMassMatrix = inertiaParams.added_mass_matrix && inertiaParams.added_mass_matrix !== 'None';
//                             document.getElementById('useDefaultAddedMass').checked = !hasAddedMassMatrix;
//                             document.getElementById('addedMassSection').style.display = hasAddedMassMatrix ? 'block' : 'none';
                            
//                             if (hasAddedMassMatrix) {
//                                 document.getElementById('addedMass11').value = inertiaParams.added_mass_matrix[0][0] || 0;
//                                 document.getElementById('addedMass12').value = inertiaParams.added_mass_matrix[0][1] || 0;
//                                 document.getElementById('addedMass13').value = inertiaParams.added_mass_matrix[0][2] || 0;
//                                 document.getElementById('addedMass21').value = inertiaParams.added_mass_matrix[1][0] || 0;
//                                 document.getElementById('addedMass22').value = inertiaParams.added_mass_matrix[1][1] || 0;
//                                 document.getElementById('addedMass23').value = inertiaParams.added_mass_matrix[1][2] || 0;
//                                 document.getElementById('addedMass31').value = inertiaParams.added_mass_matrix[2][0] || 0;
//                                 document.getElementById('addedMass32').value = inertiaParams.added_mass_matrix[2][1] || 0;
//                                 document.getElementById('addedMass33').value = inertiaParams.added_mass_matrix[2][2] || 0;
//                             }
//                         }
                        
//                         showNotification("Inertia parameters file loaded successfully", "success");
//                     } catch (error) {
//                         console.error("Error parsing inertia file:", error);
//                         showNotification("Error parsing file: " + error.message, "error");
//                     }
//                 };
//                 reader.readAsText(file);
//             }
//         });
        
//         // Add handler for Save Inertia Parameters button
//         document.getElementById('btnSaveInertia')?.addEventListener('click', function() {
//             try {
//                 // Get values from the form fields
//                 const mass = parseFloat(document.getElementById('vesselMass').value);
//                 const buoyancyMass = parseFloat(document.getElementById('vesselBuoyancyMass').value);
                
//                 // Check for invalid values
//                 if (isNaN(mass) || isNaN(buoyancyMass) || mass <= 0 || buoyancyMass <= 0) {
//                     throw new Error("Mass and buoyancy mass must be positive numbers");
//                 }
                
//                 // Handle inertia matrix
//                 const useDefaultInertia = document.getElementById('useDefaultInertia').checked;
//                 let inertiaMatrix = null;
                
//                 if (!useDefaultInertia) {
//                     // Build inertia matrix from form inputs
//                     inertiaMatrix = [
//                         [
//                             parseFloat(document.getElementById('inertia11').value),
//                             parseFloat(document.getElementById('inertia12').value),
//                             parseFloat(document.getElementById('inertia13').value)
//                         ],
//                         [
//                             parseFloat(document.getElementById('inertia21').value),
//                             parseFloat(document.getElementById('inertia22').value),
//                             parseFloat(document.getElementById('inertia23').value)
//                         ],
//                         [
//                             parseFloat(document.getElementById('inertia31').value),
//                             parseFloat(document.getElementById('inertia32').value),
//                             parseFloat(document.getElementById('inertia33').value)
//                         ]
//                     ];
                    
//                     // Check for invalid inertia matrix values
//                     const hasInvalidInertia = inertiaMatrix.flat().some(val => isNaN(val));
//                     if (hasInvalidInertia) {
//                         throw new Error("Inertia matrix contains invalid values");
//                     }
//                 }
                
//                 // Handle added mass matrix
//                 const useDefaultAddedMass = document.getElementById('useDefaultAddedMass').checked;
//                 let addedMassMatrix = null;
                
//                 if (!useDefaultAddedMass) {
//                     // Build added mass matrix from form inputs
//                     addedMassMatrix = [
//                         [
//                             parseFloat(document.getElementById('addedMass11').value),
//                             parseFloat(document.getElementById('addedMass12').value),
//                             parseFloat(document.getElementById('addedMass13').value)
//                         ],
//                         [
//                             parseFloat(document.getElementById('addedMass21').value),
//                             parseFloat(document.getElementById('addedMass22').value),
//                             parseFloat(document.getElementById('addedMass23').value)
//                         ],
//                         [
//                             parseFloat(document.getElementById('addedMass31').value),
//                             parseFloat(document.getElementById('addedMass32').value),
//                             parseFloat(document.getElementById('addedMass33').value)
//                         ]
//                     ];
                    
//                     // Check for invalid added mass matrix values
//                     const hasInvalidAddedMass = addedMassMatrix.flat().some(val => isNaN(val));
//                     if (hasInvalidAddedMass) {
//                         throw new Error("Added mass matrix contains invalid values");
//                     }
//                 }
                
//                 // Update the vessel model with the new inertia parameters
//                 window.currentVesselModel.updatePhysicalProperties(
//                     mass,
//                     buoyancyMass,
//                     window.currentVesselModel.config.inertia.CG || [0, 0, 0],
//                     window.currentVesselModel.config.inertia.CB || [0, 0, 0]
//                 );
                
//                 // Also directly update the inertia matrices
//                 window.currentVesselModel.config.inertia.inertia_matrix = inertiaMatrix;
//                 window.currentVesselModel.config.inertia.added_mass_matrix = addedMassMatrix;
                
//                 // Generate the inertia.yml directly and update the preview
//                 const yamlGenerator = new YAMLGenerator();
//                 const inertiaYaml = yamlGenerator.toYAML(
//                     yamlGenerator.generateInertiaYAML(window.currentVesselModel.config.inertia)
//                 );
//                 document.getElementById('inertia-yaml-content').textContent = inertiaYaml;
                
//                 // Also update all other YAML previews
//                 generateYAMLPreviews();
                
//                 // Close the modal
//                 const inertiaModal = bootstrap.Modal.getInstance(document.getElementById('inertiaModal'));
//                 inertiaModal.hide();
                
//                 // Show success notification
//                 showNotification("Inertia parameters updated successfully", "success");
//             } catch (error) {
//                 console.error("Error saving inertia parameters:", error);
//                 showNotification("Error: " + error.message, "error");
//             }
//         });
        
//         // YAML Generator modal
//         const yamlModal = document.getElementById('yamlModal');
//         if (yamlModal) {
//             yamlModal.addEventListener('shown.bs.modal', function() {
//                 // Generate previews
//                 generateYAMLPreviews();
                
//                 // Update the included files section
//                 updateYAMLIncludedFilesInfo();
//             });
//         }
        
//         // Add validation and automatic ID management for component IDs
//         function setupComponentIdValidation() {
//             // Input elements
//             const surfaceIdInput = document.getElementById('fbxSurfaceId');
//             const thrusterIdInput = document.getElementById('fbxThrusterId');
//             const sensorIdInput = document.getElementById('fbxSensorId');
            
//             // Error message elements
//             const surfaceIdError = document.createElement('div');
//             surfaceIdError.className = 'invalid-feedback';
//             surfaceIdInput.after(surfaceIdError);
            
//             const thrusterIdError = document.createElement('div');
//             thrusterIdError.className = 'invalid-feedback';
//             thrusterIdInput.after(thrusterIdError);
            
//             const sensorIdError = document.createElement('div');
//             sensorIdError.className = 'invalid-feedback';
//             sensorIdInput.after(sensorIdError);
            
//             // Validate control surface ID
//             surfaceIdInput.addEventListener('input', function() {
//                 const id = parseInt(this.value);
//                 if (!id || isNaN(id)) return;
                
//                 const vesselModel = window.currentVesselModel;
//                 if (!vesselModel) return;
                
//                 const existingSurface = vesselModel.getControlSurface(id);
                
//                 // Get the modal and check if we're reconfiguring an existing component
//                 const modal = document.getElementById('fbxComponentModal');
//                 const objectUuid = modal?.dataset.objectUuid;
//                 const isReconfiguring = modal && objectUuid;
                
//                 // Get the current object being edited
//                 let currentObject = null;
//                 if (isReconfiguring) {
//                     currentObject = threeScene.getObjectByProperty('uuid', objectUuid);
//                 }
                
//                 // If we're reconfiguring and the ID matches the component we're editing, don't show error
//                 const isEditingSameComponent = currentObject && 
//                                              currentObject.userData.componentId === id && 
//                                              currentObject.userData.componentType === 'controlSurface';
                
//                 if (existingSurface && !isEditingSameComponent) {
//                     // ID conflict found with a different component
//                     this.classList.add('is-invalid');
//                     surfaceIdError.textContent = `ID ${id} is already used by ${existingSurface.control_surface_name || 'another control surface'}`;
//                 } else {
//                     this.classList.remove('is-invalid');
//                 }
//             });
            
//             // Validate thruster ID
//             thrusterIdInput.addEventListener('input', function() {
//                 const id = parseInt(this.value);
//                 if (!id || isNaN(id)) return;
                
//                 const vesselModel = window.currentVesselModel;
//                 if (!vesselModel) return;
                
//                 const existingThruster = vesselModel.getThruster(id);
                
//                 // Get the modal and check if we're reconfiguring an existing component
//                 const modal = document.getElementById('fbxComponentModal');
//                 const objectUuid = modal?.dataset.objectUuid;
//                 const isReconfiguring = modal && objectUuid;
                
//                 // Get the current object being edited
//                 let currentObject = null;
//                 if (isReconfiguring) {
//                     currentObject = threeScene.getObjectByProperty('uuid', objectUuid);
//                 }
                
//                 // If we're reconfiguring and the ID matches the component we're editing, don't show error
//                 const isEditingSameComponent = currentObject && 
//                                              currentObject.userData.componentId === id && 
//                                              currentObject.userData.componentType === 'thruster';
                
//                 if (existingThruster && !isEditingSameComponent) {
//                     // ID conflict found with a different component
//                     this.classList.add('is-invalid');
//                     thrusterIdError.textContent = `ID ${id} is already used by ${existingThruster.thruster_name || 'another thruster'}`;
//                 } else {
//                     this.classList.remove('is-invalid');
//                 }
//             });
            
//             // Validate sensor ID
//             sensorIdInput.addEventListener('input', function() {
//                 const id = parseInt(this.value);
//                 if (!id || isNaN(id)) return;
                
//                 const vesselModel = window.currentVesselModel;
//                 if (!vesselModel) return;
                
//                 const existingSensor = vesselModel.getSensor(id);
                
//                 // Get the modal and check if we're reconfiguring an existing component
//                 const modal = document.getElementById('fbxComponentModal');
//                 const objectUuid = modal?.dataset.objectUuid;
//                 const isReconfiguring = modal && objectUuid;
                
//                 // Get the current object being edited
//                 let currentObject = null;
//                 if (isReconfiguring) {
//                     currentObject = threeScene.getObjectByProperty('uuid', objectUuid);
//                 }
                
//                 // If we're reconfiguring and the ID matches the component we're editing, don't show error
//                 const isEditingSameComponent = currentObject && 
//                                              currentObject.userData.componentId === id && 
//                                              currentObject.userData.componentType === 'sensor';
                
//                 if (existingSensor && !isEditingSameComponent) {
//                     // ID conflict found with a different component
//                     this.classList.add('is-invalid');
//                     sensorIdError.textContent = `ID ${id} is already used by ${existingSensor.sensor_name || 'another sensor'}`;
//                 } else {
//                     this.classList.remove('is-invalid');
//                 }
//             });
            
//             // Function to suggest next available ID when opening the component modal
//             function setNextAvailableComponentId(componentType) {
//                 const vesselModel = window.currentVesselModel;
//                 if (!vesselModel) return;
                
//                 if (componentType === 'controlSurface') {
//                     // For new components, use the next available ID from vessel model
//                     surfaceIdInput.value = vesselModel.nextControlSurfaceId || 1;
//                 } else if (componentType === 'thruster') {
//                     thrusterIdInput.value = vesselModel.nextThrusterId || 1;
//                 } else if (componentType === 'sensor') {
//                     sensorIdInput.value = vesselModel.nextSensorId || 1;
//                 }
//             }
            
//             // When component modal shows, set the next available ID if this is a new component
//             document.getElementById('fbxComponentModal').addEventListener('show.bs.modal', function(event) {
//                 const button = event.relatedTarget;
//                 const object = button ? threeScene.getSelectedObject() : null;
                
//                 // Get component type being configured
//                 const componentType = document.getElementById('fbxComponentType').value;
                
//                 // Only suggest IDs for new components, not existing ones
//                 const isReconfiguring = object && object.children.some(child => child.userData.isComponentAxes);
                
//                 if (!isReconfiguring) {
//                     setNextAvailableComponentId(componentType);
//                 }
                
//                 // Trigger validation for any preset values
//                 surfaceIdInput.dispatchEvent(new Event('input'));
//                 thrusterIdInput.dispatchEvent(new Event('input'));
//                 sensorIdInput.dispatchEvent(new Event('input'));
//             });
//         }
        
//         setupComponentIdValidation();
//     }
    
//     // Initialize file handlers
//     function initializeFileHandlers(threeScene) {
//         // Basic vessel properties form
//         const vesselNameInput = document.getElementById('vesselName');
//         const vesselTypeSelect = document.getElementById('vesselType');
        
//         if (vesselNameInput) {
//             vesselNameInput.addEventListener('change', function() {
//                 window.currentVesselModel.setBasicInfo(this.value, vesselTypeSelect.value);
//             });
//         }
        
//         if (vesselTypeSelect) {
//             vesselTypeSelect.addEventListener('change', function() {
//                 window.currentVesselModel.setBasicInfo(vesselNameInput.value, this.value);
//             });
//         }
        
//         // FBX file upload - with performance optimization (debounce loading events)
//         const fbxFileInput = document.getElementById('fbxFile');
//         if (fbxFileInput) {
//             fbxFileInput.addEventListener('change', function() {
//                 if (this.files && this.files[0]) {
//                     const file = this.files[0];
                    
//                     // Show loading indicator
//                     showNotification('Loading 3D model...', 'info');
                    
//                     // Use setTimeout to allow UI to update before starting heavy operation
//                     setTimeout(async () => {
//                         try {
//                             // Load the model if threeScene has the method
//                             if (typeof threeScene.loadFBXModel === 'function') {
//                                 const result = await threeScene.loadFBXModel(file);
//                                 const model = result.vessel;
//                                 const dimensions = result.dimensions;
                                
//                                 // Store file reference
//                                 window.currentVesselModel.setModelFile('fbx', file);
                                
//                                 // Update the vessel model dimensions if we have them
//                                 if (dimensions) {
//                                     console.log("Updating vessel model with FBX dimensions:", dimensions);
                                    
//                                     // Update the vessel model
//                                     window.currentVesselModel.updateDimensions(
//                                         dimensions.length,
//                                         dimensions.breadth,
//                                         dimensions.depth
//                                     );
                                    
//                                     // Store axis configuration in the vessel model
//                                     window.currentVesselModel.config.geometry.axisConfig = dimensions.axes;
                                    
//                                     // Store bounding box information
//                                     window.currentVesselModel.config.geometry.boundingBox = dimensions.boundingBox;
                                    
//                                     // Update gyration values based on the new dimensions
//                                     if (typeof recalculateGyrationValues === 'function') {
//                                         recalculateGyrationValues();
//                                     }
                                    
//                                     // If geometry modal is open, update its values
//                                     const geometryModal = document.getElementById('geometryModal');
//                                     if (geometryModal && geometryModal.classList.contains('show')) {
//                                         // Get UI elements
//                                         const vesselLength = document.getElementById('vesselLength');
//                                         const vesselBreadth = document.getElementById('vesselBreadth');
//                                         const vesselDepth = document.getElementById('vesselDepth');
//                                         const lengthAxis = document.getElementById('lengthAxis');
//                                         const breadthAxis = document.getElementById('breadthAxis');
//                                         const depthAxis = document.getElementById('depthAxis');
//                                         const gdfFileInfo = document.getElementById('gdfFileInfo');
                                        
//                                         if (vesselLength && vesselBreadth && vesselDepth) {
//                                             // Update dimension inputs
//                                             vesselLength.value = dimensions.length.toFixed(2);
//                                             vesselBreadth.value = dimensions.breadth.toFixed(2);
//                                             vesselDepth.value = dimensions.depth.toFixed(2);
//                                         }
                                        
//                                         if (lengthAxis && breadthAxis && depthAxis) {
//                                             // Update axis dropdowns
//                                             lengthAxis.value = dimensions.axes.length;
//                                             breadthAxis.value = dimensions.axes.breadth;
//                                             depthAxis.value = dimensions.axes.depth;
//                                         }
                                        
//                                         // Update gyration display if available
//                                         const gyrationX = document.getElementById('gyrationX');
//                                         const gyrationY = document.getElementById('gyrationY');
//                                         const gyrationZ = document.getElementById('gyrationZ');
                                        
//                                         if (gyrationX && gyrationY && gyrationZ) {
//                                             const gyrationValues = window.currentVesselModel.config.geometry.gyration || [0, 0, 0];
//                                             gyrationX.value = gyrationValues[0].toFixed(2);
//                                             gyrationY.value = gyrationValues[1].toFixed(2);
//                                             gyrationZ.value = gyrationValues[2].toFixed(2);
//                                         }
                                        
//                                         // Show info message in the modal
//                                         if (gdfFileInfo) {
//                                             gdfFileInfo.style.display = 'block';
//                                             gdfFileInfo.innerHTML = `
//                                                 <i class="bi bi-info-circle"></i> <strong>Dimensions detected from FBX file:</strong><br>
//                                                 Length: ${dimensions.length.toFixed(2)}m (${lengthAxis.value}-axis)<br>
//                                                 Breadth: ${dimensions.breadth.toFixed(2)}m (${breadthAxis.value}-axis)<br>
//                                                 Depth: ${dimensions.depth.toFixed(2)}m (${depthAxis.value}-axis)<br>
//                                                 <small class="text-muted">Based on the bounding box of the geometry</small>
//                                             `;
//                                         }
//                                     }
//                                 }
                                
//                                 showNotification(`3D model loaded successfully. Dimensions: L=${dimensions.length.toFixed(2)}m, B=${dimensions.breadth.toFixed(2)}m, D=${dimensions.depth.toFixed(2)}m`, 'success');
                                
//                                 // Update scene hierarchy
//                                 threeScene.updateSceneHierarchy();
//                             } else {
//                                 console.warn("No dimensions extracted from FBX model");
//                                 showNotification('3D model loaded successfully, but could not extract dimensions', 'warning');
//                             }
//                         } catch (error) {
//                             console.error('Error loading 3D model:', error);
//                             showNotification('Failed to load 3D model: ' + error.message, 'error');
//                         }
//                     }, 100);
//                 }
//             });
//         }
        
//         // Geometry visibility toggle
//         const toggleGeometryVisibility = document.getElementById('toggleGeometryVisibility');
//         if (toggleGeometryVisibility && threeScene) {
//             toggleGeometryVisibility.addEventListener('change', function() {
//                 if (threeScene.vessel) {
//                     // Use the explicit state from the checkbox
//                     threeScene.toggleGeometry(this.checked);
//                     showNotification(`3D model ${this.checked ? 'shown' : 'hidden'}`, 'info');
//                 } else {
//                     showNotification('No 3D model loaded', 'warning');
//                     // Reset the toggle to active state if no model is loaded
//                     this.classList.add('active');
//                 }
//             });
//         }
        
//         // Toggle model selectability
//         document.getElementById('toggleModelSelectable')?.addEventListener('change', function() {
//             if (threeScene.vessel) {
//                 threeScene.setModelSelectable(this.checked);
//                 showNotification(`3D model ${this.checked ? 'is now selectable' : 'is no longer selectable'}`, 'info');
//             } else {
//                 showNotification('No 3D model loaded', 'warning');
//                 this.checked = true;
//             }
//         });
        
//         // Model transparency slider
//         document.getElementById('modelTransparency')?.addEventListener('input', function() {
//             if (threeScene.vessel) {
//                 const transparencyValue = parseInt(this.value, 10);
//                 threeScene.setModelTransparency(transparencyValue);
//             }
//         });
        
//         // Hydra output zip upload
//         const hydraZipInput = document.getElementById('hydraZip');
//         if (hydraZipInput) {
//             hydraZipInput.addEventListener('change', function() {
//                 if (this.files && this.files[0]) {
//                     const file = this.files[0];
                    
//                     // Show loading notification
//                     showNotification('Loading Hydra output...', 'info');
                    
//                     // Use setTimeout to allow UI to update before starting heavy operation
//                     setTimeout(async () => {
//                         try {
//                             // Load the model if threeScene has the method
//                             if (typeof threeScene.loadHydraModel === 'function') {
//                                 const result = await threeScene.loadHydraModel(file);
//                                 const model = result.vessel;
//                                 const dimensions = result.dimensions;
                                
//                                 // Store file reference
//                                 window.currentVesselModel.setModelFile('hydra', file);
                                
//                                 // Update the vessel model dimensions if we have them
//                                 if (dimensions) {
//                                     console.log("Updating vessel model with Hydra dimensions:", dimensions);
                                    
//                                     // Update the vessel model
//                                     window.currentVesselModel.updateDimensions(
//                                         dimensions.length,
//                                         dimensions.width,
//                                         dimensions.height
//                                     );
                                    
//                                     // Store axis configuration in the vessel model
//                                     window.currentVesselModel.config.geometry.axisConfig = dimensions.axes;
                                    
//                                     // Store bounding box information
//                                     window.currentVesselModel.config.geometry.boundingBox = dimensions.boundingBox;
                                    
//                                     // Update gyration values based on the new dimensions
//                                     if (typeof recalculateGyrationValues === 'function') {
//                                         recalculateGyrationValues();
//                                     }
                                    
//                                     // If geometry modal is open, update its values
//                                     const geometryModal = document.getElementById('geometryModal');
//                                     if (geometryModal && geometryModal.classList.contains('show')) {
//                                         // Get UI elements
//                                         const vesselLength = document.getElementById('vesselLength');
//                                         const vesselBreadth = document.getElementById('vesselBreadth');
//                                         const vesselDepth = document.getElementById('vesselDepth');
//                                         const lengthAxis = document.getElementById('lengthAxis');
//                                         const breadthAxis = document.getElementById('breadthAxis');
//                                         const depthAxis = document.getElementById('depthAxis');
//                                         const gdfFileInfo = document.getElementById('gdfFileInfo');
                                        
//                                         if (vesselLength && vesselBreadth && vesselDepth) {
//                                             // Update dimension inputs
//                                             vesselLength.value = dimensions.length.toFixed(2);
//                                             vesselBreadth.value = dimensions.breadth.toFixed(2);
//                                             vesselDepth.value = dimensions.depth.toFixed(2);
//                                         }
                                        
//                                         if (lengthAxis && breadthAxis && depthAxis) {
//                                             // Update axis dropdowns
//                                             lengthAxis.value = dimensions.axes.length;
//                                             breadthAxis.value = dimensions.axes.breadth;
//                                             depthAxis.value = dimensions.axes.depth;
//                                         }
                                        
//                                         // Update gyration display if available
//                                         const gyrationX = document.getElementById('gyrationX');
//                                         const gyrationY = document.getElementById('gyrationY');
//                                         const gyrationZ = document.getElementById('gyrationZ');
                                        
//                                         if (gyrationX && gyrationY && gyrationZ) {
//                                             const gyrationValues = window.currentVesselModel.config.geometry.gyration || [0, 0, 0];
//                                             gyrationX.value = gyrationValues[0].toFixed(2);
//                                             gyrationY.value = gyrationValues[1].toFixed(2);
//                                             gyrationZ.value = gyrationValues[2].toFixed(2);
//                                         }
                                        
//                                         // Show info message in the modal
//                                         if (gdfFileInfo) {
//                                             gdfFileInfo.style.display = 'block';
//                                             gdfFileInfo.innerHTML = `
//                                                 <i class="bi bi-info-circle"></i> <strong>Dimensions detected from Hydra file:</strong><br>
//                                                 Length: ${dimensions.length.toFixed(2)}m (${lengthAxis.value}-axis)<br>
//                                                 Breadth: ${dimensions.breadth.toFixed(2)}m (${breadthAxis.value}-axis)<br>
//                                                 Depth: ${dimensions.depth.toFixed(2)}m (${depthAxis.value}-axis)<br>
//                                                 <small class="text-muted">Based on the bounding box of the geometry</small>
//                                             `;
//                                         }
//                                     }
//                                 }
                                
//                                 showNotification(`Hydra model loaded successfully. Dimensions: L=${dimensions.length.toFixed(2)}m, B=${dimensions.breadth.toFixed(2)}m, D=${dimensions.depth.toFixed(2)}m`, 'success');
                                
//                                 // Update scene hierarchy
//                                 threeScene.updateSceneHierarchy();
//                             } else {
//                                 console.warn("No dimensions extracted from Hydra model");
//                                 showNotification('Hydra model loaded successfully, but could not extract dimensions', 'warning');
//                             }
//                         } catch (error) {
//                             console.error('Error loading Hydra model:', error);
//                             showNotification('Failed to load Hydra model: ' + error.message, 'error');
//                         }
//                     }, 100);
//                 }
//             });
//         }
        
//         // NACA airfoil file upload
//         const nacaFileInput = document.getElementById('nacaFileUpload');
//         if (nacaFileInput) {
//             nacaFileInput.addEventListener('change', function() {
//                 if (this.files && this.files[0]) {
//                     const file = this.files[0];
                    
//                     try {
//                         // Store file reference in vessel model - this also handles extracting NACA number from filename
//                         window.currentVesselModel.setModelFile('naca', file);
                        
//                         // Update the input field with the NACA number if it was extracted
//                         const nacaNumber = window.currentVesselModel.config.control_surfaces.naca_number;
//                         if (document.getElementById('nacaNumber')) {
//                             document.getElementById('nacaNumber').value = nacaNumber;
//                         }
                        
//                         showNotification(`NACA file loaded: ${file.name}. Profile: ${nacaNumber}`, 'success');
//                     } catch (error) {
//                         console.error('Error processing NACA file:', error);
//                         showNotification('Error processing NACA file: ' + error.message, 'error');
//                     }
//                 }
//             });
//         }
        
//         // NACA profile number
//         const nacaNumberInput = document.getElementById('nacaNumber');
//         if (nacaNumberInput) {
//             nacaNumberInput.addEventListener('change', function() {
//                 const nacaNumber = this.value.trim();
//                 if (nacaNumber) {
//                     window.currentVesselModel.setNacaProfile(nacaNumber);
                    
//                     // Update all control surface NACA profiles
//                     const controlSurfaces = window.currentVesselModel.config.control_surfaces.control_surfaces;
//                     if (controlSurfaces.length > 0) {
//                         // Update existing control surfaces with new NACA profile
//                         showNotification(`Updated NACA profile for all control surfaces to ${nacaNumber}`, 'info');
//                     }
//                 }
//             });
//         }
        
//         // Simulation parameters - with debounced updates to reduce lag
//         let simParamsDebounce;
//         function updateSimParams() {
//             clearTimeout(simParamsDebounce);
//             simParamsDebounce = setTimeout(() => {
//                 const simParams = {
//                     sim_time: parseFloat(document.getElementById('simTime')?.value || 100),
//                     time_step: parseFloat(document.getElementById('timeStep')?.value || 0.01),
//                     density: parseFloat(document.getElementById('density')?.value || 1025),
//                     gravity: parseFloat(document.getElementById('gravity')?.value || 9.80665),
//                     world_size: [
//                         parseFloat(document.getElementById('worldSizeX')?.value || 1000),
//                         parseFloat(document.getElementById('worldSizeY')?.value || 1000),
//                         parseFloat(document.getElementById('worldSizeZ')?.value || 100)
//                     ],
//                     gps_datum: [
//                         parseFloat(document.getElementById('gpsDatumLat')?.value || 12.99300),
//                         parseFloat(document.getElementById('gpsDatumLon')?.value || 80.23913),
//                         parseFloat(document.getElementById('gpsDatumAlt')?.value || -87.0)
//                     ]
//                 };
                
//                 window.currentVesselModel.updateSimulationParams(simParams);
//             }, 300); // 300ms debounce
//         }
        
//         document.querySelectorAll('#simTime, #timeStep, #density, #gravity, #worldSizeX, #worldSizeY, #worldSizeZ, #gpsDatumLat, #gpsDatumLon, #gpsDatumAlt').forEach(input => {
//             input?.addEventListener('change', updateSimParams);
//             input?.addEventListener('input', updateSimParams);
//         });
//     }
    
//     // Initialize YAML generation
//     function initializeYAMLGeneration() {
//         document.getElementById('btnGenerateYaml')?.addEventListener('click', async function() {
//             const vesselName = window.currentVesselModel.config.name;
//             const outputPath = document.getElementById('outputPath').value;
            
//             if (!vesselName) {
//                 window.currentVesselModel.config.name = 'vessel';
//                 document.getElementById('vesselName').value = 'vessel';
//                 showNotification('Using default vessel name "vessel"', 'warning');
//             }
            
//             try {
//                 // Generate YAML files
//                 const yamlFiles = window.currentVesselModel.generateYAMLFiles();
                
//                 if (!yamlFiles || Object.keys(yamlFiles).length === 0) {
//                     throw new Error('No YAML files were generated');
//                 }
                
//                 // Update the included files section
//                 updateYAMLIncludedFilesInfo();
                
//                 // Create a notification with the result
//                 let message = `YAML files generated for vessel: ${vesselName}\n`;
//                 message += `Output path: ${outputPath}${vesselName}/`;
                
//                 // Show files that were generated
//                 message += `\nGenerated files: ${Object.keys(yamlFiles).join(', ')}`;
                
//                 showNotification(message, 'success');
                
//                 // Hide the modal
//                 bootstrap.Modal.getInstance(document.getElementById('yamlModal')).hide();
//             } catch (error) {
//                 console.error('Error generating YAML files:', error);
//                 showNotification('Failed to generate YAML files: ' + error.message, 'error');
//             }
//         });
        
//         // Add handler for the refresh previews button
//         document.getElementById('btnRefreshPreviews')?.addEventListener('click', function() {
//             showNotification('Refreshing YAML previews...', 'info');
            
//             try {
//                 // Generate the previews
//                 const yamlFiles = generateYAMLPreviews();
                
//                 if (yamlFiles && Object.keys(yamlFiles).length > 0) {
//                     showNotification(`Successfully refreshed ${Object.keys(yamlFiles).length} YAML previews`, 'success');
//                 } else {
//                     showNotification('No YAML previews were generated', 'warning');
//                 }
//             } catch (error) {
//                 console.error('Error refreshing previews:', error);
//                 showNotification('Failed to refresh previews: ' + error.message, 'error');
//             }
//         });
        
//         document.getElementById('btnDownloadZip')?.addEventListener('click', async function() {
//             const vesselName = window.currentVesselModel.config.name;
            
//             if (!vesselName) {
//                 showNotification('Please enter a vessel name before downloading YAML files', 'error');
//                 return;
//             }
            
//             showNotification('Preparing ZIP file...', 'info');
            
//             try {
//                 // Update the included files section before download
//                 updateYAMLIncludedFilesInfo();
                
//                 // Create a zip file with all YAML files - use setTimeout to allow UI to update
//                 setTimeout(async () => {
//                     try {
//                         const zipBlob = await window.currentVesselModel.createYAMLZip();
                        
//                         // Create download link
//                         const url = URL.createObjectURL(zipBlob);
//                         const a = document.createElement('a');
//                         a.href = url;
//                         a.download = `${vesselName}_config.zip`;
//                         a.click();
                        
//                         URL.revokeObjectURL(url);
                        
//                         // Show notification
//                         showNotification(`YAML files downloaded as ${vesselName}_config.zip`, 'success');
                        
//                         // Hide the modal
//                         bootstrap.Modal.getInstance(document.getElementById('yamlModal')).hide();
//                     } catch (error) {
//                         console.error('Error creating ZIP file:', error);
//                         showNotification('Failed to create ZIP file: ' + error.message, 'error');
//                     }
//                 }, 100);
//             } catch (error) {
//                 console.error('Error preparing ZIP file:', error);
//                 showNotification('Failed to prepare ZIP file: ' + error.message, 'error');
//             }
//         });
//     }
    
//     // Generate YAML previews in the modal
//     function generateYAMLPreviews() {
//         // Check if the vessel has a name
//         if (!window.currentVesselModel || !window.currentVesselModel.config || !window.currentVesselModel.config.name) {
//             console.warn('Vessel has no name set. Using default "vessel".');
//             if (window.currentVesselModel && window.currentVesselModel.config) {
//                 window.currentVesselModel.config.name = 'vessel';
//             }
//         }
        
//         try {
//             // Generate YAML files
//             const yamlFiles = window.currentVesselModel.generateYAMLFiles();
            
//             if (!yamlFiles || Object.keys(yamlFiles).length === 0) {
//                 console.warn('No YAML files were generated');
//                 return {};
//             }
            
//             // Define mapping from file names to element IDs
//             const fileToElementMap = {
//                 'simulation_input.yml': 'simulation-yaml-content',
//                 'geometry.yml': 'geometry-yaml-content',
//                 'inertia.yml': 'inertia-yaml-content',
//                 'hydrodynamics.yml': 'hydrodynamics-yaml-content',
//                 'thrusters.yml': 'thrusters-yaml-content',  // Map to thrusters-yaml-content
//                 'control_surfaces.yml': 'control-surfaces-yaml-content',
//                 'sensors.yml': 'sensors-yaml-content',
//                 'initial_conditions.yml': 'initial-conditions-yaml-content',
//                 'guidance.yml': 'guidance-yaml-content',
//                 'control.yml': 'control-yaml-content'
//             };
            
//             // Update each preview field
//             for (const [fileName, content] of Object.entries(yamlFiles)) {
//                 const elementId = fileToElementMap[fileName];
                
//                 if (elementId) {
//                     const previewElement = document.getElementById(elementId);
                    
//                     if (previewElement) {
//                         previewElement.textContent = content;
//                     } else {
//                         console.warn(`Preview element with ID ${elementId} not found for file ${fileName}`);
//                     }
//                 } else {
//                     console.warn(`No mapping found for file ${fileName}`);
//                 }
//             }
            
//             // Update included files information
//             updateYAMLIncludedFilesInfo();
            
//             return yamlFiles;
//         } catch (error) {
//             console.error('Error generating YAML previews:', error);
//             return {};
//         }
//     }
    
//     /**
//      * Update the included files section in the YAML generation modal
//      */
//     function updateYAMLIncludedFilesInfo() {
//         const includedFilesElement = document.getElementById('yamlIncludedFiles');
//         if (!includedFilesElement) return;
        
//         // Create a list of included files
//         let filesList = '<ul class="mb-0">';
        
//         // Check for NACA file
//         if (window.currentVesselModel.modelData.nacaFile) {
//             const nacaNumber = window.currentVesselModel.config.control_surfaces.naca_number || '0015';
//             filesList += `<li><i class="bi bi-file-earmark-text"></i> <strong>NACA${nacaNumber}.csv</strong> - Custom NACA airfoil data</li>`;
//         } else {
//             const nacaNumber = window.currentVesselModel.config.control_surfaces.naca_number || '0015';
//             filesList += `<li><i class="bi bi-file-earmark-text"></i> <strong>NACA${nacaNumber}.csv</strong> - Default NACA airfoil data</li>`;
//         }
        
//         // Check for HydRA zip
//         if (window.currentVesselModel.modelData.hydraZipFile) {
//             filesList += `<li><i class="bi bi-file-earmark-zip"></i> <strong>HydRA/</strong> - Extracted HydRA output files</li>`;
//         } else {
//             filesList += `<li><i class="bi bi-folder"></i> <strong>HydRA/</strong> - Empty HydRA folder</li>`;
//         }
        
//         // Add all the YAML files
//         filesList += `<li><i class="bi bi-file-earmark-code"></i> <strong>*.yml</strong> - All configuration YAML files</li>`;
        
//         filesList += '</ul>';
        
//         // Update the element content
//         includedFilesElement.innerHTML = filesList;
//     }
    
//     // --------- Utility Functions ---------
    
//     // Show notification to the user
//     function showNotification(message, type = 'info') {
//         const colors = {
//             success: '#28a745',
//             error: '#dc3545',
//             warning: '#ffc107',
//             info: '#17a2b8'
//         };
        
//         // Create notification element
//         const notification = document.createElement('div');
//         notification.className = 'notification';
//         notification.style.position = 'fixed';
//         notification.style.top = '20px';
//         notification.style.right = '20px';
//         notification.style.padding = '12px 20px';
//         notification.style.background = colors[type] || colors.info;
//         notification.style.color = '#fff';
//         notification.style.borderRadius = '4px';
//         notification.style.boxShadow = '0 4px 8px rgba(0, 0, 0, 0.2)';
//         notification.style.zIndex = '9999';
//         notification.style.maxWidth = '400px';
//         notification.style.wordWrap = 'break-word';
        
//         // Format message for multiline if needed
//         notification.innerHTML = message.replace(/\n/g, '<br>');
        
//         // Add to DOM
//         document.body.appendChild(notification);
        
//         // Remove after 5 seconds
//         setTimeout(() => {
//             notification.style.opacity = '0';
//             notification.style.transition = 'opacity 0.5s';
//             setTimeout(() => {
//                 document.body.removeChild(notification);
//             }, 500);
//         }, 5000);
//     }

//     // Function to debug the vessel model structure
//     function logComponentCounts() {
//         if (window.currentVesselModel && window.currentVesselModel.config) {
//             const config = window.currentVesselModel.config;
            
//             console.log('------- Current Component Counts -------');
//             // Control surfaces
//             if (config.control_surfaces && Array.isArray(config.control_surfaces.control_surfaces)) {
//                 console.log(`Control Surfaces: ${config.control_surfaces.control_surfaces.length}`);
//                 console.log(config.control_surfaces.control_surfaces);
//                 } else {
//                 console.log('Control Surfaces: Not properly initialized');
//                 console.log(config.control_surfaces);
//             }
            
//             // Thrusters
//             if (config.thrusters && Array.isArray(config.thrusters.thrusters)) {
//                 console.log(`Thrusters: ${config.thrusters.thrusters.length}`);
//             } else {
//                 console.log('Thrusters: Not properly initialized');
//             }
            
//             // Sensors
//             if (config.sensors && Array.isArray(config.sensors.sensors)) {
//                 console.log(`Sensors: ${config.sensors.sensors.length}`);
//             } else {
//                 console.log('Sensors: Not properly initialized');
//             }
//             console.log('---------------------------------------');
//         } else {
//             console.log('Vessel model not initialized');
//         }
//     }

//     // Add handler for Advanced Simulation Parameters button
//     document.getElementById('btnAdvancedSimParams')?.addEventListener('click', function() {
//         try {
//             const yamlGenerator = new YAMLGenerator();
            
//             // Get current simulation parameters from the vessel model
//             // Either use from config or get them from the UI if not yet saved
//             let simParams = window.currentVesselModel.config.simulation || {};
//             if (Object.keys(simParams).length === 0) {
//                 simParams = {
//                     sim_time: parseFloat(document.getElementById('simTime')?.value || 100),
//                     time_step: parseFloat(document.getElementById('timeStep')?.value || 0.01),
//                     density: parseFloat(document.getElementById('density')?.value || 1025),
//                     gravity: parseFloat(document.getElementById('gravity')?.value || 9.80665),
//                     world_size: [
//                         parseFloat(document.getElementById('worldSizeX')?.value || 1000),
//                         parseFloat(document.getElementById('worldSizeY')?.value || 1000),
//                         parseFloat(document.getElementById('worldSizeZ')?.value || 100)
//                     ],
//                     gps_datum: [
//                         parseFloat(document.getElementById('gpsDatumLat')?.value || 12.99300),
//                         parseFloat(document.getElementById('gpsDatumLon')?.value || 80.23913),
//                         parseFloat(document.getElementById('gpsDatumAlt')?.value || -87.0)
//                     ],
//                     geofence: yamlGenerator.defaultSimParams.geofence || [],
//                     nagents: 1,
//                     agents: [{
//                         name: window.currentVesselModel.config.name || 'vessel',
//                         type: window.currentVesselModel.config.type || 'auv'
//                     }]
//                 };
//             }
            
//             // Populate basic parameters tab
//             document.getElementById('simTimeSetting').value = simParams.sim_time || 100;
//             document.getElementById('timeStepSetting').value = simParams.time_step || 0.01;
//             document.getElementById('densitySetting').value = simParams.density || 1025;
//             document.getElementById('gravitySetting').value = simParams.gravity || 9.80665;
            
//             if (simParams.world_size && Array.isArray(simParams.world_size)) {
//                 document.getElementById('worldSizeXSetting').value = simParams.world_size[0] || 1000;
//                 document.getElementById('worldSizeYSetting').value = simParams.world_size[1] || 1000;
                
//                 // Handle infinite Z dimension (either 'Inf' or a number)
//                 const zValue = simParams.world_size[2];
//                 const isInfinite = zValue === 'Inf' || zValue === Infinity;
//                 document.getElementById('infiniteZSetting').checked = isInfinite;
//                 document.getElementById('worldSizeZSetting').value = isInfinite ? 100 : (zValue || 100);
//             }
            
//             if (simParams.gps_datum && Array.isArray(simParams.gps_datum)) {
//                 document.getElementById('gpsDatumLatSetting').value = simParams.gps_datum[0] || 12.99300;
//                 document.getElementById('gpsDatumLonSetting').value = simParams.gps_datum[1] || 80.23913;
//                 document.getElementById('gpsDatumAltSetting').value = simParams.gps_datum[2] || -87.0;
//             }
            
//             document.getElementById('numAgentsSetting').value = simParams.nagents || 1;
            
//             // Populate geofence table
//             const geofenceTableBody = document.getElementById('geofenceTableBody');
//             geofenceTableBody.innerHTML = ''; // Clear existing rows
            
//             if (simParams.geofence && Array.isArray(simParams.geofence)) {
//                 simParams.geofence.forEach((point, index) => {
//                     if (Array.isArray(point) && point.length >= 2) {
//                         addGeofenceTableRow(point[0], point[1]);
//                     }
//                 });
//             } else {
//                 // Add default empty row if no geofence exists
//                 addGeofenceTableRow(12.993496, 80.239007);
//             }
            
//             // Generate YAML string from current simulation parameters for the YAML editor tab
//             // Use the formatted YAML generator instead of jsyaml.dump
//             const yamlString = yamlGenerator.generateFormattedSimulationYAML(simParams);
//             document.getElementById('simParamsYaml').value = yamlString;
            
//             console.log("Opening simulation params modal with:", simParams);
            
//             // Show the modal
//             const simParamsModal = new bootstrap.Modal(document.getElementById('simParamsModal'));
//             simParamsModal.show();
            
//             // Initialize the map after the modal is shown
//             document.getElementById('simParamsModal').addEventListener('shown.bs.modal', function() {
//                 // Initialize geofence map when the modal is shown
//                 initializeGeofenceMap();
                
//                 // Add listeners for tab switches
//                 const tabList = document.getElementById('simParamsTabs');
//                 if (tabList) {
//                     const tabs = tabList.querySelectorAll('button[data-bs-toggle="tab"]');
//                     tabs.forEach(tab => {
//                         tab.addEventListener('shown.bs.tab', function(event) {
//                             if (event.target.id === 'geofence-tab') {
//                                 // When switching to geofence tab, make sure the map is initialized
//                                 setTimeout(() => {
//                                     if (geofenceMap) {
//                                         geofenceMap.invalidateSize();
                                        
//                                         // If we have a polygon, zoom to fit its bounds
//                                         if (geofencePoints.length >= 3) {
//                                             const polygon = L.polygon(geofencePoints);
//                                             geofenceMap.fitBounds(polygon.getBounds());
//                                         }
//                                     }
//                                 }, 100); // Small delay to ensure DOM is ready
//                             } else if (event.target.id === 'yaml-params-tab') {
//                                 // When switching to YAML tab, update from current form values
//                                 if (document.getElementById('basic-params').classList.contains('active')) {
//                                     updateYAMLFromBasicParams();
//                                 } else if (document.getElementById('geofence-params').classList.contains('active')) {
//                                     updateGeofenceYaml();
//                                 }
//                             }
//                         });
//                     });
//                 }
//             }, { once: true });
            
//         } catch (error) {
//             console.error("Error opening simulation parameters modal:", error);
//             showNotification("Error loading simulation parameters", "error");
//         }
//     });
    
//     // Helper function to add a row to the geofence table
//     function addGeofenceTableRow(lat = 0, lon = 0) {
//         const geofenceTableBody = document.getElementById('geofenceTableBody');
//         const rowIndex = geofenceTableBody.children.length;
        
//         const newRow = document.createElement('tr');
//         newRow.innerHTML = `
//             <td>
//                 <input type="number" class="form-control form-control-sm geofence-lat" 
//                        value="${lat}" step="0.000001" min="-90" max="90">
//             </td>
//             <td>
//                 <input type="number" class="form-control form-control-sm geofence-lon" 
//                        value="${lon}" step="0.000001" min="-180" max="180">
//             </td>
//             <td class="text-center">
//                 <button type="button" class="btn btn-sm btn-danger remove-geofence-point">
//                     <i class="bi bi-trash"></i>
//                 </button>
//             </td>
//         `;
//         geofenceTableBody.appendChild(newRow);
        
//         // Add event listener to the remove button
//         const removeButton = newRow.querySelector('.remove-geofence-point');
//         removeButton?.addEventListener('click', function() {
//             // Remove the row from the table
//             newRow.remove();
            
//             // Update the YAML
//             updateGeofenceYaml();
            
//             // Also update the map if it's initialized
//             if (geofenceMap) {
//                 // Rebuild geofencePoints array from the table
//                 const rows = document.getElementById('geofenceTableBody').querySelectorAll('tr');
//                 geofencePoints = [];
//                 rows.forEach(row => {
//                     const latInput = row.querySelector('.geofence-lat');
//                     const lonInput = row.querySelector('.geofence-lon');
                    
//                     if (latInput && lonInput) {
//                         const lat = parseFloat(latInput.value);
//                         const lon = parseFloat(lonInput.value);
//                         geofencePoints.push([lat, lon]);
//                     }
//                 });
                
//                 // Redraw the geofence on the map
//                 drawGeofenceOnMap();
//             }
//         });
        
//         // Add event listeners to the lat/lon inputs to update the YAML when they change
//         const latInput = newRow.querySelector('.geofence-lat');
//         const lonInput = newRow.querySelector('.geofence-lon');
        
//         const updateInputHandler = function() {
//             updateGeofenceYaml();
            
//             // Also update the map if it's initialized
//             if (geofenceMap) {
//                 // Rebuild geofencePoints array from the table
//                 const rows = document.getElementById('geofenceTableBody').querySelectorAll('tr');
//                 geofencePoints = [];
//                 rows.forEach(row => {
//                     const latInput = row.querySelector('.geofence-lat');
//                     const lonInput = row.querySelector('.geofence-lon');
                    
//                     if (latInput && lonInput) {
//                         const lat = parseFloat(latInput.value);
//                         const lon = parseFloat(lonInput.value);
//                         geofencePoints.push([lat, lon]);
//                     }
//                 });
                
//                 // Redraw the geofence on the map
//                 drawGeofenceOnMap();
//             }
//         };
        
//         latInput?.addEventListener('change', updateInputHandler);
//         lonInput?.addEventListener('change', updateInputHandler);
//     }
    
//     // Function to update the YAML editor when geofence points change
//     function updateGeofenceYaml() {
//         // Skip if we're not on the geofence tab
//         if (!document.getElementById('geofence-params').classList.contains('active')) {
//             return;
//         }
        
//         try {
//             // Get current YAML
//             const yamlText = document.getElementById('simParamsYaml').value;
//             const simParams = jsyaml.load(yamlText) || {};
            
//             // Get geofence points from table
//             const geofencePoints = [];
//             const rows = document.getElementById('geofenceTableBody').querySelectorAll('tr');
//             rows.forEach(row => {
//                 const latInput = row.querySelector('.geofence-lat');
//                 const lonInput = row.querySelector('.geofence-lon');
                
//                 if (latInput && lonInput) {
//                     const lat = parseFloat(latInput.value);
//                     const lon = parseFloat(lonInput.value);
//                     geofencePoints.push([lat, lon]);
//                 }
//             });
            
//             // Update geofence in simParams
//             simParams.geofence = geofencePoints;
            
//             // Use the YAMLGenerator for consistent formatting
//             const yamlGenerator = new YAMLGenerator();
//             const yamlString = yamlGenerator.generateFormattedSimulationYAML(simParams);
            
//             document.getElementById('simParamsYaml').value = yamlString;
//         } catch (error) {
//             console.error("Error updating geofence YAML:", error);
//         }
//     }
    
//     // Add handler for Add Geofence Point button
//     document.getElementById('addGeofencePoint')?.addEventListener('click', function() {
//         // Get the last point's coordinates to use as a starting point
//         const rows = document.getElementById('geofenceTableBody').querySelectorAll('tr');
//         let lat = 12.993496, lon = 80.239007; // Default if no points
        
//         if (rows.length > 0) {
//             const lastRow = rows[rows.length - 1];
//             const latInput = lastRow.querySelector('.geofence-lat');
//             const lonInput = lastRow.querySelector('.geofence-lon');
            
//             if (latInput && lonInput) {
//                 lat = parseFloat(latInput.value);
//                 lon = parseFloat(lonInput.value);
                
//                 // Slightly offset the new point to make it visible
//                 lat += 0.0001;
//                 lon += 0.0001;
//             }
//         }
        
//         addGeofenceTableRow(lat, lon);
//         updateGeofenceYaml();
//     });
    
//     // Add handler for simulation parameters file upload
//     document.getElementById('simParamsFileUpload')?.addEventListener('change', function(event) {
//         const file = event.target.files[0];
//         if (file) {
//             const reader = new FileReader();
//             reader.onload = function(e) {
//                 const yamlText = e.target.result;
                
//                 try {
//                     // Parse the YAML to update the UI
//                     const simParams = jsyaml.load(yamlText);
//                     if (simParams) {
//                         // Format the YAML using our formatter to ensure consistent format
//                         const yamlGenerator = new YAMLGenerator();
//                         const formattedYaml = yamlGenerator.generateFormattedSimulationYAML(simParams);
                        
//                         // Set the textarea value to the formatted YAML
//                         document.getElementById('simParamsYaml').value = formattedYaml;
                        
//                         // Update UI elements
//                         updateSimParamsUIFromYAML(simParams);
                        
//                         showNotification("Simulation parameters loaded successfully", "success");
//                     }
//                 } catch (error) {
//                     console.error("Error parsing uploaded YAML:", error);
//                     showNotification("Error parsing YAML file", "error");
//                 }
//             };
//             reader.readAsText(file);
//         }
//     });
    
//     // Function to update UI from YAML
//     function updateSimParamsUIFromYAML(simParams) {
//         // Update basic parameters tab
//         if (simParams.sim_time !== undefined) document.getElementById('simTimeSetting').value = simParams.sim_time;
//         if (simParams.time_step !== undefined) document.getElementById('timeStepSetting').value = simParams.time_step;
//         if (simParams.density !== undefined) document.getElementById('densitySetting').value = simParams.density;
//         if (simParams.gravity !== undefined) document.getElementById('gravitySetting').value = simParams.gravity;
        
//         if (simParams.world_size && Array.isArray(simParams.world_size)) {
//             document.getElementById('worldSizeXSetting').value = simParams.world_size[0] || 1000;
//             document.getElementById('worldSizeYSetting').value = simParams.world_size[1] || 1000;
            
//             // Handle infinite Z dimension
//             const zValue = simParams.world_size[2];
//             const isInfinite = zValue === 'Inf' || zValue === Infinity;
//             document.getElementById('infiniteZSetting').checked = isInfinite;
//             document.getElementById('worldSizeZSetting').value = isInfinite ? 100 : (zValue || 100);
//         }
        
//         if (simParams.gps_datum && Array.isArray(simParams.gps_datum)) {
//             document.getElementById('gpsDatumLatSetting').value = simParams.gps_datum[0] || 12.99300;
//             document.getElementById('gpsDatumLonSetting').value = simParams.gps_datum[1] || 80.23913;
//             document.getElementById('gpsDatumAltSetting').value = simParams.gps_datum[2] || -87.0;
//         }
        
//         if (simParams.nagents !== undefined) document.getElementById('numAgentsSetting').value = simParams.nagents;
        
//         // Update geofence table
//         if (simParams.geofence && Array.isArray(simParams.geofence)) {
//             const geofenceTableBody = document.getElementById('geofenceTableBody');
//             geofenceTableBody.innerHTML = ''; // Clear existing rows
            
//             simParams.geofence.forEach((point, index) => {
//                 if (Array.isArray(point) && point.length >= 2) {
//                     addGeofenceTableRow(point[0], point[1]);
//                 }
//             });
//         }
//     }
    
//     // Add handler for the YAML text area to update UI when YAML changes
//     document.getElementById('simParamsYaml')?.addEventListener('change', function() {
//         try {
//             const yamlText = this.value;
//             const simParams = jsyaml.load(yamlText);
//             if (simParams) {
//                 // First update the UI elements
//                 updateSimParamsUIFromYAML(simParams);
                
//                 // Then reformat the YAML to ensure consistent formatting
//                 // but only if not on the YAML editor tab (to avoid disrupting manual editing)
//                 if (!document.getElementById('yaml-params').classList.contains('active')) {
//                     const yamlGenerator = new YAMLGenerator();
//                     const formattedYaml = yamlGenerator.generateFormattedSimulationYAML(simParams);
//                     this.value = formattedYaml;
//                 }
//             }
//         } catch (error) {
//             console.error("Error parsing YAML:", error);
//         }
//     });
    
//     // Add handlers for tab switching
//     document.getElementById('basic-params-tab')?.addEventListener('click', function() {
//         // When switching to basic params tab, update the YAML from UI
//         updateYAMLFromBasicParams();
//     });
    
//     document.getElementById('geofence-tab')?.addEventListener('click', function() {
//         // When switching to geofence tab, make sure the table is up to date
//         updateGeofenceTableFromYAML();
//     });
    
//     // Function to update YAML from basic parameters
//     function updateYAMLFromBasicParams() {
//         try {
//             // Get current YAML
//             const yamlText = document.getElementById('simParamsYaml').value;
//             const simParams = jsyaml.load(yamlText) || {};
            
//             // Update basic parameters
//             simParams.sim_time = parseFloat(document.getElementById('simTimeSetting').value);
//             simParams.time_step = parseFloat(document.getElementById('timeStepSetting').value);
//             simParams.density = parseFloat(document.getElementById('densitySetting').value);
//             simParams.gravity = parseFloat(document.getElementById('gravitySetting').value);
            
//             // Handle world size
//             const isInfiniteZ = document.getElementById('infiniteZSetting').checked;
//             simParams.world_size = [
//                 parseFloat(document.getElementById('worldSizeXSetting').value),
//                 parseFloat(document.getElementById('worldSizeYSetting').value),
//                 isInfiniteZ ? 'Inf' : parseFloat(document.getElementById('worldSizeZSetting').value)
//             ];
            
//             // Handle GPS datum
//             simParams.gps_datum = [
//                 parseFloat(document.getElementById('gpsDatumLatSetting').value),
//                 parseFloat(document.getElementById('gpsDatumLonSetting').value),
//                 parseFloat(document.getElementById('gpsDatumAltSetting').value)
//             ];
            
//             // Handle nagents
//             simParams.nagents = parseInt(document.getElementById('numAgentsSetting').value);
            
//             // Use the YAMLGenerator for consistent formatting
//             const yamlGenerator = new YAMLGenerator();
//             const yamlString = yamlGenerator.generateFormattedSimulationYAML(simParams);
            
//             document.getElementById('simParamsYaml').value = yamlString;
//         } catch (error) {
//             console.error("Error updating YAML from basic parameters:", error);
//         }
//     }
    
//     // Function to update geofence table from YAML
//     function updateGeofenceTableFromYAML() {
//         try {
//             // Get current YAML
//             const yamlText = document.getElementById('simParamsYaml').value;
//             const simParams = jsyaml.load(yamlText) || {};
            
//             // Update geofence table
//             if (simParams.geofence && Array.isArray(simParams.geofence)) {
//                 const geofenceTableBody = document.getElementById('geofenceTableBody');
//                 geofenceTableBody.innerHTML = ''; // Clear existing rows
                
//                 simParams.geofence.forEach((point, index) => {
//                     if (Array.isArray(point) && point.length >= 2) {
//                         addGeofenceTableRow(point[0], point[1]);
//                     }
//                 });
//             }
//         } catch (error) {
//             console.error("Error updating geofence table from YAML:", error);
//         }
//     }
    
//     // Add handler for Save Simulation Parameters button
//     document.getElementById('btnSaveSimParams')?.addEventListener('click', function() {
//         try {
//             // Update YAML from UI first
//             if (document.getElementById('basic-params').classList.contains('active')) {
//                 updateYAMLFromBasicParams();
//             } else if (document.getElementById('geofence-params').classList.contains('active')) {
//                 updateGeofenceYaml();
//             }
            
//             // Get the YAML text from the textarea
//             const yamlText = document.getElementById('simParamsYaml').value;
            
//             // Parse the YAML into an object
//             const simParams = jsyaml.load(yamlText);
//             console.log("Parsed simulation parameters:", simParams);
            
//             if (!simParams || typeof simParams !== 'object') {
//                 throw new Error("Invalid YAML format");
//             }
            
//             // Create a clean copy without extra fields
//             const cleanParams = { ...simParams };
//             delete cleanParams.fullReplace; // Remove this field if it exists
            
//             // Add the fullReplace flag internally for the update process
//             simParams.fullReplace = true;
            
//             // Update the vessel model with the new parameters
//             window.currentVesselModel.updateSimulationParams(simParams);
            
//             // Update the UI inputs to reflect the new values
//             if (simParams.sim_time) document.getElementById('simTime').value = simParams.sim_time;
//             if (simParams.time_step) document.getElementById('timeStep').value = simParams.time_step;
//             if (simParams.density) document.getElementById('density').value = simParams.density;
//             if (simParams.gravity) document.getElementById('gravity').value = simParams.gravity;
            
//             if (simParams.world_size && Array.isArray(simParams.world_size)) {
//                 document.getElementById('worldSizeX').value = simParams.world_size[0] || 1000;
//                 document.getElementById('worldSizeY').value = simParams.world_size[1] || 1000;
//                 document.getElementById('worldSizeZ').value = simParams.world_size[2] === 'Inf' ? 100 : simParams.world_size[2] || 100;
//             }
            
//             if (simParams.gps_datum && Array.isArray(simParams.gps_datum)) {
//                 document.getElementById('gpsDatumLat').value = simParams.gps_datum[0] || 12.99300;
//                 document.getElementById('gpsDatumLon').value = simParams.gps_datum[1] || 80.23913;
//                 document.getElementById('gpsDatumAlt').value = simParams.gps_datum[2] || -87.0;
//             }
            
//             // Generate the simulation_input.yml directly and update the preview
//             const yamlGenerator = new YAMLGenerator();
//             const simInputYaml = yamlGenerator.generateSimulationInputYAML(window.currentVesselModel.config);
//             document.getElementById('simulation-yaml-content').textContent = simInputYaml;
            
//             // Also update all other YAML previews
//             generateYAMLPreviews();
            
//             // Close the modal
//             const simParamsModal = bootstrap.Modal.getInstance(document.getElementById('simParamsModal'));
//             simParamsModal.hide();
            
//             // Show success notification
//             showNotification("Simulation parameters updated successfully", "success");
//         } catch (error) {
//             console.error("Error saving simulation parameters:", error);
//             showNotification("Error: " + error.message, "error");
//         }
//     });

//     // Add handler for theme toggle
//     document.getElementById('btn-toggle-theme')?.addEventListener('click', function() {
//         // Check if dark mode is currently enabled
//         const isDarkMode = document.documentElement.getAttribute('data-bs-theme') === 'dark';
        
//         // Toggle the theme
//         document.documentElement.setAttribute('data-bs-theme', isDarkMode ? 'light' : 'dark');
        
//         // Save preference in localStorage
//         localStorage.setItem('theme', isDarkMode ? 'light' : 'dark');
        
//         // Show notification
//         showNotification(`Switched to ${isDarkMode ? 'light' : 'dark'} mode`, 'info');
//     });
    
//     // Initialize theme based on saved preference or system preference
//     function initializeTheme() {
//         // Check if theme is saved in localStorage
//         const savedTheme = localStorage.getItem('theme');
        
//         if (savedTheme) {
//             // Use saved preference
//             document.documentElement.setAttribute('data-bs-theme', savedTheme);
//         } else {
//             // Default to dark mode instead of system preference
//             document.documentElement.setAttribute('data-bs-theme', 'dark');
            
//             // Listen for system preference changes
//             window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', e => {
//                 if (!localStorage.getItem('theme')) {
//                     // Only apply system preference if user hasn't manually set a preference
//                     const newMode = e.matches ? 'dark' : 'light';
//                     document.documentElement.setAttribute('data-bs-theme', newMode);
//                     showNotification(`Matched system theme: ${newMode} mode`, 'info');
//                 }
//             });
//         }
        
//         // Apply theme-specific adjustments for components that need special handling
//         applyThemeSpecificAdjustments();
//     }
    
//     // Apply additional theme-specific adjustments to components that need special handling
//     function applyThemeSpecificAdjustments() {
//         const isDarkMode = document.documentElement.getAttribute('data-bs-theme') === 'dark';
        
//         // Adjust the Three.js scene background
//         if (window.threeScene && window.threeScene.scene) {
//             // Keep the 3D scene background dark in both themes, but slightly lighter in light mode
//             window.threeScene.scene.background = new THREE.Color(isDarkMode ? '#111111' : '#1e1e1e');
//         }
        
//         // Fix code preview areas for YAML
//         document.querySelectorAll('.yaml-preview, .code-example').forEach(el => {
//             el.style.border = `1px solid var(--border-color)`;
//             el.style.backgroundColor = `var(--code-bg)`;
//             el.style.color = `var(--text-color)`;
//         });
        
//         // Apply simple syntax highlighting to code examples
//         document.querySelectorAll('.code-example').forEach(el => {
//             // Skip if already processed
//             if (el.dataset.highlighted === 'true') return;
            
//             let content = el.innerHTML;
            
//             // Highlight comments
//             content = content.replace(/(#.+)$/gm, '<span class="comment">$1</span>');
            
//             // Highlight keys and values in YAML
//             content = content.replace(/^(\s*)([A-Za-z0-9_-]+)(:)/gm, '$1<span class="key">$2</span>$3');
            
//             // Highlight numbers
//             content = content.replace(/: (-?\d+(\.\d+)?)/g, ': <span class="number">$1</span>');
            
//             // Highlight arrays/lists
//             content = content.replace(/(\[.+?\])/g, '<span class="string">$1</span>');
            
//             el.innerHTML = content;
//             el.dataset.highlighted = 'true';
//         });
        
//         // Fix inline code elements in alerts
//         document.querySelectorAll('.alert code').forEach(el => {
//             el.style.backgroundColor = isDarkMode ? 'rgba(0, 0, 0, 0.2)' : 'rgba(255, 255, 255, 0.5)';
//             el.style.color = `var(--text-color)`;
//             el.style.borderColor = isDarkMode ? 'rgba(0, 0, 0, 0.3)' : 'rgba(0, 0, 0, 0.1)';
//         });
        
//         // Ensure alert text is visible in dark mode
//         if (isDarkMode) {
//             document.querySelectorAll('.alert').forEach(el => {
//                 el.style.setProperty('--alert-text-opacity', '1');
//             });
//         }
        
//         // Fix placeholder text color
//         document.querySelectorAll('input, textarea').forEach(el => {
//             el.style.setProperty('--placeholder-opacity', isDarkMode ? '0.7' : '0.5');
//         });
        
//         // Apply theme to any SVG icons
//         document.querySelectorAll('svg').forEach(svg => {
//             if (!svg.hasAttribute('data-no-theme')) {
//                 svg.style.fill = `var(--text-color)`;
//             }
//         });
//     }
    
//     // Call the initializeTheme function when the page loads
//     document.addEventListener('DOMContentLoaded', initializeTheme);
    
//     // Also listen for theme changes to apply specific adjustments
//     document.addEventListener('themeChange', applyThemeSpecificAdjustments);
    
//     // Observe theme attribute changes and dispatch custom event
//     const observer = new MutationObserver(mutations => {
//         mutations.forEach(mutation => {
//             if (mutation.attributeName === 'data-bs-theme') {
//                 document.dispatchEvent(new CustomEvent('themeChange', {
//                     detail: { theme: document.documentElement.getAttribute('data-bs-theme') }
//                 }));
//                 applyThemeSpecificAdjustments();
//             }
//         });
//     });
    
//     // Start observing data-bs-theme attribute changes
//     observer.observe(document.documentElement, { attributes: true });

//     // Geometry modal functionality
//     const btnEditGeometry = document.getElementById('btnEditGeometry');
//     const geometryModal = new bootstrap.Modal(document.getElementById('geometryModal'));
//     const geometryFileUpload = document.getElementById('geometryFileUpload');
//     const btnSaveGeometry = document.getElementById('btnSaveGeometry');
//     const gdfFileInfo = document.getElementById('gdfFileInfo');

//     // Elements for axis direction selection
//     const lengthAxis = document.getElementById('lengthAxis');
//     const breadthAxis = document.getElementById('breadthAxis');
//     const depthAxis = document.getElementById('depthAxis');

//     // Dimension inputs
//     const vesselLength = document.getElementById('vesselLength');
//     const vesselBreadth = document.getElementById('vesselBreadth');
//     const vesselDepth = document.getElementById('vesselDepth');

//     // Gyration inputs
//     const gyrationX = document.getElementById('gyrationX');
//     const gyrationY = document.getElementById('gyrationY');
//     const gyrationZ = document.getElementById('gyrationZ');

//     // Center point inputs
//     const coX = document.getElementById('coX');
//     const coY = document.getElementById('coY');
//     const coZ = document.getElementById('coZ');
//     const cgX = document.getElementById('cgX');
//     const cgY = document.getElementById('cgY');
//     const cgZ = document.getElementById('cgZ');
//     const cbX = document.getElementById('cbX');
//     const cbY = document.getElementById('cbY');
//     const cbZ = document.getElementById('cbZ');
    
//     // Center point orientation inputs
//     const coRoll = document.getElementById('coRoll');
//     const coPitch = document.getElementById('coPitch');
//     const coYaw = document.getElementById('coYaw');
//     const cgRoll = document.getElementById('cgRoll');
//     const cgPitch = document.getElementById('cgPitch');
//     const cgYaw = document.getElementById('cgYaw');
//     const cbRoll = document.getElementById('cbRoll');
//     const cbPitch = document.getElementById('cbPitch');
//     const cbYaw = document.getElementById('cbYaw');

//     // Geometry file path
//     const geometryFilePath = document.getElementById('geometryFilePath');

//     if (btnEditGeometry) {
//         btnEditGeometry.addEventListener('click', function() {
//             // Load current geometry data from the vessel model
//             const geometry = window.currentVesselModel.config.geometry;
            
//             // Populate the form fields
//             vesselLength.value = geometry.length.toFixed(2);
//             vesselBreadth.value = geometry.breadth.toFixed(2);
//             vesselDepth.value = geometry.depth.toFixed(2);
            
//             // Set gyration values
//             gyrationX.value = geometry.gyration[0].toFixed(2);
//             gyrationY.value = geometry.gyration[1].toFixed(2);
//             gyrationZ.value = geometry.gyration[2].toFixed(2);
            
//             // Set center coordinates (position)
//             const coPos = geometry.CO_position || geometry.CO || [0, 0, 0];
//             coX.value = coPos[0].toFixed(2);
//             coY.value = coPos[1].toFixed(2);
//             coZ.value = coPos[2].toFixed(2);
            
//             const cgPos = geometry.CG_position || geometry.CG || [0, 0, 0];
//             cgX.value = cgPos[0].toFixed(2);
//             cgY.value = cgPos[1].toFixed(2);
//             cgZ.value = cgPos[2].toFixed(2);
            
//             const cbPos = geometry.CB_position || geometry.CB || [0, 0, 0];
//             cbX.value = cbPos[0].toFixed(2);
//             cbY.value = cbPos[1].toFixed(2);
//             cbZ.value = cbPos[2].toFixed(2);
            
//             // Set center orientations
//             const coOri = geometry.CO_orientation || [0, 0, 0];
//             coRoll.value = coOri[0].toFixed(2);
//             coPitch.value = coOri[1].toFixed(2);
//             coYaw.value = coOri[2].toFixed(2);
            
//             const cgOri = geometry.CG_orientation || [0, 0, 0];
//             cgRoll.value = cgOri[0].toFixed(2);
//             cgPitch.value = cgOri[1].toFixed(2);
//             cgYaw.value = cgOri[2].toFixed(2);
            
//             const cbOri = geometry.CB_orientation || [0, 0, 0];
//             cbRoll.value = cbOri[0].toFixed(2);
//             cbPitch.value = cbOri[1].toFixed(2);
//             cbYaw.value = cbOri[2].toFixed(2);
            
//             // Set geometry file path
//             geometryFilePath.value = geometry.geometry_file || '';
            
//             // Set axis directions if available
//             if (geometry.axisConfig) {
//                 lengthAxis.value = geometry.axisConfig.length || 'x';
//                 breadthAxis.value = geometry.axisConfig.breadth || 'y';
//                 depthAxis.value = geometry.axisConfig.depth || 'z';
//             } else {
//                 // Default to X = length, Y = breadth, Z = depth
//                 lengthAxis.value = 'x';
//                 breadthAxis.value = 'y';
//                 depthAxis.value = 'z';
//             }
            
//             // Show dimension source info if available
//             if (gdfFileInfo) {
//                 // Check which file type was used for dimension extraction
//                 let sourceFile = 'Unknown';
//                 if (window.currentVesselModel.modelData.gdfFile) {
//                     sourceFile = 'GDF';
//                 } else if (window.currentVesselModel.modelData.fbxFile) {
//                     sourceFile = 'FBX';
//                 } else if (window.currentVesselModel.modelData.stlFile) {
//                     sourceFile = 'STL';
//                 }
                
//                 if (sourceFile !== 'Unknown') {
//                     gdfFileInfo.style.display = 'block';
//                     gdfFileInfo.innerHTML = `
//                         <i class="bi bi-info-circle"></i> <strong>Dimensions from ${sourceFile} file:</strong><br>
//                         Length: ${geometry.length.toFixed(2)}m (${lengthAxis.value}-axis)<br>
//                         Breadth: ${geometry.breadth.toFixed(2)}m (${breadthAxis.value}-axis)<br>
//                         Depth: ${geometry.depth.toFixed(2)}m (${depthAxis.value}-axis)<br>
//                         <small class="text-muted">Based on the bounding box of the ${sourceFile.toLowerCase()} model</small>
//                     `;
//                 } else {
//                     gdfFileInfo.style.display = 'none';
//                 }
//             }
            
//             // Show the modal
//             geometryModal.show();
//         });
//     }

//     // Handler for geometry file upload
//     if (geometryFileUpload) {
//         geometryFileUpload.addEventListener('change', function() {
//             const file = this.files[0];
//             if (file) {
//                 const reader = new FileReader();
//                 reader.onload = function(e) {
//                     try {
//                         const content = e.target.result;
//                         const geometryData = jsyaml.load(content);
                        
//                         // Update the form with the loaded values
//                         if (geometryData) {
//                             if (geometryData.length) vesselLength.value = parseFloat(geometryData.length).toFixed(2);
//                             if (geometryData.breadth) vesselBreadth.value = parseFloat(geometryData.breadth).toFixed(2);
//                             if (geometryData.depth) vesselDepth.value = parseFloat(geometryData.depth).toFixed(2);
                            
//                             if (geometryData.gyration) {
//                                 const gyration = parseArrayFromString(geometryData.gyration);
//                                 if (gyration.length >= 3) {
//                                     gyrationX.value = parseFloat(gyration[0]).toFixed(2);
//                                     gyrationY.value = parseFloat(gyration[1]).toFixed(2);
//                                     gyrationZ.value = parseFloat(gyration[2]).toFixed(2);
//                                 }
//                             }
                            
//                             // Handle CO (Center of Origin)
//                             if (geometryData.CO) {
//                                 if (typeof geometryData.CO === 'object' && geometryData.CO.position) {
//                                     // New format with position and orientation
//                                     const position = parseArrayFromString(geometryData.CO.position);
//                                     if (position.length >= 3) {
//                                         coX.value = parseFloat(position[0]).toFixed(2);
//                                         coY.value = parseFloat(position[1]).toFixed(2);
//                                         coZ.value = parseFloat(position[2]).toFixed(2);
//                                     }
                                    
//                                     if (geometryData.CO.orientation) {
//                                         const orientation = parseArrayFromString(geometryData.CO.orientation);
//                                         if (orientation.length >= 3) {
//                                             coRoll.value = parseFloat(orientation[0]).toFixed(2);
//                                             coPitch.value = parseFloat(orientation[1]).toFixed(2);
//                                             coYaw.value = parseFloat(orientation[2]).toFixed(2);
//                                         }
//                                     }
//                                 } else {
//                                     // Legacy format with just position array
//                                     const co = parseArrayFromString(geometryData.CO);
//                                     if (co.length >= 3) {
//                                         coX.value = parseFloat(co[0]).toFixed(2);
//                                         coY.value = parseFloat(co[1]).toFixed(2);
//                                         coZ.value = parseFloat(co[2]).toFixed(2);
//                                     }
//                                 }
//                             }
                            
//                             // Handle CG (Center of Gravity)
//                             if (geometryData.CG) {
//                                 if (typeof geometryData.CG === 'object' && geometryData.CG.position) {
//                                     // New format with position and orientation
//                                     const position = parseArrayFromString(geometryData.CG.position);
//                                     if (position.length >= 3) {
//                                         cgX.value = parseFloat(position[0]).toFixed(2);
//                                         cgY.value = parseFloat(position[1]).toFixed(2);
//                                         cgZ.value = parseFloat(position[2]).toFixed(2);
//                                     }
                                    
//                                     if (geometryData.CG.orientation) {
//                                         const orientation = parseArrayFromString(geometryData.CG.orientation);
//                                         if (orientation.length >= 3) {
//                                             cgRoll.value = parseFloat(orientation[0]).toFixed(2);
//                                             cgPitch.value = parseFloat(orientation[1]).toFixed(2);
//                                             cgYaw.value = parseFloat(orientation[2]).toFixed(2);
//                                         }
//                                     }
//                                 } else {
//                                     // Legacy format with just position array
//                                     const cg = parseArrayFromString(geometryData.CG);
//                                     if (cg.length >= 3) {
//                                         cgX.value = parseFloat(cg[0]).toFixed(2);
//                                         cgY.value = parseFloat(cg[1]).toFixed(2);
//                                         cgZ.value = parseFloat(cg[2]).toFixed(2);
//                                     }
//                                 }
//                             }
                            
//                             // Handle CB (Center of Buoyancy)
//                             if (geometryData.CB) {
//                                 if (typeof geometryData.CB === 'object' && geometryData.CB.position) {
//                                     // New format with position and orientation
//                                     const position = parseArrayFromString(geometryData.CB.position);
//                                     if (position.length >= 3) {
//                                         cbX.value = parseFloat(position[0]).toFixed(2);
//                                         cbY.value = parseFloat(position[1]).toFixed(2);
//                                         cbZ.value = parseFloat(position[2]).toFixed(2);
//                                     }
                                    
//                                     if (geometryData.CB.orientation) {
//                                         const orientation = parseArrayFromString(geometryData.CB.orientation);
//                                         if (orientation.length >= 3) {
//                                             cbRoll.value = parseFloat(orientation[0]).toFixed(2);
//                                             cbPitch.value = parseFloat(orientation[1]).toFixed(2);
//                                             cbYaw.value = parseFloat(orientation[2]).toFixed(2);
//                                         }
//                                     }
//                                 } else {
//                                     // Legacy format with just position array
//                                     const cb = parseArrayFromString(geometryData.CB);
//                                     if (cb.length >= 3) {
//                                         cbX.value = parseFloat(cb[0]).toFixed(2);
//                                         cbY.value = parseFloat(cb[1]).toFixed(2);
//                                         cbZ.value = parseFloat(cb[2]).toFixed(2);
//                                     }
//                                 }
//                             }
                            
//                             if (geometryData.geometry_file) geometryFilePath.value = geometryData.geometry_file;
//                         }
                        
//                         // Show success message
//                         showToast('Geometry file loaded successfully', 'success');
//                     } catch (error) {
//                         console.error('Error parsing geometry YAML:', error);
//                         showToast('Error loading geometry file: ' + error.message, 'error');
//                     }
//                 };
//                 reader.readAsText(file);
//             }
//         });
//     }

//     // Helper function to parse array from string like [1.0, 2.0, 3.0]
//     function parseArrayFromString(str) {
//         if (Array.isArray(str)) return str;
//         if (typeof str !== 'string') return [];
        
//         try {
//             // Remove brackets and split by comma
//             const cleanStr = str.replace(/[\[\]]/g, '').trim();
//             return cleanStr.split(',').map(item => parseFloat(item.trim()));
//         } catch (error) {
//             console.error('Error parsing array string:', error);
//             return [];
//         }
//     }

//     // Add function to update dimensions based on axis changes
//     function updateDimensionsBasedOnAxis(currentAxis, previousAxis) {
//         // If we don't have the previous axis configuration, we can't remap dimensions
//         if (!previousAxis) return;
        
//         console.log(`Axis changed from ${previousAxis} to ${currentAxis}`);
        
//         // Get current dimension values
//         const currentLength = parseFloat(vesselLength.value) || 0;
//         const currentBreadth = parseFloat(vesselBreadth.value) || 0;
//         const currentDepth = parseFloat(vesselDepth.value) || 0;
        
//         // Get current axis settings
//         const currentLengthAxis = lengthAxis.value;
//         const currentBreadthAxis = breadthAxis.value;
//         const currentDepthAxis = depthAxis.value;
        
//         // Create dimension maps - mapping from axis to dimension value
//         const previousDimensionMap = {
//             length: previousAxis.length,
//             breadth: previousAxis.breadth,
//             depth: previousAxis.depth
//         };
        
//         const currentDimensionMap = {
//             length: currentLengthAxis,
//             breadth: currentBreadthAxis,
//             depth: currentDepthAxis
//         };
        
//         // Get previous dimension values for each axis
//         const oldValueMap = {
//             [previousAxis.length]: currentLength,
//             [previousAxis.breadth]: currentBreadth,
//             [previousAxis.depth]: currentDepth
//         };
        
//         console.log("Previous axis to dimension map:", previousDimensionMap);
//         console.log("Current axis to dimension map:", currentDimensionMap);
//         console.log("Old values by axis:", oldValueMap);
        
//         // Remap the dimensions based on axis changes
//         const newLength = oldValueMap[currentDimensionMap.length] || currentLength;
//         const newBreadth = oldValueMap[currentDimensionMap.breadth] || currentBreadth;
//         const newDepth = oldValueMap[currentDimensionMap.depth] || currentDepth;
        
//         console.log(`Remapped dimensions: Length: ${newLength}, Breadth: ${newBreadth}, Depth: ${newDepth}`);
        
//         // Update the UI
//         vesselLength.value = newLength.toFixed(2);
//         vesselBreadth.value = newBreadth.toFixed(2);
//         vesselDepth.value = newDepth.toFixed(2);
        
//         // Update info message if visible
//         if (gdfFileInfo && gdfFileInfo.style.display === 'block') {
//             const infoHtml = gdfFileInfo.innerHTML;
//             // Update the axis information in the info text
//             const updatedHtml = infoHtml.replace(
//                 /Length: ([\d.]+)m \(([xyz])-axis\)/,
//                 `Length: ${newLength.toFixed(2)}m (${currentLengthAxis}-axis)`
//             ).replace(
//                 /Breadth: ([\d.]+)m \(([xyz])-axis\)/,
//                 `Breadth: ${newBreadth.toFixed(2)}m (${currentBreadthAxis}-axis)`
//             ).replace(
//                 /Depth: ([\d.]+)m \(([xyz])-axis\)/,
//                 `Depth: ${newDepth.toFixed(2)}m (${currentDepthAxis}-axis)`
//             );
            
//             gdfFileInfo.innerHTML = updatedHtml;
//         }
        
//         // Recalculate gyration after dimension change
//         recalculateGyrationValues();
//     }

//     // Update the axis selection event listeners
//     if (lengthAxis && breadthAxis && depthAxis) {
//         // Store the current axis configuration for comparison when changed
//         let previousAxisConfig = {
//             length: lengthAxis.value || 'x',
//             breadth: breadthAxis.value || 'y',
//             depth: depthAxis.value || 'z'
//         };
        
//         lengthAxis.addEventListener('change', function() {
//             // If both other axes are set to the same as this one, switch one of them
//             if (breadthAxis.value === this.value && depthAxis.value === this.value) {
//                 // Find an available axis
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value);
                
//                 breadthAxis.value = availableAxes[0];
//                 depthAxis.value = availableAxes[1];
//             } 
//             // If just one other axis is the same, switch it
//             else if (breadthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value && a !== depthAxis.value);
//                 breadthAxis.value = availableAxes[0];
//             }
//             else if (depthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value && a !== breadthAxis.value);
//                 depthAxis.value = availableAxes[0];
//             }
            
//             // Get current axis configuration
//             const currentAxisConfig = {
//                 length: lengthAxis.value,
//                 breadth: breadthAxis.value,
//                 depth: depthAxis.value
//             };
            
//             // Update dimensions based on axis changes
//             updateDimensionsBasedOnAxis(currentAxisConfig, previousAxisConfig);
            
//             // Update the previous config for next change
//             previousAxisConfig = { ...currentAxisConfig };
//         });
        
//         breadthAxis.addEventListener('change', function() {
//             // Handle axis conflicts similar to length axis
//             if (lengthAxis.value === this.value && depthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value);
                
//                 lengthAxis.value = availableAxes[0];
//                 depthAxis.value = availableAxes[1];
//             } 
//             else if (lengthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value && a !== depthAxis.value);
//                 lengthAxis.value = availableAxes[0];
//             }
//             else if (depthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value && a !== lengthAxis.value);
//                 depthAxis.value = availableAxes[0];
//             }
            
//             // Get current axis configuration
//             const currentAxisConfig = {
//                 length: lengthAxis.value,
//                 breadth: breadthAxis.value,
//                 depth: depthAxis.value
//             };
            
//             // Update dimensions based on axis changes
//             updateDimensionsBasedOnAxis(currentAxisConfig, previousAxisConfig);
            
//             // Update the previous config for next change
//             previousAxisConfig = { ...currentAxisConfig };
//         });
        
//         depthAxis.addEventListener('change', function() {
//             // Handle axis conflicts similar to other axes
//             if (lengthAxis.value === this.value && breadthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value);
                
//                 lengthAxis.value = availableAxes[0];
//                 breadthAxis.value = availableAxes[1];
//             } 
//             else if (lengthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value && a !== breadthAxis.value);
//                 lengthAxis.value = availableAxes[0];
//             }
//             else if (breadthAxis.value === this.value) {
//                 const axes = ['x', 'y', 'z'];
//                 const availableAxes = axes.filter(a => a !== this.value && a !== lengthAxis.value);
//                 breadthAxis.value = availableAxes[0];
//             }
            
//             // Get current axis configuration
//             const currentAxisConfig = {
//                 length: lengthAxis.value,
//                 breadth: breadthAxis.value,
//                 depth: depthAxis.value
//             };
            
//             // Update dimensions based on axis changes
//             updateDimensionsBasedOnAxis(currentAxisConfig, previousAxisConfig);
            
//             // Update the previous config for next change
//             previousAxisConfig = { ...currentAxisConfig };
//         });
        
//         // Also recalculate when dimensions change
//         vesselLength.addEventListener('change', recalculateGyrationValues);
//         vesselBreadth.addEventListener('change', recalculateGyrationValues);
//         vesselDepth.addEventListener('change', recalculateGyrationValues);
        
//         // If vessel type changes, also recalculate
//         const vesselTypeSelect = document.getElementById('vesselType');
//         if (vesselTypeSelect) {
//             vesselTypeSelect.addEventListener('change', recalculateGyrationValues);
//         }
//     }

//     /**
//      * Recalculate gyration values based on vessel dimensions
//      */
//     function recalculateGyrationValues() {
//         try {
//             if (!window.currentVesselModel || !window.currentVesselModel.config) {
//                 console.warn("Cannot recalculate gyration values: vessel model not available");
//                 return;
//             }
            
//             // Get current dimensions
//             const length = parseFloat(vesselLength.value) || 0;
//             const breadth = parseFloat(vesselBreadth.value) || 0;
//             const depth = parseFloat(vesselDepth.value) || 0;
            
//             if (length <= 0 || breadth <= 0 || depth <= 0) {
//                 console.warn("Cannot recalculate gyration: invalid dimensions");
//                 return;
//             }
            
//             // Get current axis configuration
//             const lAxis = lengthAxis.value;
//             const bAxis = breadthAxis.value;
//             const dAxis = depthAxis.value;
            
//             // Create a mapping of dimensions to axes
//             const dimensionMap = {
//                 x: 0,
//                 y: 0,
//                 z: 0
//             };
            
//             // Map each axis to its corresponding dimension value
//             dimensionMap[lAxis] = length;
//             dimensionMap[bAxis] = breadth;
//             dimensionMap[dAxis] = depth;
            
//             console.log("Dimension map for gyration calculation:", dimensionMap);
            
//             // Calculate gyration values based on vessel type and dimensions
//             const vesselType = window.currentVesselModel.config.vesselType || 'generic';
//             let gyrationValues = [0, 0, 0];
            
//             switch (vesselType.toLowerCase()) {
//                 case 'tug':
//                 case 'tugboat':
//                     // For tugs: kxx ≈ 0.35-0.4*B, kyy ≈ 0.25*L, kzz ≈ 0.25-0.3*L
//                     gyrationValues[0] = 0.38 * dimensionMap['y']; // kxx = 0.38*B
//                     gyrationValues[1] = 0.25 * dimensionMap['x']; // kyy = 0.25*L
//                     gyrationValues[2] = 0.28 * dimensionMap['x']; // kzz = 0.28*L
//                     break;
                    
//                 case 'container':
//                 case 'containership':
//                     // For container ships: kxx ≈ 0.4*B, kyy ≈ 0.25*L, kzz ≈ 0.26*L
//                     gyrationValues[0] = 0.4 * dimensionMap['y'];  // kxx = 0.4*B
//                     gyrationValues[1] = 0.25 * dimensionMap['x']; // kyy = 0.25*L
//                     gyrationValues[2] = 0.26 * dimensionMap['x']; // kzz = 0.26*L
//                     break;
                    
//                 case 'tanker':
//                     // For tankers: kxx ≈ 0.35-0.4*B, kyy ≈ 0.24-0.26*L, kzz ≈ 0.24-0.26*L
//                     gyrationValues[0] = 0.38 * dimensionMap['y'];  // kxx = 0.38*B
//                     gyrationValues[1] = 0.25 * dimensionMap['x']; // kyy = 0.25*L
//                     gyrationValues[2] = 0.25 * dimensionMap['x']; // kzz = 0.25*L
//                     break;
                    
//                 case 'ferry':
//                     // For ferries: kxx ≈ 0.4-0.45*B, kyy ≈ 0.25-0.3*L, kzz ≈ 0.25-0.3*L
//                     gyrationValues[0] = 0.42 * dimensionMap['y'];  // kxx = 0.42*B
//                     gyrationValues[1] = 0.28 * dimensionMap['x']; // kyy = 0.28*L
//                     gyrationValues[2] = 0.28 * dimensionMap['x']; // kzz = 0.28*L
//                     break;
                    
//                 case 'sailboat':
//                 case 'sailship':
//                     // For sailboats: kxx ≈ 0.4-0.45*B, kyy ≈ 0.2-0.25*L, kzz ≈ 0.2-0.25*L
//                     gyrationValues[0] = 0.43 * dimensionMap['y'];  // kxx = 0.43*B
//                     gyrationValues[1] = 0.23 * dimensionMap['x']; // kyy = 0.23*L
//                     gyrationValues[2] = 0.23 * dimensionMap['x']; // kzz = 0.23*L
//                     break;
                    
//                 case 'generic':
//                 default:
//                     // Generic ship: kxx ≈ 0.4*B, kyy ≈ 0.25*L, kzz ≈ 0.25*L
//                     gyrationValues[0] = 0.4 * dimensionMap['y'];  // kxx = 0.4*B
//                     gyrationValues[1] = 0.25 * dimensionMap['x']; // kyy = 0.25*L
//                     gyrationValues[2] = 0.25 * dimensionMap['x']; // kzz = 0.25*L
//                     break;
//             }
            
//             // Update the input fields
//             if (gyrationX && gyrationY && gyrationZ) {
//                 gyrationX.value = gyrationValues[0].toFixed(2);
//                 gyrationY.value = gyrationValues[1].toFixed(2);
//                 gyrationZ.value = gyrationValues[2].toFixed(2);
//             }
            
//             // Also update the model configuration
//             window.currentVesselModel.config.geometry.gyration = gyrationValues.map(v => parseFloat(v.toFixed(2)));
            
//             console.log(`Updated gyration values for ${vesselType}: [${gyrationValues.map(v => v.toFixed(2)).join(', ')}]`);
//         } catch (error) {
//             console.error("Error recalculating gyration values:", error);
//         }
//     }

//     // Add a new function to initialize center points controls
//     function initializeCenterPointsControls(threeScene) {
//         const toggleCenterPoints = document.getElementById('toggleCenterPoints');
//         if (toggleCenterPoints) {
//             toggleCenterPoints.addEventListener('change', function() {
//                 if (threeScene && typeof threeScene.updateCenterPointsVisibility === 'function') {
//                     threeScene.updateCenterPointsVisibility(this.checked);
//                     showNotification(`Center points ${this.checked ? 'visible' : 'hidden'}`, 'info');
//                 }
//             });
//         }
//     }

//     // Add handler for Copy from Map button
//     document.getElementById('copyFromMapGeofence')?.addEventListener('click', function() {
//         // Since there's no actual map integration yet, use default geofence coordinates
//         // In a real implementation, this would get coordinates from a map selection
//         const yamlGenerator = new YAMLGenerator();
//         const defaultGeofence = [
//             [12.993496, 80.239007],
//             [12.993500, 80.238903],
//             [12.993485, 80.238829],
//             [12.993359, 80.238699],
//             [12.993547, 80.238437],
//             [12.993716, 80.238475],
//             [12.994077, 80.239559],
//             [12.993809, 80.239807],
//             [12.993271, 80.240646],
//             [12.993050, 80.240423],
//             [12.993193, 80.239274],
//             [12.993044, 80.239217],
//             [12.993054, 80.239120],
//             [12.993335, 80.239098],
//             [12.993496, 80.239007] // Closing point to complete the polygon
//         ];
        
//         // Clear existing geofence points
//         const geofenceTableBody = document.getElementById('geofenceTableBody');
//         geofenceTableBody.innerHTML = '';
        
//         // Add new points from the default geofence
//         defaultGeofence.forEach(point => {
//             addGeofenceTableRow(point[0], point[1]);
//         });
        
//         // Update the YAML
//         updateGeofenceYaml();
        
//         // Show notification
//         showNotification("Geofence coordinates imported", "success");
//     });

//     // Initialize and handle the geofence map
//     let geofenceMap = null;
//     let geofenceLayer = null;
//     let datumMarker = null;
//     let geofencePoints = [];
    
//     // Fullscreen map variables
//     let fullscreenGeofenceMap = null;
//     let fullscreenGeofenceLayer = null;
//     let fullscreenDatumMarker = null;
    
//     function initializeGeofenceMap() {
//         // Skip if the map is already initialized
//         if (geofenceMap) {
//             return;
//         }
        
//         // Create a map centered on a default location (Chennai, India)
//         geofenceMap = L.map('geofence-map').setView([12.9930, 80.2392], 15);
        
//         // Add OpenStreetMap as the base layer
//         L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
//             attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
//         }).addTo(geofenceMap);
        
//         // Create a feature group for the geofence polygon
//         geofenceLayer = L.featureGroup().addTo(geofenceMap);

//         // Add handler for the expand map button
//         document.getElementById('expandMapBtn')?.addEventListener('click', function() {
//             openFullscreenMap();
//         });
        
//         // Get current simulation parameters
//         const simParams = window.currentVesselModel.config.simulation || {};
        
//         // Initialize with GPS datum if available
//         if (simParams.gps_datum && Array.isArray(simParams.gps_datum) && simParams.gps_datum.length >= 2) {
//             const [lat, lon] = simParams.gps_datum;
//             datumMarker = L.marker([lat, lon], { 
//                 title: 'GPS Datum',
//                 draggable: true,
//                 icon: L.divIcon({
//                     className: 'datum-marker', 
//                     html: '<i class="bi bi-geo-alt-fill" style="color: blue; font-size: 24px;"></i>',
//                     iconSize: [24, 24],
//                     iconAnchor: [12, 24]
//                 })
//             }).addTo(geofenceMap);
            
//             // Update form values when the datum marker is dragged
//             datumMarker.on('dragend', function(e) {
//                 const position = datumMarker.getLatLng();
//                 document.getElementById('gpsDatumLatSetting').value = position.lat.toFixed(6);
//                 document.getElementById('gpsDatumLonSetting').value = position.lng.toFixed(6);
//                 updateYAMLFromBasicParams();
//             });
            
//             // Center the map on the GPS datum
//             geofenceMap.setView([lat, lon], 15);
//         }
        
//         // Initialize with geofence if available
//         if (simParams.geofence && Array.isArray(simParams.geofence) && simParams.geofence.length >= 3) {
//             geofencePoints = simParams.geofence.map(point => {
//                 if (Array.isArray(point) && point.length >= 2) {
//                     return [point[0], point[1]];
//                 }
//                 return null;
//             }).filter(point => point !== null);
            
//             drawGeofenceOnMap();
//         }
        
//         // Click handler for adding geofence points
//         geofenceMap.on('click', function(e) {
//             const latlng = e.latlng;
            
//             // Add to our geofence points array
//             geofencePoints.push([latlng.lat, latlng.lng]);
            
//             // Redraw the geofence
//             drawGeofenceOnMap();
            
//             // Update the geofence table
//             updateGeofenceTableFromMap();
//         });
        
//         // Add handler for Set GPS Datum button
//         document.getElementById('setGpsDatum')?.addEventListener('click', function() {
//             // Enable datum selection mode
//             showNotification("Click on the map to set GPS Datum location", "info");
            
//             // We need to use a one-time event handler that stops propagation to prevent adding a geofence point
//             const mapClickHandler = function(e) {
//                 // Stop propagation to prevent the regular map click handler from firing
//                 e.originalEvent.stopPropagation();
//                 e.originalEvent.preventDefault();
                
//                 const latlng = e.latlng;
                
//                 // Remove existing marker if present
//                 if (datumMarker) {
//                     datumMarker.remove();
//                 }
                
//                 // Create new marker
//                 datumMarker = L.marker([latlng.lat, latlng.lng], { 
//                     title: 'GPS Datum',
//                     draggable: true,
//                     icon: L.divIcon({
//                         className: 'datum-marker', 
//                         html: '<i class="bi bi-geo-alt-fill" style="color: blue; font-size: 24px;"></i>',
//                         iconSize: [24, 24],
//                         iconAnchor: [12, 24]
//                     })
//                 }).addTo(geofenceMap);
                
//                 // Update form values
//                 document.getElementById('gpsDatumLatSetting').value = latlng.lat.toFixed(6);
//                 document.getElementById('gpsDatumLonSetting').value = latlng.lng.toFixed(6);
                
//                 // Update the YAML
//                 updateYAMLFromBasicParams();
                
//                 // Update marker drag event
//                 datumMarker.on('dragend', function(e) {
//                     const position = datumMarker.getLatLng();
//                     document.getElementById('gpsDatumLatSetting').value = position.lat.toFixed(6);
//                     document.getElementById('gpsDatumLonSetting').value = position.lng.toFixed(6);
//                     updateYAMLFromBasicParams();
//                 });
                
//                 showNotification("GPS Datum updated", "success");
                
//                 // Remove this one-time handler
//                 geofenceMap.off('click', mapClickHandler);
//             };
            
//             // Use this handler once
//             geofenceMap.once('click', mapClickHandler);
//         });
        
//         // Add handler for Clear Geofence button
//         document.getElementById('clearGeofence')?.addEventListener('click', function() {
//             geofencePoints = [];
//             drawGeofenceOnMap();
//             updateGeofenceTableFromMap();
//             showNotification("Geofence cleared", "info");
//         });
        
//         // Add handler for Update from Map button
//         document.getElementById('copyFromMapGeofence')?.addEventListener('click', function() {
//             updateGeofenceTableFromMap();
//             showNotification("Geofence table updated from map", "success");
//         });
//     }
    
//     // Initialize fullscreen map
//     function openFullscreenMap() {
//         // Show the modal
//         const fullscreenMapModal = new bootstrap.Modal(document.getElementById('fullscreenMapModal'));
//         fullscreenMapModal.show();
        
//         // Initialize the map after the modal is shown
//         document.getElementById('fullscreenMapModal').addEventListener('shown.bs.modal', function() {
//             // Create a new map instance for the fullscreen view
//             if (!fullscreenGeofenceMap) {
//                 fullscreenGeofenceMap = L.map('fullscreen-geofence-map');
                
//                 // Add OpenStreetMap as the base layer
//                 L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
//                     attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
//                 }).addTo(fullscreenGeofenceMap);
                
//                 // Create a feature group for the geofence polygon
//                 fullscreenGeofenceLayer = L.featureGroup().addTo(fullscreenGeofenceMap);
                
//                 // Click handler for adding geofence points
//                 fullscreenGeofenceMap.on('click', function(e) {
//                     const latlng = e.latlng;
                    
//                     // Add to our geofence points array
//                     geofencePoints.push([latlng.lat, latlng.lng]);
                    
//                     // Redraw the geofence on both maps
//                     drawGeofenceOnMap();
//                     drawGeofenceOnFullscreenMap();
                    
//                     // Update the geofence table
//                     updateGeofenceTableFromMap();
//                 });
                
//                 // Add handler for the fullscreen Set GPS Datum button
//                 document.getElementById('fullscreenSetGpsDatum')?.addEventListener('click', function() {
//                     // Enable datum selection mode
//                     showNotification("Click on the map to set GPS Datum location", "info");
                    
//                     // We need to use a one-time event handler that stops propagation
//                     const mapClickHandler = function(e) {
//                         // Stop propagation to prevent the regular map click handler from firing
//                         e.originalEvent.stopPropagation();
//                         e.originalEvent.preventDefault();
                        
//                         const latlng = e.latlng;
                        
//                         // Remove existing markers if present
//                         if (datumMarker) {
//                             datumMarker.remove();
//                         }
//                         if (fullscreenDatumMarker) {
//                             fullscreenDatumMarker.remove();
//                         }
                        
//                         // Create new markers on both maps
//                         datumMarker = L.marker([latlng.lat, latlng.lng], { 
//                             title: 'GPS Datum',
//                             draggable: true,
//                             icon: L.divIcon({
//                                 className: 'datum-marker', 
//                                 html: '<i class="bi bi-geo-alt-fill" style="color: blue; font-size: 24px;"></i>',
//                                 iconSize: [24, 24],
//                                 iconAnchor: [12, 24]
//                             })
//                         }).addTo(geofenceMap);
                        
//                         fullscreenDatumMarker = L.marker([latlng.lat, latlng.lng], { 
//                             title: 'GPS Datum',
//                             draggable: true,
//                             icon: L.divIcon({
//                                 className: 'datum-marker', 
//                                 html: '<i class="bi bi-geo-alt-fill" style="color: blue; font-size: 24px;"></i>',
//                                 iconSize: [24, 24],
//                                 iconAnchor: [12, 24]
//                             })
//                         }).addTo(fullscreenGeofenceMap);
                        
//                         // Update form values
//                         document.getElementById('gpsDatumLatSetting').value = latlng.lat.toFixed(6);
//                         document.getElementById('gpsDatumLonSetting').value = latlng.lng.toFixed(6);
                        
//                         // Update the YAML
//                         updateYAMLFromBasicParams();
                        
//                         // Update drag events for both markers
//                         datumMarker.on('dragend', function(e) {
//                             const position = datumMarker.getLatLng();
//                             document.getElementById('gpsDatumLatSetting').value = position.lat.toFixed(6);
//                             document.getElementById('gpsDatumLonSetting').value = position.lng.toFixed(6);
                            
//                             // Also update the fullscreen marker
//                             if (fullscreenDatumMarker) {
//                                 fullscreenDatumMarker.setLatLng(position);
//                             }
                            
//                             updateYAMLFromBasicParams();
//                         });
                        
//                         fullscreenDatumMarker.on('dragend', function(e) {
//                             const position = fullscreenDatumMarker.getLatLng();
//                             document.getElementById('gpsDatumLatSetting').value = position.lat.toFixed(6);
//                             document.getElementById('gpsDatumLonSetting').value = position.lng.toFixed(6);
                            
//                             // Also update the regular marker
//                             if (datumMarker) {
//                                 datumMarker.setLatLng(position);
//                             }
                            
//                             updateYAMLFromBasicParams();
//                         });
                        
//                         showNotification("GPS Datum updated", "success");
                        
//                         // Remove this one-time handler
//                         fullscreenGeofenceMap.off('click', mapClickHandler);
//                     };
                    
//                     // Use this handler once
//                     fullscreenGeofenceMap.once('click', mapClickHandler);
//                 });
                
//                 // Add handler for the fullscreen Clear Geofence button
//                 document.getElementById('fullscreenClearGeofence')?.addEventListener('click', function() {
//                     geofencePoints = [];
//                     drawGeofenceOnMap();
//                     drawGeofenceOnFullscreenMap();
//                     updateGeofenceTableFromMap();
//                     showNotification("Geofence cleared", "info");
//                 });
//             }
            
//             // Sync the view with the regular map
//             if (geofenceMap) {
//                 const center = geofenceMap.getCenter();
//                 const zoom = geofenceMap.getZoom();
//                 fullscreenGeofenceMap.setView(center, zoom);
//             } else {
//                 fullscreenGeofenceMap.setView([12.9930, 80.2392], 15);
//             }
            
//             // Draw the current geofence
//             drawGeofenceOnFullscreenMap();
            
//             // Add the datum marker if it exists
//             if (datumMarker) {
//                 const position = datumMarker.getLatLng();
                
//                 // Remove existing marker if present
//                 if (fullscreenDatumMarker) {
//                     fullscreenDatumMarker.remove();
//                 }
                
//                 // Create new marker
//                 fullscreenDatumMarker = L.marker([position.lat, position.lng], { 
//                     title: 'GPS Datum',
//                     draggable: true,
//                     icon: L.divIcon({
//                         className: 'datum-marker', 
//                         html: '<i class="bi bi-geo-alt-fill" style="color: blue; font-size: 24px;"></i>',
//                         iconSize: [24, 24],
//                         iconAnchor: [12, 24]
//                     })
//                 }).addTo(fullscreenGeofenceMap);
                
//                 // Update drag event
//                 fullscreenDatumMarker.on('dragend', function(e) {
//                     const newPosition = fullscreenDatumMarker.getLatLng();
//                     document.getElementById('gpsDatumLatSetting').value = newPosition.lat.toFixed(6);
//                     document.getElementById('gpsDatumLonSetting').value = newPosition.lng.toFixed(6);
                    
//                     // Also update the regular marker
//                     if (datumMarker) {
//                         datumMarker.setLatLng(newPosition);
//                     }
                    
//                     updateYAMLFromBasicParams();
//                 });
//             }
            
//             // Fix the map display (needed when a map is initialized inside a hidden element)
//             setTimeout(() => {
//                 fullscreenGeofenceMap.invalidateSize();
                
//                 // Zoom to fit markers if we have a geofence
//                 if (geofencePoints.length >= 3) {
//                     const polygon = L.polygon(geofencePoints);
//                     fullscreenGeofenceMap.fitBounds(polygon.getBounds());
//                 } else if (geofencePoints.length > 0) {
//                     // Otherwise zoom to include all markers
//                     const group = L.featureGroup(fullscreenGeofenceLayer.getLayers());
//                     if (group.getBounds().isValid()) {
//                         fullscreenGeofenceMap.fitBounds(group.getBounds(), {
//                             padding: [50, 50]
//                         });
//                     }
//                 }
//             }, 100);
//         });
        
//         // Handle modal close event
//         document.getElementById('fullscreenMapModal').addEventListener('hidden.bs.modal', function() {
//             // Refresh the regular map view to sync any changes
//             if (geofenceMap) {
//                 setTimeout(() => {
//                     geofenceMap.invalidateSize();
//                     drawGeofenceOnMap();
//                 }, 100);
//             }
//         });
//     }
    
//     // Draw the geofence on the fullscreen map
//     function drawGeofenceOnFullscreenMap() {
//         if (!fullscreenGeofenceMap || !fullscreenGeofenceLayer) {
//             return;
//         }
        
//         // Clear existing geofence
//         fullscreenGeofenceLayer.clearLayers();
        
//         // Always draw the markers for each point
//         geofencePoints.forEach((point, index) => {
//             L.marker(point, {
//                 icon: L.divIcon({
//                     className: 'geofence-vertex', 
//                     html: `<div style="background-color: #FF5722; color: white; border-radius: 50%; width: 20px; height: 20px; text-align: center; line-height: 20px; font-size: 12px;">${index + 1}</div>`,
//                     iconSize: [20, 20],
//                     iconAnchor: [10, 10]
//                 })
//             }).addTo(fullscreenGeofenceLayer);
//         });
        
//         // If we have at least 2 points, draw lines connecting them
//         if (geofencePoints.length >= 2) {
//             // Create a polyline from the points
//             L.polyline(geofencePoints, {
//                 color: '#FF5722',
//                 weight: 2,
//                 opacity: 0.7,
//                 dashArray: '5, 5'
//             }).addTo(fullscreenGeofenceLayer);
//         }
        
//         // If we have at least 3 points, also draw a polygon
//         if (geofencePoints.length >= 3) {
//             // Create a polygon from the points
//             const polygon = L.polygon(geofencePoints, {
//                 color: '#FF5722',
//                 weight: 3,
//                 opacity: 0.7,
//                 fillColor: '#FF8A65',
//                 fillOpacity: 0.3
//             }).addTo(fullscreenGeofenceLayer);
//         }
//     }
    
//     // Draw the geofence on the map
//     function drawGeofenceOnMap() {
//         // Clear existing geofence
//         geofenceLayer.clearLayers();
        
//         // Always draw the markers for each point
//         geofencePoints.forEach((point, index) => {
//             L.marker(point, {
//                 icon: L.divIcon({
//                     className: 'geofence-vertex', 
//                     html: `<div style="background-color: #FF5722; color: white; border-radius: 50%; width: 20px; height: 20px; text-align: center; line-height: 20px; font-size: 12px;">${index + 1}</div>`,
//                     iconSize: [20, 20],
//                     iconAnchor: [10, 10]
//                 })
//             }).addTo(geofenceLayer);
//         });
        
//         // If we have at least 2 points, draw lines connecting them
//         if (geofencePoints.length >= 2) {
//             // Create a polyline from the points
//             L.polyline(geofencePoints, {
//                 color: '#FF5722',
//                 weight: 2,
//                 opacity: 0.7,
//                 dashArray: '5, 5'
//             }).addTo(geofenceLayer);
//         }
        
//         // If we have at least 3 points, also draw a polygon
//         if (geofencePoints.length >= 3) {
//             // Create a polygon from the points
//             const polygon = L.polygon(geofencePoints, {
//                 color: '#FF5722',
//                 weight: 3,
//                 opacity: 0.7,
//                 fillColor: '#FF8A65',
//                 fillOpacity: 0.3
//             }).addTo(geofenceLayer);
            
//             // Fit the map to the geofence bounds
//             geofenceMap.fitBounds(polygon.getBounds());
//         } else if (geofencePoints.length > 0) {
//             // If we don't have enough points for a polygon, but we have some points,
//             // at least zoom to include all points
//             const group = L.featureGroup(geofenceLayer.getLayers());
//             if (group.getBounds().isValid()) {
//                 geofenceMap.fitBounds(group.getBounds(), {
//                     padding: [50, 50]
//                 });
//             }
//         }
//     }
    
//     // Update the geofence table from the map points
//     function updateGeofenceTableFromMap() {
//         const geofenceTableBody = document.getElementById('geofenceTableBody');
//         geofenceTableBody.innerHTML = ''; // Clear existing rows
        
//         // Add each point to the table
//         geofencePoints.forEach((point, index) => {
//             addGeofenceTableRow(point[0], point[1]);
//         });
        
//         // Make sure we also update the YAML
//         updateGeofenceYaml();
//     }

//     /**
//      * Center the camera on a specific point
//      */
//     function centerCameraOnPoint(point) {
//         if (!threeScene || !threeScene.camera || !threeScene.controls) return;
        
//         // Set the orbit controls target to the point
//         threeScene.controls.target.copy(point);
//         threeScene.controls.update();
//     }

//     // GDF dimension extraction function removed
    
//     /**
//      * Format a value with proper units
//      */
//     function formatWithUnits(value, units) {
//         return `${parseFloat(value).toFixed(2)} ${units}`;
//     }

//     // Save geometry settings when the save button is clicked
//     if (btnSaveGeometry) {
//         btnSaveGeometry.addEventListener('click', function() {
//             try {
//                 // Validate dimensions
//                 const length = parseFloat(vesselLength.value);
//                 const breadth = parseFloat(vesselBreadth.value);
//                 const depth = parseFloat(vesselDepth.value);
                
//                 if (isNaN(length) || length <= 0 || isNaN(breadth) || breadth <= 0 || isNaN(depth) || depth <= 0) {
//                     throw new Error('Dimensions must be positive values');
//                 }
                
//                 // Get gyration values
//                 const gyration = [
//                     parseFloat(gyrationX.value),
//                     parseFloat(gyrationY.value),
//                     parseFloat(gyrationZ.value)
//                 ];
                
//                 // Get center positions
//                 const CO = [
//                     parseFloat(coX.value),
//                     parseFloat(coY.value),
//                     parseFloat(coZ.value)
//                 ];
                
//                 const CG = [
//                     parseFloat(cgX.value),
//                     parseFloat(cgY.value),
//                     parseFloat(cgZ.value)
//                 ];
                
//                 const CB = [
//                     parseFloat(cbX.value),
//                     parseFloat(cbY.value),
//                     parseFloat(cbZ.value)
//                 ];
                
//                 // Get center orientations (from UI in degrees)
//                 const CO_orientation_deg = [
//                     parseFloat(coRoll.value),
//                     parseFloat(coPitch.value),
//                     parseFloat(coYaw.value)
//                 ];
                
//                 const CG_orientation_deg = [
//                     parseFloat(cgRoll.value),
//                     parseFloat(cgPitch.value),
//                     parseFloat(cgYaw.value)
//                 ];
                
//                 const CB_orientation_deg = [
//                     parseFloat(cbRoll.value),
//                     parseFloat(cbPitch.value),
//                     parseFloat(cbYaw.value)
//                 ];
                
//                 // Get axis configuration
//                 const axisConfig = {
//                     length: lengthAxis.value,
//                     breadth: breadthAxis.value,
//                     depth: depthAxis.value
//                 };
                
//                 // Update vessel model with new geometry data
//                 window.currentVesselModel.config.geometry.length = length;
//                 window.currentVesselModel.config.geometry.breadth = breadth;
//                 window.currentVesselModel.config.geometry.depth = depth;
//                 window.currentVesselModel.config.geometry.gyration = gyration;
                
//                 // Update both legacy and new format
//                 window.currentVesselModel.config.geometry.CO = CO;
//                 window.currentVesselModel.config.geometry.CG = CG;
//                 window.currentVesselModel.config.geometry.CB = CB;
                
//                 window.currentVesselModel.config.geometry.CO_position = CO;
//                 window.currentVesselModel.config.geometry.CG_position = CG;
//                 window.currentVesselModel.config.geometry.CB_position = CB;
                
//                 // Pass degree values to the 3D scene update
//                 // (updateCenterPoint will convert them to radians internally for storage)
//                 if (window.threeScene) {
//                     // Update the ThreeScene center points
//                     window.threeScene.updateCenterPoint('CO', CO, CO_orientation_deg);
//                     window.threeScene.updateCenterPoint('CG', CG, CG_orientation_deg);
//                     window.threeScene.updateCenterPoint('CB', CB, CB_orientation_deg);
                    
//                     // Force a rendering update
//                     window.threeScene.render();
//                 }
                
//                 window.currentVesselModel.config.geometry.geometry_file = geometryFilePath.value;
                
//                 // Store axis direction configuration in the model
//                 window.currentVesselModel.config.geometry.axisConfig = axisConfig;
                
//                 // Close the modal
//                 geometryModal.hide();
                
//                 // Show success message
//                 showToast('Geometry parameters saved successfully', 'success');
//             } catch (error) {
//                 console.error('Error saving geometry parameters:', error);
//                 showToast('Error: ' + error.message, 'error');
//             }
//         });
//     }
// }); 
// ===========================
// main.js (FULL corrected)
// Multi-vessel visualization using VesselManager & VesselVisualizer
// Works with state_test (or vessel_state for backward compatibility) format:
// [u, v, w, p, r, x, y, z, phi, theta, psi, rudder_command]
// ===========================

// ---- Config / Colors ----
const DEFAULT_ROSBRIDGE_URL = 'ws://localhost:9090';
const DEFAULT_MAX_POINTS = 200; // points per chart dataset
const vesselColorMap = {
  sookshma_01: '#1E88E5',
  sookshma2_01: '#D81B60',
  sookshma3_02: '#43A047',
  evader_03: '#FDD835'
};
const fallbackVesselTopics = [
  '/sookshma_01/state_test',
  '/sookshma2_01/state_test',
  '/sookshma3_02/state_test',
  '/evader_03/state_test'
];

// ---- Globals used by classes ----
let lastMessageTime = Date.now();
let globalCharts = {}; // created once and shared among all visualizers
let ros = null; // ROSLIB.Ros instance

// Utility: ensure Chart plugins registration is attempted
function registerChartPluginsIfNeeded() {
  if (typeof Chart === 'undefined') {
    console.warn('Chart.js not found - charts won\'t work.');
    return;
  }
  // zoom plugin might be available on window.ChartZoom
  try {
    if (window.ChartZoom && !Chart.registry.plugins.get('zoom')) {
      Chart.register(window.ChartZoom);
      console.log('Chart Zoom plugin registered');
    }
    if (window.ChartAnnotation && !Chart.registry.plugins.get('annotation')) {
      Chart.register(window.ChartAnnotation);
      console.log('Chart Annotation plugin registered');
    }
  } catch (e) {
    console.warn('Could not register Chart plugins', e);
  }
}

// ---- Create shared charts (path, position, velocity, angularVelocity, orientation) ----
function createGlobalCharts() {
  registerChartPluginsIfNeeded();

  // PATH CHART (scatter / line) - will contain one dataset per vessel for trajectory
  const pathCtx = document.getElementById('pathChart').getContext('2d');
  const pathChart = new Chart(pathCtx, {
    type: 'line',
    data: { datasets: [] }, // datasets added per vessel
    options: {
      animation: false,
      maintainAspectRatio: false,
      responsive: true,
      interaction: { mode: 'nearest', intersect: false, axis: 'xy' },
      plugins: {
        legend: { position: 'top' },
        tooltip: {
          callbacks: {
            label: ctx => {
              const p = ctx.raw;
              if (p && typeof p.x === 'number') return `${ctx.dataset.label}: (${p.x.toFixed(2)}, ${p.y.toFixed(2)})`;
              return ctx.dataset.label;
            }
          }
        },
        zoom: {
          pan: { enabled: true, mode: 'xy', modifierKey: 'ctrl' },
          zoom: {
            wheel: { enabled: true },
            pinch: { enabled: true },
            mode: 'xy'
          }
        }
      },
      scales: {
        x: { type: 'linear', title: { display: true, text: 'X Position (m)' } },
        y: { type: 'linear', title: { display: true, text: 'Y Position (m)' } }
      }
    }
  });

  // POSITION CHART: will show x,y,z per vessel (datasets added dynamically)
  const posCtx = document.getElementById('positionChart').getContext('2d');
  const positionChart = new Chart(posCtx, {
    type: 'line',
    data: { datasets: [] },
    options: {
      animation: false,
      maintainAspectRatio: false,
      responsive: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { position: 'top' } },
      scales: { x: { title: { display: true, text: 'Sample' } }, y: { title: { display: true, text: 'Position (m)' } } }
    }
  });

  // VELOCITY CHART: speed per vessel
  const velCtx = document.getElementById('velocityChart').getContext('2d');
  const velocityChart = new Chart(velCtx, {
    type: 'line',
    data: { datasets: [] },
    options: {
      animation: false,
      maintainAspectRatio: false,
      responsive: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { position: 'top' } },
      scales: { x: { title: { display: true, text: 'Sample' } }, y: { title: { display: true, text: 'Speed (m/s)' } } }
    }
  });

  // ANGULAR VELOCITY CHART: r (yaw rate) per vessel
  const angCtx = document.getElementById('angularVelocityChart').getContext('2d');
  const angularVelocityChart = new Chart(angCtx, {
    type: 'line',
    data: { datasets: [] },
    options: {
      animation: false,
      maintainAspectRatio: false,
      responsive: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { position: 'top' } },
      scales: { x: { title: { display: true, text: 'Sample' } }, y: { title: { display: true, text: 'Angular velocity (rad/s)' } } }
    }
  });

  // ORIENTATION CHART: phi,theta,psi per vessel => we'll add 3 datasets per vessel
  const oriCtx = document.getElementById('orientationChart').getContext('2d');
  const orientationChart = new Chart(oriCtx, {
    type: 'line',
    data: { datasets: [] },
    options: {
      animation: false,
      maintainAspectRatio: false,
      responsive: true,
      interaction: { mode: 'index', intersect: false },
      plugins: { legend: { position: 'top' } },
      scales: { x: { title: { display: true, text: 'Sample' } }, y: { title: { display: true, text: 'Angle (deg)' } } }
    }
  });

  globalCharts = {
    path: pathChart,
    position: positionChart,
    velocity: velocityChart,
    angularVelocity: angularVelocityChart,
    orientation: orientationChart
  };
}

// ---- VesselManager (keeps map of VesselVisualizer instances) ----
class VesselManager {
  constructor() {
    this.vessels = new Map(); // name -> VesselVisualizer
    this.userConfig = { controlSurfaces: 0, thrusters: 0 };
    this.selectorHandlerAdded = false;
    this.configurationHandlerAdded = false;

    // Create shared charts once
    createGlobalCharts();

    // Connect to ros
    this.connectToROS();

    // Setup config panel (retain your UI)
    this.setupConfigurationPanel();
  }

  connectToROS() {
    try {
      ros = new ROSLIB.Ros({ url: DEFAULT_ROSBRIDGE_URL });
      ros.on('connection', () => {
        console.log('Connected to rosbridge.');
        document.getElementById('connectionStatus').className = 'status-indicator status-connected';
        document.getElementById('connectionText').textContent = 'Connected';
        // discover topics after a short delay
        setTimeout(() => this.discoverVessels(), 250);
      });

      ros.on('error', (err) => {
        console.error('ROS connection error', err);
        document.getElementById('connectionStatus').className = 'status-indicator status-disconnected';
        document.getElementById('connectionText').textContent = 'Disconnected';
      });

      ros.on('close', () => {
        console.warn('ROS connection closed');
        document.getElementById('connectionStatus').className = 'status-indicator status-disconnected';
        document.getElementById('connectionText').textContent = 'Disconnected';
        // remove all visualizers
        this.removeAllVisualizers();
        // attempt reconnect after a few seconds
        setTimeout(() => this.connectToROS(), 3000);
      });
    } catch (e) {
      console.error('Failed to create ROS connection', e);
    }
  }

  discoverVessels() {
    if (!ros) {
      console.warn('ROS not connected - skipping discovery');
      this.createVisualizersFromList(fallbackVesselTopics);
      return;
    }

    // Prefer getTopicsAndTypes where available
    if (typeof ros.getTopicsAndTypes === 'function') {
      ros.getTopicsAndTypes((result) => {
        if (!result || !Array.isArray(result.topics)) {
          console.warn('Unexpected topics result - using fallback', result);
          this.createVisualizersFromList(fallbackVesselTopics);
          return;
        }
        // Flatten topic names list
        const topics = result.topics.map(t => t.name || t);
        const vesselTopics = topics.filter(t =>
          t.endsWith('/state_test') ||
          t.endsWith('/vessel_state') ||
          t.endsWith('/odometry_test') ||
          t.endsWith('/odometry_sim')
        );
        if (vesselTopics.length === 0) {
          this.createVisualizersFromList(fallbackVesselTopics);
        } else {
          this.createVisualizersFromList(vesselTopics);
        }
      });
    } else if (typeof ros.getTopics === 'function') {
      ros.getTopics((result) => {
        const topics = result && result.topics ? result.topics : [];
        const vesselTopics = topics.filter(t =>
          t.endsWith('/state_test') ||
          t.endsWith('/vessel_state') ||
          t.endsWith('/odometry_test') ||
          t.endsWith('/odometry_sim')
        );
        if (vesselTopics.length === 0) {
          this.createVisualizersFromList(fallbackVesselTopics);
        } else {
          this.createVisualizersFromList(vesselTopics);
        }
      });
    } else {
      // fallback
      this.createVisualizersFromList(fallbackVesselTopics);
    }
  }

  createVisualizersFromList(topicList) {
    // topicList can contain '/name/state_test', '/name/vessel_state', '/name/odometry_test', or '/name/odometry_sim'
    const names = new Set();
    topicList.forEach(t => {
      try {
        const parts = t.split('/');
        if (parts.length >= 2 && parts[1]) names.add(parts[1]);
      } catch (e) {}
    });

    // If we didn't detect anything, fallback to known ones
    if (names.size === 0) {
      fallbackVesselTopics.forEach(t => names.add(t.split('/')[1]));
    }

    // Create visualizers for each detected vessel if not already present
    names.forEach(name => {
      if (!this.vessels.has(name)) {
        const vis = new VesselVisualizer(name, ros, this.userConfig);
        this.vessels.set(name, vis);
        vis.subscribe(); // start subscribing immediately
      }
    });

    // Update selector UI (optional)
    this.updateVesselSelector(Array.from(names));
  }

  updateVesselSelector(names) {
    const vesselSelect = document.getElementById('vesselSelect');
    if (!vesselSelect) return;
    vesselSelect.innerHTML = '<option value="">All vessels (shown)</option>';
    names.forEach(n => {
      const opt = document.createElement('option');
      opt.value = n;
      opt.textContent = n;
      vesselSelect.appendChild(opt);
    });

    if (!this.selectorHandlerAdded) {
      vesselSelect.addEventListener('change', (e) => {
        // If you want dropdown to focus on single vessel, implement here.
        // By default we display all; selecting a vessel will "highlight" it (reduce opacity of others).
        const selected = e.target.value;
        this.highlightVessel(selected);
      });
      this.selectorHandlerAdded = true;
    }
  }

  highlightVessel(name) {
    // reduce dataset opacity for all but selected
    Object.entries(globalCharts).forEach(([k, chart]) => {
      chart.data.datasets.forEach(ds => {
        if (!name || ds._vessel === name) {
          ds._hiddenByHighlight = false;
          ds.borderWidth = 2.5;
          ds.borderColor = ds._baseColor || ds.borderColor;
          ds.hidden = false;
        } else {
          ds._hiddenByHighlight = true;
          ds.borderWidth = 1;
          // dim color
          ds.borderColor = hexToRgbaString(ds._baseColor || ds.borderColor, 0.25);
        }
      });
      chart.update('none');
    });
  }

  setupConfigurationPanel() {
    const applyButton = document.getElementById('applyConfig');
    const controlSurfacesInput = document.getElementById('controlSurfacesInput');
    const thrustersInput = document.getElementById('thrustersInput');

    if (this.configurationHandlerAdded) return;
    this.configurationHandlerAdded = true;

    applyButton.addEventListener('click', () => {
      const cs = parseInt(controlSurfacesInput.value) || 0;
      const thr = parseInt(thrustersInput.value) || 0;
      this.userConfig.controlSurfaces = cs;
      this.userConfig.thrusters = thr;
      // Propagate to existing visualizers
      this.vessels.forEach(vis => vis.setConfiguration(cs, thr));
      console.log('Applied configuration to all vessels:', this.userConfig);
    });

    // dynamic calculation based on latest state length is optional
    controlSurfacesInput.addEventListener('input', () => {
      const cs = parseInt(controlSurfacesInput.value) || 0;
      if (this.latestStateLength) {
        const rem = this.latestStateLength - 12;
        thrustersInput.value = Math.max(0, rem - cs);
      }
    });

    thrustersInput.addEventListener('input', () => {
      const thr = parseInt(thrustersInput.value) || 0;
      if (this.latestStateLength) {
        const rem = this.latestStateLength - 12;
        controlSurfacesInput.value = Math.max(0, rem - thr);
      }
    });
  }

  updateLatestStateLength(length) {
    this.latestStateLength = length;
    // propagate default if UI is empty
    const controlSurfacesInput = document.getElementById('controlSurfacesInput');
    const thrustersInput = document.getElementById('thrustersInput');
    if (controlSurfacesInput && thrustersInput &&
        parseInt(controlSurfacesInput.value) === 0 && parseInt(thrustersInput.value) === 0) {
      const remaining = Math.max(0, length - 12);
      const cs = Math.floor(remaining / 2);
      const thr = remaining - cs;
      controlSurfacesInput.value = cs;
      thrustersInput.value = thr;
      this.userConfig = { controlSurfaces: cs, thrusters: thr };
      // propagate config
      this.vessels.forEach(vis => vis.setConfiguration(cs, thr));
    }
  }

  removeAllVisualizers() {
    this.vessels.forEach((vis, name) => {
      try { vis.unsubscribe(); } catch (e) {}
    });
    this.vessels.clear();
    // clear global charts
    Object.values(globalCharts).forEach(chart => {
      chart.data.datasets = [];
      chart.update('none');
    });
  }
}

// ---- VesselVisualizer: one instance per vessel, but shares global charts ----
class VesselVisualizer {
  constructor(vesselName, rosInstance, userConfig) {
    this.vesselName = vesselName;
    this.ros = rosInstance;
    this.config = Object.assign({}, userConfig || { controlSurfaces: 0, thrusters: 0 });
    this.isCollectingData = false;
    this.sampleIndex = 0;
    this.odomTopic = null;
    this.stateTopic = null;
    this.datasetIndices = {}; // mapping: chartKey -> dataset index(s)
    this.setupSharedDatasets(); // add datasets to globalCharts for this vessel
    this.dataMonitorInterval = null;
  }

  setConfiguration(controlSurfaces, thrusters) {
    this.config.controlSurfaces = controlSurfaces || 0;
    this.config.thrusters = thrusters || 0;
  }

  setupSharedDatasets() {
    // Add dataset(s) to the shared charts and store their indices
    const name = this.vesselName;
    const baseColor = vesselColorMap[name] || randomColorForName(name);

    // PATH dataset (trajectory) - one dataset (objects {x,y})
    const pathDataset = {
      label: name,
      data: [],
      borderColor: baseColor,
      backgroundColor: baseColor,
      fill: false,
      tension: 0.2,
      showLine: true,
      pointRadius: 0,
      _vessel: name,
      _baseColor: baseColor
    };
    globalCharts.path.data.datasets.push(pathDataset);
    this.datasetIndices.path = globalCharts.path.data.datasets.length - 1;

    // POSITION datasets: x,y,z -> three datasets
    const posLabels = ['x', 'y', 'z'];
    this.datasetIndices.position = [];
    posLabels.forEach((lbl, i) => {
      const ds = {
        label: `${name} ${lbl}`,
        data: [],
        borderColor: shadeColor(baseColor, -10 * (i + 1)),
        backgroundColor: hexToRgbaString(shadeColor(baseColor, -10 * (i + 1)), 0.12),
        fill: false,
        tension: 0.2,
        pointRadius: 0,
        _vessel: name,
        _baseColor: baseColor
      };
      globalCharts.position.data.datasets.push(ds);
      this.datasetIndices.position.push(globalCharts.position.data.datasets.length - 1);
    });

    // VELOCITY (speed) dataset
    const velDs = {
      label: `${name} speed`,
      data: [],
      borderColor: baseColor,
      backgroundColor: hexToRgbaString(baseColor, 0.12),
      fill: false,
      tension: 0.2,
      pointRadius: 0,
      _vessel: name,
      _baseColor: baseColor
    };
    globalCharts.velocity.data.datasets.push(velDs);
    this.datasetIndices.velocity = globalCharts.velocity.data.datasets.length - 1;

    // ANGULAR (r) dataset
    const angDs = {
      label: `${name} yaw_rate (r)`,
      data: [],
      borderColor: baseColor,
      backgroundColor: hexToRgbaString(baseColor, 0.08),
      fill: false,
      tension: 0.2,
      pointRadius: 0,
      _vessel: name,
      _baseColor: baseColor
    };
    globalCharts.angularVelocity.data.datasets.push(angDs);
    this.datasetIndices.angularVelocity = globalCharts.angularVelocity.data.datasets.length - 1;

    // ORIENTATION datasets: phi, theta, psi
    const oriLabels = ['phi', 'theta', 'psi'];
    this.datasetIndices.orientation = [];
    oriLabels.forEach((lbl, i) => {
      const ds = {
        label: `${name} ${lbl}`,
        data: [],
        borderColor: shadeColor(baseColor, 15 * (i - 1)),
        backgroundColor: hexToRgbaString(shadeColor(baseColor, 15 * (i - 1)), 0.08),
        fill: false,
        tension: 0.2,
        pointRadius: 0,
        _vessel: name,
        _baseColor: baseColor
      };
      globalCharts.orientation.data.datasets.push(ds);
      this.datasetIndices.orientation.push(globalCharts.orientation.data.datasets.length - 1);
    });

    // Update charts after dataset additions
    Object.values(globalCharts).forEach(ch => ch.update('none'));
  }

  subscribe() {
    if (!this.ros) {
      console.warn('No ros instance available for', this.vesselName);
      return;
    }

    // Subscribe to state_test (preferred) or vessel_state; the message is std_msgs/Float64MultiArray (per your info)
    const stateTopicName = `/${this.vesselName}/state_test`;
    try {
      this.stateTopic = new ROSLIB.Topic({ ros: this.ros, name: stateTopicName, messageType: 'std_msgs/Float64MultiArray' });
      this.stateTopic.subscribe(msg => this.handleVesselState(msg));
      console.log(`Subscribed to ${stateTopicName}`);
    } catch (e) {
      console.error('Failed to subscribe to', stateTopicName, e);
    }

    // Start monitoring data (staleness)
    this.startDataCollection();
  }

  unsubscribe() {
    // unsubscribe from topics and remove datasets
    try {
      if (this.stateTopic) {
        this.stateTopic.unsubscribe();
        this.stateTopic = null;
      }
    } catch (e) {}

    // Remove the datasets we added from each global chart
    // Because datasets indices may shift when others removed, safest is to filter by _vessel property
    Object.values(globalCharts).forEach(chart => {
      chart.data.datasets = chart.data.datasets.filter(ds => ds._vessel !== this.vesselName);
      chart.update('none');
    });

    // stop collection
    this.stopDataCollection();
  }

  startDataCollection() {
    if (this.isCollectingData) return;
    this.isCollectingData = true;
    this.sampleIndex = 0;
    lastMessageTime = Date.now();

    // data monitor to mark stale connection per-visualizer
    if (!this.dataMonitorInterval) {
      this.dataMonitorInterval = setInterval(() => {
        const dt = Date.now() - lastMessageTime;
        if (dt > 5000) {
          // show overlays / annotations if needed
          this.addDisconnectOverlayToContainers();
        }
      }, 1000);
    }
  }

  stopDataCollection() {
    this.isCollectingData = false;
    if (this.dataMonitorInterval) {
      clearInterval(this.dataMonitorInterval);
      this.dataMonitorInterval = null;
    }
  }

  addDisconnectOverlayToContainers() {
    // create overlay for path & other charts' containers (if not exists)
    ['pathChart', 'positionChart', 'velocityChart', 'angularVelocityChart', 'orientationChart'].forEach(id => {
      const canvas = document.getElementById(id);
      if (!canvas) return;
      const container = canvas.parentNode;
      if (!container.querySelector('.chart-disconnected')) {
        const overlay = document.createElement('div');
        overlay.className = 'chart-disconnected';
        overlay.innerHTML = '<div class="disconnect-message">Simulator Disconnected</div>';
        container.appendChild(overlay);
        setTimeout(() => overlay.classList.add('active'), 50);
      }
    });
  }

  handleVesselState(message) {
    try {
      // update last message time (global)
      lastMessageTime = Date.now();

      // message.data expected array
      if (!message || !Array.isArray(message.data)) {
        console.warn('Received non-array vessel_state:', message);
        return;
      }

      // Use your given ordering:
      // [u, v, w, p, r, x, y, z, phi, theta, psi, rudder_command]
      const d = message.data;
      // guard missing fields
      const u = Number(d[0] || 0);
      const v = Number(d[1] || 0);
      const w = Number(d[2] || 0);
      const p = Number(d[3] || 0);
      const q = Number(d[4] || 0);
      const r = Number(d[5] || 0);
      const x = Number(d[6] || 0);
      const y = Number(d[7] || 0);
      const z = Number(d[8] || 0);
      const phi = Number(d[9] || 0);
      const theta = Number(d[10] || 0);
      const psi = Number(d[11] || 0); 
      const rudder = Number(d[1] || 0);

      // update sample index
      this.sampleIndex++;

      // PATH: append {x,y}
      const pathDataset = globalCharts.path.data.datasets[this.datasetIndices.path];
      if (pathDataset) {
        pathDataset.data.push({ x, y });
        if (pathDataset.data.length > DEFAULT_MAX_POINTS) pathDataset.data.shift();
      }

      // POSITION: push x,y,z to respective datasets
      ['x', 'y', 'z'].forEach((lab, i) => {
        const idx = this.datasetIndices.position[i];
        const ds = globalCharts.position.data.datasets[idx];
        if (ds) {
          const val = lab === 'x' ? x : lab === 'y' ? y : z;
          ds.data.push({ x: this.sampleIndex, y: val });
          if (ds.data.length > DEFAULT_MAX_POINTS) ds.data.shift();
        }
      });

      // VELOCITY: push scalar speed
      {
        const idx = this.datasetIndices.velocity;
        const ds = globalCharts.velocity.data.datasets[idx];
        if (ds) {
          const speed = Math.sqrt(u * u + v * v + w * w);
          ds.data.push({ x: this.sampleIndex, y: speed });
          if (ds.data.length > DEFAULT_MAX_POINTS) ds.data.shift();
        }
      }

      // ANGULAR VELOCITY: push r (yaw rate)
      {
        const idx = this.datasetIndices.angularVelocity;
        const ds = globalCharts.angularVelocity.data.datasets[idx];
        if (ds) {
          ds.data.push({ x: this.sampleIndex, y: r });
          if (ds.data.length > DEFAULT_MAX_POINTS) ds.data.shift();
        }
      }

      // ORIENTATION: phi,theta,psi
      [phi, theta, psi].forEach((val, i) => {
        const idx = this.datasetIndices.orientation[i];
        const ds = globalCharts.orientation.data.datasets[idx];
        if (ds) {
          ds.data.push({ x: this.sampleIndex, y: val });
          if (ds.data.length > DEFAULT_MAX_POINTS) ds.data.shift();
        }
      });

      // Update the charts efficiently (no animation)
      Object.values(globalCharts).forEach(c => c.update('none'));

      // Update control surfaces / thrusters if provided via config mapping
      // Your config says control surfaces begin after 12 states; but your state vector here is only 12 long,
      // so control surfaces / thrusters won't be present unless your state is longer elsewhere.
      // We'll check for any extras beyond index 11 and split them based on config
      const extras = d.length > 12 ? d.slice(12) : [];
      if (this.config.controlSurfaces > 0 || this.config.thrusters > 0) {
        // determine how many controls are present
        const totalControlsAvailable = extras.length;
        // allocate controls and thrusters based on config but ensure not to read out of bounds
        const csCount = Math.min(this.config.controlSurfaces, totalControlsAvailable);
        const thrCount = Math.min(this.config.thrusters, Math.max(0, totalControlsAvailable - csCount));
        const csVals = extras.slice(0, csCount);
        const thrVals = extras.slice(csCount, csCount + thrCount);
        if (csCount > 0) this.updateControlSurfaces(csVals);
        if (thrCount > 0) this.updateThrusters(thrVals);
      } else {
        // If extras but config is zero, attempt to estimate? We won't guess; we keep control charts empty.
      }

      // Update vessel info panel for the selected vessel area - same container used by your HTML
      this.updateVesselInfo({
        pose: { pose: { position: { x, y, z }, orientation: { x: 0, y: 0, z: 0, w: 1 } } },
        twist: { twist: { linear: { x: u, y: v, z: w }, angular: { x: p, y: 0, z: r } } },
        rawState: d
      });

      // Inform manager about state length for config auto-calculation
      if (typeof vesselManager !== 'undefined') {
        vesselManager.updateLatestStateLength(d.length);
      }

    } catch (err) {
      console.error('Error handling vessel_state for', this.vesselName, err);
    }
  }

  updateControlSurfaces(controlStates) {
    // Similar to your earlier updateControlSurfaces but per vessel we show latest values in the controlSurfacesChart container
    const container = document.getElementById('controlSurfacesChart');
    if (!container) return;

    // Build a small per-vessel display (chart) or reuse a combined chart — here we use combined per-vessel rows
    // We'll show the values as text if too many to create separate datasets
    const boxId = `cs-box-${this.vesselName}`;
    let box = container.querySelector(`#${boxId}`);
    if (!box) {
      box = document.createElement('div');
      box.id = boxId;
      box.style.padding = '8px';
      box.style.borderBottom = '1px solid #eee';
      box.style.fontSize = '12px';
      container.appendChild(box);
    }
    if (!controlStates || controlStates.length === 0) {
      box.innerHTML = `<strong>${this.vesselName}</strong>: No control surfaces`;
      return;
    }
    // show as CSV
    box.innerHTML = `<strong>${this.vesselName}</strong> CS: ${controlStates.map(v => Number(v).toFixed(3)).join(', ')}`;
  }

  updateThrusters(thrusterStates) {
    const container = document.getElementById('thrustersChart');
    if (!container) return;
    const boxId = `thr-box-${this.vesselName}`;
    let box = container.querySelector(`#${boxId}`);
    if (!box) {
      box = document.createElement('div');
      box.id = boxId;
      box.style.padding = '8px';
      box.style.borderBottom = '1px solid #eee';
      box.style.fontSize = '12px';
      container.appendChild(box);
    }
    if (!thrusterStates || thrusterStates.length === 0) {
      box.innerHTML = `<strong>${this.vesselName}</strong>: No thrusters`;
      return;
    }
    box.innerHTML = `<strong>${this.vesselName}</strong> Thrusters: ${thrusterStates.map(v => Number(v).toFixed(3)).join(', ')}`;
  }

  updateVesselInfo(message) {
    // update the info card in your UI
    try {
      const container = document.getElementById('vesselInfo');
      if (!container) return;

      // Minimal safe extraction from the "message" wrapper used above
      const pos = (message.pose && message.pose.pose && message.pose.pose.position) ? message.pose.pose.position : { x: 0, y: 0, z: 0 };
      const lin = (message.twist && message.twist.twist && message.twist.twist.linear) ? message.twist.twist.linear : { x: 0, y: 0, z: 0 };
      const ang = (message.twist && message.twist.twist && message.twist.twist.angular) ? message.twist.twist.angular : { x: 0, y: 0, z: 0 };

      // Update a compact card - only one card used in your HTML so we'll show last-updated vessel
      container.innerHTML = `
        <div class="info-item">
          <div class="info-label">Vessel Name</div>
          <div class="info-value">${this.vesselName}</div>
        </div>
        <div class="info-item">
          <div class="info-label">Position</div>
          <div class="info-value">X: ${Number(pos.x).toFixed(2)} m</div>
          <div class="info-value">Y: ${Number(pos.y).toFixed(2)} m</div>
          <div class="info-value">Z: ${Number(pos.z).toFixed(2)} m</div>
        </div>
        <div class="info-item">
          <div class="info-label">Velocity</div>
          <div class="info-value">U: ${Number(lin.x).toFixed(2)} m/s</div>
          <div class="info-value">V: ${Number(lin.y).toFixed(2)} m/s</div>
          <div class="info-value">W: ${Number(lin.z).toFixed(2)} m/s</div>
        </div>
        <div class="info-item">
          <div class="info-label">Angular Velocity</div>
          <div class="info-value">P: ${Number(ang.x).toFixed(2)} rad/s</div>
          <div class="info-value">R: ${Number(ang.z).toFixed(2)} rad/s</div>
        </div>
        <div class="info-item">
          <div class="info-label">Last Update</div>
          <div class="info-value">${new Date().toLocaleTimeString()}</div>
        </div>
      `;
    } catch (e) {
      console.error('Error updating vessel info UI', e);
    }
  }
}

// ---- Utility helpers ----
function hexToRgbaString(hex, alpha) {
  if (!hex) return `rgba(128,128,128,${alpha})`;
  // handle rgb strings passed in
  if (hex.startsWith('rgba')) return hex.replace(/rgba\(([^)]+)\)/, (m, inner) => {
    const parts = inner.split(',').map(s => s.trim());
    parts[3] = alpha;
    return `rgba(${parts.join(',')})`;
  });
  // remove '#'
  if (hex.indexOf('#') === 0) {
    const bigint = parseInt(hex.slice(1), 16);
    const r = (bigint >> 16) & 255;
    const g = (bigint >> 8) & 255;
    const b = bigint & 255;
    return `rgba(${r}, ${g}, ${b}, ${alpha})`;
  }
  return `rgba(128,128,128,${alpha})`;
}

function shadeColor(hex, percent) {
  // accepts hex like '#rrggbb', returns hex
  if (!hex || hex[0] !== '#') return hex;
  const num = parseInt(hex.slice(1), 16);
  const r = Math.max(0, Math.min(255, ((num >> 16) & 0xFF) + percent));
  const g = Math.max(0, Math.min(255, ((num >> 8) & 0xFF) + percent));
  const b = Math.max(0, Math.min(255, (num & 0xFF) + percent));
  return `#${((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1)}`;
}

function randomColorForName(name) {
  // deterministic-ish color generation
  let hash = 0;
  for (let i = 0; i < name.length; i++) hash = name.charCodeAt(i) + ((hash << 5) - hash);
  const c = (hash & 0x00FFFFFF).toString(16).toUpperCase();
  return `#${'00000'.slice(0, 6 - c.length) + c}`;
}

// ---- Instantiate manager globally so VesselVisualizer can refer to it ----
const vesselManager = new VesselManager();

// Expose helper for debugging
window._vesselManager = vesselManager;
window._globalCharts = globalCharts;

console.log('main.js loaded - multi-vessel manager created.');
