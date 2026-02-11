// Initialize the map
const map = L.map('map').setView([12.99300425860631, 80.23913114094384], 15);

// Add multiple tile layers
const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
});

const satelliteLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
    attribution: '© Esri'
});

// Default to satellite view
satelliteLayer.addTo(map);

// Add layer control
const baseMaps = {
    "Satellite": satelliteLayer,
    "Street": osmLayer
};

L.control.layers(baseMaps).addTo(map);

// Initialize drawing layers
const datum = {
    marker: null
};

const waypoints = {
    points: [],
    layer: L.layerGroup().addTo(map),
    polyline: L.polyline([], { color: 'blue', weight: 3 }).addTo(map)
};

const geofence = {
    points: [],
    layer: L.layerGroup().addTo(map),
    polygon: L.polygon([], { color: 'red', fillOpacity: 0.2 }).addTo(map)
};

const obstacles = {
    points: [],
    layer: L.layerGroup().addTo(map)
};

// Current mode
let currentMode = 'datum';

// Update mode when selector changes
document.getElementById('drawingMode').addEventListener('change', (e) => {
    currentMode = e.target.value;
});

// Create draggable marker with delete and edit functionality
function createDraggableMarker(lat, lng, type, index) {
    const marker = L.marker([lat, lng], { draggable: true });
    
    // Handle drag events
    marker.on('drag', () => {
        updatePolylines();
    });

    // Handle right-click (delete)
    marker.on('contextmenu', (e) => {
        e.target.remove();
        removePoint(type, index);
    });

    // Handle double-click (edit coordinates)
    marker.on('dblclick', (e) => {
        const pos = e.target.getLatLng();
        const popupContent = `
            <div class="coordinate-editor">
                <input type="number" id="edit-lat" value="${pos.lat}" step="any">
                <input type="number" id="edit-lng" value="${pos.lng}" step="any">
                <button onclick="updateMarkerPosition(${index}, '${type}')">Update</button>
            </div>
        `;
        e.target.bindPopup(popupContent).openPopup();
    });

    return marker;
}

// Update marker position from popup
window.updateMarkerPosition = function(index, type) {
    const lat = parseFloat(document.getElementById('edit-lat').value);
    const lng = parseFloat(document.getElementById('edit-lng').value);
    
    if (isNaN(lat) || isNaN(lng)) {
        alert('Please enter valid coordinates');
        return;
    }

    const collection = type === 'waypoint' ? waypoints :
                      type === 'geofence' ? geofence :
                      type === 'obstacle' ? obstacles : null;
    
    if (collection && collection.points[index]) {
        collection.points[index] = [lat, lng];
        collection.layer.clearLayers();
        redrawPoints(type);
    }
};

// Remove point from collection
function removePoint(type, index) {
    const collection = type === 'waypoint' ? waypoints :
                      type === 'geofence' ? geofence :
                      type === 'obstacle' ? obstacles : null;
    
    if (collection) {
        collection.points.splice(index, 1);
        collection.layer.clearLayers();
        redrawPoints(type);
    }
}

// Update polylines after drag
function updatePolylines() {
    // Update waypoints polyline
    const waypointLatLngs = [];
    waypoints.layer.eachLayer((layer) => {
        waypointLatLngs.push(layer.getLatLng());
    });
    waypoints.polyline.setLatLngs(waypointLatLngs);
    waypoints.points = waypointLatLngs.map(ll => [ll.lat, ll.lng]);

    // Update geofence polygon
    const geofenceLatLngs = [];
    geofence.layer.eachLayer((layer) => {
        geofenceLatLngs.push(layer.getLatLng());
    });
    if (geofenceLatLngs.length > 2) {
        geofence.polygon.setLatLngs([...geofenceLatLngs, geofenceLatLngs[0]]);
    }
    geofence.points = geofenceLatLngs.map(ll => [ll.lat, ll.lng]);
}

// Redraw points for a specific type
function redrawPoints(type) {
    if (type === 'waypoint') {
        waypoints.points.forEach((point, i) => {
            const marker = createDraggableMarker(point[0], point[1], 'waypoint', i);
            marker.bindPopup(`Waypoint ${i + 1}`);
            marker.addTo(waypoints.layer);
        });
        waypoints.polyline.setLatLngs(waypoints.points);
    } else if (type === 'geofence') {
        geofence.points.forEach((point, i) => {
            const marker = createDraggableMarker(point[0], point[1], 'geofence', i);
            marker.bindPopup(`Geofence ${i + 1}`);
            marker.addTo(geofence.layer);
        });
        if (geofence.points.length > 2) {
            geofence.polygon.setLatLngs([...geofence.points, geofence.points[0]]);
        }
    } else if (type === 'obstacle') {
        obstacles.points.forEach((point, i) => {
            const marker = createDraggableMarker(point[0], point[1], 'obstacle', i);
            marker.bindPopup(`Obstacle ${i + 1}`);
            marker.addTo(obstacles.layer);
        });
    }
}

// Handle map clicks
map.on('click', (e) => {
    const lat = e.latlng.lat;
    const lng = e.latlng.lng;
    
    document.getElementById('currentCoords').textContent = `Lat: ${lat.toFixed(6)}, Lng: ${lng.toFixed(6)}`;

    switch(currentMode) {
        case 'datum':
            if (datum.marker) {
                datum.marker.remove();
            }
            datum.marker = L.marker([lat, lng], {
                icon: L.divIcon({
                    className: 'datum-marker',
                    html: '<i class="fas fa-crosshairs" style="color: #3498db; font-size: 24px;"></i>'
                })
            }).addTo(map);
            document.getElementById('refLat').value = lat;
            document.getElementById('refLon').value = lng;
            break;
        case 'waypoints':
            const waypointMarker = createDraggableMarker(lat, lng, 'waypoint', waypoints.points.length);
            waypointMarker.bindPopup(`Waypoint ${waypoints.points.length + 1}`).openPopup();
            waypointMarker.addTo(waypoints.layer);
            waypoints.points.push([lat, lng]);
            waypoints.polyline.setLatLngs(waypoints.points);
            break;
        case 'geofence':
            const geofenceMarker = createDraggableMarker(lat, lng, 'geofence', geofence.points.length);
            geofenceMarker.bindPopup(`Geofence ${geofence.points.length + 1}`).openPopup();
            geofenceMarker.addTo(geofence.layer);
            geofence.points.push([lat, lng]);
            if (geofence.points.length > 2) {
                geofence.polygon.setLatLngs([...geofence.points, geofence.points[0]]);
            }
            break;
        case 'obstacles':
            const obstacleMarker = createDraggableMarker(lat, lng, 'obstacle', obstacles.points.length);
            obstacleMarker.bindPopup(`Obstacle ${obstacles.points.length + 1}`).openPopup();
            obstacleMarker.addTo(obstacles.layer);
            obstacles.points.push([lat, lng]);
            break;
    }
});

// Clear current layer
document.getElementById('clearBtn').addEventListener('click', () => {
    switch(currentMode) {
        case 'datum':
            if (datum.marker) {
                datum.marker.remove();
                datum.marker = null;
            }
            document.getElementById('refLat').value = '';
            document.getElementById('refLon').value = '';
            break;
        case 'waypoints':
            waypoints.layer.clearLayers();
            waypoints.polyline.setLatLngs([]);
            waypoints.points = [];
            break;
        case 'geofence':
            geofence.layer.clearLayers();
            geofence.polygon.setLatLngs([]);
            geofence.points = [];
            break;
        case 'obstacles':
            obstacles.layer.clearLayers();
            obstacles.points = [];
            break;
    }
});

// Clear all layers
document.getElementById('clearAllBtn').addEventListener('click', () => {
    if (datum.marker) {
        datum.marker.remove();
        datum.marker = null;
    }
    document.getElementById('refLat').value = '';
    document.getElementById('refLon').value = '';
    
    waypoints.layer.clearLayers();
    waypoints.polyline.setLatLngs([]);
    waypoints.points = [];
    
    geofence.layer.clearLayers();
    geofence.polygon.setLatLngs([]);
    geofence.points = [];
    
    obstacles.layer.clearLayers();
    obstacles.points = [];
});

// Convert GPS coordinates to NED
function gpsToNed(lat, lon, alt, refLat, refLon, refAlt) {
    const R = 6371000; // Earth's radius in meters
    const dLat = (lat - refLat) * Math.PI / 180;
    const dLon = (lon - refLon) * Math.PI / 180;
    
    const north = R * dLat;
    const east = R * Math.cos(refLat * Math.PI / 180) * dLon;
    const down = refAlt - alt;
    
    return [north, east, down];
}

// Generate configuration files
document.getElementById('generateBtn').addEventListener('click', () => {
    const refLat = parseFloat(document.getElementById('refLat').value);
    const refLon = parseFloat(document.getElementById('refLon').value);
    const refAlt = parseFloat(document.getElementById('refAlt').value);
    
    if (isNaN(refLat) || isNaN(refLon) || isNaN(refAlt)) {
        alert('Please set a valid GPS reference point (datum)');
        return;
    }

    // Generate simulation_input.yml content
    let simInputContent = `#Simulation Specific Inputs
sim_time: 100
time_step: 0.1
density: 1000
gravity: 9.80665
world_size: [100, 100, Inf]
gps_datum: [${refLat}, ${refLon}, ${refAlt}]

`;

    if (geofence.points.length > 0) {
        simInputContent += 'geofence:\n';
        geofence.points.forEach(point => {
            simInputContent += `  - [${point[0].toFixed(6)}, ${point[1].toFixed(6)}]\n`;
        });
        // Close the geofence by adding the first point again
        simInputContent += `  - [${geofence.points[0][0].toFixed(6)}, ${geofence.points[0][1].toFixed(6)}]\n\n`;
    }

    if (obstacles.points.length > 0) {
        simInputContent += 'static_obstacle:\n';
        obstacles.points.forEach(point => {
            simInputContent += `  - [${point[0].toFixed(6)}, ${point[1].toFixed(6)}]\n`;
        });
        simInputContent += '\n';
    }

    // Generate guidance.yml content
    let guidanceContent = 'waypoints:\n';
    waypoints.points.forEach(point => {
        const ned = gpsToNed(point[0], point[1], refAlt, refLat, refLon, refAlt);
        guidanceContent += `- [${ned[0].toFixed(2)}, ${ned[1].toFixed(2)}, ${ned[2].toFixed(2)}]\n`;
    });
    guidanceContent += 'waypoints_type: \'NED\'\n';

    // Create download links
    const simInputBlob = new Blob([simInputContent], { type: 'text/yaml' });
    const guidanceBlob = new Blob([guidanceContent], { type: 'text/yaml' });
    
    const simInputUrl = URL.createObjectURL(simInputBlob);
    const guidanceUrl = URL.createObjectURL(guidanceBlob);
    
    // Create and trigger downloads
    const simInputLink = document.createElement('a');
    simInputLink.href = simInputUrl;
    simInputLink.download = 'simulation_input.yml';
    document.body.appendChild(simInputLink);
    simInputLink.click();
    document.body.removeChild(simInputLink);
    
    const guidanceLink = document.createElement('a');
    guidanceLink.href = guidanceUrl;
    guidanceLink.download = 'guidance.yml';
    document.body.appendChild(guidanceLink);
    guidanceLink.click();
    document.body.removeChild(guidanceLink);
}); 