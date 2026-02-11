// Vessel management class
// Global variable for tracking last message time
let lastMessageTime = Date.now();

class VesselManager {
  constructor() {
    this.vessels = new Map();
    this.configurationHandlerAdded = false;
    this.defaultVessels = ['evader_03', 'sookshma2_01', 'sookshma3_02', 'sookshma_00'];
    this.colorPalette = ['#FF6384', '#36A2EB', '#4BC0C0', '#FF9F40', '#9966FF', '#8BC34A', '#F06292', '#26A69A'];
    this.vesselColors = new Map();
    this.globalPathChart = null;
    this.globalChartDirty = false;
    this.globalPathLimit = 1500;
    this.userConfig = {
      controlSurfaces: 0,
      thrusters: 0
    };
    this.dashboardTemplate = document.getElementById('vesselDashboardTemplate');
    this.dashboardHost = document.getElementById('dashboardHost');
    this.emptyState = document.getElementById('emptyState');
    this.vesselSelect = document.getElementById('vesselSelect');
    this.activeVesselList = document.getElementById('activeVesselsList');
    this.setupSelectorControls();
    this.connectToROS();
    this.setupConfigurationPanel();
    this.initGlobalPathChart();
    this.syncActiveList();
  }

  setupSelectorControls() {
    const addButton = document.getElementById('addVesselBtn');
    const clearButton = document.getElementById('clearVesselsBtn');
    if (addButton) {
      addButton.addEventListener('click', () => this.handleAddVessel());
    }
    if (clearButton) {
      clearButton.addEventListener('click', () => {
        Array.from(this.vessels.keys()).forEach(name => this.toggleVessel(name, false));
        this.syncActiveList();
      });
    }
  }

  connectToROS() {
    this.ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });
  
    this.setupRosConnection();
  }

  setupRosConnection() {
    this.ros.on('connection', () => {
    console.log('Connected to rosbridge.');
      document.getElementById('connectionStatus').className = 'status-indicator status-connected';
      document.getElementById('connectionText').textContent = 'Connected';
      
      // Begin vessel discovery after connection
      setTimeout(() => this.discoverVessels(), 1000);
  });

    this.ros.on('error', (error) => {
    console.error('Error connecting to rosbridge: ', error);
      document.getElementById('connectionStatus').className = 'status-indicator status-disconnected';
      document.getElementById('connectionText').textContent = 'Disconnected';
      
      // Clear active vessel when disconnected
      this.handleDisconnection();
    });
    
    this.ros.on('close', () => {
      console.log('Connection to rosbridge closed.');
      document.getElementById('connectionStatus').className = 'status-indicator status-disconnected';
      document.getElementById('connectionText').textContent = 'Disconnected';
      
      // Clear active vessel when disconnected
      this.handleDisconnection();
      
      // Attempt to reconnect after a delay
      setTimeout(() => this.connectToROS(), 3000);
    });
    
    // Add a heartbeat monitor to detect stale connections
    this.setupHeartbeatMonitor();
  }

  // Setup a heartbeat to monitor connection health
  setupHeartbeatMonitor() {
    // Clear any existing heartbeat
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
    }
    
    // Setup connection timeout detection
    lastMessageTime = Date.now();
    
    // We won't subscribe to /rosout - instead we'll use the vessel-specific topics
    // Connection health will be monitored through updates to lastMessageTime
    // which will be updated in the handleOdometry and handleVesselState methods
    
    // Check connection status every 5 seconds
    this.heartbeatInterval = setInterval(() => {
      const currentTime = Date.now();
      const timeSinceLastMessage = currentTime - lastMessageTime;
      
      // If no messages for 10 seconds, consider disconnected
      if (timeSinceLastMessage > 10000) {
        console.warn('No messages received for 10 seconds, connection may be stale');
        
        // Update UI to show potentially disconnected state
        document.getElementById('connectionStatus').className = 'status-indicator status-disconnected';
        document.getElementById('connectionText').textContent = 'Connection Stale';
        
        // Clear active vessel data
        this.handleDisconnection();
        
        // Try to reconnect
        this.ros.close();
      }
    }, 5000);
  }
  
  // Handle disconnection by cleaning up vessel data
  handleDisconnection() {
    // Stop data collection for active vessels
    this.vessels.forEach(vessel => vessel.stopDataCollection());
  }

  async discoverVessels() {
    try {
      console.log('Starting vessel discovery...');
      
      // Try the getTopicsAndTypes method first
      if (typeof this.ros.getTopicsAndTypes === 'function') {
        this.ros.getTopicsAndTypes(this.processTopics.bind(this));
      } 
      // Fall back to getTopics if getTopicsAndTypes isn't available
      else if (typeof this.ros.getTopics === 'function') {
        this.ros.getTopics(this.processTopicsLegacy.bind(this));
      }
      // Manual fallback for testing
      else {
        console.warn('No topic discovery methods available, using fallback');
        this.manualVesselDiscovery();
      }
    } catch (error) {
      console.error('Error discovering vessels:', error);
      this.manualVesselDiscovery();
    }
  }
  
  processTopics(result) {
    if (!result || !result.topics || !Array.isArray(result.topics)) {
      console.error('Invalid topic result format:', result);
      this.manualVesselDiscovery();
      return;
    }
    
    const topicsAndTypes = result.topics;
    
    // Find vessel topics
    const vesselTopics = topicsAndTypes
      .filter(topic => 
        topic.name.endsWith('/odometry_sim') || 
        topic.name.endsWith('/vessel_state')
      )
      .map(topic => topic.name);

    console.log('Found vessel topics:', vesselTopics);
    this.updateVesselSelector(vesselTopics);
  }
  
  processTopicsLegacy(result) {
    if (!result || !result.topics || !Array.isArray(result.topics)) {
      console.error('Invalid topic result format:', result);
      this.manualVesselDiscovery();
      return;
    }
    
    const topics = result.topics;
    
    // Find vessel topics
    const vesselTopics = topics.filter(topic => 
      topic.endsWith('/odometry_sim') || 
      topic.endsWith('/vessel_state')
    );

    console.log('Found vessel topics (legacy):', vesselTopics);
    this.updateVesselSelector(vesselTopics);
  }
  
  manualVesselDiscovery() {
    // Hard-coded fallback for testing
    console.log('Using manual vessel discovery');
    const vesselTopics = [
      '/evader_03/odometry_sim',
      '/evader_03/vessel_state',
      '/sookshma2_01/odometry_sim',
      '/sookshma2_01/vessel_state',
      '/sookshma3_02/odometry_sim',
      '/sookshma3_02/vessel_state',
      '/sookshma_00/odometry_sim',
      '/sookshma_00/vessel_state'
    ];
    this.updateVesselSelector(vesselTopics);
  }
  
  updateVesselSelector(vesselTopics) {
    const vesselNames = new Set();
    vesselTopics.forEach(topic => {
      const parts = topic.split('/');
      if (parts.length >= 2) {
        vesselNames.add(parts[1]);
      }
    });

    console.log('Found vessels:', Array.from(vesselNames));

    if (!this.vesselSelect) return;
    this.vesselSelect.innerHTML = '<option value="">Select Vessel</option>';

    if (vesselNames.size === 0) {
      const option = document.createElement('option');
      option.value = '';
      option.textContent = 'No vessels found';
      option.disabled = true;
      this.vesselSelect.appendChild(option);
    } else {
      vesselNames.forEach(name => {
        const option = document.createElement('option');
        option.value = name;
        option.textContent = name;
        this.vesselSelect.appendChild(option);
      });
    }

    // Auto-enable defaults the first time we discover vessels
    if (this.vessels.size === 0) {
      this.defaultVessels.forEach(name => {
        if (vesselNames.has(name)) {
          this.toggleVessel(name, true);
        }
      });
    }

    this.syncActiveList();
  }

  setupConfigurationPanel() {
    const applyButton = document.getElementById('applyConfig');
    const controlSurfacesInput = document.getElementById('controlSurfacesInput');
    const thrustersInput = document.getElementById('thrustersInput');

    if (!this.configurationHandlerAdded) {
      applyButton.addEventListener('click', () => {
        const controlSurfaces = parseInt(controlSurfacesInput.value) || 0;
        const thrusters = parseInt(thrustersInput.value) || 0;
        
        this.updateConfiguration(controlSurfaces, thrusters);
      });
      
      // Auto-calculate the other value when one input changes
      controlSurfacesInput.addEventListener('input', () => {
        if (this.latestStateLength && controlSurfacesInput.value) {
          const controlSurfaces = parseInt(controlSurfacesInput.value) || 0;
          const remainingStates = this.latestStateLength - 12; // 12 for position and orientation
          const calculatedThrusters = Math.max(0, remainingStates - controlSurfaces);
          thrustersInput.value = calculatedThrusters;
        }
      });
      
      thrustersInput.addEventListener('input', () => {
        if (this.latestStateLength && thrustersInput.value) {
          const thrusters = parseInt(thrustersInput.value) || 0;
          const remainingStates = this.latestStateLength - 12; // 12 for position and orientation
          const calculatedControlSurfaces = Math.max(0, remainingStates - thrusters);
          controlSurfacesInput.value = calculatedControlSurfaces;
        }
      });
      
      this.configurationHandlerAdded = true;
    }
  }
  
  updateConfiguration(controlSurfaces, thrusters) {
    console.log(`Updating configuration: ${controlSurfaces} control surfaces, ${thrusters} thrusters`);
    this.userConfig = {
      controlSurfaces,
      thrusters
    };
    
    // Update all active vessels with the new configuration
    this.vessels.forEach(vessel => vessel.setConfiguration(controlSurfaces, thrusters));
  }
  
  // When vessel state is received, store its length for auto-calculation
  updateLatestStateLength(length) {
    this.latestStateLength = length;
    
    // Update calculations if needed
    if (this.latestStateLength) {
      const remainingStates = this.latestStateLength - 12;
      const controlSurfacesInput = document.getElementById('controlSurfacesInput');
      const thrustersInput = document.getElementById('thrustersInput');
      
      // If user hasn't set values yet, provide initial estimate
      if (parseInt(controlSurfacesInput.value) === 0 && parseInt(thrustersInput.value) === 0) {
        // Default distribution: half and half
        const estimatedControls = Math.floor(remainingStates / 2);
        const estimatedThrusters = remainingStates - estimatedControls;
        
        controlSurfacesInput.value = estimatedControls;
        thrustersInput.value = estimatedThrusters;
      }
    }
  }

  handleAddVessel() {
    if (!this.vesselSelect) return;
    const vesselName = this.vesselSelect.value;
    if (!vesselName) return;
    this.toggleVessel(vesselName, true);
  }

  toggleVessel(vesselName, enable) {
    if (!vesselName) return;

    if (enable) {
      if (this.vessels.has(vesselName)) return;
      const color = this.getVesselColor(vesselName);
      const panelInfo = this.createVesselPanel(vesselName, color);
      if (!panelInfo) return;
      panelInfo.color = color;
      const visualizer = new VesselVisualizer(vesselName, this.ros, this.userConfig, panelInfo, this);
      this.vessels.set(vesselName, visualizer);
      visualizer.subscribe();
    } else {
      const visualizer = this.vessels.get(vesselName);
      if (!visualizer) return;
      visualizer.destroy();
      this.vessels.delete(vesselName);
      this.removeGlobalDataset(vesselName);
    }
    this.syncActiveList();
  }

  createVesselPanel(vesselName, color) {
    if (!this.dashboardTemplate || !this.dashboardHost) return null;
    const fragment = this.dashboardTemplate.content.cloneNode(true);
    const panel = fragment.querySelector('.vessel-panel');
    if (!panel) return null;
    panel.dataset.vessel = vesselName;
    const title = panel.querySelector('.vessel-title');
    if (title) {
      title.textContent = vesselName;
    }
    if (color) {
      const header = panel.querySelector('.panel-header');
      if (header) {
        header.style.borderLeft = `4px solid ${color}`;
        header.style.paddingLeft = '16px';
      }
    }
    const removeButton = panel.querySelector('[data-role="remove"]');
    if (removeButton) {
      removeButton.addEventListener('click', () => this.toggleVessel(vesselName, false));
    }

    const slug = this.slugifyVessel(vesselName);
    panel.querySelectorAll('[id]').forEach((element) => {
      element.id = `${slug}-${element.id}`;
    });

    this.dashboardHost.appendChild(panel);
    this.updateEmptyState();
    return { panel, domPrefix: slug, color };
  }

  slugifyVessel(name) {
    return `vessel-${name.replace(/[^a-zA-Z0-9_-]/g, '-')}`;
  }

  syncActiveList() {
    if (!this.activeVesselList) return;
    this.activeVesselList.innerHTML = '';
    if (this.vessels.size === 0) {
      const placeholder = document.createElement('span');
      placeholder.style.color = 'var(--text-secondary)';
      placeholder.textContent = 'No vessels selected';
      this.activeVesselList.appendChild(placeholder);
    } else {
      this.vessels.forEach((_, name) => {
        const chip = document.createElement('div');
        chip.className = 'vessel-chip';
        const color = this.getVesselColor(name);
        chip.style.borderColor = color;
        chip.style.color = color;
        const label = document.createElement('span');
        label.textContent = name;
        const closeBtn = document.createElement('button');
        closeBtn.type = 'button';
        closeBtn.textContent = '×';
        closeBtn.addEventListener('click', () => this.toggleVessel(name, false));
        chip.appendChild(label);
        chip.appendChild(closeBtn);
        this.activeVesselList.appendChild(chip);
      });
    }
    this.updateEmptyState();
  }

  updateEmptyState() {
    if (!this.emptyState) return;
    this.emptyState.style.display = this.vessels.size === 0 ? 'block' : 'none';
  }

  getVesselColor(name) {
    if (this.vesselColors.has(name)) return this.vesselColors.get(name);
    const color = this.colorPalette[this.vesselColors.size % this.colorPalette.length] || '#36A2EB';
    this.vesselColors.set(name, color);
    return color;
  }

  initGlobalPathChart() {
    const canvas = document.getElementById('globalPathChart');
    if (!canvas || typeof Chart === 'undefined') return;
    this.globalPathChart = new Chart(canvas, {
      type: 'scatter',
      data: {
        datasets: []
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: {
          mode: 'nearest',
          axis: 'xy',
          intersect: false
        },
        plugins: {
          legend: {
            position: 'bottom',
            labels: {
              usePointStyle: true,
              font: {
                size: 12
              }
            }
          },
          tooltip: {
            callbacks: {
              label: (context) => {
                const label = context.dataset.label || 'Series';
                const x = context.parsed.x !== undefined ? context.parsed.x.toFixed(2) : '0.00';
                const y = context.parsed.y !== undefined ? context.parsed.y.toFixed(2) : '0.00';
                return `${label}: (${x}, ${y})`;
              }
            }
          }
        },
        scales: {
          x: {
            title: { display: true, text: 'X Position (m)' },
            type: 'linear',
            grid: { color: 'rgba(0,0,0,0.05)' }
          },
          y: {
            title: { display: true, text: 'Y Position (m)' },
            type: 'linear',
            grid: { color: 'rgba(0,0,0,0.05)' }
          }
        }
      }
    });
  }

  ensureGlobalDataset(vesselName) {
    if (!this.globalPathChart) return null;
    let dataset = this.globalPathChart.data.datasets.find(ds => ds._vessel === vesselName);
    if (!dataset) {
      const color = this.getVesselColor(vesselName);
      dataset = {
        label: vesselName,
        data: [],
        showLine: true,
        borderColor: color,
        backgroundColor: color,
        borderWidth: 2,
        pointRadius: 0,
        fill: false,
        tension: 0.2,
        _vessel: vesselName
      };
      this.globalPathChart.data.datasets.push(dataset);
    }
    return dataset;
  }

  updateGlobalPath(vesselName, point) {
    if (!this.globalPathChart) return;
    const dataset = this.ensureGlobalDataset(vesselName);
    if (!dataset) return;
    dataset.data.push(point);
    if (dataset.data.length > this.globalPathLimit) {
      dataset.data.splice(0, dataset.data.length - this.globalPathLimit);
    }
    this.globalChartDirty = true;
  }

  updateEvaderCaptureCircle(vesselName, centerX, centerY, radius = 1) {
    if (!this.globalPathChart) return;
    let dataset = this.globalPathChart.data.datasets.find(ds => ds._captureFor === vesselName);
    if (!dataset) {
      const color = this.getVesselColor(vesselName);
      dataset = {
        label: `${vesselName} Capture Radius`,
        data: [],
        showLine: true,
        borderColor: color,
        backgroundColor: 'transparent',
        borderWidth: 1.5,
        borderDash: [6, 4],
        pointRadius: 0,
        tension: 0.2,
        fill: false,
        _captureFor: vesselName
      };
      this.globalPathChart.data.datasets.push(dataset);
    }
    dataset.data = this.generateCirclePoints(centerX, centerY, radius);
    this.globalChartDirty = true;
  }

  updateGlobalGoal(vesselName, point) {
    if (!this.globalPathChart || !point) return;
    let dataset = this.globalPathChart.data.datasets.find(ds => ds._goalFor === vesselName);
    if (!dataset) {
      const color = this.getVesselColor(vesselName);
      dataset = {
        label: `${vesselName} Goal`,
        data: [],
        showLine: false,
        borderColor: color,
        backgroundColor: color,
        pointRadius: 8,
        pointHoverRadius: 10,
        pointStyle: 'star',
        pointBorderColor: '#111',
        pointBorderWidth: 1.5,
        _goalFor: vesselName
      };
      this.globalPathChart.data.datasets.push(dataset);
    }
    dataset.data = [point];
    this.globalChartDirty = true;
  }

  generateCirclePoints(cx, cy, radius) {
    const points = [];
    for (let angle = 0; angle <= 360; angle += 5) {
      const rad = angle * Math.PI / 180;
      points.push({
        x: cx + radius * Math.cos(rad),
        y: cy + radius * Math.sin(rad)
      });
    }
    return points;
  }

  commitGlobalPathUpdate() {
    if (this.globalPathChart && this.globalChartDirty) {
      this.globalPathChart.update('none');
      this.globalChartDirty = false;
    }
  }

  removeGlobalDataset(vesselName) {
    if (!this.globalPathChart) return;
    const originalLength = this.globalPathChart.data.datasets.length;
    this.globalPathChart.data.datasets = this.globalPathChart.data.datasets.filter(ds => ds._vessel !== vesselName && ds._captureFor !== vesselName && ds._goalFor !== vesselName);
    if (this.globalPathChart.data.datasets.length !== originalLength) {
      this.globalChartDirty = true;
      this.commitGlobalPathUpdate();
    }
  }

  isEvader(vesselName) {
    if (!vesselName) return false;
    return vesselName.toLowerCase().includes('evader');
  }
}

// Vessel visualization class
class VesselVisualizer {
  constructor(vesselName, ros, userConfig, domContext, manager) {
    this.vesselName = vesselName;
    this.ros = ros;
    this.charts = new Map();
    this.sampleIndex = 0;
    this.setConfiguration(userConfig.controlSurfaces, userConfig.thrusters);
    this.setupChartPlugins();
    this.panel = domContext ? domContext.panel : null;
    this.domPrefix = domContext ? domContext.domPrefix : '';
    this.manager = manager || null;
    this.color = (domContext && domContext.color) || this.manager?.getVesselColor(this.vesselName) || '#36A2EB';
    this.statusLabel = this.panel ? this.panel.querySelector('[data-role="status"]') : null;
    this.setupCharts();
    this.isCollectingData = false;
    this.lastMessageTime = Date.now();
    this.dataMonitorInterval = null;
  }

  getElement(id) {
    if (!id) return null;
    const actualId = this.domPrefix ? `${this.domPrefix}-${id}` : id;
    return document.getElementById(actualId);
  }
  
  setConfiguration(controlSurfaces, thrusters) {
    this.config = {
      controlSurfaces: controlSurfaces || 0,
      thrusters: thrusters || 0
    };
    console.log(`Set vessel configuration: ${this.config.controlSurfaces} control surfaces, ${this.config.thrusters} thrusters`);
  }

  setupChartPlugins() {
    // Register the zoom plugin if it exists but isn't registered
    if (window.ChartJS && !Chart.registry.plugins.get('zoom')) {
      try {
        Chart.register(window.ChartZoom);
        console.log("Chart.js Zoom plugin registered");
      } catch (e) {
        console.warn("Could not register Chart.js Zoom plugin:", e);
      }
    }
  }

  setupCharts() {
    // Common chart options for better visualization
    Chart.defaults.font.family = "'Roboto', sans-serif";
    Chart.defaults.color = '#666';
    Chart.defaults.elements.line.borderWidth = 2;
    Chart.defaults.elements.point.radius = 0; // Hide points for smoother lines
    Chart.defaults.elements.point.hoverRadius = 4; // Show points on hover
    
    // Better color scheme
    const colorSchemes = {
      position: ['#FF6384', '#36A2EB', '#4BC0C0'],
      velocity: ['#FF9F40', '#9966FF', '#FF6384'],
      angular: ['#FFCD56', '#4BC0C0', '#36A2EB'],
    };

    // Setup path chart with improved styling
    const pathColor = this.color || '#FF6384';
    this.charts.set('path', new Chart(this.getElement('pathChart'), {
    type: 'scatter',
    data: {
        datasets: [
          {
        label: 'Path',
        data: [],
            borderColor: pathColor,
            backgroundColor: 'transparent', // Remove fill color
        showLine: true,
            fill: false, // No fill
            tension: 0.4, // Smooth curves
            borderWidth: 2.5, // Slightly thicker line for better visibility
            pointRadius: 0 // Hide points on path
          },
          {
            label: 'Current Position',
            data: [],
            borderColor: pathColor,
            backgroundColor: pathColor,
            pointRadius: 6,
            pointHoverRadius: 8,
            showLine: false
          },
          {
            label: 'Goal',
            data: [],
            borderColor: '#111',
            backgroundColor: '#111',
            pointRadius: 8,
            pointHoverRadius: 10,
            pointStyle: 'star',
            pointBorderColor: '#fff',
            pointBorderWidth: 1.5,
            showLine: false
          }
        ]
    },
    options: {
      animation: false,
      responsive: true,
        maintainAspectRatio: false,
        interaction: {
          mode: 'nearest',
          intersect: false,
          axis: 'xy'
        },
        plugins: {
          legend: {
            display: false
          },
          tooltip: {
            enabled: true,
            mode: 'nearest',
            callbacks: {
              label: function(context) {
                const point = context.raw;
                return `Position: (${point.x.toFixed(2)}, ${point.y.toFixed(2)})`;
              }
            }
          },
          zoom: {
            pan: {
              enabled: true,
              mode: 'xy',
              modifierKey: 'ctrl'
            },
            zoom: {
              wheel: {
                enabled: true,
              },
              pinch: {
                enabled: true
              },
              mode: 'xy',
            },
            limits: {
              y: {min: 'original', max: 'original'},
              x: {min: 'original', max: 'original'}
            }
          }
        },
      scales: {
          x: { 
            display: true, 
            title: { display: true, text: 'X Position (m)' },
            grid: {
              color: 'rgba(0, 0, 0, 0.05)',
              tickColor: 'rgba(0, 0, 0, 0.15)'
            },
            ticks: {
              maxTicksLimit: 10
            },
            type: 'linear',
            offset: true // Better alignment
          },
          y: { 
            display: true, 
            title: { display: true, text: 'Y Position (m)' },
            grid: {
              color: 'rgba(0, 0, 0, 0.05)',
              tickColor: 'rgba(0, 0, 0, 0.15)'
            },
            ticks: {
              maxTicksLimit: 10
            },
            type: 'linear',
            offset: true // Better alignment
          }
        }
      }
    }));

    // Create path controls container
    const pathContainer = this.getElement('pathChart').parentNode;
    const controlsContainer = document.createElement('div');
    controlsContainer.className = 'path-controls';
    
    // Create reset zoom button
    const resetButton = document.createElement('button');
    resetButton.textContent = 'Reset Zoom';
    resetButton.className = 'chart-control-btn';
    resetButton.onclick = () => {
      const pathChart = this.charts.get('path');
      if (pathChart) {
        pathChart.resetZoom();
      }
    };
    
    // Create clear path button
    const clearButton = document.createElement('button');
    clearButton.textContent = 'Clear Path';
    clearButton.className = 'chart-control-btn';
    clearButton.onclick = () => {
      const pathChart = this.charts.get('path');
      if (pathChart) {
        pathChart.data.datasets[0].data = [];
        
        // Keep the current position (last point)
        if (pathChart.data.datasets[1].data.length > 0) {
          const currentPos = pathChart.data.datasets[1].data[0];
          pathChart.data.datasets[0].data.push(currentPos);
        }
        
        pathChart.update();
      }
    };
    
    // Create follow toggle button
    const followButton = document.createElement('button');
    followButton.textContent = 'Auto Follow: ON';
    followButton.className = 'chart-control-btn';
    followButton.dataset.following = 'true';
    followButton.onclick = () => {
      const isFollowing = followButton.dataset.following === 'true';
      followButton.dataset.following = isFollowing ? 'false' : 'true';
      followButton.textContent = `Auto Follow: ${isFollowing ? 'OFF' : 'ON'}`;
      
      // If turning following back on, reset the zoom to show current position
      if (!isFollowing) {
        const pathChart = this.charts.get('path');
        if (pathChart) {
          pathChart.resetZoom();
          this.adjustPathScaling(pathChart);
          pathChart.update();
        }
      }
    };
    
    // Add buttons to controls container
    controlsContainer.appendChild(resetButton);
    controlsContainer.appendChild(clearButton);
    controlsContainer.appendChild(followButton);
    
    // Add controls to the path container
    pathContainer.appendChild(controlsContainer);
    
    // Store the follow button for later reference
    this.pathFollowButton = followButton;

    // Common chart config for line charts
    const createLineChartConfig = (id, labels, colors) => {
      return {
    type: 'line',
    data: {
      labels: [],
          datasets: labels.map((label, i) => ({
            label,
            data: [],
            borderColor: colors[i],
            backgroundColor: this.hexToRgba(colors[i], 0.1),
            fill: false,
            tension: 0.3, // Smooth line transitions
            borderWidth: 2,
            pointRadius: 0,
            pointHoverRadius: 4
          }))
        },
        options: {
          animation: false,
          responsive: true,
          maintainAspectRatio: false,
          interaction: {
            mode: 'index',
            intersect: false
          },
          plugins: {
            legend: {
              position: 'top',
              labels: {
                boxWidth: 12,
                usePointStyle: true,
                padding: 10,
                font: {
                  size: 11
                }
              }
            },
            tooltip: {
              mode: 'index',
              intersect: false,
              callbacks: {
                label: function(context) {
                  return `${context.dataset.label}: ${context.parsed.y.toFixed(2)}`;
                }
              }
            }
          },
          scales: {
            x: { 
              display: false,
              type: 'linear',
              ticks: {
                maxTicksLimit: 6
              }
            },
            y: {
              beginAtZero: false,
              grid: {
                color: 'rgba(0, 0, 0, 0.05)',
                tickColor: 'rgba(0, 0, 0, 0.15)'
              },
              ticks: {
                maxTicksLimit: 8
              }
            }
          }
        }
      };
    };

    // Create charts with consistent styling
    const charts = [
      { id: 'position', title: 'Position (m)', labels: ['x', 'y', 'z'], colors: colorSchemes.position },
      { id: 'velocity', title: 'Velocity (m/s)', labels: ['u', 'v', 'w'], colors: colorSchemes.velocity },
      { id: 'angularVelocity', title: 'Angular Velocity (rad/s)', labels: ['p', 'q', 'r'], colors: colorSchemes.angular },
      { id: 'orientation', title: 'Euler Angles (degrees)', labels: ['Roll', 'Pitch', 'Yaw'], colors: colorSchemes.angular }
    ];

    charts.forEach(({ id, title, labels, colors }) => {
      const config = createLineChartConfig(id, labels, colors);
      config.options.plugins.title = {
        display: true,
        text: title,
        font: {
          size: 14,
          weight: 'normal'
        }
      };
      this.charts.set(id, new Chart(this.getElement(`${id}Chart`), config));
    });
  }

  // Convert hex color to rgba for backgrounds
  hexToRgba(hex, alpha) {
    // Standard colors
    if (!hex.startsWith('#')) {
      const colors = {
        red: [255, 0, 0],
        green: [0, 128, 0],
        blue: [0, 0, 255],
        yellow: [255, 255, 0],
        orange: [255, 165, 0],
        purple: [128, 0, 128],
        cyan: [0, 255, 255],
      };
      
      if (colors[hex]) {
        const [r, g, b] = colors[hex];
        return `rgba(${r}, ${g}, ${b}, ${alpha})`;
      }
      
      // For HSL colors
      if (hex.startsWith('hsl')) {
        // Extract values using regex
        const match = hex.match(/hsl\((\d+),\s*(\d+)%,\s*(\d+)%\)/);
        if (match) {
          return `hsla(${match[1]}, ${match[2]}%, ${match[3]}%, ${alpha})`;
        }
        return `${hex.replace(')', `, ${alpha})`).replace('hsl', 'hsla')}`;
      }
      
      return `rgba(128, 128, 128, ${alpha})`;
    }
    
    // Convert hex to rgba
    try {
      const r = parseInt(hex.slice(1, 3), 16);
      const g = parseInt(hex.slice(3, 5), 16);
      const b = parseInt(hex.slice(5, 7), 16);
      return `rgba(${r}, ${g}, ${b}, ${alpha})`;
    } catch (e) {
      return `rgba(128, 128, 128, ${alpha})`;
    }
  }

  // Method to remove disconnect overlays when reconnecting
  removeDisconnectOverlays() {
    // Remove overlays from chart containers
    const scope = this.panel || document;
    scope.querySelectorAll('.chart-disconnected').forEach(overlay => {
      overlay.classList.remove('active');
      // Remove after fade-out animation completes
      setTimeout(() => {
        overlay.remove();
      }, 300);
    });
  }

  // Update the startDataCollection method to remove overlays when reconnecting
  startDataCollection() {
    if (this.isCollectingData) return;
    
    this.isCollectingData = true;
    lastMessageTime = Date.now();
    
    // Remove any disconnect overlays when starting data collection
    this.removeDisconnectOverlays();
    
    // Start monitoring data flow
    if (!this.dataMonitorInterval) {
      this.dataMonitorInterval = setInterval(() => {
        const timeSinceLastMessage = Date.now() - lastMessageTime;
        
        // If no messages for 3 seconds, consider connection stale
        if (timeSinceLastMessage > 3000) {
          console.warn(`No data received for ${timeSinceLastMessage}ms, stopping collection`);
          this.stopDataCollection();
          this.showConnectionStatus('stale');
        }
      }, 1000);
    }
  }

  // Update showConnectionStatus to handle more connection states
  showConnectionStatus(status) {
    const container = this.getElement('vesselInfo');
    let statusClass = '';
    let statusText = '';
    
    if (status === true || status === 'connected') {
      statusClass = 'connected';
      statusText = 'Connected';
      // Remove disconnect overlays when connected
      this.removeDisconnectOverlays();
    } else if (status === 'stale') {
      statusClass = 'stale';
      statusText = 'Connection Stale';
    } else {
      statusClass = 'disconnected';
      statusText = 'Disconnected';
      // Add disconnect overlays when disconnected
      this.addDisconnectOverlays();
    }
    
    if (this.statusLabel) {
      this.statusLabel.textContent = statusText;
      this.statusLabel.classList.remove('connected', 'stale', 'disconnected');
      if (statusClass) {
        this.statusLabel.classList.add(statusClass);
      }
    }

    if (!container) return;
    
    // Remove existing status indicator
    const existingStatus = container.querySelector('.status-indicator');
    if (existingStatus) {
      existingStatus.remove();
    }
    
    const statusElement = document.createElement('div');
    statusElement.className = `status-indicator ${statusClass}`;
    statusElement.innerHTML = `<span class="status-dot"></span><span class="status-text">${statusText}</span>`;
    
    container.prepend(statusElement);
  }

  subscribe() {
    try {
      console.log(`Attempting to subscribe to vessel ${this.vesselName}`);
      
      // Try different message types for odometry
      const odomTypes = [
        'nav_msgs/Odometry', 
        'nav_msgs/msg/Odometry'
      ];
      
      // Try different message types for vessel state
      const stateTypes = [
        'std_msgs/Float64MultiArray',
        'std_msgs/msg/Float64MultiArray'
      ];
      
      // Subscribe to odometry with flexible message type
      this.subscribeWithTypes(`/${this.vesselName}/odometry_sim`, odomTypes, 
        (message) => this.handleOdometry(message), 
        (error) => console.error('Failed to subscribe to odometry:', error)
      );
      
      // Subscribe to vessel state with flexible message type
      this.subscribeWithTypes(`/${this.vesselName}/vessel_state`, stateTypes,
        (message) => this.handleVesselState(message),
        (error) => console.error('Failed to subscribe to vessel state:', error)
      );

      // Subscribe to goal updates for evader only
      if (this.manager && this.manager.isEvader(this.vesselName)) {
        const goalTypes = [
          'geometry_msgs/Point',
          'geometry_msgs/msg/Point',
          'geometry_msgs/Pose',
          'geometry_msgs/msg/Pose',
          'geometry_msgs/PoseStamped',
          'geometry_msgs/msg/PoseStamped',
          'std_msgs/Float64MultiArray',
          'std_msgs/msg/Float64MultiArray'
        ];
        this.subscribeWithTypes(`/${this.vesselName}/goal`, goalTypes,
          (message) => this.handleGoal(message),
          () => {}
        );
      }
      
      // Start data collection tracking
      this.startDataCollection();
      
      console.log(`Subscribed to vessel ${this.vesselName}`);
    } catch (error) {
      console.error(`Error subscribing to vessel ${this.vesselName}:`, error);
    }
  }
  
  subscribeWithTypes(topicName, messageTypes, callback, errorCallback) {
    // Try to subscribe with each message type until successful
    const trySubscribe = (index) => {
      if (index >= messageTypes.length) {
        errorCallback(new Error(`Failed to subscribe to ${topicName} with any message type`));
        return;
      }
      
      const topic = new ROSLIB.Topic({
        ros: this.ros,
        name: topicName,
        messageType: messageTypes[index]
      });
      
      try {
        topic.subscribe((message) => {
          // Successfully subscribed
          console.log(`Subscribed to ${topicName} with type ${messageTypes[index]}`);
          
          // Store the topic for later unsubscription
          if (topicName.endsWith('/odometry_sim')) {
            this.odomTopic = topic;
          } else if (topicName.endsWith('/vessel_state')) {
            this.stateTopic = topic;
          } else if (topicName.endsWith('/goal')) {
            this.goalTopic = topic;
          }
          
          callback(message);
        });
      } catch (error) {
        console.warn(`Failed to subscribe to ${topicName} with type ${messageTypes[index]}:`, error);
        // Try the next message type
        trySubscribe(index + 1);
      }
    };
    
    // Start with the first message type
    trySubscribe(0);
  }

  unsubscribe() {
    // Stop data collection
    this.stopDataCollection();
    
    // Unsubscribe from topics
    if (this.odomTopic) {
      this.odomTopic.unsubscribe();
      this.odomTopic = null;
    }
    
    if (this.stateTopic) {
      this.stateTopic.unsubscribe();
      this.stateTopic = null;
    }

    if (this.goalTopic) {
      this.goalTopic.unsubscribe();
      this.goalTopic = null;
    }
    
    // Add disconnect indicator to vessel info
    this.showConnectionStatus(false);
    
    // Add disconnect overlays
    this.addDisconnectOverlays();
    
    console.log(`Unsubscribed from vessel ${this.vesselName}`);
  }

  destroy() {
    this.unsubscribe();
    this.charts.forEach(chart => chart.destroy());
    this.charts.clear();
    if (this.panel && this.panel.parentNode) {
      this.panel.remove();
    }
  }

  handleOdometry(message) {
    // Update last message time
    lastMessageTime = Date.now();
    
    // Ensure we're collecting data
    if (!this.isCollectingData) {
      this.startDataCollection();
      this.showConnectionStatus(true);
    }
    
    this.sampleIndex++;

    // Extract data
    const x = message.pose.pose.position.x;
    const y = message.pose.pose.position.y;
    const z = message.pose.pose.position.z;
    const u = message.twist.twist.linear.x;
    const v = message.twist.twist.linear.y;
    const w = message.twist.twist.linear.z;
    const p = message.twist.twist.angular.x;
    const q = message.twist.twist.angular.y;
    const r = message.twist.twist.angular.z;
    
    // Convert quaternion to Euler angles
    const euler = this.quaternionToEuler(message.pose.pose.orientation);

    // Update charts
    this.updateChart('path', [{x, y}]);
    this.updateChart('position', [x, y, z]);
    this.updateChart('velocity', [u, v, w]);
    this.updateChart('angularVelocity', [p, q, r]);
    this.updateChart('orientation', [euler.roll, euler.pitch, euler.yaw]);

    if (this.manager) {
      this.manager.updateGlobalPath(this.vesselName, { x, y });
      if (this.manager.isEvader(this.vesselName)) {
        this.manager.updateEvaderCaptureCircle(this.vesselName, x, y);
      }
      this.manager.commitGlobalPathUpdate();
    }

    // Update vessel info
    this.updateVesselInfo(message);
  }

  handleVesselState(message) {
    console.log('Received vessel state:', message);
    
    // Update last message time
    lastMessageTime = Date.now();
    
    // Check if message has data property (standard ROS message)
    if (!message.data || !Array.isArray(message.data)) {
      console.warn('Vessel state message has no data array');
      return;
    }

    // Extract control surfaces and thrusters from vessel state
    const state = message.data;
    
    // Update manager with the latest state length for auto-calculation
    if (vesselManager) {
      vesselManager.updateLatestStateLength(state.length);
    }
    
    // Handle the case where state might be shorter than expected
    if (state.length < 12) {
      console.warn(`State vector length (${state.length}) is too short`);
      return;
    }

    // Use the user configuration instead of guessing
    const controlStart = 12; // After position and orientation
    const thrusterStart = controlStart + this.config.controlSurfaces;
    
    // Only update if we have controls in config and enough state data
    if (this.config.controlSurfaces > 0 && state.length > controlStart) {
      // Ensure we don't try to access beyond array bounds
      const end = Math.min(thrusterStart, state.length);
      this.updateControlSurfaces(state.slice(controlStart, end));
    } else {
      // Clear control surfaces chart if no controls
    const container = this.getElement('controlSurfacesChart');
      container.innerHTML = '<div style="text-align: center; padding: 20px;">No control surfaces configured</div>';
    }
    
    // Only update if we have thrusters in config and enough state data
    if (this.config.thrusters > 0 && state.length > thrusterStart) {
      this.updateThrusters(state.slice(thrusterStart, thrusterStart + this.config.thrusters));
    } else {
      // Clear thrusters chart if no thrusters
    const container = this.getElement('thrustersChart');
      container.innerHTML = '<div style="text-align: center; padding: 20px;">No thrusters configured</div>';
    }
  }

  handleGoal(message) {
    const point = this.extractGoalPoint(message);
    if (!point) return;

    const pathChart = this.charts.get('path');
    if (pathChart && pathChart.data && pathChart.data.datasets[2]) {
      pathChart.data.datasets[2].data = [point];
      pathChart.update('none');
    }

    if (this.manager) {
      this.manager.updateGlobalGoal(this.vesselName, point);
      this.manager.commitGlobalPathUpdate();
    }
  }

  extractGoalPoint(message) {
    if (!message) return null;

    // geometry_msgs/Point
    if (typeof message.x === 'number' && typeof message.y === 'number') {
      return { x: message.x, y: message.y };
    }

    // geometry_msgs/Pose or PoseStamped
    if (message.pose && message.pose.position) {
      const pos = message.pose.position;
      if (typeof pos.x === 'number' && typeof pos.y === 'number') {
        return { x: pos.x, y: pos.y };
      }
    }

    // std_msgs/Float64MultiArray [x, y, ...]
    if (Array.isArray(message.data) && message.data.length >= 2) {
      const x = Number(message.data[0]);
      const y = Number(message.data[1]);
      if (!Number.isNaN(x) && !Number.isNaN(y)) {
        return { x, y };
      }
    }

    return null;
  }

  updateChart(chartId, values) {
    const chart = this.charts.get(chartId);
    if (!chart) return;

    // Maximum number of points to show
    const MAX_POINTS = 200;

    if (chartId === 'path') {
      // For path chart with current position tracking
      const newPoint = values[0];
      
      // Update the path (dataset 0)
      chart.data.datasets[0].data.push(newPoint);
      
      // Keep the last MAX_POINTS for the path
      if (chart.data.datasets[0].data.length > MAX_POINTS) {
        chart.data.datasets[0].data.shift();
      }
      
      // Update the current position marker (dataset 1)
      chart.data.datasets[1].data = [newPoint];
      
      // Auto-scale the axes if auto-follow is enabled
      const isFollowing = this.pathFollowButton 
        ? this.pathFollowButton.dataset.following === 'true'
        : true;
      
      if (isFollowing && chart.data.datasets[0].data.length > 1) {
        this.adjustPathScaling(chart);
      }
    } else {
      // For other charts, maintain continuous time series
      chart.data.labels.push(this.sampleIndex);
      
      // Add data to each dataset
      values.forEach((value, i) => {
        if (i < chart.data.datasets.length) {
          chart.data.datasets[i].data.push(value);
          if (chart.data.datasets[i].data.length > MAX_POINTS) {
            chart.data.datasets[i].data.shift();
          }
        }
      });
      
      // Remove excess labels to match data length
      while (chart.data.labels.length > chart.data.datasets[0].data.length) {
        chart.data.labels.shift();
      }
      
      // Adjust x-axis for better visualization
      if (chart.options.scales.x.type === 'linear') {
        const minIndex = Math.min(...chart.data.labels);
        const maxIndex = Math.max(...chart.data.labels);
        chart.options.scales.x.min = minIndex;
        chart.options.scales.x.max = maxIndex;
      }
    }

    chart.update('none'); // 'none' mode for better performance
  }

  // Method to automatically adjust path chart scaling
  adjustPathScaling(chart) {
    // Check if auto-follow is disabled
    const isFollowing = this.pathFollowButton 
      ? this.pathFollowButton.dataset.following === 'true'
      : true;
      
    if (!isFollowing) {
      return;
    }
    
    // Don't auto-adjust if the user has manually zoomed or panned
    if (chart._zoom && chart._zoom.panning) {
      return;
    }
    
    const data = chart.data.datasets[0].data;
    
    // If no data, don't adjust
    if (!data || data.length === 0) {
      return;
    }
    
    // Calculate min/max values with padding
    let minX = Number.MAX_VALUE;
    let maxX = Number.MIN_VALUE;
    let minY = Number.MAX_VALUE;
    let maxY = Number.MIN_VALUE;
    
    data.forEach(point => {
      if (point && typeof point.x === 'number' && typeof point.y === 'number') {
        minX = Math.min(minX, point.x);
        maxX = Math.max(maxX, point.x);
        minY = Math.min(minY, point.y);
        maxY = Math.max(maxY, point.y);
      }
    });
    
    // Handle case where all values are the same
    if (minX === maxX) {
      minX -= 10;
      maxX += 10;
    }
    if (minY === maxY) {
      minY -= 10;
      maxY += 10;
    }
    
    // Add padding (15%)
    const rangeX = maxX - minX;
    const rangeY = maxY - minY;
    
    const paddingX = rangeX * 0.15;
    const paddingY = rangeY * 0.15;
    
    // Set min/max on axes to ensure equal scaling (maintain aspect ratio)
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    
    // Ensure the scale is the same for both axes (use the larger range with padding)
    const maxRange = Math.max(rangeX + 2 * paddingX, rangeY + 2 * paddingY);
    
    // Calculate the new bounds, ensuring they're centered around the centroid
    const newMinX = centerX - maxRange / 2;
    const newMaxX = centerX + maxRange / 2;
    const newMinY = centerY - maxRange / 2;
    const newMaxY = centerY + maxRange / 2;
    
    // Only update if there are significant changes or on first run
    if (!chart._lastBounds || 
        Math.abs(chart._lastBounds.minX - newMinX) > 0.05 ||
        Math.abs(chart._lastBounds.maxX - newMaxX) > 0.05 ||
        Math.abs(chart._lastBounds.minY - newMinY) > 0.05 ||
        Math.abs(chart._lastBounds.maxY - newMaxY) > 0.05) {
      
      // Set the new bounds
      chart.options.scales.x.min = newMinX;
      chart.options.scales.x.max = newMaxX;
      chart.options.scales.y.min = newMinY;
      chart.options.scales.y.max = newMaxY;
      
      // Store the last bounds to avoid unnecessary updates
      chart._lastBounds = { minX: newMinX, maxX: newMaxX, minY: newMinY, maxY: newMaxY };
    }
  }

  updateControlSurfaces(controlStates) {
    const container = this.getElement('controlSurfacesChart');
    
    if (!controlStates || controlStates.length === 0) {
      container.innerHTML = '<div style="text-align: center; padding: 20px;">No control surfaces available</div>';
      return;
    }
    
    const existingChart = this.charts.get('controlSurfaces');
    if (existingChart) {
      if (existingChart.data.datasets.length === controlStates.length) {
        // Update existing chart data
        existingChart.data.labels.push(this.sampleIndex);
        
        controlStates.forEach((value, i) => {
          existingChart.data.datasets[i].data.push(value);
          // Keep only the last 200 points for smoother rendering
          if (existingChart.data.datasets[i].data.length > 200) {
            existingChart.data.datasets[i].data.shift();
          }
        });
        
        // Remove excess labels
        while (existingChart.data.labels.length > existingChart.data.datasets[0].data.length) {
          existingChart.data.labels.shift();
        }
        
        existingChart.update('none'); // Use 'none' mode for better performance
        return;
      }
      existingChart.destroy();
    }
    
    // Create new canvas and chart
    container.innerHTML = '';
    const canvas = document.createElement('canvas');
    container.appendChild(canvas);

    // Create color scheme based on number of control surfaces
    const colors = controlStates.map((_, i) => 
      `hsl(${(i * 360) / Math.max(controlStates.length, 1)}, 70%, 50%)`);

    // Create chart with enhanced styling
    const chart = new Chart(canvas, {
    type: 'line',
    data: {
        labels: [this.sampleIndex],
        datasets: controlStates.map((value, i) => ({
          label: `Control Surface ${i + 1}`,
          data: [value],
          borderColor: colors[i],
          backgroundColor: this.hexToRgba(colors[i], 0.1),
          fill: false,
          tension: 0.3,
          borderWidth: 2,
          pointRadius: 0,
          pointHoverRadius: 4
        }))
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: {
          mode: 'index',
          intersect: false
        },
        plugins: {
          legend: {
            position: 'top',
            labels: {
              boxWidth: 12,
              usePointStyle: true,
              font: {
                size: 11
              }
            }
          },
          tooltip: {
            mode: 'index',
            intersect: false
          }
        },
        scales: {
          x: { 
            display: false,
            type: 'linear'
          },
          y: {
            grid: {
              color: 'rgba(0, 0, 0, 0.05)',
              tickColor: 'rgba(0, 0, 0, 0.15)'
            }
          }
        }
      }
    });

    this.charts.set('controlSurfaces', chart);
  }

  updateThrusters(thrusterStates) {
    const container = this.getElement('thrustersChart');
    
    if (!thrusterStates || thrusterStates.length === 0) {
      container.innerHTML = '<div style="text-align: center; padding: 20px;">No thrusters available</div>';
      return;
    }
    
    const existingChart = this.charts.get('thrusters');
    if (existingChart) {
      if (existingChart.data.datasets.length === thrusterStates.length) {
        // Update existing chart
        existingChart.data.labels.push(this.sampleIndex);
        
        thrusterStates.forEach((value, i) => {
          existingChart.data.datasets[i].data.push(value);
          if (existingChart.data.datasets[i].data.length > 200) {
            existingChart.data.datasets[i].data.shift();
          }
        });
        
        // Remove excess labels
        while (existingChart.data.labels.length > existingChart.data.datasets[0].data.length) {
          existingChart.data.labels.shift();
        }
        
        existingChart.update('none'); // Use 'none' mode for better performance
        return;
      }
      existingChart.destroy();
    }
    
    // Create new canvas and chart
    container.innerHTML = '';
    const canvas = document.createElement('canvas');
    container.appendChild(canvas);

    // Create color scheme with more vibrant colors for thrusters
    const colors = thrusterStates.map((_, i) => 
      `hsl(${(i * 360) / Math.max(thrusterStates.length, 1) + 180}, 80%, 50%)`);

    // Create chart with enhanced styling
    const chart = new Chart(canvas, {
    type: 'line',
    data: {
        labels: [this.sampleIndex],
        datasets: thrusterStates.map((value, i) => ({
          label: `Thruster ${i + 1}`,
          data: [value],
          borderColor: colors[i],
          backgroundColor: this.hexToRgba(colors[i], 0.1),
          fill: false,
          tension: 0.3,
          borderWidth: 2,
          pointRadius: 0,
          pointHoverRadius: 4
        }))
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,
        interaction: {
          mode: 'index',
          intersect: false
        },
        plugins: {
          legend: {
            position: 'top',
            labels: {
              boxWidth: 12,
              usePointStyle: true,
              font: {
                size: 11
              }
            }
          },
          tooltip: {
            mode: 'index',
            intersect: false
          }
        },
        scales: {
          x: { 
            display: false,
            type: 'linear'
          },
          y: {
            grid: {
              color: 'rgba(0, 0, 0, 0.05)',
              tickColor: 'rgba(0, 0, 0, 0.15)'
            }
          }
        }
      }
    });

    this.charts.set('thrusters', chart);
  }

  updateVesselInfo(message) {
    try {
      const container = this.getElement('vesselInfo');
      if (!container) return;
      
      if (!message || !message.pose || !message.pose.pose || !message.twist || !message.twist.twist) {
        console.warn('Incomplete odometry message received');
        return;
      }
      
      // Clear connection status indicators
      const existingStatus = container.querySelector('.status-indicator');
      if (existingStatus) {
        existingStatus.remove();
      }
      
      container.innerHTML = `
        <div class="info-item">
          <div class="info-label">Vessel Name</div>
          <div class="info-value">${this.vesselName}</div>
        </div>
        <div class="info-item">
          <div class="info-label">Position</div>
          <div class="info-value">X: ${message.pose.pose.position.x.toFixed(2)}m</div>
          <div class="info-value">Y: ${message.pose.pose.position.y.toFixed(2)}m</div>
          <div class="info-value">Z: ${message.pose.pose.position.z.toFixed(2)}m</div>
        </div>
        <div class="info-item">
          <div class="info-label">Velocity</div>
          <div class="info-value">U: ${message.twist.twist.linear.x.toFixed(2)}m/s</div>
          <div class="info-value">V: ${message.twist.twist.linear.y.toFixed(2)}m/s</div>
          <div class="info-value">W: ${message.twist.twist.linear.z.toFixed(2)}m/s</div>
        </div>
        <div class="info-item">
          <div class="info-label">Angular Velocity</div>
          <div class="info-value">P: ${message.twist.twist.angular.x.toFixed(2)}rad/s</div>
          <div class="info-value">Q: ${message.twist.twist.angular.y.toFixed(2)}rad/s</div>
          <div class="info-value">R: ${message.twist.twist.angular.z.toFixed(2)}rad/s</div>
        </div>
        <div class="info-item">
          <div class="info-label">Last Update</div>
          <div class="info-value">${new Date().toLocaleTimeString()}</div>
        </div>
      `;
    } catch (error) {
      console.error('Error updating vessel info:', error);
    }
  }

  quaternionToEuler(q) {
    const x = q.x, y = q.y, z = q.z, w = q.w;
    const sinr_cosp = 2 * (w * x + y * z);
    const cosr_cosp = 1 - 2 * (x * x + y * y);
    const roll = Math.atan2(sinr_cosp, cosr_cosp) * 180 / Math.PI;

    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * 90 : Math.asin(sinp) * 180 / Math.PI;

    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI;
    
    return { roll, pitch, yaw };
  }

  // Add disconnect overlay to all chart containers
  addDisconnectOverlays() {
    // Add disconnect overlay to each chart container
    this.charts.forEach((chart, id) => {
      const canvas = this.getElement(`${id}Chart`);
      if (canvas) {
        const container = canvas.parentNode;
        
        // Only add if not already present
        if (!container.querySelector('.chart-disconnected')) {
          const overlay = document.createElement('div');
          overlay.className = 'chart-disconnected';
          overlay.innerHTML = '<div class="disconnect-message">Simulator Disconnected</div>';
          container.appendChild(overlay);
          
          // Activate after a short delay for animation
          setTimeout(() => {
            overlay.classList.add('active');
          }, 50);
        }
      }
    });
    
    // Also handle control surfaces and thrusters containers
    ['controlSurfacesChart', 'thrustersChart'].forEach(id => {
      const container = this.getElement(id);
      if (container && !container.querySelector('.chart-disconnected')) {
        const overlay = document.createElement('div');
        overlay.className = 'chart-disconnected';
        overlay.innerHTML = '<div class="disconnect-message">Simulator Disconnected</div>';
        container.appendChild(overlay);
        
        // Activate after a short delay for animation
        setTimeout(() => {
          overlay.classList.add('active');
        }, 50);
      }
    });
  }

  // Stop data collection and clean up
  stopDataCollection() {
    this.isCollectingData = false;
    
    if (this.dataMonitorInterval) {
      clearInterval(this.dataMonitorInterval);
      this.dataMonitorInterval = null;
    }
    
    // Update the vessel charts to show disconnected state
    this.updateAllChartsWithDisconnectStatus();
  }

  // Update showConnectionStatus to handle more connection states
  showConnectionStatus(status) {
    const container = this.getElement('vesselInfo');
    if (!container) return;
    
    // Remove existing status indicator
    const existingStatus = container.querySelector('.status-indicator');
    if (existingStatus) {
      existingStatus.remove();
    }
    
    let statusClass = '';
    let statusText = '';
    
    if (status === true || status === 'connected') {
      statusClass = 'connected';
      statusText = 'Connected';
      // Remove disconnect overlays when connected
      this.removeDisconnectOverlays();
    } else if (status === 'stale') {
      statusClass = 'stale';
      statusText = 'Connection Stale';
    } else {
      statusClass = 'disconnected';
      statusText = 'Disconnected';
      // Add disconnect overlays when disconnected
      this.addDisconnectOverlays();
    }
    
    const statusElement = document.createElement('div');
    statusElement.className = `status-indicator ${statusClass}`;
    statusElement.innerHTML = `<span class="status-dot"></span><span class="status-text">${statusText}</span>`;
    
    container.prepend(statusElement);
  }

  // Add a disconnect annotation to a chart
  addDisconnectAnnotation(chart) {
    try {
      // Check if Chart.js annotation plugin is available
      if (!Chart.registry.plugins.get('annotation')) {
        console.warn('Chart.js annotation plugin not registered, cannot add disconnect annotation');
        return;
      }
      
      // Only add if not already present
      if (chart.options.plugins.annotation && 
          chart.options.plugins.annotation.annotations && 
          chart.options.plugins.annotation.annotations.disconnectLine) {
        return;
      }
      
      // Initialize annotation plugin options if not already present
      if (!chart.options.plugins.annotation) {
        chart.options.plugins.annotation = {
          annotations: {}
        };
      } else if (!chart.options.plugins.annotation.annotations) {
        chart.options.plugins.annotation.annotations = {};
      }
      
      // For regular time-series charts
      if (chart.data.labels && chart.data.labels.length > 0) {
        const lastTimeIndex = chart.data.labels[chart.data.labels.length - 1];
        
        // Add a vertical line at the last timestamp
        chart.options.plugins.annotation.annotations.disconnectLine = {
          type: 'line',
          xMin: lastTimeIndex,
          xMax: lastTimeIndex,
          borderColor: 'rgba(255, 0, 0, 0.5)',
          borderWidth: 2,
          borderDash: [5, 5],
          label: {
            content: 'Disconnected',
            enabled: true,
            position: 'top',
            backgroundColor: 'rgba(255, 0, 0, 0.7)',
            font: {
              size: 11
            }
          }
        };
      } 
      // For path chart
      else if (chart.type === 'scatter' || chart.id === 'path') {
        // If there's data in the current position dataset
        if (chart.data.datasets.length > 1 && 
            chart.data.datasets[1] && 
            chart.data.datasets[1].data && 
            chart.data.datasets[1].data.length > 0) {
          
          const lastPoint = chart.data.datasets[1].data[0];
          
          // Add a point annotation at the last position
          chart.options.plugins.annotation.annotations.disconnectPoint = {
            type: 'point',
            xValue: lastPoint.x,
            yValue: lastPoint.y,
            backgroundColor: 'rgba(255, 0, 0, 0.7)',
            radius: 8,
            borderWidth: 2,
            borderColor: 'white',
            label: {
              content: 'Disconnected',
              enabled: true,
              position: 'top',
              backgroundColor: 'rgba(255, 0, 0, 0.7)',
              font: {
                size: 11
              }
            }
          };
        }
      }
      
      // Update the chart with minimal animation
      chart.update('none');
    } catch (error) {
      console.error('Error adding disconnect annotation:', error);
    }
  }
}

// Initialize the vessel manager
const vesselManager = new VesselManager();
