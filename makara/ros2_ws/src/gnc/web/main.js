// Connect to rosbridge websocket (adjust the URL as needed)
var ros = new ROSLIB.Ros({
  url: 'ws://192.168.0.226:9090'
 // url: 'ws://localhost:9090'
});

ros.on('connection', function () {
  console.log('Connected to rosbridge.');
  // Get all topics once connected
  getTopics();
});
ros.on('error', function (error) {
  console.error('Error connecting to rosbridge: ', error);
});

// Store all active subscribers
var odomSubscribers = {};
var actuatorSubscribers = {};

// Get all topics and filter for odometry and actuator topics
function getTopics() {
  ros.getTopics(function(topics) {
    var odomTopics = topics.topics.filter(topic => topic.endsWith('/odometry') || topic.endsWith('/odometry_sim'));
    var actuatorTopics = topics.topics.filter(topic => topic.endsWith('/actuator_cmd'));
    
    // Subscribe to all odometry topics
    odomTopics.forEach(topic => {
      if (!odomSubscribers[topic]) {
        var subscriber = new ROSLIB.Topic({
          ros: ros,
          name: topic,
          messageType: 'nav_msgs/Odometry'
        });
        
        subscriber.subscribe(function(message) {
          processOdomMessage(topic, message);
        });
        
        odomSubscribers[topic] = subscriber;
        console.log('Subscribed to odometry topic:', topic);
      }
    });
    
    // Subscribe to all actuator topics
    actuatorTopics.forEach(topic => {
      if (!actuatorSubscribers[topic]) {
        var subscriber = new ROSLIB.Topic({
          ros: ros,
          name: topic,
          messageType: 'interfaces/Actuator'
        });
        
        subscriber.subscribe(function(message) {
          processActuatorMessage(topic, message);
        });
        
        actuatorSubscribers[topic] = subscriber;
        console.log('Subscribed to actuator topic:', topic);
      }
    });
  });
}

// Helper: convert quaternion to Euler angles
function quaternionToEuler(q) {
  var x = q.x, y = q.y, z = q.z, w = q.w;
  var sinr_cosp = 2 * (w * x + y * z);
  var cosr_cosp = 1 - 2 * (x * x + y * y);
  var roll = Math.atan2(sinr_cosp, cosr_cosp) * 180 / Math.PI;

  var sinp = 2 * (w * y - z * x);
  var pitch;
  if (Math.abs(sinp) >= 1) {
    pitch = Math.sign(sinp) * 90; // 90 degrees (π/2 in radians converted to degrees)
  } else {
    pitch = Math.asin(sinp) * 180 / Math.PI;
  }

  var siny_cosp = 2 * (w * z + x * y);
  var cosy_cosp = 1 - 2 * (y * y + z * z);
  var yaw = Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI;
  
  return { roll: roll, pitch: pitch, yaw: yaw };
}

// Get canvas contexts for individual charts
var pathCtx = document.getElementById('pathChart').getContext('2d');
var xPositionCtx = document.getElementById('xPositionChart').getContext('2d');
var yPositionCtx = document.getElementById('yPositionChart').getContext('2d');
var zPositionCtx = document.getElementById('zPositionChart').getContext('2d');
var uVelocityCtx = document.getElementById('uVelocityChart').getContext('2d');
var vVelocityCtx = document.getElementById('vVelocityChart').getContext('2d');
var wVelocityCtx = document.getElementById('wVelocityChart').getContext('2d');
var rollCtx = document.getElementById('rollChart').getContext('2d');
var pitchCtx = document.getElementById('pitchChart').getContext('2d');
var yawCtx = document.getElementById('yawChart').getContext('2d');
var pAngularVelocityCtx = document.getElementById('pAngularVelocityChart').getContext('2d');
var qAngularVelocityCtx = document.getElementById('qAngularVelocityChart').getContext('2d');
var rAngularVelocityCtx = document.getElementById('rAngularVelocityChart').getContext('2d');

// Create charts for each data type
var pathChart = new Chart(pathCtx, {
  type: 'scatter',
  data: {
    datasets: []
  },
  options: {
    animation: false,
    responsive: true,
    aspectRatio: 1,
    scales: {
      x: { display: true, title: { display: true, text: 'X Position' } },
      y: { display: true, title: { display: true, text: 'Y Position' } }
    }
  }
});

// Create individual charts for each variable
function createSingleVariableChart(ctx, label) {
  return new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: []
    },
    options: {
      animation: false,
      responsive: true,
      scales: { x: { display: false } }
    }
  });
}

// Create all individual charts
var xPositionChart = createSingleVariableChart(xPositionCtx, 'X Position');
var yPositionChart = createSingleVariableChart(yPositionCtx, 'Y Position');
var zPositionChart = createSingleVariableChart(zPositionCtx, 'Z Position');
var uVelocityChart = createSingleVariableChart(uVelocityCtx, 'U Velocity');
var vVelocityChart = createSingleVariableChart(vVelocityCtx, 'V Velocity');
var wVelocityChart = createSingleVariableChart(wVelocityCtx, 'W Velocity');
var rollChart = createSingleVariableChart(rollCtx, 'Roll');
var pitchChart = createSingleVariableChart(pitchCtx, 'Pitch');
var yawChart = createSingleVariableChart(yawCtx, 'Yaw');
var pAngularVelocityChart = createSingleVariableChart(pAngularVelocityCtx, 'P Angular Velocity');
var qAngularVelocityChart = createSingleVariableChart(qAngularVelocityCtx, 'Q Angular Velocity');
var rAngularVelocityChart = createSingleVariableChart(rAngularVelocityCtx, 'R Angular Velocity');

// Create actuator charts container
var actuatorCharts = {};

function createActuatorChart(actuatorName, topicName) {
  var canvas = document.createElement('canvas');
  canvas.id = actuatorName + 'Chart';
  document.getElementById('actuatorCharts').appendChild(canvas);
  
  var ctx = canvas.getContext('2d');
  actuatorCharts[actuatorName] = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: []
    },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      scales: { 
        x: { display: false },
        y: {
          beginAtZero: true,
          ticks: {
            maxTicksLimit: 5
          }
        }
      },
      plugins: {
        legend: {
          display: true,
          position: 'top'
        }
      }
    }
  });
}

function updateActuatorChart(actuatorName, topicName, value) {
  if (!actuatorCharts[actuatorName]) {
    createActuatorChart(actuatorName, topicName);
  }
  
  var chart = actuatorCharts[actuatorName];
  var datasetIndex = chart.data.datasets.findIndex(ds => ds.label === topicName);
  
  if (datasetIndex === -1) {
    // Generate a random color for the new dataset
    var color = '#' + Math.floor(Math.random()*16777215).toString(16);
    chart.data.datasets.push({
      label: topicName,
      data: [],
      borderColor: color,
      fill: false
    });
    datasetIndex = chart.data.datasets.length - 1;
  }
  
  chart.data.labels.push(sampleIndex);
  chart.data.datasets[datasetIndex].data.push(value);
  
  if (chart.data.labels.length > 100) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds => ds.data.shift());
  }
  chart.update();
}

var sampleIndex = 0;

function updatePathChart(topicName, x, y) {
  var datasetIndex = pathChart.data.datasets.findIndex(ds => ds.label === topicName);
  
  if (datasetIndex === -1) {
    // Generate a random color for the new dataset
    var color = '#' + Math.floor(Math.random()*16777215).toString(16);
    pathChart.data.datasets.push({
      label: topicName,
      data: [],
      borderColor: color,
      backgroundColor: color,
      showLine: true,
      fill: false
    });
    datasetIndex = pathChart.data.datasets.length - 1;
  }
  
  pathChart.data.datasets[datasetIndex].data.push({x: x, y: y});
  if (pathChart.data.datasets[datasetIndex].data.length > 100) {
    pathChart.data.datasets[datasetIndex].data.shift();
  }
  pathChart.update();
}

function updateSingleVariableChart(chart, topicName, value) {
  var datasetIndex = chart.data.datasets.findIndex(ds => ds.label === topicName);
  
  if (datasetIndex === -1) {
    // Generate a random color for the new dataset
    var color = '#' + Math.floor(Math.random()*16777215).toString(16);
    chart.data.datasets.push({
      label: topicName,
      data: [],
      borderColor: color,
      fill: false
    });
    datasetIndex = chart.data.datasets.length - 1;
  }
  
  chart.data.labels.push(sampleIndex);
  chart.data.datasets[datasetIndex].data.push(value);
  
  if (chart.data.labels.length > 100) {
    chart.data.labels.shift();
    chart.data.datasets.forEach(ds => ds.data.shift());
  }
  chart.update();
}

function processOdomMessage(topicName, message) {
  sampleIndex++;
  
  // Extract data from message
  var x = message.pose.pose.position.x;
  var y = message.pose.pose.position.y;
  var z = message.pose.pose.position.z;
  var u = message.twist.twist.linear.x;
  var v = message.twist.twist.linear.y;
  var w = message.twist.twist.linear.z;
  var p = message.twist.twist.angular.x;
  var q = message.twist.twist.angular.y;
  var r = message.twist.twist.angular.z;
  var euler = quaternionToEuler(message.pose.pose.orientation);
  
  // Update all charts
  updatePathChart(topicName, x, y);
  updateSingleVariableChart(xPositionChart, topicName, x);
  updateSingleVariableChart(yPositionChart, topicName, y);
  updateSingleVariableChart(zPositionChart, topicName, z);
  updateSingleVariableChart(uVelocityChart, topicName, u);
  updateSingleVariableChart(vVelocityChart, topicName, v);
  updateSingleVariableChart(wVelocityChart, topicName, w);
  updateSingleVariableChart(rollChart, topicName, euler.roll);
  updateSingleVariableChart(pitchChart, topicName, euler.pitch);
  updateSingleVariableChart(yawChart, topicName, euler.yaw);
  updateSingleVariableChart(pAngularVelocityChart, topicName, p);
  updateSingleVariableChart(qAngularVelocityChart, topicName, q);
  updateSingleVariableChart(rAngularVelocityChart, topicName, r);
}

function processActuatorMessage(topicName, message) {
  for (var i = 0; i < message.actuator_names.length; i++) {
    updateActuatorChart(message.actuator_names[i], topicName, message.actuator_values[i]);
  }
}
