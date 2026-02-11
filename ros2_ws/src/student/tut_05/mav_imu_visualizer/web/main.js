// Connect to rosbridge websocket (adjust the URL as needed)
var ros = new ROSLIB.Ros({
    url: 'ws://192.168.0.30:9090'
  });
  
  ros.on('connection', function () {
    console.log('Connected to rosbridge.');
  });
  ros.on('error', function (error) {
    console.error('Error connecting to rosbridge: ', error);
  });
  
  // Create a subscriber to the /imu/data topic
  var imuTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/imu/data',
    messageType: 'sensor_msgs/Imu'
  });
  
  // Helper: convert quaternion to Euler angles
  function quaternionToEuler(q) {
    var x = q.x, y = q.y, z = q.z, w = q.w;
    var sinr_cosp = 2 * (w * x + y * z);
    var cosr_cosp = 1 - 2 * (x * x + y * y);
    var roll = Math.atan2(sinr_cosp, cosr_cosp) * 180 / Math.PI;
  
    var sinp = 2 * (w * y - z * x);
    var pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp) * 180 / Math.PI;
  
    var siny_cosp = 2 * (w * z + x * y);
    var cosy_cosp = 1 - 2 * (y * y + z * z);
    var yaw = Math.atan2(siny_cosp, cosy_cosp) * 180 / Math.PI;
    
    return { roll: roll, pitch: pitch, yaw: yaw };
  }
  
  // Set up Chart.js charts
  function createChart(ctx, label, borderColor) {
    return new Chart(ctx, {
      type: 'line',
      data: {
        labels: [], // time or sample index
        datasets: [{
          label: label,
          data: [],
          borderColor: borderColor,
          fill: false,
          tension: 0.1,
        }]
      },
      options: {
        animation: false,
        responsive: true,
        scales: { x: { display: false } }
      }
    });
  }
  
  // Get canvas contexts
  var accCtx = document.getElementById('accChart').getContext('2d');
  var gyroCtx = document.getElementById('gyroChart').getContext('2d');
  var orientationCtx = document.getElementById('rollChart').getContext('2d');
  
  // Create charts for each data type.
  // For acceleration and angular velocity, we use one chart per topic (each with three datasets for x, y, z).
  var accChart = new Chart(accCtx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Acc X', data: [], borderColor: 'red', fill: false },
        { label: 'Acc Y', data: [], borderColor: 'green', fill: false },
        { label: 'Acc Z', data: [], borderColor: 'blue', fill: false },
      ]
    },
    options: { animation: false, responsive: true, scales: { x: { display: false } } }
  });
  
  var gyroChart = new Chart(gyroCtx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Gyro X', data: [], borderColor: 'red', fill: false },
        { label: 'Gyro Y', data: [], borderColor: 'green', fill: false },
        { label: 'Gyro Z', data: [], borderColor: 'blue', fill: false },
      ]
    },
    options: { animation: false, responsive: true, scales: { x: { display: false } } }
  });
  
  // Create one chart for all orientation angles
  var orientationChart = new Chart(orientationCtx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'Roll', data: [], borderColor: 'purple', fill: false },
        { label: 'Pitch', data: [], borderColor: 'orange', fill: false },
        { label: 'Yaw', data: [], borderColor: 'brown', fill: false },
      ]
    },
    options: { animation: false, responsive: true, scales: { x: { display: false } } }
  });
  
  var sampleIndex = 0;
  
  function updateAccChart(ax, ay, az) {
    accChart.data.labels.push(sampleIndex);
    accChart.data.datasets[0].data.push(ax);
    accChart.data.datasets[1].data.push(ay);
    accChart.data.datasets[2].data.push(az);
    if (accChart.data.labels.length > 100) {
      accChart.data.labels.shift();
      accChart.data.datasets.forEach(ds => ds.data.shift());
    }
    accChart.update();
  }
  
  function updateGyroChart(gx, gy, gz) {
    gyroChart.data.labels.push(sampleIndex);
    gyroChart.data.datasets[0].data.push(gx);
    gyroChart.data.datasets[1].data.push(gy);
    gyroChart.data.datasets[2].data.push(gz);
    if (gyroChart.data.labels.length > 100) {
      gyroChart.data.labels.shift();
      gyroChart.data.datasets.forEach(ds => ds.data.shift());
    }
    gyroChart.update();
  }

  function updateOrientationChart(roll, pitch, yaw) {
    orientationChart.data.labels.push(sampleIndex);
    orientationChart.data.datasets[0].data.push(roll);
    orientationChart.data.datasets[1].data.push(pitch);
    orientationChart.data.datasets[2].data.push(yaw);
    if (orientationChart.data.labels.length > 100) {
      orientationChart.data.labels.shift();
      orientationChart.data.datasets.forEach(ds => ds.data.shift());
    }
    orientationChart.update();
  }
  
  // Subscribe to the /imu/data topic
  imuTopic.subscribe(function (message) {
    // Increment our sample index for the x-axis
    sampleIndex++;
  
    // Extract accelerations and angular velocities
    var acc = message.linear_acceleration; // {x, y, z}
    var gyro = message.angular_velocity;     // {x, y, z}
    
    // Convert quaternion to Euler angles
    var euler = quaternionToEuler(message.orientation); // returns {roll, pitch, yaw}
    
    // Update the charts
    updateAccChart(acc.x, acc.y, acc.z);
    updateGyroChart(gyro.x, gyro.y, gyro.z);
    updateOrientationChart(euler.roll, euler.pitch, euler.yaw);
  });
