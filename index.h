#ifndef INDEX_H
#define INDEX_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>DTrac Rotor Setting</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
    background: #f5f5f5;
    padding: 20px;
    color: #333;
  }
  .container {
    max-width: 480px;
    margin: 0 auto;
    background: #fff;
    border-radius: 12px;
    padding: 24px;
    box-shadow: 0 2px 12px rgba(0,0,0,0.08);
  }
  h1 {
    text-align: center;
    color: #1a237e;
    font-size: 22px;
    margin-bottom: 24px;
    font-weight: 600;
  }
  .section {
    margin-bottom: 20px;
  }
  .section-title {
    font-size: 14px;
    color: #666;
    margin-bottom: 12px;
    font-weight: 500;
  }
  .row {
    display: flex;
    align-items: center;
    margin-bottom: 10px;
    min-height: 36px;
  }
  .row label {
    width: 100px;
    font-size: 13px;
    color: #555;
    text-align: right;
    padding-right: 12px;
    flex-shrink: 0;
  }
  .row input[type="text"],
  .row input[type="password"],
  .row select {
    flex: 1;
    padding: 8px 10px;
    border: 1px solid #ddd;
    border-radius: 6px;
    font-size: 13px;
    outline: none;
    transition: border-color 0.2s;
  }
  .row input:focus, .row select:focus {
    border-color: #4CAF50;
  }
  .row input[readonly] {
    background: #f0f0f0;
    color: #666;
  }
  .radio-group {
    display: flex;
    gap: 16px;
    align-items: center;
  }
  .radio-group label {
    width: auto;
    display: flex;
    align-items: center;
    gap: 4px;
    cursor: pointer;
    font-size: 13px;
  }
  .radio-group input[type="radio"] {
    width: 16px;
    height: 16px;
    accent-color: #4CAF50;
  }
  .slider-row {
    display: flex;
    align-items: center;
    margin-bottom: 10px;
  }
  .slider-row label {
    width: 60px;
    font-size: 13px;
    color: #555;
    text-align: right;
    padding-right: 12px;
  }
  .slider-row input[type="range"] {
    flex: 1;
    height: 6px;
    -webkit-appearance: none;
    appearance: none;
    background: #e0e0e0;
    border-radius: 3px;
    outline: none;
  }
  .slider-row input[type="range"]::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 18px;
    height: 18px;
    background: #666;
    border-radius: 50%;
    cursor: pointer;
  }
  .slider-row .value {
    width: 40px;
    text-align: center;
    font-size: 13px;
    color: #333;
    font-weight: 500;
  }
  .btn {
    padding: 10px 20px;
    border: none;
    border-radius: 6px;
    font-size: 14px;
    cursor: pointer;
    transition: opacity 0.2s;
  }
  .btn:hover { opacity: 0.9; }
  .btn-refresh {
    background: #607d8b;
    color: white;
    padding: 8px 16px;
    font-size: 13px;
  }
  .btn-submit {
    background: #4CAF50;
    color: white;
    width: 100%;
    padding: 14px;
    font-size: 16px;
    margin-top: 16px;
    border-radius: 8px;
  }
  .btn-submit:disabled {
    background: #ccc;
    cursor: not-allowed;
  }
  .status {
    font-size: 12px;
    color: #4CAF50;
    margin-top: 8px;
    text-align: center;
    min-height: 18px;
  }
  .status.error { color: #f44336; }
  .divider {
    height: 1px;
    background: #eee;
    margin: 16px 0;
  }
  .ap-list-row {
    display: flex;
    align-items: center;
    gap: 8px;
  }
  .ap-list-row select {
    flex: 1;
  }
  .ap-list-row .btn-refresh {
    flex-shrink: 0;
  }
  @media (max-width: 400px) {
    body { padding: 10px; }
    .container { padding: 16px; }
    .row label { width: 80px; font-size: 12px; }
  }
</style>
</head>
<body>
<div class="container">
  <h1>旋转器设置</h1>

  <!-- WiFi Status -->
  <div class="section">
    <div class="section-title">自身WiFi状态:</div>
    <div class="row">
      <label>热点IP:</label>
      <input type="text" id="staIp" readonly value="%STA_IP%">
    </div>
    <div class="row">
      <label>热点MAC地址:</label>
      <input type="text" id="staMac" readonly value="%STA_MAC%">
    </div>
  </div>

  <div class="divider"></div>

  <!-- External Sensor -->
  <div class="section">
    <div class="row">
      <label>外置传感器？</label>
      <div class="radio-group">
        <label><input type="radio" name="extSensor" value="0" %SENSOR_NO%> 不使用</label>
        <label><input type="radio" name="extSensor" value="1" %SENSOR_YES%> 使用</label>
      </div>
    </div>
  </div>

  <div class="divider"></div>

  <!-- Motor Gain -->
  <div class="section">
    <div class="section-title">电机增益:</div>
    <div class="slider-row">
      <label>azGain:</label>
      <input type="range" id="azGain" min="1" max="100" value="%AZ_GAIN%" oninput="updateSlider('azGain')">
      <span class="value" id="azGainVal">%AZ_GAIN%</span>
    </div>
    <div class="slider-row">
      <label>elGain:</label>
      <input type="range" id="elGain" min="1" max="100" value="%EL_GAIN%" oninput="updateSlider('elGain')">
      <span class="value" id="elGainVal">%EL_GAIN%</span>
    </div>
  </div>

  <div class="divider"></div>


  <!-- STA Setting -->
  <div class="section">
    <div class="section-title">接入点设置:</div>
    <div class="row">
      <label>热点名称:</label>
      <input type="text" id="staSsid" value="%STA_SSID%" placeholder="WiFi名称">
    </div>
    <div class="row">
      <label>附近的WiFi:</label>
      <div class="ap-list-row">
        <select id="apList" onchange="selectAP()">
          <option value="">-- 选择网络 --</option>
          %AP_OPTIONS%
        </select>
        <button class="btn btn-refresh" onclick="scanAP()" id="scanBtn">刷新</button>
      </div>
    </div>
    <div class="row">
      <label>WiFi密码:</label>
      <input type="password" id="staPass" value="%STA_PASS%" placeholder="密码">
    </div>
  </div>

  <button class="btn btn-submit" onclick="submitConfig()" id="submitBtn">保存</button>
  <div style="text-align:center;margin-top:16px;font-size:12px;color:#999;">
    固件版本: %FIRMWARE_VERSION%
  </div>
  <div class="status" id="status"></div>
</div>

<script>
function updateSlider(id) {
  document.getElementById(id + 'Val').textContent = document.getElementById(id).value;
}

function selectAP() {
  var sel = document.getElementById('apList');
  if (sel.value) {
    document.getElementById('staSsid').value = sel.value;
  }
}

function scanAP() {
  var btn = document.getElementById('scanBtn');
  btn.disabled = true;
  btn.textContent = '扫描中...';
  fetch('/scan')
    .then(r => r.json())
    .then(data => {
      var sel = document.getElementById('apList');
      sel.innerHTML = '<option value="">-- 选择网络 --</option>';
      data.aps.forEach(function(ap) {
        var opt = document.createElement('option');
        opt.value = ap.ssid;
        opt.textContent = ap.ssid + ' (' + ap.rssi + 'dBm)';
        sel.appendChild(opt);
      });
      showStatus('扫描结束，找到' + data.aps.length + '个热点');
    })
    .catch(e => showStatus('扫描失败: ' + e, true))
    .finally(() => {
      btn.disabled = false;
      btn.textContent = '刷新';
    });
}

function submitConfig() {
  var btn = document.getElementById('submitBtn');
  btn.disabled = true;
  btn.textContent = '保存中...';

  var extSensor = document.querySelector('input[name="extSensor"]:checked').value;
  var data = {
    extSensor: parseInt(extSensor),
    azGain: parseInt(document.getElementById('azGain').value),
    elGain: parseInt(document.getElementById('elGain').value),

    staSsid: document.getElementById('staSsid').value,
    staPass: document.getElementById('staPass').value
  };

  fetch('/config', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(data)
  })
  .then(r => r.text())
  .then(t => {
    showStatus('配置已保存，正在重启...');
    setTimeout(() => location.reload(), 3000);
  })
  .catch(e => {
    showStatus('保存失败: ' + e, true);
    btn.disabled = false;
    btn.textContent = 'Submit';
  });
}

function showStatus(msg, isError) {
  var el = document.getElementById('status');
  el.textContent = msg;
  el.className = isError ? 'status error' : 'status';
}

// 实时更新仪表盘数据
setInterval(function() {
  fetch('/AzEl')
    .then(r => r.text())
    .catch(() => {});
}, 1000);
</script>
</body>
</html>
)rawliteral";

#endif
