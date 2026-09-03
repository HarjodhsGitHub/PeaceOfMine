const GATEWAY_ID = 253;
const MAX_POSITION = 4095;
const BAUDS = [[1_000_000, 1], [57_600, 34]];
const HISTORY_SECONDS = 60;
const SERIES_COLORS = ['#54a6ff', '#7be0a8', '#ffbf69', '#e98cff', '#ff7786', '#55d6d6'];

let port, writer, reader, readerTask;
let buffer = [];
let servos = new Map();
let chosen = new Set();
let connected = false;
let connecting = false;
let busy = false;
let serialTail = Promise.resolve();
let failures = 0;
let chart;
let history = [];

const $ = id => document.getElementById(id);
const checksum = bytes => (255 - (bytes.reduce((sum, byte) => sum + byte, 0) % 256)) & 255;
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));

function packet(id, instruction, ...parameters) {
  const length = parameters.length + 2;
  return Uint8Array.from([255, 255, id, length, instruction, ...parameters, checksum([id, length, instruction, ...parameters])]);
}

function transact(id, instruction, parameters, timeout = 250) {
  const job = serialTail.then(() => sendNow(id, instruction, parameters, timeout));
  serialTail = job.catch(() => undefined);
  return job;
}

async function sendNow(id, instruction, parameters, timeout) {
  if (!writer) throw Error('Serial writer is not available');
  await writer.write(packet(id, instruction, ...parameters));
  return id === 254 ? null : receive(id, timeout);
}

async function receive(id, timeout) {
  const deadline = Date.now() + timeout;
  while (Date.now() < deadline) {
    while (buffer.length >= 6) {
      const start = buffer.findIndex((byte, index) => index < buffer.length - 1 && byte === 255 && buffer[index + 1] === 255);
      if (start < 0) { buffer = []; break; }
      if (start > 0) buffer.splice(0, start);
      const frameLength = buffer[3] + 4;
      if (buffer.length < frameLength) break;
      const frame = buffer.splice(0, frameLength);
      if (frame[2] === id && checksum(frame.slice(2, -1)) === frame.at(-1)) return frame.slice(5, -1);
    }
    await sleep(2);
  }
  throw Error(`ID ${id} did not reply within ${timeout} ms`);
}

const read = (id, address, length, timeout) => transact(id, 2, [address, length], timeout);
const write = (id, address, ...values) => transact(id, 3, [address, ...values]);
const baudValue = baud => BAUDS.find(([rate]) => rate === baud)?.[1];

function setStatus(message, isBusy = false) {
  $('status').textContent = message;
  $('status').classList.toggle('busy', isBusy);
}

function setCommandBusy(isBusy) {
  busy = isBusy;
  document.querySelectorAll('[data-action], #estop, #position, #speed').forEach(control => control.disabled = isBusy || !connected);
}

async function gatewayReady() {
  setStatus('Waiting for ArbotiX ROS firmware…', true);
  for (let attempt = 0; attempt < 60; attempt++) {
    try { if ((await read(GATEWAY_ID, 0, 1, 50))[0] === 44) return; }
    catch (_) { /* The gateway needs roughly 1.7 seconds after reset before it replies. */ }
    await sleep(50);
  }
  throw Error('ArbotiX ROS firmware did not respond at 115200 baud');
}

async function discover() {
  servos.clear(); history = []; destroyChart();
  for (const [baud, value] of BAUDS) {
    setStatus(`Scanning IDs at ${baud.toLocaleString()} baud…`, true);
    await write(GATEWAY_ID, 4, value);
    for (let id = 0; id < 253; id++) {
      if (id % 16 === 0) setStatus(`Scanning ${baud.toLocaleString()} baud — ID ${id}/252…`, true);
      try { if ((await read(id, 3, 1, 20))[0] === id) servos.set(id, {id, baud}); }
      catch (_) { /* A non-reply is the expected result for an unused ID. */ }
    }
  }
  if (!servos.size) throw Error('No Protocol 1.0 servos replied at 1,000,000 or 57,600 baud');
  setStatus('Reading servo configuration…', true);
  for (const servo of servos.values()) {
    await write(GATEWAY_ID, 4, baudValue(servo.baud));
    const [limits, position, speed] = await Promise.all([read(servo.id, 6, 4), read(servo.id, 36, 2), read(servo.id, 32, 2)]);
    servo.low = limits[0] + 256 * limits[1];
    servo.high = limits[2] + 256 * limits[3];
    servo.wheel = servo.low === 0 && servo.high === 0;
    servo.pos = position[0] + 256 * position[1];
    servo.speed = (speed[0] + 256 * speed[1]) & 1023;
    servo.load = 0;
  }
  chosen = new Set(servos.keys());
  render();
  setCommandBusy(false);
  setStatus(`Connected — ${servos.size} servo${servos.size === 1 ? '' : 's'} found`);
}

async function eachServo(callback) {
  for (const id of chosen) {
    const servo = servos.get(id);
    if (!servo) continue;
    await write(GATEWAY_ID, 4, baudValue(servo.baud));
    await callback(servo);
  }
}

async function setMode(wheel) {
  await eachServo(async servo => {
    await write(servo.id, 24, 0);
    await write(servo.id, 6, 0, 0);
    await write(servo.id, 8, wheel ? 0 : 255, wheel ? 0 : 15);
    servo.wheel = wheel; servo.torque = false;
  });
}

async function runAction(action) {
  if (!connected || busy) return;
  setCommandBusy(true);
  try {
    setStatus('Sending command…', true);
    if (action === 'joint') await setMode(false);
    if (action === 'wheel') await setMode(true);
    if (action === 'torque') await eachServo(servo => write(servo.id, 24, servo.torque ? 0 : 1));
    if (action === 'led') await eachServo(servo => write(servo.id, 25, servo.led ? 0 : 1));
    if (action === 'jog-' || action === 'jog+') await eachServo(async servo => {
      if (!servo.wheel) {
        const target = Math.max(servo.low, Math.min(servo.high, servo.pos + (action === 'jog-' ? -114 : 114)));
        await write(servo.id, 30, target & 255, target >> 8); await write(servo.id, 24, 1);
      }
    });
    if (['ccw', 'cw', 'stop'].includes(action)) await eachServo(servo => {
      if (!servo.wheel) return;
      const speed = action === 'stop' ? 0 : servo.speed | (action === 'ccw' ? 1024 : 0);
      return write(servo.id, 32, speed & 255, speed >> 8);
    });
    await poll(true);
    setStatus('Command sent — telemetry updated');
  } catch (error) { setStatus(`Command failed: ${error.message}`); }
  finally { setCommandBusy(false); }
}

function decodeLoad(bytes) {
  const raw = bytes[0] + 256 * bytes[1];
  const magnitude = raw & 1023;
  return raw & 1024 ? -magnitude : magnitude;
}
const loadPercent = load => load / 10.23;

async function poll(allowBusy = false) {
  if (!connected || (busy && !allowBusy)) return;
  try {
    await eachServo(async servo => {
      const [position, voltage, temperature, load, torque, led] = await Promise.all([
        read(servo.id, 36, 2), read(servo.id, 42, 1), read(servo.id, 43, 1),
        read(servo.id, 40, 2), read(servo.id, 24, 1), read(servo.id, 25, 1),
      ]);
      servo.pos = position[0] + 256 * position[1]; servo.voltage = voltage[0] / 10;
      servo.temp = temperature[0]; servo.load = decodeLoad(load);
      servo.torque = torque[0] > 0; servo.led = led[0] > 0;
    });
    failures = 0; recordHistory(); render();
  } catch (error) {
    failures++;
    setStatus(`Telemetry retry ${failures}/3: ${error.message}`, true);
    if (failures >= 3) { connected = false; setCommandBusy(false); setStatus('Connection unavailable — click Connect ArbotiX to retry'); }
  }
}

function recordHistory() {
  const timestamp = Date.now() / 1000;
  const loads = {};
  for (const servo of servos.values()) loads[servo.id] = loadPercent(servo.load);
  history.push({timestamp, loads});
  history = history.filter(sample => sample.timestamp >= timestamp - HISTORY_SECONDS);
}

function destroyChart() { if (chart) chart.destroy(); chart = undefined; }

function renderChart() {
  const container = $('load-chart');
  const ids = [...chosen].sort((a, b) => a - b);
  if (!ids.length || !history.length) {
    destroyChart(); container.innerHTML = '<p class="chart-empty">Select a servo to plot its signed load history.</p>'; return;
  }
  const data = [history.map(sample => sample.timestamp), ...ids.map(id => history.map(sample => sample.loads[id] ?? null))];
  const series = [{label: 'Time'}, ...ids.map((id, index) => ({label: `ID ${id}`, stroke: SERIES_COLORS[index % SERIES_COLORS.length], width: 2, points: {show: false}}))];
  const width = Math.max(container.clientWidth, 260);
  if (!chart || chart.series.length !== series.length) {
    destroyChart(); container.innerHTML = '';
    chart = new uPlot({
      width, height: 260, series, scales: {x: {time: true}, y: {range: [-100, 100]}},
      axes: [{stroke: '#9eafc7', grid: {stroke: '#2b3b53'}}, {label: 'Signed load estimate (%)', stroke: '#9eafc7', grid: {stroke: '#2b3b53'}}],
      legend: {show: true},
    }, data, container);
  } else { chart.setSize({width, height: 260}); chart.setData(data); }
}

function render() {
  const selected = [...chosen].map(id => servos.get(id)).filter(Boolean);
  $('title').textContent = selected.length ? `Controlling ID ${selected.map(servo => servo.id).join(', ID ')}` : 'No servos selected';
  $('servos').innerHTML = [...servos.values()].map(servo => `<label class="servo ${chosen.has(servo.id) ? 'on' : ''}"><input type="checkbox" ${chosen.has(servo.id) ? 'checked' : ''} onchange="pick(${servo.id},this.checked)"><span><b>ID ${servo.id}</b><br><small>${servo.baud.toLocaleString()} baud · ${servo.wheel ? 'WHEEL' : 'JOINT'}</small></span></label>`).join('');
  $('telemetry').innerHTML = selected.map(servo => `<div class="metric"><span>ID ${servo.id}</span><b>${(servo.pos / MAX_POSITION * 360).toFixed(1)}°</b><small>${(servo.voltage ?? 0).toFixed(1)} V · ${servo.temp ?? 0}°C</small><strong class="${servo.load < 0 ? 'negative' : ''}">${servo.load >= 0 ? '+' : ''}${loadPercent(servo.load).toFixed(1)}% load</strong></div>`).join('');
  renderChart();
}

function pick(id, selected) { selected ? chosen.add(id) : chosen.delete(id); render(); }

async function disconnect(message = 'Disconnected') {
  connected = false; failures = 0;
  try {
    if (reader) await reader.cancel();
    if (readerTask) await readerTask.catch(() => undefined);
    if (reader) reader.releaseLock();
    if (writer) writer.releaseLock();
    if (port) await port.close();
  } catch (_) { /* Closing an already-disconnected Web Serial port can reject. */ }
  reader = writer = port = readerTask = undefined; buffer = [];
  $('connect').textContent = 'Connect ArbotiX'; setCommandBusy(false); setStatus(message);
}

async function emergencyStop() {
  if (!connected || busy) return;
  setCommandBusy(true);
  try {
    setStatus('Emergency stop — disabling torque…', true);
    for (const servo of servos.values()) {
      await write(GATEWAY_ID, 4, baudValue(servo.baud));
      if (servo.wheel) await write(servo.id, 32, 0, 0);
      await write(servo.id, 24, 0);
      servo.torque = false;
    }
    await poll(true); setStatus('Emergency stop sent — torque disabled');
  } catch (error) { setStatus(`Emergency stop failed: ${error.message}`); }
  finally { setCommandBusy(false); }
}

$('connect').onclick = async () => {
  if (connecting) return;
  if (connected) return disconnect();
  connecting = true; $('connect').disabled = true;
  try {
    setStatus('Choose the ArbotiX USB serial device…', true);
    port = await navigator.serial.requestPort({filters: [{usbVendorId: 0x0403}]});
    setStatus('Opening serial port…', true); await port.open({baudRate: 115200});
    writer = port.writable.getWriter(); reader = port.readable.getReader();
    readerTask = (async () => { while (true) { const {value, done} = await reader.read(); if (done) break; if (value) buffer.push(...value); } })();
    await gatewayReady(); connected = true; $('connect').textContent = 'Disconnect'; await discover();
  } catch (error) { await disconnect(error.message); }
  finally { connecting = false; $('connect').disabled = false; }
};

$('estop').onclick = emergencyStop;
document.querySelectorAll('[data-action]').forEach(button => button.onclick = () => runAction(button.dataset.action));
$('position').oninput = event => {
  if (!connected || busy) return;
  eachServo(async servo => { if (!servo.wheel) { const target = Math.round(servo.low + event.target.value / 1000 * (servo.high - servo.low)); await write(servo.id, 30, target & 255, target >> 8); } }).catch(error => setStatus(`Position command failed: ${error.message}`));
};
$('speed').oninput = event => {
  if (!connected || busy) return;
  const speed = Number(event.target.value);
  eachServo(servo => write(servo.id, 32, speed & 255, speed >> 8)).catch(error => setStatus(`Speed command failed: ${error.message}`));
};

new ResizeObserver(() => { if (chart) renderChart(); }).observe($('load-chart'));
setInterval(poll, 600);
