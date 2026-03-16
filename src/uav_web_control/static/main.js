const mapStates = {}
const goalDrafts = {}
let activeModalId = null
let modalMapState = null
const mapOffsets = {}
let teleopInterval = null
let teleopState = null
const manualMode = {}

function getInputEl(id, selector) {
  if (activeModalId === id) {
    const modalWrap = document.getElementById('uav-modal-controls')
    if (modalWrap) {
      const el = modalWrap.querySelector(selector)
      if (el) return el
    }
  }
  return document.querySelector(selector)
}

function getGoalFromInputs(id) {
  const latEl = getInputEl(id, `.geo-lat[data-id="${id}"]`)
  const lonEl = getInputEl(id, `.geo-lon[data-id="${id}"]`)
  const altEl = getInputEl(id, `.geo-alt-ft[data-id="${id}"]`)
  if (!latEl || !lonEl) return null
  const lat = parseFloat(latEl.value)
  const lon = parseFloat(lonEl.value)
  if (isNaN(lat) || isNaN(lon)) return null
  const goal = { lat: lat, lon: lon }
  if (altEl) {
    const altFt = parseFloat(altEl.value)
    if (!isNaN(altFt)) {
      goal.alt_ft = altFt
    } else {
      goal.alt_ft = 20.0
      altEl.value = '20'
    }
  }
  return goal
}

function updateGoalMarker(id, goal) {
  const state = mapStates[id]
  if (!state || !goal) return
  const ll = applyOffset(id, goal.lat, goal.lon)
  if (state.goalMarker) {
    state.goalMarker.setLatLng(ll)
  } else {
    state.goalMarker = L.circleMarker(ll, { radius: 6, color: '#f97316', fillColor: '#f97316', fillOpacity: 0.8 }).addTo(state.map)
  }
  updateModalGoalMarker(id, goal)
}

function updateModalGoalMarker(id, goal) {
  if (activeModalId !== id || !modalMapState || !goal) return
  const ll = applyOffset(id, goal.lat, goal.lon)
  if (modalMapState.goalMarker) {
    modalMapState.goalMarker.setLatLng(ll)
  } else {
    modalMapState.goalMarker = L.circleMarker(ll, { radius: 7, color: '#f97316', fillColor: '#f97316', fillOpacity: 0.85 }).addTo(modalMapState.map)
  }
}

function updateModalCurrentMarker(id, lat, lon) {
  if (activeModalId !== id || !modalMapState) return
  const ll = applyOffset(id, lat, lon)
  if (!modalMapState.currentMarker) {
    modalMapState.currentMarker = L.circleMarker(ll, { radius: 6, color: '#38bdf8', fillColor: '#38bdf8', fillOpacity: 0.9 }).addTo(modalMapState.map)
    modalMapState.map.setView(ll, 16)
  } else {
    modalMapState.currentMarker.setLatLng(ll)
  }
}

function sendFly(id, goal) {
  const statusEl = document.getElementById('status-' + id)
  statusEl.textContent = 'sending...'
  const body = { id, lat: goal.lat, lon: goal.lon }
  if (goal.alt_ft !== undefined) {
    body.alt_ft = goal.alt_ft
    body.alt_frame = 'agl'
  }

  fetch('/fly', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'command sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}

function updateDraftFromInputs(id) {
  const goal = getGoalFromInputs(id)
  if (!goal) return
  goalDrafts[id] = goal
  updateGoalMarker(id, goal)
}

document.addEventListener('click', function (e) {
  if (!e.target.matches('.draft-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('status-' + id)
  const goal = getGoalFromInputs(id)
  if (!goal) {
    statusEl.textContent = 'enter lat/lon'
    return
  }
  goalDrafts[id] = goal
  updateGoalMarker(id, goal)
  statusEl.textContent = 'goal drafted'
})

document.addEventListener('click', function (e) {
  if (!e.target.matches('.fly-btn')) return
  const id = e.target.dataset.id
  const inputGoal = getGoalFromInputs(id)
  const goal = inputGoal || goalDrafts[id]
  const statusEl = document.getElementById('status-' + id)
  if (!goal) {
    statusEl.textContent = 'enter lat/lon'
    return
  }
  if (inputGoal) {
    goalDrafts[id] = inputGoal
    updateGoalMarker(id, inputGoal)
  }
  sendFly(id, goal)
})

document.addEventListener('click', function (e) {
  if (!e.target.matches('.max-btn')) return
  const id = e.target.dataset.id
  openModalFor(id)
})

document.addEventListener('click', function (e) {
  if (!e.target.matches('.arm-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('arm-status-' + id)
  statusEl.textContent = 'sending offboard...'
  fetch('/arm', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'offboard command sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}
)

document.addEventListener('click', function (e) {
  if (!e.target.matches('.disarm-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('arm-status-' + id)
  statusEl.textContent = 'sending disarm...'
  fetch('/disarm', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'disarm command sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}
)

document.addEventListener('click', function (e) {
  if (!e.target.matches('.force-disarm-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('arm-status-' + id)
  statusEl.textContent = 'sending force disarm...'
  fetch('/force_disarm', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'force disarm sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}
)

document.addEventListener('click', function (e) {
  if (!e.target.matches('.home-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('status-' + id)
  statusEl.textContent = 'sending home...'
  fetch('/home', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'home command sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}
)

document.addEventListener('click', function (e) {
  if (!e.target.matches('.land-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('land-status-' + id)
  statusEl.textContent = 'sending land...'
  fetch('/land', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'land command sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}
)


// Poll status endpoint and update UI: disable buttons when drone not connected
function updateStatusUI(statusData) {
  Object.keys(statusData).forEach(key => {
    const info = statusData[key]
    const id = key // e.g. 'uav1'
    const connected = info.connected
    const flyBtn = document.querySelector(`.fly-btn[data-id="${id}"]`)
    const armBtn = document.querySelector(`.arm-btn[data-id="${id}"]`)
    const draftBtn = document.querySelector(`.draft-btn[data-id="${id}"]`)
    const disarmBtn = document.querySelector(`.disarm-btn[data-id="${id}"]`)
    const forceDisarmBtn = document.querySelector(`.force-disarm-btn[data-id="${id}"]`)
    const homeBtn = document.querySelector(`.home-btn[data-id="${id}"]`)
    const statusEl = document.getElementById('status-' + id)
    const armStatusEl = document.getElementById('arm-status-' + id)
    const goalStatusEl = document.getElementById('goal-status-' + id)
    // Disarm UI removed: keep original behavior for arm/status
    if (flyBtn) flyBtn.disabled = !connected
    if (armBtn) armBtn.disabled = !connected
    if (draftBtn) draftBtn.disabled = !connected
    if (disarmBtn) disarmBtn.disabled = !connected
    if (forceDisarmBtn) forceDisarmBtn.disabled = !connected
    if (homeBtn) homeBtn.disabled = !connected
    if (statusEl && !goalDrafts[id]) statusEl.textContent = connected ? 'connected' : 'no node'
    if (armStatusEl) armStatusEl.textContent = info.armed ? 'armed' : (connected ? 'idle' : 'no node')
    if (armStatusEl && !info.armed && connected) armStatusEl.textContent = 'disarmed'

    if (info.lat !== undefined && info.lon !== undefined) {
      ensureMap(id, info.lat, info.lon)
    }
    if (goalStatusEl) {
      updateGoalStatus(id, info, goalStatusEl)
    }
    if (activeModalId === id) {
      renderModalStatus(id, info)
    }
  })
}

function pollStatus() {
  fetch('/status').then(r => r.json()).then(data => {
    if (data.status === 'ok' && data.data) {
      updateStatusUI(data.data)
    }
  }).catch(err => console.error('status poll error', err))
}

// start polling every 2 seconds
setInterval(pollStatus, 2000)
// run once immediately
pollStatus()

function ensureMap(id, lat, lon) {
  const state = mapStates[id]
  if (!state) return
  const ll = applyOffset(id, lat, lon)
  state.lastLatLon = ll
  if (!state.currentMarker) {
    state.currentMarker = L.circleMarker(ll, { radius: 5, color: '#38bdf8', fillColor: '#38bdf8', fillOpacity: 0.9 }).addTo(state.map)
    state.map.setView(ll, 16)
  } else {
    state.currentMarker.setLatLng(ll)
  }
  updateModalCurrentMarker(id, lat, lon)
}

function initMaps() {
  const maps = document.querySelectorAll('.gps-map')
  maps.forEach(el => {
    const id = el.dataset.id
    if (!id || mapStates[id]) return
    const map = L.map(el, { zoomControl: false, attributionControl: false }).setView([0, 0], 2)
    L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 19
    }).addTo(map)
    map.on('click', function (ev) {
      const lat = ev.latlng.lat
      const lon = ev.latlng.lng
      const latEl = document.querySelector(`.geo-lat[data-id="${id}"]`)
      const lonEl = document.querySelector(`.geo-lon[data-id="${id}"]`)
      const [rawLat, rawLon] = removeOffset(id, lat, lon)
      if (latEl) latEl.value = rawLat.toFixed(6)
      if (lonEl) lonEl.value = rawLon.toFixed(6)
      const goal = getGoalFromInputs(id)
      if (goal) {
        goalDrafts[id] = goal
        updateGoalMarker(id, goal)
        const statusEl = document.getElementById('status-' + id)
        if (statusEl) statusEl.textContent = 'goal drafted (map)'
      }
    })
    mapStates[id] = { map: map, currentMarker: null, goalMarker: null, lastLatLon: null }
  })
}

document.addEventListener('DOMContentLoaded', function () {
  initMaps()
})

document.addEventListener('input', function (e) {
  if (!e.target.matches('.geo-lat, .geo-lon, .geo-alt-ft')) return
  const id = e.target.dataset.id
  if (!id) return
  updateDraftFromInputs(id)
})

function openModalFor(id) {
  const overlay = document.getElementById('uav-modal')
  const title = document.getElementById('uav-modal-title')
  const mapEl = document.getElementById('uav-modal-map')
  const controlsEl = document.getElementById('uav-modal-controls')
  const teleopFooter = document.getElementById('uav-teleop-footer')
  if (!overlay || !title || !mapEl || !controlsEl) return

  activeModalId = id
  title.textContent = `UAV ${id}`
  overlay.style.display = 'flex'

  controlsEl.innerHTML = ''
  const srcControls = document.querySelector(`#card-${id} .gps-controls`)
  if (srcControls) {
    const clone = srcControls.cloneNode(true)
    controlsEl.appendChild(clone)
    const srcLat = document.querySelector(`#card-${id} .geo-lat[data-id="${id}"]`)
    const srcLon = document.querySelector(`#card-${id} .geo-lon[data-id="${id}"]`)
    const srcAlt = document.querySelector(`#card-${id} .geo-alt-ft[data-id="${id}"]`)
    const dstLat = controlsEl.querySelector(`.geo-lat[data-id="${id}"]`)
    const dstLon = controlsEl.querySelector(`.geo-lon[data-id="${id}"]`)
    const dstAlt = controlsEl.querySelector(`.geo-alt-ft[data-id="${id}"]`)
    if (srcLat && dstLat) dstLat.value = srcLat.value
    if (srcLon && dstLon) dstLon.value = srcLon.value
    if (srcAlt && dstAlt) dstAlt.value = srcAlt.value
  }
  const statusPanel = document.createElement('div')
  statusPanel.id = 'uav-modal-status'
  statusPanel.className = 'status-panel'
  statusPanel.textContent = 'status loading...'
  controlsEl.appendChild(statusPanel)

  const teleopWrap = document.createElement('div')
  teleopWrap.id = 'uav-teleop'
  teleopWrap.className = 'teleop'
  teleopWrap.innerHTML = `
    <div class="joy-area" id="joy-area"><div class="joy-knob" id="joy-knob"></div></div>
    <div class="teleop-meta">
      <label>Speed (m/s)
        <input type="range" id="teleop-speed" min="0.5" max="5" step="0.5" value="2">
      </label>
      <label>Altitude (ft)
        <input type="range" id="teleop-alt-ft" min="10" max="40" step="0.5" value="15">
      </label>
      <label class="toggle">
        <input type="checkbox" id="teleop-manual">
        Manual Mode
      </label>
      <div id="teleop-readout">x:0 y:0 alt_ft:15</div>
    </div>
  `
  if (teleopFooter) {
    teleopFooter.innerHTML = ''
    teleopFooter.appendChild(teleopWrap)
  } else {
    controlsEl.appendChild(teleopWrap)
  }
  setupTeleop(id)

  if (modalMapState && modalMapState.map) {
    modalMapState.map.remove()
  }
  mapEl.innerHTML = ''
  const map = L.map(mapEl, { zoomControl: true, attributionControl: false }).setView([0, 0], 2)
  L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', { maxZoom: 19 }).addTo(map)
  modalMapState = { map: map, currentMarker: null, goalMarker: null }
  updateCalibrateStatus(id)

  const state = mapStates[id]
  if (state && state.lastLatLon) {
    updateModalCurrentMarker(id, state.lastLatLon[0], state.lastLatLon[1])
  }
  if (goalDrafts[id]) {
    updateModalGoalMarker(id, goalDrafts[id])
  }

  map.on('click', function (ev) {
    const lat = ev.latlng.lat
    const lon = ev.latlng.lng
    const latEl = controlsEl.querySelector(`.geo-lat[data-id="${id}"]`)
    const lonEl = controlsEl.querySelector(`.geo-lon[data-id="${id}"]`)
    const [rawLat, rawLon] = removeOffset(id, lat, lon)
    if (latEl) latEl.value = rawLat.toFixed(6)
    if (lonEl) lonEl.value = rawLon.toFixed(6)
    const goal = getGoalFromInputs(id)
    if (goal) {
      goalDrafts[id] = goal
      updateModalGoalMarker(id, goal)
      const statusEl = document.getElementById('status-' + id)
      if (statusEl) statusEl.textContent = 'goal drafted (map)'
    }
  })
}

document.addEventListener('click', function (e) {
  if (e.target && e.target.id === 'uav-modal-close') {
    closeModal()
  }
  if (e.target && e.target.id === 'uav-modal') {
    closeModal()
  }
})

function closeModal() {
  const overlay = document.getElementById('uav-modal')
  if (overlay) overlay.style.display = 'none'
  if (activeModalId) {
    const id = activeModalId
    const modalWrap = document.getElementById('uav-modal-controls')
    if (modalWrap) {
      const srcLat = modalWrap.querySelector(`.geo-lat[data-id="${id}"]`)
      const srcLon = modalWrap.querySelector(`.geo-lon[data-id="${id}"]`)
      const srcAlt = modalWrap.querySelector(`.geo-alt-ft[data-id="${id}"]`)
      const dstLat = document.querySelector(`#card-${id} .geo-lat[data-id="${id}"]`)
      const dstLon = document.querySelector(`#card-${id} .geo-lon[data-id="${id}"]`)
      const dstAlt = document.querySelector(`#card-${id} .geo-alt-ft[data-id="${id}"]`)
      if (srcLat && dstLat) dstLat.value = srcLat.value
      if (srcLon && dstLon) dstLon.value = srcLon.value
      if (srcAlt && dstAlt) dstAlt.value = srcAlt.value
    }
  }
  if (modalMapState && modalMapState.map) {
    modalMapState.map.remove()
  }
  modalMapState = null
  activeModalId = null
  stopTeleop()
}

function setupTeleop(id) {
  const area = document.getElementById('joy-area')
  const knob = document.getElementById('joy-knob')
  const speedEl = document.getElementById('teleop-speed')
  const altEl = document.getElementById('teleop-alt-ft')
  const readout = document.getElementById('teleop-readout')
  const manualEl = document.getElementById('teleop-manual')
  if (!area || !knob || !speedEl || !altEl || !readout) return
  if (manualEl) {
    manualEl.checked = !!manualMode[id]
    manualEl.onchange = () => {
      manualMode[id] = !!manualEl.checked
      if (manualMode[id]) {
        const altFt = parseFloat(altEl.value || '10')
        teleopState = { x: 0, y: 0, alt_ft: altFt, speed: parseFloat(speedEl.value || '2') }
        updateReadout(0, 0, altFt)
        start()
      } else {
        stopTeleop()
      }
      fetch('/manual_mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ id, enabled: manualMode[id] })
      }).catch(() => {})
    }
  }

  const updateReadout = (x, y, altFt) => {
    readout.textContent = `x:${x.toFixed(2)} y:${y.toFixed(2)} alt_ft:${altFt.toFixed(1)}`
  }

  const send = () => {
    if (!teleopState) return
    sendTeleop(id, teleopState.x, teleopState.y, teleopState.alt_ft, teleopState.speed)
  }

  const start = () => {
    if (teleopInterval) return
    teleopInterval = setInterval(send, 200)
  }

  const stop = () => {
    const altFt = parseFloat(altEl.value || '15')
    teleopState = { x: 0, y: 0, alt_ft: altFt, speed: parseFloat(speedEl.value || '2'), drag: { active: false } }
    updateReadout(0, 0, altFt)
    if (knob) {
      knob.style.left = '50%'
      knob.style.top = '50%'
    }
    start()
    sendTeleop(id, 0, 0, altFt, parseFloat(speedEl.value || '2'))
  }

  const handleMove = (ev) => {
    if (!teleopState || !teleopState.drag || !teleopState.drag.active) return
    const rect = area.getBoundingClientRect()
    const cx = rect.left + rect.width / 2
    const cy = rect.top + rect.height / 2
    const max = rect.width / 2 - 12
    const ox = ev.clientX - cx
    const oy = ev.clientY - cy
    const clampedX = Math.max(-max, Math.min(max, ox))
    const clampedY = Math.max(-max, Math.min(max, oy))
    const nx = clampedX / max
    const ny = clampedY / max
    // joystick: up is negative screen y -> positive north (x)
    const x = -ny
    const y = nx
    const altFt = parseFloat(altEl.value || '15')
    const speed = parseFloat(speedEl.value || '2')
    teleopState = { x, y, alt_ft: altFt, speed, drag: { active: true } }
    if (!manualMode[id]) {
      if (manualEl) manualEl.checked = true
      manualMode[id] = true
      fetch('/manual_mode', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ id, enabled: true })
      }).catch(() => {})
    }
    knob.style.left = `${50 + (nx * 40)}%`
    knob.style.top = `${50 + (ny * 40)}%`
    updateReadout(x, y, altFt)
    start()
  }

  area.onpointerdown = (ev) => {
    area.setPointerCapture(ev.pointerId)
    teleopState = teleopState || { x: 0, y: 0, alt_ft: parseFloat(altEl.value || '15'), speed: parseFloat(speedEl.value || '2'), drag: { active: false } }
    teleopState.drag.active = true
    handleMove(ev)
  }
  area.onpointermove = (ev) => {
    if (teleopState && teleopState.drag && teleopState.drag.active) handleMove(ev)
  }
  area.onpointerup = () => stop()
  area.onpointerleave = () => stop()
}

function stopTeleop() {
  if (teleopInterval) {
    clearInterval(teleopInterval)
    teleopInterval = null
  }
  teleopState = null
}

function sendTeleop(id, x, y, altFt, speed) {
  if (!manualMode[id]) return
  fetch('/teleop', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id, x, y, alt_ft: altFt, speed })
  }).catch(() => {})
}

function applyOffset(id, lat, lon) {
  const off = mapOffsets[id]
  if (!off) return [lat, lon]
  return [lat + off.lat, lon + off.lon]
}

function removeOffset(id, lat, lon) {
  const off = mapOffsets[id]
  if (!off) return [lat, lon]
  return [lat - off.lat, lon - off.lon]
}

function updateCalibrateStatus(id) {
  const el = document.getElementById('uav-calibrate-status')
  if (!el) return
  if (mapOffsets[id]) {
    el.textContent = `calibrated (${mapOffsets[id].lat.toFixed(6)}, ${mapOffsets[id].lon.toFixed(6)})`
  } else {
    el.textContent = 'not calibrated'
  }
}

function updateGoalStatus(id, info, el) {
  const goal = goalDrafts[id]
  if (!goal || info.lat === undefined || info.lon === undefined) {
    el.textContent = 'idle'
    el.classList.remove('good')
    return
  }
  const d = haversineMeters(info.lat, info.lon, goal.lat, goal.lon)
  if (d <= 3.0) {
    el.textContent = 'reached'
    el.classList.add('good')
  } else {
    el.textContent = 'flying'
    el.classList.remove('good')
  }
}

function haversineMeters(lat1, lon1, lat2, lon2) {
  const R = 6378137.0
  const toRad = v => v * Math.PI / 180
  const dLat = toRad(lat2 - lat1)
  const dLon = toRad(lon2 - lon1)
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2
  return 2 * R * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
}

function renderModalStatus(id, info) {
  const panel = document.getElementById('uav-modal-status')
  if (!panel || activeModalId !== id) return
  const vs = info.vehicle_status || {}
  const vcm = info.vehicle_control_mode || {}
  const batt = info.battery_status || {}
  const gps = info.vehicle_gps_position || {}
  const land = info.vehicle_land_detected || {}
  const takeoff = info.takeoff_status || {}
  const gpsLat = (gps.lat !== undefined) ? gps.lat : info.lat
  const gpsLon = (gps.lon !== undefined) ? gps.lon : info.lon
  const gpsAlt = (gps.alt !== undefined) ? gps.alt : info.alt
  const goal = goalDrafts[id]
  let goalLine = ['goal_status', '-']
  if (goal && info.lat !== undefined && info.lon !== undefined) {
    const d = haversineMeters(info.lat, info.lon, goal.lat, goal.lon)
    goalLine = ['goal_status', d <= 3.0 ? 'reached' : 'flying']
  }
  const lines = [
    goalLine,
    ['armed', info.armed],
    ['nav_state', vs.nav_state],
    ['arming_state', vs.arming_state],
    ['failsafe', vs.failsafe],
    ['pre_flight_checks_pass', vs.pre_flight_checks_pass],
    ['control_offboard', vcm.flag_control_offboard_enabled],
    ['control_auto', vcm.flag_control_auto_enabled],
    ['control_manual', vcm.flag_control_manual_enabled],
    ['battery_voltage_v', batt.voltage_v],
    ['battery_current_a', batt.current_a],
    ['battery_remaining', batt.remaining],
    ['gps_lat', gpsLat],
    ['gps_lon', gpsLon],
    ['gps_alt', gpsAlt],
    ['gps_fix_type', gps.fix_type],
    ['gps_satellites_used', gps.satellites_used],
    ['landed', land.landed],
    ['maybe_landed', land.maybe_landed],
    ['freefall', land.freefall],
    ['takeoff_state', takeoff.takeoff_state],
  ]
  panel.innerHTML = lines.map(([k, v]) => {
    const val = (v === undefined || v === null || v === '') ? '-' : v
    const cls = (k === 'goal_status' && val === 'reached') ? 'status-good' : ''
    return `<div class="${cls}"><strong>${k}</strong>: ${val}</div>`
  }).join('')
}

document.addEventListener('click', function (e) {
  if (e.target && e.target.id === 'uav-calibrate-btn') {
    if (!activeModalId) return
    const id = activeModalId
    const statusEl = document.getElementById('status-' + id)
    const latEl = document.querySelector(`#card-${id} .geo-lat[data-id="${id}"]`)
    const lonEl = document.querySelector(`#card-${id} .geo-lon[data-id="${id}"]`)
    if (!latEl || !lonEl) return
    const desiredLat = parseFloat(latEl.value)
    const desiredLon = parseFloat(lonEl.value)
    const state = mapStates[id]
    if (!state || !state.lastLatLon || isNaN(desiredLat) || isNaN(desiredLon)) {
      if (statusEl) statusEl.textContent = 'need UAV GPS + click map'
      return
    }
    const actualLat = state.lastLatLon[0]
    const actualLon = state.lastLatLon[1]
    mapOffsets[id] = { lat: desiredLat - actualLat, lon: desiredLon - actualLon }
    updateCalibrateStatus(id)
    // refresh markers with new offset
    ensureMap(id, actualLat, actualLon)
    if (goalDrafts[id]) updateGoalMarker(id, goalDrafts[id])
    if (statusEl) statusEl.textContent = 'map calibrated'
  }
})
