const mapStates = {}
const goalDrafts = {}
let activeModalId = null
let modalMapState = null
const mapOffsets = {}

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
  const statusEl = document.getElementById('disarm-status-' + id)
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
    const statusEl = document.getElementById('status-' + id)
    const armStatusEl = document.getElementById('arm-status-' + id)
    const disarmStatusEl = document.getElementById('disarm-status-' + id)
    // Disarm UI removed: keep original behavior for arm/status
    if (flyBtn) flyBtn.disabled = !connected
    if (armBtn) armBtn.disabled = !connected
    if (draftBtn) draftBtn.disabled = !connected
    if (disarmBtn) disarmBtn.disabled = !connected
    if (statusEl && !goalDrafts[id]) statusEl.textContent = connected ? 'connected' : 'no node'
    if (armStatusEl) armStatusEl.textContent = info.armed ? 'armed' : (connected ? 'idle' : 'no node')
    if (disarmStatusEl) disarmStatusEl.textContent = info.armed ? 'armed' : (connected ? 'disarmed' : 'no node')

    if (info.lat !== undefined && info.lon !== undefined) {
      ensureMap(id, info.lat, info.lon)
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
