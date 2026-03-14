document.addEventListener('click', function (e) {
  if (!e.target.matches('.fly-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('status-' + id)
  statusEl.textContent = 'sending...'
  // include selected setpoint index if present
  const sel = document.querySelector(`.sp-select[data-id="${id}"]`)
  const body = { id }
  if (sel) body.index = parseInt(sel.value)

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

/* Disarm handler commented out per request
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
*/

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
    const statusEl = document.getElementById('status-' + id)
    const armStatusEl = document.getElementById('arm-status-' + id)
    // Disarm UI removed: keep original behavior for arm/status
    if (flyBtn) flyBtn.disabled = !connected
    if (armBtn) armBtn.disabled = !connected
    if (statusEl) statusEl.textContent = connected ? 'connected' : 'no node'
    if (armStatusEl) armStatusEl.textContent = info.armed ? 'armed' : (connected ? 'idle' : 'no node')
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
