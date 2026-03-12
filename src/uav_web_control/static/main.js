document.addEventListener('click', function (e) {
  if (!e.target.matches('.fly-btn')) return
  const id = e.target.dataset.id
  const statusEl = document.getElementById('status-' + id)
  statusEl.textContent = 'sending...'
  fetch('/fly', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
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
  statusEl.textContent = 'sending arm...'
  fetch('/arm', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  }).then(r => r.json())
    .then(data => {
      if (data.status === 'ok') {
        statusEl.textContent = 'arm command sent'
      } else {
        statusEl.textContent = 'error: ' + (data.message || 'unknown')
      }
    }).catch(err => {
      statusEl.textContent = 'network error'
      console.error(err)
    })
}
)
