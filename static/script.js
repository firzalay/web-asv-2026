// ============================================================
// Pixhawk Monitor - Client Script
// Menerima data real-time via WebSocket (Socket.IO) dan
// merender ke peta, kompas, serta panel angka.
// ============================================================

const socket = io();

// ---- Setup Peta (Leaflet) ----
const map = L.map('map', { zoomControl: true }).setView([0, 0], 3);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);

// Ikon kapal sederhana
const boatIcon = L.divIcon({
    className: 'boat-marker',
    html: '<div style="font-size:28px; transform: translate(-50%,-50%);">🚤</div>',
    iconSize: [0, 0]
});

let boatMarker = null;
let trackLine = null;
const trackPoints = [];
let firstFix = true;

// Layer group khusus untuk waypoint mission (dipisah dari jejak kapal)
const missionLayer = L.layerGroup().addTo(map);

function renderMissionOnMap(items) {
    missionLayer.clearLayers();
    if (!items || items.length === 0) return;

    const latlngs = [];
    items.forEach((wp) => {
        if (wp.lat === 0 && wp.lon === 0) return; // skip item non-posisi (mis. DO_SET_SERVO)
        const latlng = [wp.lat, wp.lon];
        latlngs.push(latlng);

        const icon = L.divIcon({
            className: 'mission-num-icon',
            html: `<div>${wp.seq}</div>`,
            iconSize: [22, 22]
        });
        L.marker(latlng, { icon })
            .bindPopup(`<b>WP ${wp.seq}</b><br>${wp.command_name}<br>${wp.lat.toFixed(6)}, ${wp.lon.toFixed(6)}<br>Alt: ${wp.alt} m`)
            .addTo(missionLayer);
    });

    if (latlngs.length > 1) {
        L.polyline(latlngs, { color: '#ffb545', weight: 3, dashArray: '6 6', opacity: 0.85 }).addTo(missionLayer);
    }

    if (latlngs.length > 0 && firstFix) {
        map.fitBounds(latlngs, { padding: [40, 40] });
    }
}

function updateMap(lat, lon, heading) {
    if (lat === 0 && lon === 0) return; // belum ada fix GPS valid

    const latlng = [lat, lon];

    if (!boatMarker) {
        boatMarker = L.marker(latlng, { icon: boatIcon }).addTo(map);
        trackLine = L.polyline([], { color: '#21c9d6', weight: 3, opacity: 0.7 }).addTo(map);
    } else {
        boatMarker.setLatLng(latlng);
    }

    trackPoints.push(latlng);
    if (trackPoints.length > 500) trackPoints.shift(); // batasi jejak biar tidak berat
    trackLine.setLatLngs(trackPoints);

    if (firstFix) {
        map.setView(latlng, 17);
        firstFix = false;
    }
}

// ---- Elemen DOM ----
const el = {
    connStatus: document.getElementById('connStatus'),
    connText: document.getElementById('connText'),
    lat: document.getElementById('latVal'),
    lon: document.getElementById('lonVal'),
    cog: document.getElementById('cogVal'),
    needle: document.getElementById('needle'),
    sog: document.getElementById('sogVal'),
    sogMs: document.getElementById('sogMs'),
    speed: document.getElementById('speedVal'),
    speedMs: document.getElementById('speedMs'),
    fix: document.getElementById('fixVal'),
    sat: document.getElementById('satVal'),
    hdg: document.getElementById('hdgVal'),
    batt: document.getElementById('battVal'),
    ts: document.getElementById('tsVal'),
    modeBadge: document.getElementById('modeBadge'),
    modeVal: document.getElementById('modeVal'),
    armedBadge: document.getElementById('armedBadge'),
    armedVal: document.getElementById('armedVal'),
    btnFetchMission: document.getElementById('btnFetchMission'),
    missionStatus: document.getElementById('missionStatus'),
    missionTableBody: document.getElementById('missionTableBody'),
    cameraFeed: document.getElementById('cameraFeed'),
    cameraPlaceholder: document.getElementById('cameraPlaceholder'),
    camStatus: document.getElementById('camStatus'),
    ballCount: document.getElementById('ballCount'),
    ballTotal: document.getElementById('ballTotal'),
    ballLog: document.getElementById('ballLog'),
    btnBallPlus: document.getElementById('btnBallPlus'),
    btnBallMinus: document.getElementById('btnBallMinus'),
    btnBallReset: document.getElementById('btnBallReset'),
    btnApplyHsv: document.getElementById('btnApplyHsv'),
};

function fmt(n, digits = 6) {
    return (typeof n === 'number') ? n.toFixed(digits) : '--';
}

socket.on('connect', () => {
    console.log('Terhubung ke server monitoring');
});

socket.on('telemetry', (data) => {
    // Status koneksi Pixhawk (bukan koneksi browser-server)
    if (data.connected) {
        el.connStatus.classList.add('online');
        el.connStatus.classList.remove('offline');
        el.connText.textContent = 'Pixhawk Terhubung';
    } else {
        el.connStatus.classList.remove('online');
        el.connStatus.classList.add('offline');
        el.connText.textContent = 'Pixhawk Terputus';
    }

    // Posisi
    el.lat.textContent = fmt(data.lat, 6);
    el.lon.textContent = fmt(data.lon, 6);
    updateMap(data.lat, data.lon, data.heading);

    // COG / Kompas
    const cog = data.cog || 0;
    el.cog.textContent = cog.toFixed(1);
    el.needle.setAttribute('transform', `rotate(${cog} 100 100)`);

    // SOG
    el.sog.textContent = (data.sog || 0).toFixed(2);
    el.sogMs.textContent = ((data.sog || 0) * 0.514444).toFixed(2) + ' m/s';

    // Groundspeed
    el.speed.textContent = (data.speed_kmh || 0).toFixed(2);
    el.speedMs.textContent = (data.speed_ms || 0).toFixed(2) + ' m/s';

    // Status
    el.fix.textContent = data.fix_text || '--';
    el.sat.textContent = data.satellites ?? '--';
    el.hdg.textContent = (data.heading || 0).toFixed(1) + '°';
    el.batt.textContent = data.battery_voltage ? data.battery_voltage.toFixed(2) + ' V' : '--';

    // Flight Mode
    const mode = data.mode || 'UNKNOWN';
    el.modeVal.textContent = mode;
    el.modeBadge.className = 'mode-badge mode-' + mode.toLowerCase();

    // Armed / Disarmed
    if (data.armed) {
        el.armedVal.textContent = 'ARMED';
        el.armedBadge.className = 'armed-badge armed';
    } else {
        el.armedVal.textContent = 'DISARMED';
        el.armedBadge.className = 'armed-badge disarmed';
    }

    if (data.timestamp) {
        const d = new Date(data.timestamp * 1000);
        el.ts.textContent = d.toLocaleTimeString('id-ID');
    }
});

socket.on('disconnect', () => {
    el.connStatus.classList.remove('online');
    el.connStatus.classList.add('offline');
    el.connText.textContent = 'Server Terputus';
});

// ---- Mission / Waypoint ----

function renderMissionTable(items) {
    if (!items || items.length === 0) {
        el.missionTableBody.innerHTML = '<tr><td colspan="5" class="mission-empty">Tidak ada waypoint tersimpan di FC</td></tr>';
        return;
    }
    el.missionTableBody.innerHTML = items.map((wp) => `
        <tr>
            <td>${wp.seq}</td>
            <td>${wp.command_name}</td>
            <td>${wp.lat.toFixed(6)}</td>
            <td>${wp.lon.toFixed(6)}</td>
            <td>${wp.alt}</td>
        </tr>
    `).join('');
}

let missionRequestTimeout = null;

el.btnFetchMission.addEventListener('click', () => {
    el.btnFetchMission.disabled = true;
    el.missionStatus.textContent = 'Meminta daftar waypoint dari FC...';
    el.missionStatus.className = 'mission-status loading';
    socket.emit('request_mission');

    // Timeout jaga-jaga kalau FC tidak pernah balas (misal belum connect)
    clearTimeout(missionRequestTimeout);
    missionRequestTimeout = setTimeout(() => {
        if (el.btnFetchMission.disabled) {
            el.missionStatus.textContent = 'Timeout: tidak ada respons dari FC. Pastikan Pixhawk terhubung.';
            el.missionStatus.className = 'mission-status error';
            el.btnFetchMission.disabled = false;
        }
    }, 15000);
});

socket.on('mission_status', (data) => {
    if (data.downloading) {
        el.missionStatus.textContent = 'Mengunduh waypoint dari FC...';
        el.missionStatus.className = 'mission-status loading';
    }
});

socket.on('mission_data', (data) => {
    clearTimeout(missionRequestTimeout);
    el.btnFetchMission.disabled = false;

    if (data.error) {
        el.missionStatus.textContent = data.error;
        el.missionStatus.className = 'mission-status error';
        return;
    }

    const items = data.items || [];
    renderMissionTable(items);
    renderMissionOnMap(items);

    const timeStr = data.last_updated ? new Date(data.last_updated * 1000).toLocaleTimeString('id-ID') : '';
    el.missionStatus.textContent = `Berhasil ambil ${items.length} waypoint dari FC (${timeStr})`;
    el.missionStatus.className = 'mission-status success';
});

// ---- Floating Ball Counter (Computer Vision) ----

socket.on('camera_frame', (data) => {
    el.cameraFeed.src = 'data:image/jpeg;base64,' + data.image;
    el.cameraFeed.classList.add('active');
    el.cameraPlaceholder.style.display = 'none';
    el.camStatus.textContent = 'Kamera Terhubung';
    el.camStatus.classList.add('online');
    el.camStatus.classList.remove('offline');
});

socket.on('ball_update', (data) => {
    el.ballCount.textContent = data.count;
    el.ballTotal.textContent = data.total;

    if (!data.camera_connected) {
        el.cameraFeed.classList.remove('active');
        el.cameraPlaceholder.style.display = 'flex';
        el.cameraPlaceholder.querySelector('p').textContent = 'Kamera tidak terhubung';
        el.camStatus.textContent = 'Kamera Terputus';
        el.camStatus.classList.remove('online');
        el.camStatus.classList.add('offline');
    }

    const log = data.log || [];
    if (log.length === 0) {
        el.ballLog.innerHTML = '<li class="mission-empty">Belum ada bola terhitung</li>';
        return;
    }
    el.ballLog.innerHTML = log.map((entry) => {
        const timeStr = new Date(entry.timestamp * 1000).toLocaleTimeString('id-ID');
        const posStr = (entry.lat && entry.lon) ? `${entry.lat.toFixed(5)}, ${entry.lon.toFixed(5)}` : 'no GPS';
        return `<li class="${entry.manual ? 'manual' : ''}">
            <span class="log-seq">#${entry.seq}</span> ${timeStr}${entry.manual ? ' (manual)' : ''} · ${posStr}
        </li>`;
    }).join('');
});

el.btnBallPlus.addEventListener('click', () => socket.emit('ball_increment'));
el.btnBallMinus.addEventListener('click', () => socket.emit('ball_decrement'));
el.btnBallReset.addEventListener('click', () => {
    if (confirm('Reset hitungan bola ke 0? Log akan dihapus.')) {
        socket.emit('ball_reset');
    }
});

// ---- HSV Tuner ----

const hsvSliders = ['hMin', 'hMax', 'sMin', 'sMax', 'vMin', 'vMax'];
hsvSliders.forEach((id) => {
    const slider = document.getElementById(id);
    const label = document.getElementById(id + 'Val');
    slider.addEventListener('input', () => {
        label.textContent = slider.value;
    });
});

el.btnApplyHsv.addEventListener('click', () => {
    socket.emit('update_hsv', {
        h_min: document.getElementById('hMin').value,
        h_max: document.getElementById('hMax').value,
        s_min: document.getElementById('sMin').value,
        s_max: document.getElementById('sMax').value,
        v_min: document.getElementById('vMin').value,
        v_max: document.getElementById('vMax').value,
    });
});

socket.on('hsv_updated', () => {
    el.btnApplyHsv.textContent = '✓ Diterapkan';
    setTimeout(() => { el.btnApplyHsv.textContent = 'Terapkan'; }, 1500);
});
