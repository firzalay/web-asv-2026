"""
Pixhawk Real-time Monitoring - Backend
========================================
Membaca telemetry dari Pixhawk (via USB/Telemetry Radio/UDP) menggunakan pymavlink,
lalu broadcast ke web dashboard secara real-time via WebSocket (Flask-SocketIO).

Cocok dijalankan langsung di Legion Go yang terhubung ke Pixhawk kapal RC autonomous.
"""

from flask import Flask, render_template
from flask_socketio import SocketIO
from pymavlink import mavutil
import threading
import time
import base64

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None

app = Flask(__name__)
app.config['SECRET_KEY'] = 'pixhawk-monitor-secret'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# =========================================================================
# KONFIGURASI KONEKSI PIXHAWK - SESUAIKAN DENGAN SETUP KAMU
# =========================================================================
# Pilihan koneksi (uncomment salah satu / sesuaikan):
#
# 1) USB langsung ke Pixhawk (paling umum jika Legion Go dicolok ke Pixhawk):
#    Windows -> 'COM3', 'COM4', dst (cek di Device Manager)
#    Linux   -> '/dev/ttyACM0' atau '/dev/ttyUSB0'
#
# 2) Radio telemetry (SiK radio, 3DR, dll):
#    Windows -> 'COM5' misal, baud biasanya 57600
#    Linux   -> '/dev/ttyUSB0'
#
# 3) Koneksi UDP (misal Pixhawk sudah forward data via companion computer/WiFi,
#    atau kamu pakai MAVProxy/mavlink-router untuk broadcast UDP):
#    'udp:0.0.0.0:14550'
#
# 4) TCP:
#    'tcp:127.0.0.1:5760'

MAVLINK_CONNECTION = '/dev/acmTTY0'      # <-- GANTI SESUAI PORT DI LEGION GO KAMU
BAUD_RATE = 115200                # umumnya 57600 (radio) atau 115200 (USB langsung)

# Interval minimal update yang dikirim ke browser (detik) - biar tidak flooding
EMIT_INTERVAL = 0.2  # 5x per detik

# Set False untuk mematikan sementara fitur floating ball counter (kamera tidak akan dibuka)
ENABLE_BALL_DETECTION = False

# =========================================================================
# KONFIGURASI KAMERA - FLOATING BALL DETECTION
# =========================================================================
# CAMERA_SOURCE bisa berupa:
#   - Angka index kamera USB: 0, 1, 2, dst (0 biasanya kamera pertama yang kedetect)
#   - URL kamera IP/RTSP: 'rtsp://192.168.1.10:554/stream' atau 'http://192.168.1.10:8080/video'
CAMERA_SOURCE = 0
CAMERA_WIDTH = 480     # frame di-resize ke lebar ini sebelum diproses (biar ringan & cepat)
CAMERA_FPS_LIMIT = 8   # batas fps yang dikirim ke browser (gak perlu tinggi-tinggi)

# Total bola dalam satu misi (sesuai aturan lomba: "Floating ball set 1-10")
TOTAL_BALLS = 10

# Berapa lama (detik) minimal jeda sebelum bola berikutnya bisa dihitung lagi,
# supaya satu bola yang sama tidak ke-hitung berkali-kali selama masih di frame.
BALL_COUNT_COOLDOWN = 3.0

# Berapa frame berturut-turut bola harus konsisten terdeteksi sebelum dihitung "sah",
# supaya deteksi flicker/noise sesaat tidak ikut ke-hitung.
BALL_STABLE_FRAMES = 5

# Luas minimal kontur (dalam pixel, setelah resize) supaya dianggap bola, bukan noise
BALL_MIN_AREA = 400

# =========================================================================

latest_data = {
    'lat': 0.0,
    'lon': 0.0,
    'sog': 0.0,        # Speed Over Ground (knots)
    'cog': 0.0,        # Course Over Ground (derajat, 0-360)
    'speed_ms': 0.0,   # Groundspeed dari VFR_HUD (m/s)
    'speed_kmh': 0.0,  # konversi ke km/h biar mudah dibaca
    'heading': 0.0,    # heading kompas kapal (derajat)
    'satellites': 0,
    'fix_type': 0,
    'fix_text': 'NO FIX',
    'battery_voltage': None,
    'armed': False,
    'mode': 'UNKNOWN',
    'connected': False,
    'timestamp': 0,
}

FIX_TYPE_MAP = {
    0: 'NO GPS',
    1: 'NO FIX',
    2: '2D FIX',
    3: '3D FIX',
    4: 'DGPS',
    5: 'RTK FLOAT',
    6: 'RTK FIXED',
}

# Mapping custom_mode (dari HEARTBEAT) -> nama mode, khusus firmware ArduRover.
# Kalau Pixhawk kamu pakai ArduCopter/ArduPlane, mapping-nya beda - kasih tahu saya
# supaya bisa disesuaikan.
ROVER_MODE_MAP = {
    0: 'MANUAL',
    1: 'ACRO',
    3: 'STEERING',
    4: 'HOLD',
    5: 'LOITER',
    6: 'FOLLOW',
    7: 'SIMPLE',
    10: 'AUTO',
    11: 'RTL',
    12: 'SMART_RTL',
    15: 'GUIDED',
    16: 'INITIALISING',
}

# Nama command MAVLink yang umum dipakai di mission ArduRover/ArduPlane/ArduCopter
MAV_CMD_NAMES = {
    16: 'WAYPOINT',
    17: 'LOITER_UNLIM',
    18: 'LOITER_TURNS',
    19: 'LOITER_TIME',
    20: 'RETURN_TO_LAUNCH',
    21: 'LAND',
    22: 'TAKEOFF',
    183: 'DO_SET_SERVO',
    189: 'DO_SET_REVERSE',
    206: 'DO_GUIDED_LIMITS',
}

# Referensi koneksi MAVLink aktif, dipakai oleh route/socket handler
# untuk mengirim command mission (request list, dsb) dari thread lain.
master_ref = {'conn': None}

mission_lock = threading.Lock()
mission_state = {
    'downloading': False,
    'items': [],          # list of {seq, lat, lon, alt, command, command_name}
    'expected_count': 0,
    'last_updated': None,
    'error': None,
}

# =========================================================================
# FLOATING BALL DETECTION - state & konfigurasi warna (HSV)
# =========================================================================
# Nilai default ini untuk bola ORANYE terang - SESUAIKAN di lapangan lewat
# tuner HSV yang ada di dashboard (card "Floating Ball Counter").
# H: 0-179, S: 0-255, V: 0-255 (standar OpenCV)
ball_hsv_lock = threading.Lock()
ball_hsv_range = {
    'h_min': 5, 'h_max': 25,
    's_min': 120, 's_max': 255,
    'v_min': 120, 'v_max': 255,
}

ball_lock = threading.Lock()
ball_state = {
    'count': 0,
    'total': TOTAL_BALLS,
    'present_streak': 0,   # berapa frame berturut-turut bola stabil terdeteksi
    'last_count_time': 0,  # waktu terakhir kali berhasil menghitung bola (untuk cooldown)
    'log': [],             # list of {seq, timestamp, lat, lon}
    'camera_connected': False,
}


def reset_ball_counter():
    with ball_lock:
        ball_state['count'] = 0
        ball_state['present_streak'] = 0
        ball_state['last_count_time'] = 0
        ball_state['log'] = []
    socketio.emit('ball_update', get_ball_summary())


def get_ball_summary():
    with ball_lock:
        return {
            'count': ball_state['count'],
            'total': ball_state['total'],
            'log': list(ball_state['log']),
            'camera_connected': ball_state['camera_connected'],
        }


def manual_adjust_ball_count(delta):
    with ball_lock:
        new_count = max(0, min(ball_state['total'], ball_state['count'] + delta))
        ball_state['count'] = new_count
        if delta > 0:
            ball_state['log'].append({
                'seq': new_count,
                'timestamp': time.time(),
                'lat': latest_data['lat'],
                'lon': latest_data['lon'],
                'manual': True,
            })
    socketio.emit('ball_update', get_ball_summary())


def camera_detection_loop():
    """Thread terpisah: baca kamera USB/IP, deteksi bola warna tertentu (HSV),
    hitung otomatis, dan kirim frame + hasil deteksi ke browser secara real-time."""
    last_frame_emit = 0
    frame_interval = 1.0 / CAMERA_FPS_LIMIT

    while True:
        try:
            print(f"[Camera] Membuka kamera: {CAMERA_SOURCE} ...")
            cap = cv2.VideoCapture(CAMERA_SOURCE)
            if not cap.isOpened():
                raise RuntimeError(f"Tidak bisa membuka kamera {CAMERA_SOURCE}")

            with ball_lock:
                ball_state['camera_connected'] = True
            print("[Camera] Kamera terbuka, mulai deteksi bola...")

            while True:
                ok, frame = cap.read()
                if not ok or frame is None:
                    print("[Camera] Gagal baca frame, reconnect...")
                    break

                # Resize supaya pemrosesan ringan & cepat
                h, w = frame.shape[:2]
                scale = CAMERA_WIDTH / w
                frame = cv2.resize(frame, (CAMERA_WIDTH, int(h * scale)))

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                with ball_hsv_lock:
                    lower = np.array([ball_hsv_range['h_min'], ball_hsv_range['s_min'], ball_hsv_range['v_min']])
                    upper = np.array([ball_hsv_range['h_max'], ball_hsv_range['s_max'], ball_hsv_range['v_max']])

                mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.erode(mask, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)

                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                detected_now = False
                if contours:
                    largest = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest)
                    if area >= BALL_MIN_AREA:
                        detected_now = True
                        (x, y), radius = cv2.minEnclosingCircle(largest)
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                        cv2.putText(frame, "BALL", (int(x) - 20, int(y) - int(radius) - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # --- logika hitung otomatis dengan stabilisasi + cooldown ---
                now = time.time()
                counted_this_frame = False
                with ball_lock:
                    if detected_now:
                        ball_state['present_streak'] += 1
                    else:
                        ball_state['present_streak'] = 0

                    can_count = (
                        ball_state['present_streak'] >= BALL_STABLE_FRAMES and
                        (now - ball_state['last_count_time']) >= BALL_COUNT_COOLDOWN and
                        ball_state['count'] < ball_state['total']
                    )
                    if can_count:
                        ball_state['count'] += 1
                        ball_state['last_count_time'] = now
                        ball_state['present_streak'] = 0
                        ball_state['log'].append({
                            'seq': ball_state['count'],
                            'timestamp': now,
                            'lat': latest_data['lat'],
                            'lon': latest_data['lon'],
                            'manual': False,
                        })
                        counted_this_frame = True

                if counted_this_frame:
                    print(f"[Ball] Bola ke-{ball_state['count']} terdeteksi otomatis!")
                    socketio.emit('ball_update', get_ball_summary())

                # --- kirim frame ke browser (dibatasi FPS biar gak flooding) ---
                if now - last_frame_emit >= frame_interval:
                    ok2, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok2:
                        b64 = base64.b64encode(buf).decode('utf-8')
                        socketio.emit('camera_frame', {
                            'image': b64,
                            'detected': detected_now,
                        })
                    last_frame_emit = now

            cap.release()
            with ball_lock:
                ball_state['camera_connected'] = False
            socketio.emit('ball_update', get_ball_summary())
            time.sleep(2)

        except Exception as e:
            print(f"[Camera] Error: {e}")
            with ball_lock:
                ball_state['camera_connected'] = False
            socketio.emit('ball_update', get_ball_summary())
            time.sleep(3)


# Melacak mode sebelumnya untuk mendeteksi TRANSISI ke AUTO (bukan cuma nilai
# mode saat ini) - dipakai untuk auto-refresh & auto-verifikasi tampilan mission.
mode_tracker = {'previous_mode': None}

# State untuk setup parameter RCx_OPTION (mis. Save Waypoint) dari dashboard
rc_setup_lock = threading.Lock()
rc_setup_state = {'pending_param': None, 'pending_value': None}

SAVE_WAYPOINT_OPTION_VALUE = 7  # nilai param RCx_OPTION untuk fungsi "Save Waypoint"

# =========================================================================
# SYSTEM LOG - menangkap pesan STATUSTEXT dari FC (mis. "Saved waypoint #3")
# ArduPilot otomatis kirim STATUSTEXT saat waypoint disimpan lewat RC switch,
# arming/disarm, mode change gagal, error sensor, dll.
# =========================================================================
MAX_LOG_ENTRIES = 50
system_log_lock = threading.Lock()
system_log_state = {'entries': []}

# Kata kunci untuk menandai entry log yang terkait penyimpanan waypoint
# (ditandai khusus di UI supaya gampang dibedakan dari pesan sistem lainnya)
WAYPOINT_LOG_KEYWORDS = ['waypoint', 'wp saved', 'wp added']

MAV_SEVERITY_MAP = {
    0: 'EMERGENCY', 1: 'ALERT', 2: 'CRITICAL', 3: 'ERROR',
    4: 'WARNING', 5: 'NOTICE', 6: 'INFO', 7: 'DEBUG',
}


def add_system_log_entry(text, severity):
    is_waypoint_event = any(kw in text.lower() for kw in WAYPOINT_LOG_KEYWORDS)
    entry = {
        'text': text,
        'severity': MAV_SEVERITY_MAP.get(severity, f'SEV_{severity}'),
        'timestamp': time.time(),
        'lat': latest_data['lat'],
        'lon': latest_data['lon'],
        'is_waypoint_event': is_waypoint_event,
    }
    with system_log_lock:
        system_log_state['entries'].append(entry)
        if len(system_log_state['entries']) > MAX_LOG_ENTRIES:
            system_log_state['entries'].pop(0)
    socketio.emit('system_log_entry', entry)
    return entry


def request_mission_download():
    """Dipanggil dari socket handler (thread Flask) untuk mulai minta mission dari FC.
    Pengambilan data sesungguhnya terjadi di dalam mavlink_listener() supaya tidak
    ada dua thread yang baca/tulis serial port secara bersamaan."""
    conn = master_ref['conn']
    if conn is None:
        socketio.emit('mission_data', {
            'error': 'Pixhawk belum terhubung, tidak bisa ambil mission.',
            'items': []
        })
        return

    with mission_lock:
        mission_state['downloading'] = True
        mission_state['items'] = []
        mission_state['expected_count'] = 0
        mission_state['error'] = None

    socketio.emit('mission_status', {'downloading': True})
    print("[Mission] Meminta daftar waypoint dari FC...")
    conn.mav.mission_request_list_send(conn.target_system, conn.target_component)


def request_rc_save_waypoint_setup(channel):
    """Set parameter RCx_OPTION = Save Waypoint untuk channel RC tertentu.
    Hasilnya dikonfirmasi lewat PARAM_VALUE yang ditangkap di listener thread."""
    conn = master_ref['conn']
    if conn is None:
        socketio.emit('rc_setup_result', {
            'success': False, 'message': 'Pixhawk belum terhubung.'
        })
        return

    param_name = f'RC{channel}_OPTION'
    with rc_setup_lock:
        rc_setup_state['pending_param'] = param_name
        rc_setup_state['pending_value'] = SAVE_WAYPOINT_OPTION_VALUE

    print(f"[RC Setup] Mengirim {param_name} = {SAVE_WAYPOINT_OPTION_VALUE} (Save Waypoint)")
    conn.mav.param_set_send(
        conn.target_system, conn.target_component,
        param_name.encode('utf-8'),
        float(SAVE_WAYPOINT_OPTION_VALUE),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    socketio.emit('rc_setup_status', {'status': 'sending', 'param': param_name})


def mavlink_listener():
    """Thread terpisah yang terus-menerus membaca data dari Pixhawk."""
    global latest_data
    last_emit = 0

    while True:
        try:
            print(f"[MAVLink] Menghubungkan ke {MAVLINK_CONNECTION} ...")
            master = mavutil.mavlink_connection(MAVLINK_CONNECTION, baud=BAUD_RATE)
            master.wait_heartbeat(timeout=15)
            print(f"[MAVLink] Heartbeat diterima dari system {master.target_system}, "
                  f"component {master.target_component}")
            latest_data['connected'] = True
            master_ref['conn'] = master

            # Minta pixhawk kirim stream data lebih cepat (opsional, tergantung firmware)
            try:
                master.mav.request_data_stream_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
                )
            except Exception:
                pass

            while True:
                msg = master.recv_match(blocking=True, timeout=5)
                if msg is None:
                    print("[MAVLink] Timeout, tidak ada data masuk. Reconnect...")
                    latest_data['connected'] = False
                    master_ref['conn'] = None
                    socketio.emit('telemetry', latest_data)
                    break

                msg_type = msg.get_type()

                if msg_type == 'GLOBAL_POSITION_INT':
                    latest_data['lat'] = msg.lat / 1e7
                    latest_data['lon'] = msg.lon / 1e7
                    latest_data['heading'] = msg.hdg / 100.0 if msg.hdg != 65535 else latest_data['heading']

                elif msg_type == 'HEARTBEAT':
                    latest_data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    new_mode = ROVER_MODE_MAP.get(msg.custom_mode, f'MODE_{msg.custom_mode}')
                    latest_data['mode'] = new_mode

                    # Deteksi transisi KE AUTO (baru saja berubah, bukan sudah AUTO dari tadi)
                    if new_mode == 'AUTO' and mode_tracker['previous_mode'] != 'AUTO':
                        print("[Mode] AUTO baru saja diaktifkan -> auto-refresh tampilan mission untuk verifikasi")
                        socketio.emit('auto_engaged', {'timestamp': time.time()})
                        request_mission_download()

                    mode_tracker['previous_mode'] = new_mode

                elif msg_type == 'GPS_RAW_INT':
                    # vel dalam cm/s -> konversi ke knots
                    latest_data['sog'] = round((msg.vel / 100.0) * 1.94384, 2)
                    latest_data['cog'] = round(msg.cog / 100.0, 1)
                    latest_data['satellites'] = msg.satellites_visible
                    latest_data['fix_type'] = msg.fix_type
                    latest_data['fix_text'] = FIX_TYPE_MAP.get(msg.fix_type, 'UNKNOWN')

                elif msg_type == 'VFR_HUD':
                    latest_data['speed_ms'] = round(msg.groundspeed, 2)
                    latest_data['speed_kmh'] = round(msg.groundspeed * 3.6, 2)

                elif msg_type == 'SYS_STATUS':
                    latest_data['battery_voltage'] = round(msg.voltage_battery / 1000.0, 2)

                elif msg_type == 'STATUSTEXT':
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode('utf-8', errors='ignore')
                    text = text.rstrip('\x00').strip()
                    if text:
                        entry = add_system_log_entry(text, msg.severity)
                        if entry['is_waypoint_event']:
                            print(f"[WP Save Event] {text} @ ({entry['lat']:.6f}, {entry['lon']:.6f})")

                elif msg_type == 'PARAM_VALUE':
                    with rc_setup_lock:
                        pending_param = rc_setup_state['pending_param']
                        pending_value = rc_setup_state['pending_value']

                    if pending_param:
                        received_name = msg.param_id
                        if isinstance(received_name, bytes):
                            received_name = received_name.decode('utf-8')
                        received_name = received_name.rstrip('\x00')

                        if received_name == pending_param:
                            success = abs(msg.param_value - pending_value) < 0.5
                            print(f"[RC Setup] {pending_param} dikonfirmasi FC = {msg.param_value} "
                                  f"({'OK' if success else 'TIDAK SESUAI'})")
                            socketio.emit('rc_setup_result', {
                                'success': success,
                                'param': pending_param,
                                'value': msg.param_value,
                            })
                            with rc_setup_lock:
                                rc_setup_state['pending_param'] = None

                elif msg_type == 'MISSION_COUNT':
                    print(f"[Mission] FC melaporkan {msg.count} waypoint tersimpan")
                    with mission_lock:
                        mission_state['expected_count'] = msg.count
                        mission_state['items'] = []
                    if msg.count > 0:
                        master.mav.mission_request_int_send(master.target_system, master.target_component, 0)
                    else:
                        with mission_lock:
                            mission_state['downloading'] = False
                            mission_state['last_updated'] = time.time()
                        socketio.emit('mission_data', {
                            'items': [], 'error': None, 'last_updated': mission_state['last_updated']
                        })

                elif msg_type == 'MISSION_ITEM_INT':
                    with mission_lock:
                        item = {
                            'seq': msg.seq,
                            'lat': msg.x / 1e7,
                            'lon': msg.y / 1e7,
                            'alt': msg.z,
                            'command': msg.command,
                            'command_name': MAV_CMD_NAMES.get(msg.command, f'CMD_{msg.command}'),
                        }
                        mission_state['items'].append(item)
                        next_seq = msg.seq + 1
                        total = mission_state['expected_count']

                    if next_seq < total:
                        master.mav.mission_request_int_send(master.target_system, master.target_component, next_seq)
                    else:
                        # semua item sudah diterima -> kirim ACK, selesai
                        master.mav.mission_ack_send(
                            master.target_system, master.target_component,
                            mavutil.mavlink.MAV_MISSION_ACCEPTED
                        )
                        with mission_lock:
                            mission_state['downloading'] = False
                            mission_state['last_updated'] = time.time()
                            items_copy = list(mission_state['items'])
                        print(f"[Mission] Selesai, {len(items_copy)} waypoint diterima")
                        socketio.emit('mission_data', {
                            'items': items_copy, 'error': None,
                            'last_updated': mission_state['last_updated']
                        })

                latest_data['connected'] = True
                latest_data['timestamp'] = time.time()

                now = time.time()
                if now - last_emit >= EMIT_INTERVAL:
                    socketio.emit('telemetry', latest_data)
                    last_emit = now

        except Exception as e:
            print(f"[MAVLink] Error koneksi: {e}")
            latest_data['connected'] = False
            master_ref['conn'] = None
            socketio.emit('telemetry', latest_data)
            time.sleep(3)


@app.route('/')
def index():
    return render_template('index.html')


@socketio.on('connect')
def handle_connect():
    print('[Web] Client browser terhubung')
    socketio.emit('telemetry', latest_data)
    socketio.emit('ball_update', get_ball_summary())
    with system_log_lock:
        socketio.emit('system_log_history', list(system_log_state['entries']))


@socketio.on('request_mission')
def handle_request_mission():
    print('[Web] Client meminta download mission dari FC')
    request_mission_download()


@socketio.on('ball_increment')
def handle_ball_increment():
    manual_adjust_ball_count(1)


@socketio.on('ball_decrement')
def handle_ball_decrement():
    manual_adjust_ball_count(-1)


@socketio.on('ball_reset')
def handle_ball_reset():
    print('[Web] Reset ball counter')
    reset_ball_counter()


@socketio.on('update_hsv')
def handle_update_hsv(data):
    """Terima nilai HSV baru dari tuner di dashboard untuk kalibrasi warna bola."""
    with ball_hsv_lock:
        for key in ('h_min', 'h_max', 's_min', 's_max', 'v_min', 'v_max'):
            if key in data:
                ball_hsv_range[key] = int(data[key])
    print(f"[Camera] HSV range diupdate: {ball_hsv_range}")
    socketio.emit('hsv_updated', ball_hsv_range)


@socketio.on('setup_rc_save_wp')
def handle_setup_rc_save_wp(data):
    channel = int(data.get('channel', 10)) if data else 10
    print(f"[Web] Setup RC{channel}_OPTION = Save Waypoint diminta dari dashboard")
    request_rc_save_waypoint_setup(channel)


if __name__ == '__main__':
    listener_thread = threading.Thread(target=mavlink_listener, daemon=True)
    listener_thread.start()

    # --- Floating Ball Counter (dimatikan sementara, fokus dulu ke mode & armed status) ---
    # if ENABLE_BALL_DETECTION:
    #     camera_thread = threading.Thread(target=camera_detection_loop, daemon=True)
    #     camera_thread.start()

    print("=" * 60)
    print(" Pixhawk Monitor jalan di http://0.0.0.0:5000")
    print(" Buka di browser Legion Go: http://localhost:5000")
    print(" Atau dari HP/laptop lain di jaringan yang sama: http://<IP-Legion-Go>:5000")
    print("=" * 60)

    socketio.run(app, host='0.0.0.0', port=5000, debug=True, use_reloader=False, allow_unsafe_werkzeug=True)