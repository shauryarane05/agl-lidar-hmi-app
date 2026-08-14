import 'dart:async';
import 'dart:convert';
import 'dart:math';
import 'dart:typed_data';
import 'dart:ui' show PointMode;
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

// ================= CONFIG (edit + hot-reload) =================
// Self-contained demo: a recorded LiDAR bag (shipped under rosbag/) is played
// back on-device and bridged locally, so rosbridge_server runs on the same
// board as the app. Point at localhost. (For a live/remote source, set this to
// the host running rosbridge_server, e.g. ws://<host-ip>:9090.)
const String kRosbridgeUrl = 'ws://127.0.0.1:9090';

// The shipped bag publishes /carla/lidar as PointCloud2 (x,y,z,intensity).
// For a 2D LaserScan source instead, use '/scan' + 'sensor_msgs/msg/LaserScan'.
const String kLidarTopic = '/carla/lidar';
const String kLidarType = 'sensor_msgs/msg/PointCloud2';

const double kMaxRange = 50.0; // metres shown (matches CARLA lidar range)

// Height band (metres, relative to the sensor) kept for PointCloud2.
// The CARLA lidar sits ~2 m up, so the ground returns near z=-2 — drop them
// to declutter the top-down view; keep walls/cars/poles.
const double kMinZ = -1.6;
const double kMaxZ = 6.0;

const Color kNear = Color(0xFF00E5C3); // close returns  (warm/teal)
const Color kFar = Color(0xFF1E5AFF); // distant returns (cool/blue)
// =============================================================

void main() => runApp(const AglLidarApp());

class AglLidarApp extends StatelessWidget {
  const AglLidarApp({super.key});
  @override
  Widget build(BuildContext context) => MaterialApp(
        title: 'AGL LiDAR',
        debugShowCheckedModeBanner: false,
        theme: ThemeData.dark(useMaterial3: true),
        home: const LidarScreen(),
      );
}

/// A single LiDAR return, projected to the ground plane (x forward, y left),
/// metres. [i] is a 0..1 colour scalar (1 = close, 0 = far).
class Pt {
  final double x, y, i;
  const Pt(this.x, this.y, this.i);
}

enum Source { sim, live }

class _LiveState {
  static const connecting = 'connecting';
  static const streaming = 'streaming';
  static const waiting = 'waiting';
  static const down = 'down';
}

class LidarScreen extends StatefulWidget {
  const LidarScreen({super.key});
  @override
  State<LidarScreen> createState() => _LidarScreenState();
}

class _LidarScreenState extends State<LidarScreen>
    with SingleTickerProviderStateMixin {
  Source _source = Source.sim;
  List<Pt> _points = const [];
  String _status = 'simulated scene';
  String _live = _LiveState.connecting;
  int _hz = 0;
  int _frames = 0;
  double _lastFrameAt = 0; // seconds, monotonic-ish
  final Stopwatch _clock = Stopwatch()..start();

  final FocusNode _focus = FocusNode();

  // radar sweep animation (keeps the display feeling alive even on static data)
  late final AnimationController _spin =
      AnimationController(vsync: this, duration: const Duration(seconds: 4))
        ..repeat();

  // sim
  Timer? _simTimer;
  double _t = 0;
  // live
  WebSocketChannel? _ch;
  StreamSubscription? _sub;
  Timer? _hzTimer;

  @override
  void initState() {
    super.initState();
    // Auto-start in LIVE so CARLA data renders on launch. Falls back to
    // DOWN/waiting if rosbridge is unreachable; press S or Space for SIM.
    _source = Source.live;
    _startLive();
    _hzTimer = Timer.periodic(const Duration(seconds: 1), (_) {
      setState(() {
        _hz = _frames;
        _frames = 0;
        // if live and no frame for >1.5s, flag it as waiting
        if (_source == Source.live &&
            _live == _LiveState.streaming &&
            _clock.elapsedMilliseconds / 1000 - _lastFrameAt > 1.5) {
          _live = _LiveState.waiting;
        }
      });
    });
  }

  void _onKey(KeyEvent e) {
    if (e is! KeyDownEvent) return;
    final k = e.logicalKey;
    if (k == LogicalKeyboardKey.space ||
        k == LogicalKeyboardKey.keyG ||
        k == LogicalKeyboardKey.enter) {
      _toggle(); // toggle SIM <-> LIVE
    } else if (k == LogicalKeyboardKey.keyL && _source == Source.sim) {
      _toggle(); // L -> go live
    } else if (k == LogicalKeyboardKey.keyS && _source == Source.live) {
      _toggle(); // S -> back to sim
    }
  }

  @override
  void dispose() {
    _focus.dispose();
    _spin.dispose();
    _simTimer?.cancel();
    _hzTimer?.cancel();
    _sub?.cancel();
    _ch?.sink.close();
    super.dispose();
  }

  // ---------------- simulated LiDAR ----------------
  void _startSim() {
    _simTimer?.cancel();
    _status = 'simulated scene — corridor + moving traffic';
    _simTimer = Timer.periodic(const Duration(milliseconds: 66), (_) {
      _t += 0.066;
      _setPoints(_simulateSweep(_t));
    });
  }

  List<Pt> _simulateSweep(double t) {
    const n = 720;
    final pts = <Pt>[];
    final obstacles = <List<double>>[
      [16 + 6 * sin(t * 0.6), 3.0 * sin(t * 0.9), 1.4], // car weaving ahead
      [10, -3.4, 0.9],
      [28, 2.0 + 3 * cos(t * 0.5), 1.1],
      [38, -2.0, 1.2],
    ];
    final rnd = Random(1);
    for (int k = 0; k < n; k++) {
      final a = (k / n) * 2 * pi - pi;
      final dx = cos(a), dy = sin(a);
      double best = kMaxRange;
      for (final wall in _wallHits(a)) {
        if (wall < best) best = wall;
      }
      for (final o in obstacles) {
        final r = _rayCircle(dx, dy, o[0], o[1], o[2]);
        if (r != null && r < best) best = r;
      }
      if (best >= kMaxRange) continue;
      best += (rnd.nextDouble() - 0.5) * 0.05;
      pts.add(Pt(dx * best, dy * best, _rangeToColor(best)));
    }
    return pts;
  }

  List<double> _wallHits(double a) {
    final hits = <double>[];
    final dx = cos(a), dy = sin(a);
    void planeY(double yw) {
      if (dy.abs() < 1e-6) return;
      final r = yw / dy;
      if (r > 0) {
        final x = dx * r;
        if (x > -10 && x < 45) hits.add(r);
      }
    }

    void planeX(double xw) {
      if (dx.abs() < 1e-6) return;
      final r = xw / dx;
      if (r > 0) {
        final y = dy * r;
        if (y > -7 && y < 7) hits.add(r);
      }
    }

    planeY(7);
    planeY(-7);
    planeX(45);
    planeX(-10);
    return hits;
  }

  double? _rayCircle(double dx, double dy, double cx, double cy, double rad) {
    final b = -(dx * cx + dy * cy);
    final c = cx * cx + cy * cy - rad * rad;
    final disc = b * b - c;
    if (disc < 0) return null;
    final t = -b - sqrt(disc);
    return t > 0 ? t : null;
  }

  // ---------------- live rosbridge ----------------
  void _startLive() {
    _simTimer?.cancel();
    _live = _LiveState.connecting;
    setState(() {
      _status = 'connecting to $kRosbridgeUrl';
      _points = const [];
    });
    try {
      final ch = WebSocketChannel.connect(Uri.parse(kRosbridgeUrl));
      _ch = ch;
      ch.sink.add(jsonEncode({
        'op': 'subscribe',
        'topic': kLidarTopic,
        'type': kLidarType,
        'throttle_rate': 0, // ms; 0 = as fast as it arrives
        'queue_length': 1, // only ever hold the newest frame
      }));
      _sub = ch.stream.listen(
        _onRosMessage,
        onError: (e) => setState(() {
          _live = _LiveState.down;
          _status = 'error: $e';
        }),
        onDone: () => setState(() {
          _live = _LiveState.down;
          _status = 'disconnected — is rosbridge_server running + $kLidarTopic '
              'being published?';
        }),
      );
      setState(() {
        _live = _LiveState.waiting;
        _status = 'subscribed $kLidarTopic — waiting for data';
      });
    } catch (e) {
      setState(() {
        _live = _LiveState.down;
        _status = 'connect failed: $e';
      });
    }
  }

  void _stopLive() {
    _sub?.cancel();
    _ch?.sink.close();
    _sub = null;
    _ch = null;
  }

  void _onRosMessage(dynamic raw) {
    try {
      final data = jsonDecode(raw as String) as Map<String, dynamic>;
      if (data['op'] != 'publish') return;
      final msg = data['msg'] as Map<String, dynamic>;
      List<Pt> pts;
      if (msg.containsKey('fields') && msg.containsKey('data')) {
        pts = _parsePointCloud2(msg); // CARLA /carla/lidar
      } else if (msg.containsKey('ranges')) {
        pts = _parseLaserScan(msg); // /scan
      } else if (msg.containsKey('points')) {
        pts = _parsePointList(msg['points']); // generic {points:[[x,y]]}
      } else {
        return;
      }
      _lastFrameAt = _clock.elapsedMilliseconds / 1000;
      _live = _LiveState.streaming;
      _status = 'LIVE · $kLidarTopic';
      _setPoints(pts);
    } catch (_) {/* ignore a malformed frame */}
  }

  /// Decode a rosbridge PointCloud2. `data` arrives base64-encoded; each point
  /// is `point_step` bytes of little-endian float32 fields. We read x/y/z/
  /// intensity by their declared offsets, drop ground/roof returns by height,
  /// project to the ground plane and colour by range.
  List<Pt> _parsePointCloud2(Map<String, dynamic> m) {
    final rawData = m['data'];
    Uint8List bytes;
    if (rawData is String) {
      bytes = base64Decode(rawData);
    } else if (rawData is List) {
      bytes = Uint8List.fromList(rawData.cast<int>());
    } else {
      return const [];
    }
    final step = (m['point_step'] as num?)?.toInt() ?? 16;
    if (step <= 0 || bytes.length < step) return const [];

    int ox = 0, oy = 4, oz = 8;
    final fields = (m['fields'] as List?) ?? const [];
    for (final f in fields) {
      final fm = f as Map;
      final off = (fm['offset'] as num).toInt();
      switch (fm['name']) {
        case 'x':
          ox = off;
          break;
        case 'y':
          oy = off;
          break;
        case 'z':
          oz = off;
          break;
      }
    }

    final bd = ByteData.sublistView(bytes);
    final n = bytes.length ~/ step;
    // cap rendered points for a smooth top-down view (CARLA gives ~12k/frame)
    final stride = n > 6000 ? (n / 6000).ceil() : 1;
    final out = <Pt>[];
    for (int k = 0; k < n; k += stride) {
      final base = k * step;
      final x = bd.getFloat32(base + ox, Endian.little);
      final y = bd.getFloat32(base + oy, Endian.little);
      final z = bd.getFloat32(base + oz, Endian.little);
      if (z < kMinZ || z > kMaxZ) continue; // drop ground & high roofs
      final d = sqrt(x * x + y * y);
      if (d < 0.6 || d > kMaxRange) continue; // drop ego self-hits & far
      out.add(Pt(x, y, _rangeToColor(d)));
    }
    return out;
  }

  List<Pt> _parseLaserScan(Map<String, dynamic> m) {
    final ranges = (m['ranges'] as List).cast<num>();
    final aMin = (m['angle_min'] as num).toDouble();
    final aInc = (m['angle_increment'] as num).toDouble();
    final out = <Pt>[];
    for (int i = 0; i < ranges.length; i++) {
      final r = ranges[i].toDouble();
      if (r.isNaN || r.isInfinite || r <= 0 || r > kMaxRange) continue;
      final a = aMin + i * aInc;
      out.add(Pt(r * cos(a), r * sin(a), _rangeToColor(r)));
    }
    return out;
  }

  List<Pt> _parsePointList(dynamic list) {
    final out = <Pt>[];
    for (final p in (list as List)) {
      double x, y;
      if (p is List) {
        x = (p[0] as num).toDouble();
        y = (p[1] as num).toDouble();
      } else {
        final pm = p as Map;
        x = (pm['x'] as num).toDouble();
        y = (pm['y'] as num).toDouble();
      }
      final d = sqrt(x * x + y * y);
      if (d > kMaxRange) continue;
      out.add(Pt(x, y, _rangeToColor(d)));
    }
    return out;
  }

  double _rangeToColor(double d) => (1.0 - d / kMaxRange).clamp(0.0, 1.0);

  void _setPoints(List<Pt> pts) {
    _frames++;
    setState(() => _points = pts);
  }

  void _toggle() {
    setState(() {
      if (_source == Source.sim) {
        _source = Source.live;
        _startLive();
      } else {
        _source = Source.sim;
        _stopLive();
        _startSim();
      }
    });
  }

  ({Color color, String label}) get _badge {
    if (_source == Source.sim) return (color: kNear, label: 'SIM');
    switch (_live) {
      case _LiveState.streaming:
        return (color: const Color(0xFF35D06B), label: 'LIVE');
      case _LiveState.waiting:
      case _LiveState.connecting:
        return (color: const Color(0xFFF5A623), label: 'LINK');
      default:
        return (color: Colors.redAccent, label: 'DOWN');
    }
  }

  @override
  Widget build(BuildContext context) {
    final live = _source == Source.live;
    final badge = _badge;
    final showWaitOverlay = live &&
        _points.isEmpty &&
        (_live == _LiveState.waiting ||
            _live == _LiveState.connecting ||
            _live == _LiveState.down);

    return KeyboardListener(
      focusNode: _focus,
      autofocus: true,
      onKeyEvent: _onKey,
      child: Scaffold(
        backgroundColor: const Color(0xFF060A0F),
        body: Stack(
          children: [
            Positioned.fill(
              child: CustomPaint(
                painter: LidarPainter(_points, _spin),
              ),
            ),
            if (showWaitOverlay) _waitOverlay(),
            _legend(),
            _topBar(badge),
            _bottomBar(live, badge),
          ],
        ),
      ),
    );
  }

  Widget _topBar(({Color color, String label}) badge) => Positioned(
        top: 0,
        left: 0,
        right: 0,
        child: Container(
          padding: const EdgeInsets.fromLTRB(24, 16, 24, 16),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.topCenter,
              end: Alignment.bottomCenter,
              colors: [Colors.black.withOpacity(0.55), Colors.transparent],
            ),
          ),
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Row(children: [
                Icon(Icons.radar, color: badge.color, size: 26),
                const SizedBox(width: 10),
                const Text('AGL · LiDAR',
                    style: TextStyle(
                        fontSize: 24,
                        fontWeight: FontWeight.w800,
                        letterSpacing: 1.0)),
              ]),
              Row(children: [
                _chip('${_points.length} pts', Colors.white70),
                const SizedBox(width: 10),
                _chip('$_hz Hz', Colors.white70),
                const SizedBox(width: 10),
                _statusChip(badge),
              ]),
            ],
          ),
        ),
      );

  Widget _bottomBar(bool live, ({Color color, String label}) badge) =>
      Positioned(
        bottom: 0,
        left: 0,
        right: 0,
        child: Container(
          padding: const EdgeInsets.fromLTRB(24, 18, 24, 18),
          decoration: BoxDecoration(
            gradient: LinearGradient(
              begin: Alignment.bottomCenter,
              end: Alignment.topCenter,
              colors: [Colors.black.withOpacity(0.55), Colors.transparent],
            ),
          ),
          child: Row(
            children: [
              Icon(
                  live
                      ? (badge.label == 'LIVE'
                          ? Icons.check_circle
                          : Icons.sync)
                      : Icons.play_circle_fill,
                  size: 18,
                  color: badge.color),
              const SizedBox(width: 8),
              Expanded(
                child: Text(_status,
                    style:
                        const TextStyle(fontSize: 15, color: Colors.white70),
                    overflow: TextOverflow.ellipsis),
              ),
              const SizedBox(width: 16),
              FilledButton.icon(
                onPressed: _toggle,
                icon: Icon(live ? Icons.stop : Icons.sensors, size: 20),
                label: Text(live ? 'Use SIM  [space]' : 'Go LIVE  [space]'),
                style: FilledButton.styleFrom(
                    backgroundColor: live ? Colors.white24 : kNear,
                    foregroundColor: live ? Colors.white : Colors.black,
                    padding:
                        const EdgeInsets.symmetric(horizontal: 20, vertical: 14),
                    textStyle: const TextStyle(
                        fontSize: 15, fontWeight: FontWeight.w700)),
              ),
            ],
          ),
        ),
      );

  Widget _waitOverlay() => Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            const SizedBox(
                width: 34,
                height: 34,
                child: CircularProgressIndicator(strokeWidth: 3)),
            const SizedBox(height: 18),
            Text(
              _live == _LiveState.down
                  ? 'No connection'
                  : 'Waiting for $kLidarTopic …',
              style: const TextStyle(fontSize: 18, color: Colors.white70),
            ),
            const SizedBox(height: 6),
            const Text('Start rosbridge_server + play/stream the topic',
                style: TextStyle(fontSize: 13, color: Colors.white38)),
          ],
        ),
      );

  Widget _legend() => Positioned(
        left: 24,
        bottom: 78,
        child: Row(children: [
          const Text('near',
              style: TextStyle(color: Colors.white38, fontSize: 12)),
          const SizedBox(width: 8),
          Container(
            width: 90,
            height: 8,
            decoration: BoxDecoration(
              borderRadius: BorderRadius.circular(4),
              gradient: const LinearGradient(colors: [kNear, kFar]),
            ),
          ),
          const SizedBox(width: 8),
          const Text('far',
              style: TextStyle(color: Colors.white38, fontSize: 12)),
        ]),
      );

  Widget _statusChip(({Color color, String label}) badge) => Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
        decoration: BoxDecoration(
            border: Border.all(color: badge.color.withOpacity(0.7)),
            borderRadius: BorderRadius.circular(20)),
        child: Row(children: [
          Container(
              width: 8,
              height: 8,
              decoration:
                  BoxDecoration(color: badge.color, shape: BoxShape.circle)),
          const SizedBox(width: 7),
          Text(badge.label,
              style: TextStyle(
                  color: badge.color,
                  fontWeight: FontWeight.w800,
                  fontSize: 14)),
        ]),
      );

  Widget _chip(String s, Color c) => Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
        decoration: BoxDecoration(
            border: Border.all(color: c.withOpacity(0.5)),
            borderRadius: BorderRadius.circular(20)),
        child: Text(s,
            style: TextStyle(
                color: c, fontWeight: FontWeight.w700, fontSize: 14)),
      );
}

class LidarPainter extends CustomPainter {
  final List<Pt> points;
  final Animation<double> spin;
  LidarPainter(this.points, this.spin) : super(repaint: spin);

  static const int _bands = 6;

  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final scale = (min(size.width, size.height) / 2 - 44) / kMaxRange;

    _drawGrid(canvas, size, center, scale);
    _drawSweep(canvas, center, scale);
    _drawPoints(canvas, center, scale);
    _drawEgo(canvas, center);
  }

  void _drawGrid(Canvas canvas, Size size, Offset center, double scale) {
    final ring = Paint()
      ..color = Colors.white.withOpacity(0.09)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;
    final tp = TextPainter(textDirection: TextDirection.ltr);

    final step = kMaxRange <= 20 ? 5.0 : (kMaxRange <= 60 ? 10.0 : 20.0);
    for (double m = step; m <= kMaxRange + 0.1; m += step) {
      canvas.drawCircle(center, m * scale, ring);
      tp.text = TextSpan(
          text: '${m.toInt()}m',
          style: const TextStyle(color: Colors.white24, fontSize: 12));
      tp.layout();
      tp.paint(canvas, center + Offset(5, -m * scale - 14));
    }
    // cross axes
    canvas.drawLine(
        Offset(center.dx, 0), Offset(center.dx, size.height), ring);
    canvas.drawLine(Offset(0, center.dy), Offset(size.width, center.dy), ring);

    // forward label
    tp.text = const TextSpan(
        text: 'FWD',
        style: TextStyle(
            color: Colors.white38, fontSize: 12, fontWeight: FontWeight.w700));
    tp.layout();
    tp.paint(canvas, Offset(center.dx + 6, 8));
  }

  void _drawSweep(Canvas canvas, Offset center, double scale) {
    final ang = spin.value * 2 * pi - pi / 2; // rotate; start pointing up
    final r = kMaxRange * scale;
    final rect = Rect.fromCircle(center: center, radius: r);
    // a soft wedge trailing the leading edge
    const sweepWidth = 0.5; // radians
    final shader = SweepGradient(
      startAngle: ang - sweepWidth,
      endAngle: ang,
      colors: [Colors.transparent, kNear.withOpacity(0.16)],
      transform: GradientRotation(0),
    ).createShader(rect);
    final wedge = Paint()..shader = shader;
    final path = Path()
      ..moveTo(center.dx, center.dy)
      ..arcTo(rect, ang - sweepWidth, sweepWidth, false)
      ..close();
    canvas.drawPath(path, wedge);
    // leading line
    final lead = Paint()
      ..color = kNear.withOpacity(0.5)
      ..strokeWidth = 2;
    canvas.drawLine(
        center, center + Offset(cos(ang) * r, sin(ang) * r), lead);
  }

  void _drawPoints(Canvas canvas, Offset center, double scale) {
    if (points.isEmpty) return;
    // Bucket points into colour bands and draw each band in one batched call
    // (drawRawPoints) — far faster than a drawCircle per point.
    final buckets = List.generate(_bands, (_) => <double>[]);
    for (final pt in points) {
      // sensor x fwd -> screen up, y left -> screen left
      final sx = center.dx - pt.y * scale;
      final sy = center.dy - pt.x * scale;
      int b = (pt.i * _bands).floor();
      if (b < 0) b = 0;
      if (b >= _bands) b = _bands - 1;
      buckets[b]
        ..add(sx)
        ..add(sy);
    }
    for (int b = 0; b < _bands; b++) {
      final coords = buckets[b];
      if (coords.isEmpty) continue;
      final v = (b + 0.5) / _bands;
      final paint = Paint()
        ..color = _heat(v)
        ..strokeCap = StrokeCap.round
        ..strokeWidth = 3.4;
      canvas.drawRawPoints(
          PointMode.points, Float32List.fromList(coords), paint);
    }
  }

  void _drawEgo(Canvas canvas, Offset center) {
    // glow
    canvas.drawCircle(
        center, 16, Paint()..color = kNear.withOpacity(0.18));
    // car-ish arrow pointing forward (up)
    final body = Paint()..color = kNear;
    final path = Path()
      ..moveTo(center.dx, center.dy - 15)
      ..lineTo(center.dx - 9, center.dy + 11)
      ..lineTo(center.dx, center.dy + 5)
      ..lineTo(center.dx + 9, center.dy + 11)
      ..close();
    canvas.drawPath(path, body);
  }

  Color _heat(double v) => Color.lerp(kFar, kNear, v)!;

  @override
  bool shouldRepaint(covariant LidarPainter old) => true;
}
