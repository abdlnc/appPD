import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import 'package:latlong2/latlong.dart';
import '../models/geo_image.dart';
import '../services/gallery_api.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';
import 'map_screen.dart';

/// One entry from gallery_server.py's `/list/<kind>` response.
class GalleryImage {
  final String name;
  final int size;
  final double mtime;
  GalleryImage({required this.name, required this.size, required this.mtime});

  factory GalleryImage.fromJson(Map<String, dynamic> j) => GalleryImage(
        name: j['name'] as String,
        size: (j['size'] as num).toInt(),
        mtime: (j['mtime'] as num).toDouble(),
      );

  DateTime get time =>
      DateTime.fromMillisecondsSinceEpoch((mtime * 1000).round());

  /// Capture time + GPS position, parsed out of the filename (the Pi encodes
  /// both there -- see GeoImage). Null for anything that isn't one of the
  /// robot's photos, e.g. a saved SLAM map.
  GeoImage? get geo => GeoImage.parse(name);

  /// Prefer the filename's stamp over the file's mtime: mtime changes when
  /// the folder is copied or synced, the stamp is when the photo was taken.
  DateTime get captureTime => geo?.time ?? time;
}

/// Saved-images viewer: raw camera captures + AI-annotated detections +
/// downward soil-camera snapshots + saved SLAM maps, served by
/// gallery_server.py (read-only HTTP file server on the Pi).
class GalleryScreen extends StatefulWidget {
  const GalleryScreen({super.key});
  @override
  State<GalleryScreen> createState() => _GalleryScreenState();
}

class _GalleryScreenState extends State<GalleryScreen>
    with SingleTickerProviderStateMixin {
  late final TabController _tabs;

  @override
  void initState() {
    super.initState();
    _tabs = TabController(length: 4, vsync: this);
  }

  @override
  void dispose() {
    _tabs.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final ip = context.read<RobotSocket>().hostIp;
    return Scaffold(
      backgroundColor: AppColors.bg,
      appBar: AppBar(
        backgroundColor: AppColors.surface,
        foregroundColor: AppColors.textHi,
        title: Text("SAVED IMAGES",
            style: GoogleFonts.rajdhani(
                fontWeight: FontWeight.w700, letterSpacing: 1.5)),
        bottom: TabBar(
          controller: _tabs,
          labelColor: AppColors.lime,
          unselectedLabelColor: AppColors.textLo,
          indicatorColor: AppColors.lime,
          labelStyle:
              GoogleFonts.rajdhani(fontWeight: FontWeight.w700, letterSpacing: 1),
          tabs: const [
            Tab(text: "CAPTURES"),
            Tab(text: "DETECTIONS"),
            Tab(text: "SOIL"),
            Tab(text: "MAPS"),
          ],
        ),
      ),
      body: ip == null
          ? Center(
              child: Padding(
                padding: const EdgeInsets.all(24),
                child: Text(
                  "Not connected — set the Pi IP in the Connection screen",
                  textAlign: TextAlign.center,
                  style:
                      GoogleFonts.rajdhani(color: AppColors.textLo, fontSize: 15),
                ),
              ),
            )
          : TabBarView(
              controller: _tabs,
              children: [
                _GalleryGrid(ip: ip, kind: "captures"),
                _GalleryGrid(ip: ip, kind: "detections"),
                _GalleryGrid(ip: ip, kind: "soil"),
                _GalleryGrid(ip: ip, kind: "maps"),
              ],
            ),
    );
  }
}

class _GalleryGrid extends StatefulWidget {
  final String ip;
  final String kind; // "captures" | "detections" | "soil" | "maps"
  const _GalleryGrid({required this.ip, required this.kind});
  @override
  State<_GalleryGrid> createState() => _GalleryGridState();
}

class _GalleryGridState extends State<_GalleryGrid>
    with AutomaticKeepAliveClientMixin {
  late Future<List<GalleryImage>> _future;

  /// Session key currently being shown, or null for all of them.
  String? _sessionFilter;

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    _future = _load();
  }

  Future<List<GalleryImage>> _load() async {
    // Sessions only make sense over the whole archive: with the server's
    // default 200-image window, an older run would appear half-empty or not
    // at all, and the grouping would silently lie about what a session held.
    final uri = Uri.parse(
        'http://${widget.ip}:$kGalleryPort/list/${widget.kind}'
        '?limit=$kMapListLimit');
    final res = await http.get(uri).timeout(const Duration(seconds: 6));
    if (res.statusCode != 200) {
      throw Exception('Gallery server returned HTTP ${res.statusCode}');
    }
    final data = jsonDecode(res.body) as List;
    return data
        .map((e) => GalleryImage.fromJson(e as Map<String, dynamic>))
        .toList();
  }

  Future<void> _refresh() async {
    final next = _load();
    setState(() => _future = next);
    await next;
  }

  String _imgUrl(GalleryImage img) =>
      'http://${widget.ip}:$kGalleryPort/img/${widget.kind}/${img.name}';

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return FutureBuilder<List<GalleryImage>>(
      future: _future,
      builder: (context, snap) {
        if (snap.connectionState != ConnectionState.done) {
          return const Center(
              child: CircularProgressIndicator(color: AppColors.lime));
        }
        if (snap.hasError) {
          return _errorState(snap.error.toString());
        }
        final images = snap.data ?? [];
        if (images.isEmpty) {
          return RefreshIndicator(
            onRefresh: _refresh,
            child: ListView(
              children: [
                SizedBox(
                  height: 320,
                  child: Center(
                    child: Text("No images yet",
                        style: GoogleFonts.rajdhani(color: AppColors.textLo)),
                  ),
                ),
              ],
            ),
          );
        }
        final sections = _sections(images);
        final shown = _sessionFilter == null
            ? sections
            : sections.where((s) => s.key == _sessionFilter).toList();
        return RefreshIndicator(
          onRefresh: _refresh,
          child: CustomScrollView(
            slivers: [
              // Only worth a filter when there is more than one session.
              if (sections.length > 1)
                SliverToBoxAdapter(child: _filterRow(sections)),
              for (final s in shown) ...[
                if (!(sections.length == 1 && s.session == null))
                  SliverToBoxAdapter(child: _sessionHeader(s)),
                SliverPadding(
                  padding: const EdgeInsets.fromLTRB(10, 0, 10, 14),
                  sliver: SliverGrid(
                    gridDelegate:
                        const SliverGridDelegateWithFixedCrossAxisCount(
                      crossAxisCount: 3,
                      crossAxisSpacing: 6,
                      mainAxisSpacing: 6,
                    ),
                    delegate: SliverChildBuilderDelegate(
                      childCount: s.items.length,
                      (context, i) => _tile(s.items, i),
                    ),
                  ),
                ),
              ],
            ],
          ),
        );
      },
    );
  }

  /// A session and the images in it, as the grid renders them.
  _Section _sectionOf(PhotoSession? session, List<GalleryImage> items) =>
      _Section(session: session, items: items);

  /// Groups this tab's images into sessions, newest first.
  ///
  /// Files whose names don't parse at all -- the SLAM maps in the MAPS tab --
  /// have no capture time to group by, so they land in one trailing section
  /// that renders without a header.
  List<_Section> _sections(List<GalleryImage> images) {
    final byName = {for (final g in images) g.name: g};
    final geos = <GeoImage>[
      for (final g in images)
        if (g.geo != null) g.geo!,
    ];

    // SLAM maps are grouped by DATE, not by session: a map is the product of
    // a whole run rather than a moment inside one, and several saved on the
    // same day belong together as one album.
    if (widget.kind == 'maps') {
      final byDay = <String, List<GalleryImage>>{};
      for (final g in geos) {
        final d = g.time;
        final key = '${d.year}-${d.month.toString().padLeft(2, '0')}'
            '-${d.day.toString().padLeft(2, '0')}';
        if (byName[g.name] != null) {
          byDay.putIfAbsent(key, () => []).add(byName[g.name]!);
        }
      }
      final days = byDay.keys.toList()..sort((a, b) => b.compareTo(a));
      final out = <_Section>[
        for (final k in days)
          _Section(
            session: null,
            items: byDay[k]!..sort((a, b) =>
                b.captureTime.compareTo(a.captureTime)),
            titleOverride: _dayLabel(byDay[k]!.first.captureTime),
            keyOverride: k,
          ),
      ];
      final rest = images.where((g) => g.geo == null).toList();
      if (rest.isNotEmpty) out.add(_sectionOf(null, rest));
      return out.where((s) => s.items.isNotEmpty).toList();
    }

    final out = <_Section>[
      for (final s in groupBySession(geos))
        _sectionOf(s, <GalleryImage>[
          for (final g in s.images)
            if (byName[g.name] != null) byName[g.name]!,
        ]),
    ];
    final unparsed = images.where((g) => g.geo == null).toList();
    if (unparsed.isNotEmpty) out.add(_sectionOf(null, unparsed));
    return out.where((s) => s.items.isNotEmpty).toList();
  }

  /// "26 September 2026" -- the album date on a group of saved maps.
  static String _dayLabel(DateTime t) {
    const months = [
      'January', 'February', 'March', 'April', 'May', 'June', 'July',
      'August', 'September', 'October', 'November', 'December',
    ];
    return '${t.day} ${months[t.month - 1]} ${t.year}';
  }

  /// Confirm, then delete on the Pi and refresh the tab.
  Future<void> _confirmDelete(GalleryImage img) async {
    final twin = widget.kind == 'detections' || widget.kind == 'captures';
    final ok = await showDialog<bool>(
      context: context,
      builder: (ctx) => AlertDialog(
        backgroundColor: AppColors.surface,
        title: Text("Delete this image?",
            style: GoogleFonts.rajdhani(
                color: AppColors.textHi, fontWeight: FontWeight.w700)),
        content: Text(
          twin
              ? "${img.name}\n\nIts matching raw/annotated copy goes too, so "
                  "the same moment doesn't stay behind in the other tab.\n\n"
                  "This cannot be undone."
              : "${img.name}\n\nThis cannot be undone.",
          style: GoogleFonts.rajdhani(color: AppColors.textLo),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(ctx, false),
            child: Text("CANCEL",
                style: GoogleFonts.rajdhani(color: AppColors.textLo)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(ctx, true),
            child: Text("DELETE",
                style: GoogleFonts.rajdhani(
                    color: const Color(0xFFE53935),
                    fontWeight: FontWeight.w700)),
          ),
        ],
      ),
    );
    if (ok != true || !mounted) return;
    try {
      final gone =
          await deleteGalleryImage(widget.ip, widget.kind, img.name);
      if (!mounted) return;
      ScaffoldMessenger.of(context)
        ..hideCurrentSnackBar()
        ..showSnackBar(SnackBar(
          behavior: SnackBarBehavior.floating,
          content: Text(
            gone.length > 1 ? "Deleted ${gone.length} files" : "Deleted",
            style: GoogleFonts.rajdhani(fontWeight: FontWeight.w600),
          ),
        ));
      await _refresh();
    } catch (e) {
      if (!mounted) return;
      ScaffoldMessenger.of(context)
        ..hideCurrentSnackBar()
        ..showSnackBar(SnackBar(
          behavior: SnackBarBehavior.floating,
          backgroundColor: const Color(0xFFE53935),
          content: Text("Could not delete: $e",
              style: GoogleFonts.rajdhani(
                  color: Colors.white, fontWeight: FontWeight.w600)),
        ));
    }
  }

  Widget _filterRow(List<_Section> sections) {
    return SizedBox(
      height: 52,
      child: ListView(
        scrollDirection: Axis.horizontal,
        padding: const EdgeInsets.fromLTRB(10, 8, 10, 6),
        children: [
          _chip("ALL", null, sections.fold<int>(0, (n, s) => n + s.count)),
          for (final s in sections)
            _chip(s.label.toUpperCase(), s.key, s.count,
                inferred: s.session?.inferred ?? false),
        ],
      ),
    );
  }

  Widget _chip(String label, String? key, int count, {bool inferred = false}) {
    final on = _sessionFilter == key;
    return Padding(
      padding: const EdgeInsets.only(right: 6),
      child: GestureDetector(
        onTap: () => setState(() => _sessionFilter = key),
        child: Container(
          padding: const EdgeInsets.symmetric(horizontal: 11, vertical: 7),
          decoration: BoxDecoration(
            color: on ? AppColors.lime : AppColors.surface,
            borderRadius: BorderRadius.circular(18),
            border: Border.all(color: on ? AppColors.lime : AppColors.border),
          ),
          child: Row(
            children: [
              if (inferred) ...[
                Icon(Icons.schedule,
                    size: 12,
                    color: on ? AppColors.onLime : AppColors.textLo),
                const SizedBox(width: 4),
              ],
              Text("$label  $count",
                  style: GoogleFonts.rajdhani(
                    color: on ? AppColors.onLime : AppColors.textHi,
                    fontWeight: FontWeight.w700,
                    fontSize: 13,
                  )),
            ],
          ),
        ),
      ),
    );
  }

  Widget _sessionHeader(_Section s) {
    final ses = s.session;
    return Padding(
      padding: const EdgeInsets.fromLTRB(12, 12, 12, 6),
      child: Row(
        children: [
          Container(width: 3, height: 16, color: AppColors.lime),
          const SizedBox(width: 8),
          Text(
            s.titleOverride?.toUpperCase() ??
                (ses == null ? "NOT SESSION-TAGGED" : s.label.toUpperCase()),
            style: GoogleFonts.rajdhani(
              color: AppColors.textHi,
              fontWeight: FontWeight.w700,
              fontSize: 14,
              letterSpacing: 1.1,
            ),
          ),
          const SizedBox(width: 8),
          Expanded(
            child: Text(
              s.titleOverride != null
                  ? "${s.count} map${s.count == 1 ? '' : 's'}"
                  : ses == null
                      ? "${s.count} file${s.count == 1 ? '' : 's'}"
                  : "${s.count} photo${s.count == 1 ? '' : 's'}"
                      "${ses.withFix > 0 ? ' · ${ses.withFix} with GPS' : ''}"
                      "${ses.inferred ? ' · grouped by time' : ''}",
              overflow: TextOverflow.ellipsis,
              style: GoogleFonts.rajdhani(
                  color: AppColors.textLo, fontSize: 13),
            ),
          ),
        ],
      ),
    );
  }

  /// One thumbnail. The viewer opens on THIS session's images, so swiping
  /// stays inside the run the photo came from.
  Widget _tile(List<GalleryImage> items, int i) {
    final url = _imgUrl(items[i]);
    return GestureDetector(
                onLongPress: () => _confirmDelete(items[i]),
                onTap: () => Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (_) => _ImageViewer(
                      images: items,
                      startIndex: i,
                      urlBuilder: _imgUrl,
                      onDelete: _confirmDelete,
                    ),
                  ),
                ),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(8),
                  child: Stack(
                    fit: StackFit.expand,
                    children: [
                      Image.network(
                    url,
                    fit: BoxFit.cover,
                    loadingBuilder: (context, child, progress) =>
                        progress == null
                            ? child
                            : Container(
                                color: AppColors.surfaceHi,
                                child: const Center(
                                  child: SizedBox(
                                    width: 20,
                                    height: 20,
                                    child: CircularProgressIndicator(
                                        strokeWidth: 2, color: AppColors.lime),
                                  ),
                                ),
                              ),
                        errorBuilder: (context, error, stack) => Container(
                          color: AppColors.surfaceHi,
                          child: const Icon(Icons.broken_image,
                              color: AppColors.textLo),
                        ),
                      ),
                      // Geotag badge: at a glance, which photos carry a
                      // position and can therefore appear on the map.
                      if (items[i].geo != null)
                        Positioned(
                          left: 4,
                          bottom: 4,
                          child: _GeoBadge(geo: items[i].geo!),
                        ),
                    ],
                  ),
                ),
    );
  }

  Widget _errorState(String message) {
    return Center(
      child: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          const Icon(Icons.cloud_off, color: AppColors.textLo, size: 36),
          const SizedBox(height: 10),
          Text("Couldn't reach the gallery server",
              style: GoogleFonts.rajdhani(
                  color: AppColors.textHi, fontWeight: FontWeight.w700)),
          const SizedBox(height: 4),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 24),
            child: Text(message,
                textAlign: TextAlign.center,
                style: GoogleFonts.rajdhani(color: AppColors.textLo)),
          ),
          const SizedBox(height: 14),
          ElevatedButton(onPressed: _refresh, child: const Text("RETRY")),
        ],
      ),
    );
  }
}

class _ImageViewer extends StatefulWidget {
  final List<GalleryImage> images;
  final int startIndex;
  final String Function(GalleryImage) urlBuilder;

  /// Delete the image being viewed. The viewer closes afterwards, because the
  /// list it was paging through no longer matches what is on the Pi.
  final Future<void> Function(GalleryImage)? onDelete;

  const _ImageViewer({
    required this.images,
    required this.startIndex,
    required this.urlBuilder,
    this.onDelete,
  });

  @override
  State<_ImageViewer> createState() => _ImageViewerState();
}

class _ImageViewerState extends State<_ImageViewer> {
  late final PageController _page;
  late int _index;

  @override
  void initState() {
    super.initState();
    _index = widget.startIndex;
    _page = PageController(initialPage: _index);
  }

  @override
  void dispose() {
    _page.dispose();
    super.dispose();
  }

  String _fmtTime(DateTime t) =>
      "${t.year}-${t.month.toString().padLeft(2, '0')}-${t.day.toString().padLeft(2, '0')} "
      "${t.hour.toString().padLeft(2, '0')}:${t.minute.toString().padLeft(2, '0')}:"
      "${t.second.toString().padLeft(2, '0')}";

  @override
  Widget build(BuildContext context) {
    final img = widget.images[_index];
    return Scaffold(
      backgroundColor: Colors.black,
      appBar: AppBar(
        backgroundColor: Colors.black,
        foregroundColor: Colors.white,
        title: Text(img.name,
            overflow: TextOverflow.ellipsis,
            style: GoogleFonts.rajdhani(fontSize: 15)),
        actions: [
          if (widget.onDelete != null)
            IconButton(
              icon: const Icon(Icons.delete_outline),
              tooltip: "Delete this image",
              onPressed: () async {
                final nav = Navigator.of(context);
                await widget.onDelete!(img);
                if (nav.mounted) nav.pop();
              },
            ),
          IconButton(
            icon: const Icon(Icons.place_outlined),
            tooltip: (img.geo?.hasFix ?? false)
                ? "Show where this was taken"
                : "No GPS fix for this photo",
            // Disabled rather than hidden: a greyed-out pin tells you the
            // photo has no position, which is information; a missing button
            // just looks like the feature is broken.
            onPressed: (img.geo?.hasFix ?? false)
                ? () => Navigator.push(
                      context,
                      MaterialPageRoute(
                        builder: (_) => MapScreen(
                          focus: LatLng(img.geo!.lat!, img.geo!.lon!),
                          focusLabel: img.name,
                        ),
                      ),
                    )
                : null,
          ),
        ],
      ),
      body: Column(
        children: [
          Expanded(
            child: PageView.builder(
              controller: _page,
              itemCount: widget.images.length,
              onPageChanged: (i) => setState(() => _index = i),
              itemBuilder: (context, i) {
                final im = widget.images[i];
                return InteractiveViewer(
                  minScale: 1,
                  maxScale: 5,
                  child: Center(
                    child: Image.network(
                      widget.urlBuilder(im),
                      fit: BoxFit.contain,
                      errorBuilder: (context, error, stack) => const Icon(
                          Icons.broken_image,
                          color: Colors.white38,
                          size: 48),
                    ),
                  ),
                );
              },
            ),
          ),
          Container(
            width: double.infinity,
            padding: const EdgeInsets.symmetric(vertical: 10),
            color: const Color(0xFF111111),
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                Text(
                  "${_index + 1} / ${widget.images.length}   •   "
                  "${_fmtTime(img.captureTime)}",
                  textAlign: TextAlign.center,
                  style: GoogleFonts.rajdhani(color: Colors.white70),
                ),
                if (img.geo != null) ...[
                  const SizedBox(height: 2),
                  Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(
                        img.geo!.hasFix
                            ? Icons.place
                            : Icons.location_disabled,
                        size: 14,
                        color: img.geo!.hasFix
                            ? AppColors.lime
                            : Colors.white30,
                      ),
                      const SizedBox(width: 4),
                      Text(
                        img.geo!.coordLabel,
                        style: GoogleFonts.rajdhani(
                          color: img.geo!.hasFix
                              ? AppColors.lime
                              : Colors.white38,
                          fontWeight: FontWeight.w600,
                        ),
                      ),
                    ],
                  ),
                ],
              ],
            ),
          ),
        ],
      ),
    );
  }
}


/// Small overlay on a gallery thumbnail: does this photo carry a position?
class _GeoBadge extends StatelessWidget {
  final GeoImage geo;
  const _GeoBadge({required this.geo});

  @override
  Widget build(BuildContext context) {
    final fix = geo.hasFix;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 5, vertical: 2),
      decoration: BoxDecoration(
        color: Colors.black.withValues(alpha: 0.6),
        borderRadius: BorderRadius.circular(6),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            fix ? Icons.place : Icons.location_disabled,
            size: 11,
            color: fix ? AppColors.lime : Colors.white38,
          ),
          const SizedBox(width: 3),
          Text(
            "${geo.time.hour.toString().padLeft(2, '0')}:"
            "${geo.time.minute.toString().padLeft(2, '0')}",
            style: GoogleFonts.rajdhani(
              color: Colors.white,
              fontSize: 10,
              fontWeight: FontWeight.w600,
            ),
          ),
        ],
      ),
    );
  }
}

/// One session's worth of images, as the gallery renders them. `session` is
/// null for files with no capture time in the name (the SLAM maps).
class _Section {
  final PhotoSession? session;
  final List<GalleryImage> items;

  /// Used by the MAPS tab, which groups by calendar date instead of session.
  final String? titleOverride;
  final String? keyOverride;

  const _Section({
    required this.session,
    required this.items,
    this.titleOverride,
    this.keyOverride,
  });

  String get key => keyOverride ?? session?.key ?? 'untagged';
  String get label => titleOverride ?? session?.label ?? 'Other';
  int get count => items.length;
}
