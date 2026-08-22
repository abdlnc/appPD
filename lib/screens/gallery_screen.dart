import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'package:http/http.dart' as http;
import 'package:provider/provider.dart';
import '../services/robot_socket.dart';
import '../theme/app_theme.dart';

const int _galleryPort = 8080; // gallery_server.py on the Pi

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
}

/// Saved-images viewer: raw camera captures + AI-annotated detections +
/// downward soil-camera snapshots, served by gallery_server.py (read-only
/// HTTP file server on the Pi).
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
    _tabs = TabController(length: 3, vsync: this);
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
              ],
            ),
    );
  }
}

class _GalleryGrid extends StatefulWidget {
  final String ip;
  final String kind; // "captures" | "detections" | "soil"
  const _GalleryGrid({required this.ip, required this.kind});
  @override
  State<_GalleryGrid> createState() => _GalleryGridState();
}

class _GalleryGridState extends State<_GalleryGrid>
    with AutomaticKeepAliveClientMixin {
  late Future<List<GalleryImage>> _future;

  @override
  bool get wantKeepAlive => true;

  @override
  void initState() {
    super.initState();
    _future = _load();
  }

  Future<List<GalleryImage>> _load() async {
    final uri =
        Uri.parse('http://${widget.ip}:$_galleryPort/list/${widget.kind}');
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
      'http://${widget.ip}:$_galleryPort/img/${widget.kind}/${img.name}';

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
        return RefreshIndicator(
          onRefresh: _refresh,
          child: GridView.builder(
            padding: const EdgeInsets.all(10),
            gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(
              crossAxisCount: 3,
              crossAxisSpacing: 6,
              mainAxisSpacing: 6,
            ),
            itemCount: images.length,
            itemBuilder: (context, i) {
              final url = _imgUrl(images[i]);
              return GestureDetector(
                onTap: () => Navigator.push(
                  context,
                  MaterialPageRoute(
                    builder: (_) => _ImageViewer(
                      images: images,
                      startIndex: i,
                      urlBuilder: _imgUrl,
                    ),
                  ),
                ),
                child: ClipRRect(
                  borderRadius: BorderRadius.circular(8),
                  child: Image.network(
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
                      child:
                          const Icon(Icons.broken_image, color: AppColors.textLo),
                    ),
                  ),
                ),
              );
            },
          ),
        );
      },
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
  const _ImageViewer({
    required this.images,
    required this.startIndex,
    required this.urlBuilder,
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
            child: Text(
              "${_index + 1} / ${widget.images.length}   •   ${_fmtTime(img.time)}",
              textAlign: TextAlign.center,
              style: GoogleFonts.rajdhani(color: Colors.white70),
            ),
          ),
        ],
      ),
    );
  }
}
