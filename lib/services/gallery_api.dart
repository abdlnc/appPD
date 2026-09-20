import 'dart:convert';
import 'package:http/http.dart' as http;
import '../models/geo_image.dart';

/// Read-only HTTP access to gallery_server.py on the Pi (port 8080).
///
/// Separate from RobotSocket's WebSocket (8765) on purpose: the gallery is a
/// plain file server that keeps working whether or not the robot is driving,
/// and nothing here can send the robot a command.
const int kGalleryPort = 8080;

/// Image kinds the server exposes. "maps" is the saved SLAM maps, which are
/// not per-photo geotagged and so are not used for map pins.
const List<String> kPhotoKinds = ['captures', 'detections', 'soil'];

String galleryImgUrl(String ip, String kind, String name) =>
    'http://$ip:$kGalleryPort/img/$kind/$name';

/// The gallery server returns only its newest MAX_LIST (200) entries per
/// folder unless asked for more. The GRID is happy with that default, but the
/// MAP must ask for everything: the robot saves a soil photo every minute, so
/// a couple of hours of driving pushes older geotagged photos out of the
/// default window and they would silently disappear from the field map.
/// Server-side cap is 5000.
const int kMapListLimit = 5000;

/// Raw `/list/<kind>` -> the filenames it returned.
Future<List<String>> fetchGalleryNames(String ip, String kind,
    {int? limit, Duration timeout = const Duration(seconds: 6)}) async {
  final q = limit == null ? '' : '?limit=$limit';
  final uri = Uri.parse('http://$ip:$kGalleryPort/list/$kind$q');
  final res = await http.get(uri).timeout(timeout);
  if (res.statusCode != 200) {
    throw Exception('Gallery server returned HTTP ${res.statusCode}');
  }
  final data = jsonDecode(res.body) as List;
  return data
      .map((e) => (e as Map<String, dynamic>)['name'] as String)
      .toList();
}

/// One geotagged photo plus the kind it must be fetched from, since the URL
/// needs the kind and [GeoImage] only knows the filename.
class GalleryPhoto {
  final GeoImage geo;
  final String kind; // "captures" | "detections" | "soil"
  const GalleryPhoto(this.geo, this.kind);

  String url(String ip) => galleryImgUrl(ip, kind, geo.name);
}

/// Every photo the Pi holds, parsed, de-duplicated and ready to map.
///
/// De-duplication matters: the AI saves BOTH a raw capture and an annotated
/// `_detected` copy of the same moment, so without this every obstacle would
/// get two pins stacked on the same spot. The annotated one wins -- on a map
/// of obstructions, the image with the boxes drawn on it is the useful one.
class PhotoIndex {
  final List<GalleryPhoto> photos; // geotagged only
  final int noFixCount; // parsed fine, but had no GPS fix
  const PhotoIndex({required this.photos, required this.noFixCount});

  static const empty = PhotoIndex(photos: [], noFixCount: 0);
}

/// Loads all photo kinds concurrently and indexes them.
///
/// A kind that fails is skipped rather than failing the whole load: a missing
/// soil camera shouldn't wipe the obstacle pins off the map.
Future<PhotoIndex> fetchPhotoIndex(String ip) async {
  // Asks for kMapListLimit per kind so the map reflects the whole archive,
  // not just the most recent 200 photos per folder.
  final results = await Future.wait(
    kPhotoKinds.map((k) async {
      try {
        return MapEntry(
            k, await fetchGalleryNames(ip, k, limit: kMapListLimit));
      } catch (_) {
        return MapEntry(k, <String>[]);
      }
    }),
  );

  // eventKey -> chosen photo. Annotated copies beat raw captures.
  final byEvent = <String, GalleryPhoto>{};
  var noFix = 0;
  var any = false;

  for (final entry in results) {
    for (final name in entry.value) {
      final g = GeoImage.parse(name);
      if (g == null) continue; // not a robot photo (e.g. a SLAM map)
      any = true;
      if (!g.hasFix) {
        noFix++;
        continue;
      }
      final existing = byEvent[g.eventKey];
      if (existing == null || (g.annotated && !existing.geo.annotated)) {
        byEvent[g.eventKey] = GalleryPhoto(g, entry.key);
      }
    }
  }

  if (!any && results.every((e) => e.value.isEmpty)) {
    // Every kind failed or the Pi is empty -- indistinguishable here, and
    // both mean the same thing to the caller: nothing to draw.
    return PhotoIndex.empty;
  }

  final photos = byEvent.values.toList()
    ..sort((a, b) => b.geo.time.compareTo(a.geo.time)); // newest first
  return PhotoIndex(photos: photos, noFixCount: noFix);
}
