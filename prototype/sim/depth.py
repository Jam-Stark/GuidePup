"""Same-frame RGB-D measurements; no object recognition or navigation decisions."""
from __future__ import annotations

import base64
import colorsys
import io
from collections.abc import Mapping, Sequence

import numpy as np
from PIL import Image, ImageDraw, ImageFont


class DepthFrame:
    """An owned snapshot with depth expressed in its rectified RGB optical frame.

    Optical axes are right/down/forward; base axes are forward/left/up.
    ``camera_to_base`` maps optical 3-D points into the robot base frame.
    The caller supplies synchronized images/calibration and caches this object
    by frame_id. This module's current source is ideal MuJoCo RGB-D rendering.
    """

    columns = 6
    rows = 3
    layer_width_m = 0.5

    def __init__(
        self,
        frame_id: int | str,
        sim_time_s: float,
        pose_world: Sequence[float],
        rgb: np.ndarray,
        depth_z_m: np.ndarray,
        intrinsics: Mapping[str, float],
        camera_to_base: np.ndarray,
        world_from_base: np.ndarray,
        valid_range_m: tuple[float, float] = (0.2, 8.0),
    ):
        self.frame_id = frame_id
        self.sim_time_s = float(sim_time_s)
        self.pose_world = [float(value) for value in pose_world]
        self.rgb = np.asarray(rgb).copy()
        self.depth_z_m = np.asarray(depth_z_m, dtype=np.float32).copy()
        self.camera_to_base = np.asarray(camera_to_base, dtype=np.float32).copy()
        self.world_from_base = np.asarray(world_from_base, dtype=np.float64).copy()
        self.valid_range_m = tuple(float(value) for value in valid_range_m)
        if self.rgb.ndim != 3 or self.rgb.shape[2] != 3 or self.rgb.dtype != np.uint8:
            raise ValueError('rgb must be uint8 [height, width, 3]')
        self.height, self.width = self.rgb.shape[:2]
        if self.depth_z_m.shape != (self.height, self.width):
            raise ValueError('RGB and optical-axis depth must share the same pixel grid')
        if self.camera_to_base.shape != (4, 4):
            raise ValueError('camera_to_base must be a 4x4 optical-to-base transform')
        self.intrinsics = {key: float(intrinsics[key]) for key in ('fx', 'fy', 'cx', 'cy')}
        fx, fy, cx, cy = (self.intrinsics[key] for key in ('fx', 'fy', 'cx', 'cy'))
        near, far = self.valid_range_m
        if fx <= 0 or fy <= 0 or not 0 < near < far:
            raise ValueError('focal lengths and ordered depth limits must be positive')

        # Missing and clipped pixels remain unknown, never points at the origin
        # or at the renderer's far plane. Pixel (0, 0) is the top-left center.
        self.valid_mask = np.isfinite(self.depth_z_m) & (self.depth_z_m > near) & (self.depth_z_m < far)
        v, u = np.nonzero(self.valid_mask)
        z = self.depth_z_m[v, u]
        optical = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)).astype(np.float32)
        self._points_base = optical @ self.camera_to_base[:3, :3].T + self.camera_to_base[:3, 3]
        self._range_m = np.full((self.height, self.width), np.nan, dtype=np.float32)
        self._range_m[v, u] = np.hypot(self._points_base[:, 0], self._points_base[:, 1])

    def points_base(self) -> np.ndarray:
        """Return all valid Nx3 points in meters, without sampling or inflation.

        These include visible floor and robot surfaces. The controller adapter
        owns ground/self filtering, capture-pose alignment and occupancy memory.
        Missing pixels supply no evidence of free space.
        """
        return self._points_base.copy()

    def world_point_from_pixel(self, pixel) -> list[float] | None:
        """Measure one indicated pixel; an unavailable depth remains unknown."""
        u, v = (int(round(value)) for value in pixel)
        if not (0 <= u < self.width and 0 <= v < self.height) or not self.valid_mask[v, u]:
            return None
        z = float(self.depth_z_m[v, u])
        k = self.intrinsics
        optical = np.array([(u-k['cx'])*z/k['fx'], (v-k['cy'])*z/k['fy'], z, 1.])
        return (self.world_from_base @ self.camera_to_base @ optical)[:3].tolist()

    def project_world_point(self, point) -> dict:
        """Reproject a static observed point into this frame; no object identity claim."""
        optical = np.linalg.solve(self.world_from_base @ self.camera_to_base, np.r_[point, 1.])
        if optical[2] <= 0:
            return {'in_view': False, 'point_1000': None}
        k = self.intrinsics
        u = k['fx']*optical[0]/optical[2]+k['cx']
        v = k['fy']*optical[1]/optical[2]+k['cy']
        in_view = bool(0 <= u < self.width and 0 <= v < self.height)
        return {'in_view': in_view,
                'point_1000': [round(float(u/self.width*1000), 1), round(float(v/self.height*1000), 1)] if in_view else None}

    def _metadata(self) -> dict:
        return {
            'frame_id': self.frame_id,
            'sim_time_s': self.sim_time_s,
            'pose_world': self.pose_world,
            'width': self.width,
            'height': self.height,
            'source': 'synthetic_mujoco_rgbd',
            'synthetic': True,
            'distance_definition': 'base_planar_range_m = hypot(x_base,y_base)',
            'valid_optical_depth_range_m': list(self.valid_range_m),
            'layer_width_m': self.layer_width_m,
            'layer_note': 'D IDs identify fixed distance bands, not objects; the layer image shows their actual pixel membership',
            'statistics_note': 'p10/p90 are pixel-distance percentiles (central 80%); not a confidence interval or sensor-error bound',
            'support_bounds_definition': 'Axis-aligned world XYZ minima/maxima of actual valid depth pixels in each layer; not full object extent or entity identity.',
            'bbox_convention': '[x0,y0,x1,y1), original image pixels',
            'unknown_meaning': 'no valid depth measurement; not evidence of free space',
        }

    def _grid_cells(self):
        x_edges = np.linspace(0, self.width, self.columns + 1, dtype=int)
        y_edges = np.linspace(0, self.height, self.rows + 1, dtype=int)
        for row in range(self.rows):
            for column in range(self.columns):
                yield {
                    'cell_id': row * self.columns + column + 1,
                    'bbox_px': [int(x_edges[column]), int(y_edges[row]),
                                int(x_edges[column + 1]), int(y_edges[row + 1])],
                }

    def _bounds(self, bbox_px: Sequence[int]) -> tuple[int, int, int, int]:
        if len(bbox_px) != 4 or not all(isinstance(value, (int, np.integer)) for value in bbox_px):
            raise ValueError('bbox_px must contain four integer pixel coordinates')
        x0, y0, x1, y1 = map(int, bbox_px)
        if not (0 <= x0 < x1 <= self.width and 0 <= y0 < y1 <= self.height):
            raise ValueError(f'bbox_px must be a nonempty half-open region inside {self.width}x{self.height}')
        return x0, y0, x1, y1

    def _measure(self, bbox_px: Sequence[int], detailed: bool) -> dict:
        x0, y0, x1, y1 = self._bounds(bbox_px)
        valid = self.valid_mask[y0:y1, x0:x1]
        ranges = self._range_m[y0:y1, x0:x1][valid]
        pixel_count = int(valid.size)
        valid_count = int(ranges.size)
        layers = []
        # Fixed metric bins preserve foreground/background support separately.
        # A layer is a distance band, not an inferred object or surface label.
        bins = np.floor(ranges / self.layer_width_m).astype(np.int32)
        pixel_y, pixel_x = np.nonzero(valid)
        for index in np.unique(bins):
            selected = bins == index
            values = ranges[selected]
            low = float(index * self.layer_width_m)
            xs, ys = pixel_x[selected] + x0, pixel_y[selected] + y0
            k = self.intrinsics
            depths = self.depth_z_m[ys, xs].astype(np.float64)
            optical_points = np.column_stack(((xs-k['cx'])*depths/k['fx'],
                                              (ys-k['cy'])*depths/k['fy'], depths))
            camera_to_world = self.world_from_base @ self.camera_to_base
            points = optical_points @ camera_to_world[:3, :3].T + camera_to_world[:3, 3]
            layer = {
                'layer_id': f'D{index}',
                'range_m': [low, low + self.layer_width_m],
                'median_m': round(float(np.median(values)), 3),
                'fraction_of_valid': round(float(values.size / valid_count), 4),
                'support_world_bounds_m': [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
            }
            if detailed:
                p10, p90 = np.quantile(values, [0.1, 0.9])
                center = np.array([np.mean(xs), np.mean(ys)])
                representative = np.argmin((xs - center[0]) ** 2 + (ys - center[1]) ** 2)
                sx, sy = int(xs[representative]), int(ys[representative])
                z = float(self.depth_z_m[sy, sx])
                optical = np.array([(sx-k['cx'])*z/k['fx'], (sy-k['cy'])*z/k['fy'], z, 1.])
                base_point = self.camera_to_base @ optical
                world_point = self.world_from_base @ base_point
                surface = None
                if values.size >= 3:
                    centroid = points.mean(axis=0)
                    centered = points - centroid
                    eigenvalues, eigenvectors = np.linalg.eigh(centered.T @ centered / len(points))
                    normal = eigenvectors[:, 0]
                    if np.dot(normal, camera_to_world[:3, 3] - centroid) < 0:
                        normal = -normal
                    surface = {
                        'centroid_world_m': centroid.round(5).tolist(),
                        'normal_world': normal.round(6).tolist(),
                        'rms_m': round(float(np.sqrt(np.mean((centered @ normal) ** 2))), 5),
                        'point_count': len(points),
                        'in_plane_std_m': np.sqrt(eigenvalues[1:]).round(5).tolist(),
                        'definition': 'Least-squares plane of this distance-band pixel set; geometry only, not object segmentation.',
                    }
                layer.update(count=int(values.size), p10_m=round(float(p10), 3), p90_m=round(float(p90), 3),
                             color_rgb=self._layer_color(int(index)),
                             pixel_bbox_px=[int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1],
                             sample_pixel_px=[sx, sy],
                             sample_base_point_m=base_point[:3].round(5).tolist(),
                             sample_world_point_m=world_point[:3].round(5).tolist(),
                             surface=surface)
            layers.append(layer)
        return {
            'bbox_px': [x0, y0, x1, y1],
            'pixel_count': pixel_count,
            'valid_count': valid_count,
            'valid_ratio': round(valid_count / pixel_count, 4),
            'unknown_ratio': round(1 - valid_count / pixel_count, 4),
            'status': 'measured' if valid_count else 'unknown',
            'layers': layers,
        }

    def overview(self) -> dict:
        """Return compact 6x3 numbered-cell distributions for this RGB frame."""
        return {
            **self._metadata(),
            'grid': {'rows': self.rows, 'columns': self.columns, 'numbering': 'row_major_1_based'},
            'cells': [{**cell, **self._measure(cell['bbox_px'], detailed=False)} for cell in self._grid_cells()],
        }

    def measure_region(self, bbox_px: Sequence[int]) -> dict:
        """Measure original pixels from this cached frame, with per-layer spread."""
        return {**self._metadata(), **self._measure(bbox_px, detailed=True)}

    @staticmethod
    def _layer_color(index: int) -> list[int]:
        return [round(value * 255) for value in colorsys.hsv_to_rgb((index * 0.61803398875) % 1, 0.72, 1.0)]

    @staticmethod
    def _encode(image: Image.Image, image_format: str = 'JPEG') -> bytes:
        output = io.BytesIO()
        image.save(output, format=image_format, **({'quality': 90} if image_format == 'JPEG' else {}))
        return output.getvalue()

    @staticmethod
    def _packet(kind: str, label: str, data: bytes, mime_type: str = 'image/jpeg') -> dict:
        return {'kind': kind, 'label': label,
                'image': {'type': 'image', 'mimeType': mime_type, 'data': base64.b64encode(data).decode()}}

    def rgb_jpeg(self) -> bytes:
        return self._encode(Image.fromarray(self.rgb))

    def scene_payload(self, camera: Mapping) -> dict:
        return {
            'frame_id': self.frame_id, 'sim_time_s': self.sim_time_s,
            'pose_world': self.pose_world, 'camera': dict(camera), 'depth': self.overview(),
            'images': [
                self._packet('rgb', f'Original RGB, frame {self.frame_id}, {self.width}x{self.height}; choose object pixels on this image.', self.rgb_jpeg()),
                self._packet('grid', 'Same RGB with numbered grid. A cell can contain several objects and background; measurement accepts any pixel rectangle.', self.annotated_jpeg()),
            ],
        }

    def measurement_payload(self, bbox_px: Sequence[int]) -> dict:
        measurement = self.measure_region(bbox_px)
        return {**measurement, 'images': [
            self._packet('selection', f'Frame {self.frame_id}: cyan rectangle is the exact measured ROI {list(bbox_px)} on the full original RGB.', self.selection_jpeg(bbox_px)),
            self._packet('crop', 'Original RGB crop of the measured pixels, enlarged for inspection. Confirm which physical surface it contains.', self.region_jpeg(bbox_px)),
            self._packet('layers', 'Same crop geometry: original RGB on the left, distance-band pixels on the right. D IDs and colors match the statistics; gray means unknown. Bands are not object labels.', self.layers_png(bbox_px, measurement), 'image/png'),
        ]}

    def selection_jpeg(self, bbox_px: Sequence[int]) -> bytes:
        x0, y0, x1, y1 = self._bounds(bbox_px)
        image = Image.fromarray(self.rgb)
        draw = ImageDraw.Draw(image)
        draw.rectangle((x0, y0, x1 - 1, y1 - 1), outline=(0, 255, 255), width=3)
        return self._encode(image)

    def layers_png(self, bbox_px: Sequence[int], measurement: dict | None = None) -> bytes:
        """Render exact band membership beside RGB; unknown pixels stay gray.

        The panels use the same crop and nearest-neighbor mask scaling. Labels
        are positioned on a real member pixel, never a disconnected-mask center.
        Distinct surfaces at the same range can share a band; no semantics enter.
        """
        x0, y0, x1, y1 = self._bounds(bbox_px)
        measurement = measurement if measurement is not None else self.measure_region(bbox_px)
        valid = self.valid_mask[y0:y1, x0:x1]
        ranges = self._range_m[y0:y1, x0:x1]
        band_index = np.full(valid.shape, -1, dtype=np.int32)
        band_index[valid] = np.floor(ranges[valid] / self.layer_width_m).astype(np.int32)
        colored = np.full((*valid.shape, 3), 100, dtype=np.uint8)
        for layer in measurement['layers']:
            colored[band_index == int(layer['layer_id'][1:])] = layer['color_rgb']
        rgb_crop = Image.fromarray(self.rgb[y0:y1, x0:x1])
        scale = min(480 / rgb_crop.width, 320 / rgb_crop.height)
        panel_size = (max(1, round(rgb_crop.width * scale)), max(1, round(rgb_crop.height * scale)))
        rgb_crop = rgb_crop.resize(panel_size, Image.Resampling.BICUBIC)
        mask = Image.fromarray(colored).resize(panel_size, Image.Resampling.NEAREST)
        gap, header, line_height, panel_width = 16, 30, 23, 480
        inset = (panel_width - panel_size[0]) // 2
        legend_height = line_height * (len(measurement['layers']) + 2)
        canvas = Image.new('RGB', (2 * panel_width + gap, header + panel_size[1] + legend_height), (245, 245, 245))
        canvas.paste(rgb_crop, (inset, header))
        canvas.paste(mask, (panel_width + gap + inset, header))
        draw = ImageDraw.Draw(canvas)
        font = ImageFont.load_default(size=16)
        draw.text((3, 5), 'RGB crop', fill='black', font=font)
        draw.text((panel_width + gap + 3, 5), 'Same pixels: distance bands', fill='black', font=font)
        for number, layer in enumerate(measurement['layers']):
            sx, sy = layer['sample_pixel_px']
            location = (panel_width + gap + inset + round((sx - x0) * scale), header + round((sy - y0) * scale))
            label = layer['layer_id']
            left, top, right, bottom = draw.textbbox(location, label, font=font)
            draw.rectangle((left - 1, top - 1, right + 1, bottom + 1), fill='black')
            draw.text(location, label, fill='white', font=font)
            y = header + panel_size[1] + 5 + number * line_height
            draw.rectangle((5, y + 2, 21, y + 17), fill=tuple(layer['color_rgb']))
            low, high = layer['range_m']
            text = f"{label}: [{low:.1f}, {high:.1f}) m; median {layer['median_m']:.3f} m; {100 * layer['fraction_of_valid']:.1f}% of valid pixels"
            draw.text((28, y), text, fill='black', font=font)
        y = header + panel_size[1] + 5 + len(measurement['layers']) * line_height
        draw.rectangle((5, y + 2, 21, y + 17), fill=(100, 100, 100))
        draw.text((28, y), f"Unknown: {100 * measurement['unknown_ratio']:.1f}% of ROI. Distances: robot base horizontal range.", fill='black', font=font)
        draw.text((5, y + line_height), f'Original ROI {x0},{y0},{x1},{y1}; frame {self.frame_id}', fill='black', font=font)
        return self._encode(canvas, 'PNG')

    def region_jpeg(self, bbox_px: Sequence[int]) -> bytes:
        """Return the measured original-RGB crop, with no grid or semantic labels.

        Small crops are enlarged toward a 256-pixel short edge, with enlargement
        limited to a 1024-pixel long edge. Measurement coordinates stay original.
        """
        image = Image.fromarray(self.rgb).crop(self._bounds(bbox_px))
        scale = max(1.0, min(256 / min(image.size), 1024 / max(image.size)))
        if scale > 1:
            image = image.resize(tuple(round(size * scale) for size in image.size), Image.Resampling.BICUBIC)
        output = io.BytesIO()
        image.save(output, format='JPEG', quality=90)
        return output.getvalue()

    def annotated_jpeg(self) -> bytes:
        """Draw cell IDs/frame ID without resizing or modifying the source RGB."""
        image = Image.fromarray(self.rgb)
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default(size=18)
        for cell in self._grid_cells():
            x0, y0, x1, y1 = cell['bbox_px']
            draw.rectangle((x0, y0, x1 - 1, y1 - 1), outline=(255, 224, 64), width=1)
            location = (x0 + 4, y0 + 3)
            label = str(cell['cell_id'])
            left, top, right, bottom = draw.textbbox(location, label, font=font)
            draw.rectangle((left - 2, top - 1, right + 2, bottom + 1), fill=(0, 0, 0))
            draw.text(location, label, font=font, fill=(255, 255, 255))
        label = f'frame {self.frame_id}'
        label_width = draw.textlength(label, font=font)
        location = (self.width - label_width - 6, self.height - 24)
        left, top, right, bottom = draw.textbbox(location, label, font=font)
        draw.rectangle((left - 2, top - 1, right + 2, bottom + 1), fill=(0, 0, 0))
        draw.text(location, label, font=font, fill=(255, 255, 255))
        output = io.BytesIO()
        image.save(output, format='JPEG', quality=90)
        return output.getvalue()
