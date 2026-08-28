#!/usr/bin/env python3
"""Track / Obstacles builder GUI.

Loads a map produced by ``nav2_map_server`` / ``slam_toolbox save_map``
(``<name>.yaml`` + ``<name>.pgm``) or a pickled ``nav_msgs/OccupancyGrid``,
lets the user draw polygons on top of it, and exports them either in the
``track.yaml`` format (``stay_in`` / ``keep_out``) or in the legacy
``obstacles:`` format consumed by ``sim_lidar.py``.

Polygons are stored in *map* coordinates as soon as they are drawn, so
zooming/panning never changes the exported numbers.
"""

import os
import pickle

import numpy as np
import yaml
from PIL import Image, ImageTk
import tkinter as tk
from tkinter import messagebox
from tkinter.filedialog import askopenfilename, asksaveasfilename


class ObstaclesBuilderGUI(tk.Frame):
    """Obstacles / track builder.

    :param master: master widget (root), defaults to ``tk.Tk()``
    :type master: tkinter.Tk, optional
    """

    CANVAS_HEIGHT = 600
    CANVAS_WIDTH = 800
    CLOSE_ENOUGH_POINTS_TOLERANCE = 10  # pixels, on screen

    MIN_SCALE = 0.05
    MAX_SCALE = 60.0
    ZOOM_STEP = 1.2

    ROLES = ('stay_in', 'keep_out')
    ROLE_COLORS = {'stay_in': '#00a000', 'keep_out': '#d00000'}
    LINE_OPTIONS = {'dash': (4, 4), 'width': 3}
    POINT_RADIUS = 3

    OBSTACLES_FORMAT = 'obstacles (ros params)'
    OBSTACLES_FLAT_FORMAT = 'obstacles (flat)'
    TRACK_FORMAT = 'track (stay_in/keep_out)'
    TRACK_LIST_FORMAT = 'track (list)'
    EXPORT_FORMATS = (OBSTACLES_FORMAT, OBSTACLES_FLAT_FORMAT,
                      TRACK_FORMAT, TRACK_LIST_FORMAT)
    OBSTACLES_SUFFIX = '.obstacles.yaml'
    ## Node name pattern written into the params file
    PARAMS_NODE = '/**'
    ## Douglas-Peucker tolerance [m]; one map cell is a sensible starting point
    DEFAULT_EPSILON = 0.05
    ## A polygon can never be reduced below this many points
    MIN_POLYGON_POINTS = 3

    # Values used by map_server PGM images
    PGM_UNKNOWN = 205

    def __init__(self, master=None):
        super().__init__(master if master is not None else tk.Tk())
        self.master.title('SVEA Track / Obstacles Builder')

        ## Finished polygons, in map coordinates: [(role, [[x, y], ...]), ...]
        self.polygons = []
        ## Polygon currently being drawn, in map coordinates
        self.new_polygon = []
        ## Flag to indicate if a new polygon can be created
        self.drawing = False

        # --- map / occupancy grid state -------------------------------------
        ## PIL image of the map, row 0 == top row == largest y
        self.map_image = None
        self.map_width = self.CANVAS_WIDTH
        self.map_height = self.CANVAS_HEIGHT
        self.resolution = 1.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        ## Basename of the loaded map, e.g. 'sml' -> 'sml.obstacles.yaml'
        self.map_name = 'map'
        ## Directory the map was loaded from, used as default export location
        self.map_dir = ''

        # --- view state (canvas px = map px * scale + offset) ----------------
        self.scale = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0
        self._pan_start = None
        self._tk_image = None  # keep a reference, else Tk garbage collects it

        self.create_widgets()
        self.pack(fill=tk.BOTH, expand=True)

    # ------------------------------------------------------------------ UI --

    def create_widgets(self):
        """Creates the widgets used in the GUI."""
        self.top_frame = tk.Frame(self)
        self.top_frame.pack(fill=tk.X, expand=False)

        tk.Button(self.top_frame, text='Load map',
                  command=self._load_button_cb).pack(side=tk.LEFT)

        tk.Label(self.top_frame, text='  polygon:').pack(side=tk.LEFT)
        self.role_var = tk.StringVar(value='stay_in')
        for role in self.ROLES:
            tk.Radiobutton(self.top_frame, text=role, value=role,
                           variable=self.role_var,
                           fg=self.ROLE_COLORS[role]).pack(side=tk.LEFT)

        self.draw_button = tk.Button(self.top_frame, text='Add new polygon',
                                     command=self._toggle_drawing_cb)
        self.draw_button.pack(side=tk.LEFT, padx=(8, 0))

        tk.Button(self.top_frame, text='Undo point',
                  command=self._undo_point_cb).pack(side=tk.LEFT)
        tk.Button(self.top_frame, text='Clear all',
                  command=self._clear_cb).pack(side=tk.LEFT)
        tk.Button(self.top_frame, text='Fit view',
                  command=self._fit_view).pack(side=tk.LEFT)

        # --- point reduction -------------------------------------------------
        tk.Button(self.top_frame, text='Simplify',
                  command=self._simplify_cb).pack(side=tk.LEFT, padx=(8, 0))
        tk.Label(self.top_frame, text='eps[m]').pack(side=tk.LEFT)
        self.eps_var = tk.StringVar(value=str(self.DEFAULT_EPSILON))
        tk.Entry(self.top_frame, textvariable=self.eps_var,
                 width=6).pack(side=tk.LEFT)
        self.auto_simplify_var = tk.BooleanVar(value=False)
        tk.Checkbutton(self.top_frame, text='auto',
                       variable=self.auto_simplify_var).pack(side=tk.LEFT)

        tk.Button(self.top_frame, text='Export',
                  command=self._export_button_cb).pack(side=tk.RIGHT)
        self.format_var = tk.StringVar(value=self.OBSTACLES_FORMAT)
        tk.OptionMenu(self.top_frame, self.format_var,
                      *self.EXPORT_FORMATS).pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(self, background='white', highlightthickness=0)
        self.canvas.config(width=self.CANVAS_WIDTH, height=self.CANVAS_HEIGHT)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.focus_set()

        self.status = tk.Label(self, anchor='w', text='no map loaded')
        self.status.pack(fill=tk.X)

        # left click: add point / close polygon
        self.canvas.bind('<ButtonRelease-1>', self._on_left_click)
        # right click: finish polygon without closing on the first point
        self.canvas.bind('<ButtonRelease-3>', lambda e: self._toggle_drawing_cb())
        # middle button drag: pan
        self.canvas.bind('<ButtonPress-2>', self._on_pan_start)
        self.canvas.bind('<B2-Motion>', self._on_pan_move)
        # wheel: zoom (Windows/macOS and X11 respectively)
        self.canvas.bind('<MouseWheel>', self._on_zoom)
        self.canvas.bind('<Button-4>', self._on_zoom)
        self.canvas.bind('<Button-5>', self._on_zoom)
        self.canvas.bind('<Motion>', self._on_motion)
        self.canvas.bind('<Configure>', lambda e: self._redraw())

    # --------------------------------------------------------- coordinates --

    def canvas_to_map(self, cx, cy):
        """Canvas pixel -> map coordinate (meters)."""
        ix = (cx - self.offset_x) / self.scale
        iy = (cy - self.offset_y) / self.scale
        x = self.origin_x + ix * self.resolution
        # row 0 of the image is the top row, i.e. the largest y
        y = self.origin_y + (self.map_height - iy) * self.resolution
        return x, y

    def map_to_canvas(self, x, y):
        """Map coordinate (meters) -> canvas pixel."""
        ix = (x - self.origin_x) / self.resolution
        iy = self.map_height - (y - self.origin_y) / self.resolution
        return ix * self.scale + self.offset_x, iy * self.scale + self.offset_y

    # ------------------------------------------------------------ map load --

    def _load_button_cb(self):
        """Loads a map, either map_server YAML+PGM or a pickled OccupancyGrid."""
        filename = askopenfilename(
            filetypes=(('Map YAML', '*.yaml *.yml'),
                       ('Pickled OccupancyGrid', '*.pickle'),
                       ('All files', '*.*')))
        if not filename:
            return

        try:
            if filename.endswith(('.yaml', '.yml')):
                self._load_map_server_yaml(filename)
            else:
                self._load_occupancy_grid_pickle(filename)
        except Exception as exc:  # noqa: BLE001 - surface the error to the user
            messagebox.showerror('Failed to load map', str(exc))
            return

        self.map_dir = os.path.dirname(filename)
        # 'sml.yaml' / 'sml.pgm' / 'sml.pickle' -> 'sml'
        self.map_name = os.path.basename(filename).split('.')[0]
        self._fit_view()

    def _load_map_server_yaml(self, filename):
        """Loads a nav2_map_server / slam_toolbox map (YAML + image)."""
        with open(filename, 'r') as f:
            meta = yaml.safe_load(f)

        image_path = meta['image']
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(filename), image_path)

        image = Image.open(image_path).convert('L')

        if meta.get('negate', 0):
            image = Image.eval(image, lambda p: 255 - p)

        self.resolution = float(meta['resolution'])
        origin = meta.get('origin', [0.0, 0.0, 0.0])
        self.origin_x, self.origin_y = float(origin[0]), float(origin[1])
        if len(origin) > 2 and abs(float(origin[2])) > 1e-6:
            messagebox.showwarning(
                'Rotated map origin',
                'origin yaw != 0 is not handled; exported points will be off.')

        self.map_image = image
        self.map_width, self.map_height = image.size

    def _load_occupancy_grid_pickle(self, filename):
        """Loads a pickled ``nav_msgs/OccupancyGrid``."""
        with open(filename, 'rb') as f:
            occupancy_grid = pickle.load(f)

        info = occupancy_grid.info
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y

        data = np.reshape(np.array(occupancy_grid.data, dtype=np.int16),
                          (info.height, info.width))
        # OccupancyGrid row 0 is the *lowest* y -> flip so row 0 is the top
        data = np.flipud(data)

        # -1 unknown -> gray, 0 free -> white, 100 occupied -> black
        grey = np.where(data < 0,
                        self.PGM_UNKNOWN,
                        255 - (np.clip(data, 0, 100) * 255) // 100)
        self.map_image = Image.fromarray(grey.astype(np.uint8), mode='L')
        self.map_width, self.map_height = self.map_image.size

    # ---------------------------------------------------------------- view --

    def _fit_view(self):
        """Scales and centers the map so it fits inside the canvas."""
        cw = max(self.canvas.winfo_width(), self.CANVAS_WIDTH)
        ch = max(self.canvas.winfo_height(), self.CANVAS_HEIGHT)
        self.scale = min(cw / self.map_width, ch / self.map_height)
        self.scale = min(max(self.scale, self.MIN_SCALE), self.MAX_SCALE)
        self.offset_x = (cw - self.map_width * self.scale) / 2
        self.offset_y = (ch - self.map_height * self.scale) / 2
        self._redraw()

    def _on_zoom(self, event):
        if event.num == 5 or getattr(event, 'delta', 0) < 0:
            factor = 1 / self.ZOOM_STEP
        else:
            factor = self.ZOOM_STEP

        new_scale = min(max(self.scale * factor, self.MIN_SCALE), self.MAX_SCALE)
        factor = new_scale / self.scale
        # keep the point under the cursor fixed
        self.offset_x = event.x - (event.x - self.offset_x) * factor
        self.offset_y = event.y - (event.y - self.offset_y) * factor
        self.scale = new_scale
        self._redraw()

    def _on_pan_start(self, event):
        self._pan_start = (event.x, event.y, self.offset_x, self.offset_y)

    def _on_pan_move(self, event):
        if self._pan_start is None:
            return
        x0, y0, ox, oy = self._pan_start
        self.offset_x = ox + (event.x - x0)
        self.offset_y = oy + (event.y - y0)
        self._redraw()

    def _on_motion(self, event):
        x, y = self.canvas_to_map(event.x, event.y)
        mode = f'drawing {self.role_var.get()}' if self.drawing else 'idle'
        self.status.config(
            text=f'x={x:.3f}  y={y:.3f}   res={self.resolution:.4f}  '
                 f'origin=({self.origin_x:.3f}, {self.origin_y:.3f})   {mode}')

    def _redraw(self):
        """Redraws the map image and every polygon at the current view."""
        self.canvas.delete('all')

        if self.map_image is not None:
            w = max(int(self.map_width * self.scale), 1)
            h = max(int(self.map_height * self.scale), 1)
            resample = Image.NEAREST if self.scale >= 1 else Image.BILINEAR
            self._tk_image = ImageTk.PhotoImage(
                self.map_image.resize((w, h), resample))
            self.canvas.create_image(self.offset_x, self.offset_y,
                                     image=self._tk_image, anchor='nw')

        for role, points in self.polygons:
            self._draw_polygon(points, self.ROLE_COLORS[role], closed=True)

        if self.new_polygon:
            self._draw_polygon(self.new_polygon,
                               self.ROLE_COLORS[self.role_var.get()],
                               closed=False)

    def _draw_polygon(self, points, color, closed):
        pts = [self.map_to_canvas(x, y) for x, y in points]
        loop = pts + [pts[0]] if closed and len(pts) > 2 else pts
        for (x0, y0), (x1, y1) in zip(loop, loop[1:]):
            self.canvas.create_line(x0, y0, x1, y1, fill=color,
                                    **self.LINE_OPTIONS)
        r = self.POINT_RADIUS
        for x, y in pts:
            self.canvas.create_oval(x - r, y - r, x + r, y + r,
                                    fill=color, outline=color)

    # ------------------------------------------------------------- editing --

    def _toggle_drawing_cb(self):
        """Starts a new polygon or finishes the current one."""
        if not self.drawing:
            self.drawing = True
            self.draw_button.config(text='Finish polygon')
        else:
            if len(self.new_polygon) >= 3:
                points = self.new_polygon
                if self.auto_simplify_var.get():
                    try:
                        points = self._simplify_polygon(
                            points, float(self.eps_var.get()))
                    except ValueError:
                        pass
                self.polygons.append((self.role_var.get(), points))
            elif self.new_polygon:
                messagebox.showwarning('Discarded',
                                       'A polygon needs at least 3 points.')
            self.new_polygon = []
            self.drawing = False
            self.draw_button.config(text='Add new polygon')
        self._redraw()

    def _on_left_click(self, event):
        if not self.drawing:
            return

        x, y = self.canvas_to_map(event.x, event.y)

        if len(self.new_polygon) >= 3 and self._is_closing_shape(event.x, event.y):
            self._toggle_drawing_cb()
            return

        self.new_polygon.append([x, y])
        self._redraw()

    def _is_closing_shape(self, cx, cy):
        """True if (cx, cy) is close enough (on screen) to the first point."""
        fx, fy = self.map_to_canvas(*self.new_polygon[0])
        return (abs(fx - cx) < self.CLOSE_ENOUGH_POINTS_TOLERANCE
                and abs(fy - cy) < self.CLOSE_ENOUGH_POINTS_TOLERANCE)

    def _undo_point_cb(self):
        if self.new_polygon:
            self.new_polygon.pop()
        elif self.polygons:
            self.polygons.pop()
        self._redraw()

    def _clear_cb(self):
        if not messagebox.askyesno('Clear', 'Remove all polygons?'):
            return
        self.polygons = []
        self.new_polygon = []
        self.drawing = False
        self.draw_button.config(text='Add new polygon')
        self._redraw()

    # ---------------------------------------------------------- simplifying --

    def _simplify_cb(self):
        """Runs Douglas-Peucker on every finished polygon."""
        if not self.polygons:
            return
        try:
            epsilon = float(self.eps_var.get())
        except ValueError:
            messagebox.showerror('Bad tolerance',
                                 f'"{self.eps_var.get()}" is not a number.')
            return
        if epsilon <= 0:
            return

        before = sum(len(p) for _, p in self.polygons)
        self.polygons = [(role, self._simplify_polygon(points, epsilon))
                         for role, points in self.polygons]
        after = sum(len(p) for _, p in self.polygons)

        self._redraw()
        self.status.config(
            text=f'simplified: {before} -> {after} points '
                 f'(eps={epsilon} m, ~{135 * after} intersection tests/scan)')

    @classmethod
    def _simplify_polygon(cls, points, epsilon):
        """Douglas-Peucker for a *closed* polygon.

        The polygon has no natural endpoints, so it is split at the two points
        that are farthest apart and each chain is simplified separately;
        simplifying it as an open chain would pin whichever point happened to
        be clicked first.
        """
        if len(points) <= cls.MIN_POLYGON_POINTS:
            return points

        pts = np.asarray(points, dtype=float)
        # farthest pair from the first point, then farthest from that one
        a = int(np.argmax(np.linalg.norm(pts - pts[0], axis=1)))
        b = int(np.argmax(np.linalg.norm(pts - pts[a], axis=1)))
        i, j = sorted((a, b))

        chain1 = cls._douglas_peucker(pts[i:j + 1], epsilon)
        chain2 = cls._douglas_peucker(np.vstack([pts[j:], pts[:i + 1]]), epsilon)
        # drop the shared endpoints of the two chains
        simplified = [p.tolist() for p in chain1] + [p.tolist() for p in chain2[1:-1]]

        if len(simplified) < cls.MIN_POLYGON_POINTS:
            return points
        return simplified

    @classmethod
    def _douglas_peucker(cls, pts, epsilon):
        """Simplifies an open chain, keeping both endpoints.

        :param pts: (N, 2) array of points
        :param epsilon: max perpendicular deviation [m]
        """
        if len(pts) < 3:
            return pts

        start, end = pts[0], pts[-1]
        segment = end - start
        length = np.linalg.norm(segment)

        if length < 1e-12:
            # degenerate segment -> distance to the start point
            dists = np.linalg.norm(pts - start, axis=1)
        else:
            # perpendicular distance via the 2D cross product
            dists = np.abs(np.cross(segment, pts - start)) / length

        index = int(np.argmax(dists))
        if dists[index] <= epsilon:
            return np.vstack([start, end])

        left = cls._douglas_peucker(pts[:index + 1], epsilon)
        right = cls._douglas_peucker(pts[index:], epsilon)
        return np.vstack([left[:-1], right])

    # ------------------------------------------------------------- exporting --

    def _export_button_cb(self):
        """Exports the polygons to a YAML file in the selected format."""
        if self.drawing:
            messagebox.showwarning('Unfinished polygon',
                                   'Finish the current polygon first.')
            return
        if not self.polygons:
            return

        fmt = self.format_var.get()
        is_obstacles = fmt in (self.OBSTACLES_FORMAT, self.OBSTACLES_FLAT_FORMAT)

        if is_obstacles:
            initialfile = self.map_name + self.OBSTACLES_SUFFIX
        else:
            initialfile = self.map_name + '.track.yaml'

        filename = asksaveasfilename(
            defaultextension='.yaml',
            initialdir=self.map_dir or None,
            initialfile=initialfile,
            filetypes=(('YAML files', '*.yaml'), ('All files', '*.*')))
        if not filename:
            return

        if is_obstacles:
            text = self._dump_obstacles(
                as_params=(fmt == self.OBSTACLES_FORMAT))
        else:
            text = self._dump_track(as_list=(fmt == self.TRACK_LIST_FORMAT))
            if text is None:
                return

        with open(filename, 'w') as f:
            f.write(text)
        self.status.config(text=f'exported {len(self.polygons)} polygon(s) '
                                f'to {filename}')

    def _grouped(self):
        grouped = {role: [] for role in self.ROLES}
        for role, points in self.polygons:
            grouped[role].append(points)
        return grouped

    def _dump_track(self, as_list):
        """``stay_in`` / ``keep_out`` format expected by the Track class."""
        grouped = self._grouped()
        for role in self.ROLES:
            if len(grouped[role]) != 1:
                messagebox.showerror(
                    'Wrong number of polygons',
                    f'The track format needs exactly one "{role}" polygon, '
                    f'got {len(grouped[role])}.')
                return None

        stay_in, keep_out = grouped['stay_in'][0], grouped['keep_out'][0]

        if as_list:
            lines = ['track:']
            for poly in (stay_in, keep_out):
                lines.append('- ' + self._flow_polygon(poly))
        else:
            lines = []
            for name, poly in (('stay_in', stay_in), ('keep_out', keep_out)):
                lines.append(f'{name}:')
                lines += [f'- [{self._fmt(x)}, {self._fmt(y)}]' for x, y in poly]
        return '\n'.join(lines) + '\n'

    def _dump_obstacles(self, as_params):
        """``{map_name}.obstacles.yaml`` consumed by ``sim_lidar.py``.

        ``sim_lidar`` declares ``obstacles`` as a *string* and runs
        ``ast.literal_eval`` on it, and the file is passed through
        ``--params-file``, so the ROS params variant wraps the whole list of
        polygons in a single quoted scalar under ``/**: ros__parameters:``.
        Polygons are written open (the first point is not repeated);
        ``prepare_obstacles`` closes them at load time.
        """
        polygons = '[' + ', '.join(self._flow_polygon(poly)
                                   for _, poly in self.polygons) + ']'

        if not as_params:
            lines = ['obstacles:']
            for _, poly in self.polygons:
                lines.append('    - ' + self._flow_polygon(poly))
            return '\n'.join(lines) + '\n'

        return (f'# Generated by obstacles_builder_gui from map "{self.map_name}"\n'
                f'{self.PARAMS_NODE}:\n'
                f'  ros__parameters:\n'
                f'    obstacles: "{polygons}"\n')

    @staticmethod
    def _fmt(value):
        """Trims float noise; 4 decimals is ~0.1 mm, well below map resolution."""
        return f'{value:.4f}'

    @classmethod
    def _flow_polygon(cls, poly):
        return ('[' + ', '.join(f'[{cls._fmt(x)}, {cls._fmt(y)}]'
                                for x, y in poly) + ']')


if __name__ == '__main__':
    app = ObstaclesBuilderGUI()
    app.mainloop()