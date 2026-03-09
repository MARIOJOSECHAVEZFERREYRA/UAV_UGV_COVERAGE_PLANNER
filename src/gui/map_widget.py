from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsTextItem, QGraphicsEllipseItem, QGraphicsRectItem, QGraphicsSimpleTextItem, QGraphicsItemGroup, QGraphicsPolygonItem, QSizePolicy
from PyQt6.QtCore import Qt, QPointF, pyqtSignal, QRectF
from PyQt6.QtGui import QPen, QBrush, QColor, QFont, QPainter, QPolygonF, QWheelEvent, QMouseEvent, QPainterPath
import math
from shapely.geometry import LineString, Polygon as ShapelyPoly
from .styles import MAP_FIELD_BORDER, MAP_FIELD_FILL, MAP_MARKER_START, MAP_MARKER_END, MAP_MARKER_TRUCK, MAP_ROUTE_TRUCK, MAP_CYCLE_COLORS

class MissionMarkerItem(QGraphicsItem):
    """
    Custom marker that ignores transformations (zoom) to maintain
    constant pixel size, with clear text label.
    """
    def __init__(self, x, y, label, color, type="default"):
        super().__init__()
        self.setPos(x, y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        self.setZValue(100) # Always visible
        self.label = label
        self.color = QColor(color)
        self.text_color = QColor("white")
        
        # Short to long name mapping
        if label == "S": self.full_text = "START"
        elif label == "E": self.full_text = "END"
        elif label.startswith("R"): self.full_text = label.replace("R", "R")
        else: self.full_text = label
        
        # Specific styles
        if label == "S": self.color = QColor(MAP_MARKER_START)
        if label == "E": self.color = QColor(MAP_MARKER_END)
        if label.startswith("R"): self.color = QColor(MAP_MARKER_TRUCK)

    def boundingRect(self):
        # Approximate drawing area (marker + text bubble)
        return QRectF(-10, -50, 200, 60)

    def paint(self, painter, option, widget):
        # 1. Draw Point/Pin
        r = 6
        painter.setBrush(QBrush(self.color))
        painter.setPen(QPen(Qt.GlobalColor.white, 2))
        painter.drawEllipse(QPointF(0, 0), r, r)
        
        # 2. Draw "Callout" (Text Bubble)
        # Offset en PIXELES (fijo, no depende del zoom)
        offset_x = 10
        offset_y = -10
        
        font = QFont("Segoe UI", 9, QFont.Weight.Bold)
        painter.setFont(font)
        fm = painter.fontMetrics()
        
        text_w = fm.horizontalAdvance(self.full_text)
        text_h = fm.height()
        pad = 6
        
        # Background rectangle
        rect_x = offset_x
        rect_y = offset_y - text_h - pad
        rect_w = text_w + (pad * 2)
        rect_h = text_h + pad
        
        # Connector line (optional)
        # painter.setPen(QPen(self.color, 1))
        # painter.drawLine(0, 0, rect_x, rect_y + rect_h)
        
        # Box
        path = QPainterPath()
        path.addRoundedRect(QRectF(rect_x, rect_y, rect_w, rect_h), 4, 4)
        
        painter.setBrush(QBrush(self.color))
        painter.setPen(QPen(Qt.GlobalColor.white, 1))
        painter.drawPath(path)
        
        # Text
        painter.setPen(self.text_color)
        painter.drawText(int(rect_x + pad), int(rect_y + text_h - 2), self.full_text)

class MapWidget(QGraphicsView):
    # Signals
    map_clicked = pyqtSignal(float, float)       # Click on empty space (Add point)
    map_right_clicked = pyqtSignal(float, float) # Right click
    point_moved = pyqtSignal(int, float, float)  # Drag point (index, new_x, new_y)
    
    route_length_changed = pyqtSignal(float) # Signal for UI update

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_app = parent
        
        # State
        self.original_polygon = None
        self.truck_path_coords = []     # List of truck coords
        self.mission_cycles = None      # List of cycles (path + color + markers)
        self.recharge_markers = []      # List of dicts {lat, lon, label}
        self.swath_polygons = []        # List of coverage polygons
        self.show_swath = True          # Visibility toggle
        self.scene = QGraphicsScene(self)
        self.scene.setSceneRect(-100000, -100000, 200000, 200000)
        self.setScene(self.scene)
        
        # Configuration
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setBackgroundBrush(QBrush(QColor("#ffffff")))
        
        # Size Policy to allow maximizing
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        
        self.scale(10, -10)
        
        # Mouse Tracking for Hover Labels
        self.setMouseTracking(True)
        
        # Drawing Mode
        self.draw_mode_route = False
        self.draw_mode_obstacle = False # New mode for obstacle
        self.temp_route_points = []
        self.service_route_points = [] # Confirmed route
        
        self.temp_obstacle_points = []
        self.obstacle_polygons = [] # List of list of points
        
        # Visual Items Tracking (For clearing/redrawing)
        self.obstacle_finished_items = []
        self.obstacle_temp_items = []
        
        self.hover_group = None
        self.hover_text = None
        self.hover_bg = None
        
        # Internal drag state
        self.dragging_point_index = None

        # State
        self.zoom_level = 1.0 # Note: m11 tracks internal scale too, but we keep this for legacy logic if any
        self.pan_start = QPointF(0, 0)
        self.is_panning = False
        self.show_swath = True
        
        # Cache for redraw
        self.last_polygon_geom = None
        self.last_safe_geom = None
        self.last_mission_cycles = None

    def drawBackground(self, painter, rect):
        """Dynamic Grid"""
        super().drawBackground(painter, rect)
        scale = self.transform().m11()
        if scale <= 0: scale = 1.0
        
        step_pixel = 100
        step_world = step_pixel / scale
        exponent = math.floor(math.log10(step_world))
        fraction = step_world / (10**exponent)
        
        if fraction < 2: grid_step = 2 * 10**exponent
        elif fraction < 5: grid_step = 5 * 10**exponent
        else: grid_step = 10 * 10**exponent
            
        painter.setPen(QPen(QColor(240, 240, 240), 0))
        left = int(rect.left()) - (int(rect.left()) % int(grid_step))
        top = int(rect.top()) - (int(rect.top()) % int(grid_step))
        
        x = left
        while x < rect.right():
            painter.drawLine(int(x), int(rect.top()), int(x), int(rect.bottom()))
            x += grid_step
        y = top
        while y < rect.bottom():
            painter.drawLine(int(rect.left()), int(y), int(rect.right()), int(y))
            y += grid_step

    def drawForeground(self, painter, rect):
        """Scale Bar"""
        scale = self.transform().m11()
        if scale <= 0: return

        target_width_px = 120
        world_width = target_width_px / scale
        exponent = math.floor(math.log10(world_width))
        fraction = world_width / (10**exponent)
        
        if fraction < 2: nice_world = 1 * 10**exponent
        elif fraction < 5: nice_world = 2 * 10**exponent
        else: nice_world = 5 * 10**exponent
        
        bar_width_px = nice_world * scale
        
        vp = self.viewport().rect()
        margin = 20
        start_x = vp.width() - bar_width_px - margin
        start_y = vp.height() - margin - 10 
        
        painter.resetTransform()
        pen = QPen(QColor("black"))
        pen.setWidth(2)
        painter.setPen(pen)
        
        h = 6
        painter.drawLine(int(start_x), int(start_y - h), int(start_x), int(start_y))
        painter.drawLine(int(start_x), int(start_y), int(start_x + bar_width_px), int(start_y))
        painter.drawLine(int(start_x + bar_width_px), int(start_y), int(start_x + bar_width_px), int(start_y - h))
        
        text = f"{int(nice_world)} m"
        font = QFont("Segoe UI", 9, QFont.Weight.Bold)
        painter.setFont(font)
        fm = painter.fontMetrics()
        tw = fm.horizontalAdvance(text)
        painter.drawText(int(start_x + (bar_width_px - tw)/2), int(start_y - 8), text)

    # --- DRAWING ---

    def clear_map(self):
        self.scene.clear()
        self.hover_group = None
        self.hover_text = None
        self.hover_bg = None
        
        # Reset Route State (Fix Segfault by dropping refs to deleted C++ items)
        self.route_items = []
        self.temp_route_points = []
        self.service_route_points = []
        self.set_draw_mode_route(False)
        self.set_draw_mode_obstacle(False)
        
        # NOTE: WE DO NOT CLEAR OBSTACLES HERE TO PERSIST THEM ACROSS REDRAWS
        # self.obstacle_polygons = [] 
        self.temp_obstacle_points = [] 
        
        # Reset Visual Item references (they are deleted by scene.clear())
        self.obstacle_temp_items = []
        self.obstacle_finished_items = []
        
        # Reset Geometries to prevent Ghost Hover detection
        self.last_polygon_geom = None
        self.last_safe_geom = None
        self.mission_cycles = None 
        
        # Cache for overlapping labels
        self.label_cache = {}

    def full_reset(self):
        """Used by the CLEAR button to wipe everything including obstacles"""
        self.obstacle_polygons = []
        self.clear_map()

    def set_draw_mode_route(self, enabled):
        self.draw_mode_route = enabled
        if enabled:
            self.set_draw_mode_obstacle(False) # Mutual exclusion
            self.setCursor(Qt.CursorShape.CrossCursor)
        elif not self.draw_mode_obstacle:
            self.setCursor(Qt.CursorShape.ArrowCursor)

    def set_draw_mode_obstacle(self, enabled):
        self.draw_mode_obstacle = enabled
        if enabled:
            self.set_draw_mode_route(False) # Mutual exclusion
            self.setCursor(Qt.CursorShape.CrossCursor)
        elif not self.draw_mode_route:
            self.setCursor(Qt.CursorShape.ArrowCursor)

    def draw_obstacles(self):
        """Draws finished obstacles and current temp obstacle"""
        
        # 1. Clear previous temp items (Fix for Undo visual bug)
        if not hasattr(self, 'obstacle_temp_items'):
            self.obstacle_temp_items = []
            
        for item in self.obstacle_temp_items:
            try: self.scene.removeItem(item)
            except: pass
        self.obstacle_temp_items = []
        
        # Style for Obstacles (Red)
        pen = QPen(QColor('#c0392b')) # Red
        pen.setWidth(2)
        pen.setCosmetic(True)
        brush = QBrush(QColor(192, 57, 43, 80)) # Red transparent
        
        # 2. Finished Obstacles (These are permanent, clear_map handles them? No, we just redraw new ones? 
        # Wait, if we keep adding polygons, we duplicate them? 
        # `draw_obstacles` is called incrementally.
        # But `obstacle_polygons` are persistent.
        # We should probably clear OLD obstacle polygons too if we call this frequently?
        # NO. `draw_obstacles` is called on click. It should probably redraw EVERYTHING obstacle related or just temp?
        # Ideally, finished obstacles are static. But if we want to support deleting them later, we should track them.
        # For now, let's assume `draw_obstacles` is ONLY for the temp one + redrawing all for simplicity.
        # BUT `scene.addPolygon` adds new item.
        # FIX: We should track ALL obstacle items (finished + temp) and redraw them.
        
        # Clear finished obstacle items?
        # Currently we don't track them. They just accumulate in scene.
        # If `draw_obstacles` is called 10 times, we get 10 copies of finished obstacles!
        # WE MUST TRACK FINISHED ITEMS TOO.
        pass # Placeholder for logic below
        
        if hasattr(self, 'obstacle_finished_items'):
             for item in self.obstacle_finished_items:
                 try: self.scene.removeItem(item)
                 except: pass
        self.obstacle_finished_items = []

        # Draw Finished
        for obs_points in self.obstacle_polygons:
             if len(obs_points) < 3: continue
             poly = QPolygonF([QPointF(p[0], p[1]) for p in obs_points])
             item = self.scene.addPolygon(poly, pen, brush)
             item.setZValue(0.5) # Just above field fill (0), below everything else
             self.obstacle_finished_items.append(item)
             
        # Draw Temp
        if self.temp_obstacle_points:
             points = self.temp_obstacle_points
             for p in points:
                 # Small marker
                 r = 4
                 marker = QGraphicsEllipseItem(-r, -r, r*2, r*2)
                 marker.setBrush(QBrush(QColor('#e74c3c')))
                 marker.setPos(p[0], p[1])
                 marker.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
                 marker.setZValue(100)
                 self.scene.addItem(marker)
                 self.obstacle_temp_items.append(marker)
             
             if len(points) > 1:
                 path = QPainterPath()
                 path.moveTo(points[0][0], points[0][1])
                 for p in points[1:]:
                     path.lineTo(p[0], p[1])
                 
                 pen_dash = QPen(QColor('#e74c3c'))
                 pen_dash.setStyle(Qt.PenStyle.DashLine)
                 pen_dash.setCosmetic(True)
                 line_item = self.scene.addPath(path, pen_dash)
                 line_item.setZValue(0.6) # Slightly above finished obstacles
                 self.obstacle_temp_items.append(line_item)
            
    def draw_service_route(self, is_temp=False):
        """Draws the service route (truck)"""
        # Clean previous items
        if not hasattr(self, 'route_items'): self.route_items = []
        for item in self.route_items:
             try: self.scene.removeItem(item)
             except: pass
        self.route_items = []

        points = self.temp_route_points if is_temp else self.service_route_points
        if not points or len(points) < 2: return
        
        path = QPainterPath()
        path.moveTo(points[0][0], points[0][1])
        
        total_len = 0.0
        prev_p = points[0]
        
        for p in points[1:]:
             path.lineTo(p[0], p[1])
             dx = p[0] - prev_p[0]
             dy = p[1] - prev_p[1]
             total_len += (dx*dx + dy*dy)**0.5
             prev_p = p
             
        self.route_length_changed.emit(total_len)
             
        pen = QPen(QColor('#e67e22')) # Orange
        pen.setWidth(4 if not is_temp else 2)
        pen.setCosmetic(True)
        if is_temp: pen.setStyle(Qt.PenStyle.DashLine)
        
        item = self.scene.addPath(path, pen)
        item.setZValue(5) 
        self.route_items.append(item)
        
        # Draw points
        # Draw points
        for i, p in enumerate(points):
             r = 6
             # Create item at local origin to work correctly with setPos + IgnoreTransform
             p_item = QGraphicsEllipseItem(-r, -r, r*2, r*2)
             
             # Style look-alike to polygon editor (White center, Bold colored rim)
             p_item.setBrush(QBrush(QColor("white")))
             p_item.setPen(QPen(QColor('#d35400'), 2))
             
             p_item.setPos(p[0], p[1])
             p_item.setZValue(100) # High Z to be visible
             p_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
             
             # Store index and type for interaction
             p_item.setData(0, i) 
             p_item.setData(1, "route_point")
             
             self.scene.addItem(p_item)
             self.route_items.append(p_item)

    def draw_editor_state(self, points):
        """Draws editable polygon"""
        self.clear_map()
        self.draw_obstacles()
        if not points: return

        # 1. Lines
        if len(points) > 1:
            path = QPainterPath()
            path.moveTo(points[0][0], points[0][1])
            for p in points[1:]:
                path.lineTo(p[0], p[1])
            
            pen = QPen(QColor('#4285F4'))
            pen.setWidth(3)
            pen.setCosmetic(True)
            self.scene.addPath(path, pen).setZValue(1)
            
            if len(points) > 2:
                pen_dash = QPen(QColor('#4285F4'))
                pen_dash.setStyle(Qt.PenStyle.DashLine)
                pen_dash.setWidth(2)
                pen_dash.setCosmetic(True)
                self.scene.addLine(points[-1][0], points[-1][1], points[0][0], points[0][1], pen_dash).setZValue(1)

        # 2. Points (Interactive)
        for i, p in enumerate(points):
            self.draw_point_marker(p[0], p[1], index=i)

        # 3. Labels
        self.draw_labels(points)

    def draw_point_marker(self, x, y, index):
        """Interactive point that knows its index"""
        radius = 6 
        ellipse = QGraphicsEllipseItem(-radius, -radius, radius * 2, radius * 2)
        
        ellipse.setBrush(QBrush(QColor("white")))
        pen = QPen(QColor("#555555"))
        pen.setWidth(2)
        ellipse.setPen(pen)
        
        ellipse.setPos(x, y)
        ellipse.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        ellipse.setZValue(100) # High layer to capture clicks
        
        # SAVE INDEX IN ITEM TO IDENTIFY IT WHEN DRAGGING
        ellipse.setData(0, index) 
        
        self.scene.addItem(ellipse)

    def draw_results(self, polygon_geom, safe_geom, mission_cycles, is_static=False, road_geom=None):
        """
        Draws the results.
        road_geom: LinearRing of the truck road (offset boundary).
        """
        self.last_polygon_geom = polygon_geom
        self.last_safe_geom = safe_geom
        self.last_mission_cycles = mission_cycles
        self.last_road_geom = road_geom
        
        self.clear_map()
        
        # 0. Draw Truck Road (Limit)
        if road_geom:
             # road_geom is likely a LinearRing (from .exterior) or LineString. 
             # Ensure we iterate coords correctly.
             poly_r = QPolygonF([QPointF(x, y) for x, y in road_geom.coords])
             
             pen_r = QPen(QColor(MAP_ROUTE_TRUCK)) # Truck Route
             pen_r.setStyle(Qt.PenStyle.DashLine)
             pen_r.setWidth(1)
             pen_r.setCosmetic(True)
             
             # Use addPath or addPolygon. Since it's a ring, addPolygon is fine (closed).
             self.scene.addPolygon(poly_r, pen_r, QBrush(Qt.BrushStyle.NoBrush)).setZValue(1)

        if polygon_geom:
            poly_q = QPolygonF([QPointF(x, y) for x, y in polygon_geom.exterior.coords])
            brush = QBrush(QColor(46, 204, 113, 50))
            pen = QPen(QColor(MAP_FIELD_BORDER))
            pen.setWidth(3)
            pen.setCosmetic(True)
            self.scene.addPolygon(poly_q, pen, brush).setZValue(0)
            
            
            # 3. Field Boundary Labels (ALWAYS visible)
            self.draw_labels(list(polygon_geom.exterior.coords)[:-1])
            
            centroid = polygon_geom.centroid
            area_ha = polygon_geom.area / 10000.0
            self.draw_floating_label(centroid.x, centroid.y, f"{area_ha:.2f} ha", is_area=True)

        # 1.5 Draw Obstacles (Red)
        self.draw_obstacles()

        if safe_geom:
            poly_s = QPolygonF([QPointF(x, y) for x, y in safe_geom.exterior.coords])
            pen_s = QPen(QColor('#e74c3c'))
            pen_s.setStyle(Qt.PenStyle.DashLine)
            pen_s.setWidth(2)
            pen_s.setCosmetic(True)
            self.scene.addPolygon(poly_s, pen_s, QBrush(Qt.BrushStyle.NoBrush)).setZValue(2)

        # Color palette for cycles
        colors = MAP_CYCLE_COLORS
        
        cycle_idx = 0
        drawn_segments_history = {} # Key: ( (x1,y1), (x2,y2) ) -> Count
        for cycle in mission_cycles:
            path = cycle.get('path', [])
            truck_path_list = cycle.get('truck_path_coords', []) # Expecting list of coords
            
            if not path: continue
            
            # 1. Draw Flight Route with REAL WIDTH (SWATH)
            col = colors[cycle_idx % len(colors)]
            swath_width = cycle.get('swath_width', 5.0) # Default 5m
            
            # NEW: Use visual_groups (pre-compressed segments by draw type)
            visual_groups = cycle.get('visual_groups', [])
            
            if visual_groups:
                # Process compressed visual groups (MUCH faster than per-segment)
                # Groups are already aggregated by type (spray vs return)
                
                # AGGREGATED RETURN LABEL LOGIC (Fix for overlapping labels)
                cycle_return_total = 0.0
                cycle_return_midpoint = None
                
                for group_idx, group in enumerate(visual_groups):
                    group_path = group.get('path', [])
                    is_spraying = group.get('is_spraying', False)
                    
                    if len(group_path) < 2: continue
                    
                    # Aggregate return distances for smart labeling
                    if is_static and not is_spraying:
                        ret_line = LineString(group_path)
                        cycle_return_total += ret_line.length
                        # Use first return segment's midpoint for label placement
                        if cycle_return_midpoint is None:
                            cycle_return_midpoint = ret_line.interpolate(0.5, normalized=True)
                    
                    # 1. Draw Line
                    qpath = QPainterPath()
                    qpath.moveTo(group_path[0][0], group_path[0][1])
                    for point in group_path[1:]:
                        qpath.lineTo(point[0], point[1])
                    
                    pen_group = QPen(QColor(col))
                    
                    if is_spraying:
                        pen_group.setWidth(2)
                        pen_group.setStyle(Qt.PenStyle.SolidLine)
                    else:
                        # Return path: dashed
                        pen_group.setStyle(Qt.PenStyle.DashLine)
                        pen_group.setWidth(2)
                        
                        # Draw arrows ONLY for return paths in static mode (NEVER on spray lines)
                        if is_static and (is_spraying is False):
                            # Draw single arrow at midpoint
                            if len(group_path) >= 2:
                                mid_idx = len(group_path) // 2
                                if mid_idx > 0:
                                    self.draw_arrow(group_path[mid_idx-1], group_path[mid_idx], col)
                                else:
                                    self.draw_arrow(group_path[0], group_path[1], col)
                    
                    pen_group.setCosmetic(True)
                    self.scene.addPath(qpath, pen_group).setZValue(10)
                    
                    
                    # 2. Buffer for spray groups (if enabled)
                    # STRICT CHECK: Ensure it is explicitly True, not just Truthy
                    if self.show_swath and (is_spraying is True) and (len(group_path) >= 2):
                        try:
                            # Create single buffer for entire group (much faster than per-segment)
                            line_geom = LineString(group_path)
                            swath_poly = line_geom.buffer(
                                swath_width / 2.0,
                                cap_style=2,
                                join_style=2,
                                resolution=4  # Low resolution for speed
                            )
                            
                            if swath_poly.geom_type == 'Polygon':
                                polys = [swath_poly]
                            else:
                                polys = swath_poly.geoms
                            
                            for poly in polys:
                                qpoly = QPolygonF([QPointF(x, y) for x, y in poly.exterior.coords])
                                c = QColor(col)
                                c.setAlpha(150)
                                brush = QBrush(c)
                                pen_b = QPen(Qt.PenStyle.NoPen)
                                self.scene.addPolygon(qpoly, pen_b, brush).setZValue(9)
                        
                        except Exception as e:
                            print(f"Warning: Failed to create buffer for group {group_idx}: {e}")
                
                # STATIC MODE: Draw AGGREGATED return label (once per cycle)
                if is_static and cycle_return_total > 10.0 and cycle_return_midpoint:
                    self.draw_floating_label(
                        cycle_return_midpoint.x, 
                        cycle_return_midpoint.y, 
                        f"{cycle_return_total:.0f} m"
                    )
                
                # Process events every cycle to keep UI responsive
                if (cycle_idx + 1) % 2 == 0:  # Every 2 cycles
                    from PyQt6.QtWidgets import QApplication
                    QApplication.processEvents()

            # Cycle End marker
            label_end = "E" if cycle_idx == len(mission_cycles)-1 else f"R{cycle_idx+1}"
            
            self.draw_mission_marker(path[-1][0], path[-1][1], MAP_MARKER_END, label_end)

            # 2. Draw Truck Route (if exists for this cycle)
            if truck_path_list and len(truck_path_list) > 1:
                tpath = QPainterPath()
                tpath.moveTo(truck_path_list[0][0], truck_path_list[0][1])
                for p in truck_path_list[1:]:
                    tpath.lineTo(p[0], p[1])
                
                # Simple consistent style for truck route
                pen_t = QPen(QColor(MAP_ROUTE_TRUCK)) # Orange
                pen_t.setStyle(Qt.PenStyle.DashLine)
                pen_t.setWidth(3) 
                pen_t.setCosmetic(True)
                self.scene.addPath(tpath, pen_t).setZValue(5)
            
            cycle_idx += 1
            
        # Global Start Marker
        if mission_cycles and mission_cycles[0].get('path'):
            p0 = mission_cycles[0]['path'][0]
            self.draw_mission_marker(p0[0], p0[1], MAP_MARKER_START, "S")

    def draw_mission_marker(self, x, y, color, text):
        item = MissionMarkerItem(x, y, text, color)
        self.scene.addItem(item)

    def draw_arrow(self, p1, p2, color, check_len=True):
        """Draws an arrow. check_len=False skips the minimum length check."""
        mx = (p1[0] + p2[0]) / 2
        my = (p1[1] + p2[1]) / 2
        
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        
        if check_len:
            length = math.sqrt(dx*dx + dy*dy)
            if length < 5.0: return
        
        angle = math.atan2(dy, dx)
        # ... rest of drawing logic reused ...
        
        # Geometry in PIXELS (Fixed Screen Size)
        psize = 12
        
        # Position Logic (Relative):
        # Place arrow at 60% of the segment (Slightly forward) if we calculate from scratch.
        # But if we use draw_arrows_interpolated, p1/p2 are tiny segments around the center.
        # So we just place it at p1 (the interpolated point).
        
        # To avoid duplicating logic, let's just use p1 as center if check_len is False
        if not check_len:
             mx, my = p1[0], p1[1]
        else:
             # Classic midpoint logic
             t = 0.60
             mx = p1[0] + dx * t
             my = p1[1] + dy * t

        # Triangle pointing Right (+X), Centered on Centroid (0,0)
        # Shifted coords to align visual center with pivot
        head_px = QPolygonF([
            QPointF(-4, -4),
            QPointF(8, 0),
            QPointF(-4, 4)
        ])
        
        arrow_item = QGraphicsPolygonItem(head_px)
        arrow_item.setBrush(QBrush(QColor(color)))
        arrow_item.setPen(QPen(Qt.PenStyle.NoPen))
        
        # Transform 
        # 1. Rotation (Degrees)
        arrow_item.setRotation(-math.degrees(angle))
        
        # 2. Position (Scene Coords)
        arrow_item.setPos(mx, my)
        
        # 3. Ignore Zoom (Constant Pixel Size)
        arrow_item.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        
        arrow_item.setZValue(12) 

        self.scene.addItem(arrow_item)

    def draw_labels(self, points):
        if len(points) < 2: return
        
        # Calc Centroid
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        
        for i in range(len(points)):
            p1 = points[i]
            p2 = points[0] if i == len(points)-1 else points[i+1]
            
            dist = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            
            # Filter small segments (e.g. < 5m) to avoid clutter
            if dist < 5.0: continue
            
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2
            
            # Vector Edge
            vx = p2[0] - p1[0]
            vy = p2[1] - p1[1]
            
            # Normal (rotate 90 deg)
            nx = -vy
            ny = vx
            
            # Check direction relative to centroid
            # Vector Centroid -> Midpoint
            v_cm_x = mid_x - cx
            v_cm_y = mid_y - cy
            
            # Dot Product
            dot = nx * v_cm_x + ny * v_cm_y
            
            if dot < 0:
                nx = -nx
                ny = -ny
            
            angle = math.atan2(ny, nx)
            
            self.draw_floating_label(mid_x, mid_y, f"{dist:.1f} m", angle=angle)

    def draw_floating_label(self, x, y, text, is_area=False, angle=None):
        # Check cache for overlaps (only for distance labels, to show x2)
        if not is_area:
             key = (round(x, 1), round(y, 1))
             if hasattr(self, 'label_cache') and key in self.label_cache:
                 existing_t, orig_text = self.label_cache[key]
                 if text == orig_text:
                     # Merge and update existing label
                     new_text = f"{text} (x2)"
                     
                     border_col = "#bdc3c7"
                     bg_col = "rgba(255, 255, 255, 0.9)"
                     existing_t.setHtml(f"<div style='background-color: {bg_col}; border: 1px solid {border_col}; padding: 2px 4px; border-radius: 4px;'>{new_text}</div>")
                     
                     # Re-center (size changed)
                     rect = existing_t.boundingRect()
                     existing_t.setPos(-rect.width()/2, -rect.height()/2)
                     return

        t = QGraphicsTextItem(text)
        font = QFont("Segoe UI", 10 if is_area else 9, QFont.Weight.Bold)
        t.setFont(font)
        
        # Dark green color for area, dark gray for distances
        color = "#145A32" if is_area else "#2c3e50"
        t.setDefaultTextColor(QColor(color))
        
        # Label-style background
        border_col = "#27ae60" if is_area else "#bdc3c7"
        bg_col = "rgba(255, 255, 255, 0.9)"
        
        t.setHtml(f"<div style='background-color: {bg_col}; border: 1px solid {border_col}; padding: 2px 4px; border-radius: 4px;'>{text}</div>")
        
        # Center the text item on its origin 0,0
        rect = t.boundingRect()
        t.setPos(-rect.width()/2, -rect.height()/2)

        # Cache it
        if not is_area and hasattr(self, 'label_cache'):
            self.label_cache[key] = (t, text)

        # Container Group
        group = QGraphicsItemGroup()
        group.addToGroup(t)
        
        group.setPos(x, y)
        group.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
        group.setZValue(20)
        
        if angle is not None:
            # Offset in pixels
            offset = 45 # Aumentado para mayor separación
            dx = offset * math.cos(angle)
            dy = -offset * math.sin(angle) # Negate Y because View is Y-flipped (Scene Up = Screen Y-)
            # Apply offset to text item inside the scale-invariant group
            t.setPos(t.x() + dx, t.y() + dy)
        
        self.scene.addItem(group)
        
    # --- EVENTS ---
    
    def wheelEvent(self, event: QWheelEvent):
        """Zoom Logic"""
        factor = 1.2
        if event.angleDelta().y() > 0:
             self.scale(factor, factor)
        else: 
             self.scale(1/factor, 1/factor)
             
        self.viewport().update()

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.draw_mode_route or self.draw_mode_obstacle:
                scene_pos = self.mapToScene(event.pos())
                if self.draw_mode_route:
                    # Logic for drag start
                    item = self.scene.itemAt(scene_pos, self.transform())
                    if item and item.data(1) == "route_point":
                        self.dragging_point_index = item.data(0)
                        return
                elif self.draw_mode_obstacle:
                    # Obstacle adding
                    self.map_clicked.emit(scene_pos.x(), scene_pos.y())
                    return 
            
            # Standard click
            scene_pos = self.mapToScene(event.pos())
            self.map_clicked.emit(scene_pos.x(), scene_pos.y())
            
            # Pan logic
            self.pan_start = event.pos()
            self.is_panning = True
            self.setCursor(Qt.CursorShape.ClosedHandCursor)

        elif event.button() == Qt.MouseButton.RightButton:
            scene_pos = self.mapToScene(event.pos())
            self.map_right_clicked.emit(scene_pos.x(), scene_pos.y())
        
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event: QMouseEvent):
        super().mouseMoveEvent(event)
        
        # 0. Handle Route Drawing Interaction
        if self.draw_mode_route:
            if self.dragging_point_index is not None:
                scene_pos = self.mapToScene(event.pos())
                x, y = scene_pos.x(), scene_pos.y()
                
                # Update temp route point
                if 0 <= self.dragging_point_index < len(self.temp_route_points):
                    self.temp_route_points[self.dragging_point_index] = (x, y)
                    self.draw_service_route(is_temp=True)
            return

        if self.draw_mode_obstacle:
            return # Block hover effects while drawing obstacles
        
        # 1. Logic for Dragging Points (Priority)
        if self.dragging_point_index is not None:
            scene_pos = self.mapToScene(event.pos())
            self.point_moved.emit(self.dragging_point_index, scene_pos.x(), scene_pos.y())
            return # Skip hover if dragging

        if not self.last_polygon_geom:
            return

        scene_pos = self.mapToScene(event.pos())
        p_mouse = (scene_pos.x(), scene_pos.y())
        
        min_dist = float('inf')
        closest_edge = None # (p1, p2, dist_len, angle)
        
        coords = list(self.last_polygon_geom.exterior.coords)
        if len(coords) < 2: return
        
        for i in range(len(coords)-1):
            p1 = coords[i]
            p2 = coords[i+1]
            
            # Distance Point to Segment
            px, py = p2[0]-p1[0], p2[1]-p1[1]
            norm = px*px + py*py
            if norm == 0: continue
            
            u = ((p_mouse[0] - p1[0]) * px + (p_mouse[1] - p1[1]) * py) / norm
            u = max(min(u, 1), 0)
            
            x = p1[0] + u * px
            y = p1[1] + u * py
            
            dx = x - p_mouse[0]
            dy = y - p_mouse[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                edge_len = math.sqrt(norm)
                
                # Outward Angle Calc
                mx, my = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2
                cx, cy = self.last_polygon_geom.centroid.x, self.last_polygon_geom.centroid.y
                vx, vy = p2[0]-p1[0], p2[1]-p1[1]
                nx, ny = -vy, vx
                if nx*(mx-cx) + ny*(my-cy) < 0: nx, ny = -nx, -ny
                angle = math.atan2(ny, nx)
                
                closest_edge = ((mx, my), edge_len, angle)

        threshold = 5.0 # meters
        
        if min_dist < threshold and closest_edge:
            mid, length, ang = closest_edge
            text = f"{length:.1f} m"
            self.update_hover_label(mid[0], mid[1], text, ang)
        else:
            if self.hover_group: self.hover_group.setVisible(False)

    def update_hover_label(self, x, y, text, angle):
        if not self.hover_group:
            self.hover_text = QGraphicsSimpleTextItem()
            self.hover_text.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            self.hover_bg = QGraphicsRectItem()
            self.hover_bg.setBrush(QBrush(QColor(255, 255, 255, 220)))
            self.hover_bg.setPen(QPen(Qt.PenStyle.NoPen))
            
            self.hover_group = QGraphicsItemGroup()
            self.hover_group.addToGroup(self.hover_bg)
            self.hover_group.addToGroup(self.hover_text)
            self.hover_group.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIgnoresTransformations)
            self.hover_group.setZValue(200)
            self.scene.addItem(self.hover_group)
            
        self.hover_group.setVisible(True)
        self.hover_text.setText(text)
        
        # Layout
        r = self.hover_text.boundingRect()
        pad = 4
        self.hover_bg.setRect(r.x()-pad, r.y()-pad, r.width()+2*pad, r.height()+2*pad)
        
        # Center in Group
        off_x = -r.width()/2
        off_y = -r.height()/2
        self.hover_text.setPos(off_x, off_y)
        self.hover_bg.setPos(off_x, off_y)
        
        # Position with Angle Offset
        offset_px = 25
        dx = offset_px * math.cos(angle)
        dy = offset_px * math.sin(angle)
        
        # Since group ignores transform, x/y are scene pos, but internal matrix is pixels.
        # So we can't just add dx to x? 
        # Actually ItemIgnoresTransformations anchors the item at pos() in scene, 
        # and draws it with identity transform (pixels) at that point.
        # But if we want the TEXT to be offset by pixels, we move the item (or group) contents?
        # NO. We move the group anchor? No, group anchor is scene coord.
        # We must add pixel offset to the child items!
        
        # Reset local pos to center
        self.hover_text.setPos(off_x + dx, off_y + dy)
        self.hover_bg.setPos(off_x + dx, off_y + dy)
        
        self.hover_group.setPos(x, y)

    def mousePressEvent(self, event: QMouseEvent):
        # 0. Route Drawing Mode (Priority)
        if self.draw_mode_route:
            scene_pos = self.mapToScene(event.pos())
            x, y = scene_pos.x(), scene_pos.y()
            
            # Check for click on existing point
            item = self.scene.itemAt(scene_pos, self.transform())
            clicked_point_idx = None
            if isinstance(item, QGraphicsEllipseItem) and item.data(1) == "route_point":
                clicked_point_idx = item.data(0)

            if event.button() == Qt.MouseButton.LeftButton:
                if clicked_point_idx is not None:
                    # Start Dragging Route Point
                    self.dragging_point_index = clicked_point_idx
                    self.setDragMode(QGraphicsView.DragMode.NoDrag)
                else:
                    # Add New Point
                    self.temp_route_points.append((x, y))
                    self.draw_service_route(is_temp=True)
            
            elif event.button() == Qt.MouseButton.RightButton:
                if clicked_point_idx is not None:
                     # Delete Point
                     if 0 <= clicked_point_idx < len(self.temp_route_points):
                         self.temp_route_points.pop(clicked_point_idx)
                         self.draw_service_route(is_temp=True)
                else:
                    # Finish Drawing
                    if len(self.temp_route_points) >= 2:
                        self.service_route_points = list(self.temp_route_points)
                        self.draw_service_route(is_temp=False)
                        self.set_draw_mode_route(False)
                    else:
                        # Cancel
                        self.temp_route_points = []
                        self.set_draw_mode_route(False)
                        self.draw_service_route(is_temp=False)
            return

        # 1. Detectar si hicimos click en un PUNTO existente
        scene_pos = self.mapToScene(event.pos())
        item = self.scene.itemAt(scene_pos, self.transform())
        
        if event.button() == Qt.MouseButton.LeftButton:
            # Si es un punto (tiene datos), iniciamos arrastre
            if isinstance(item, QGraphicsEllipseItem) and item.data(0) is not None:
                self.dragging_point_index = item.data(0)
                self.setDragMode(QGraphicsView.DragMode.NoDrag) # Bloquear otros drags
                return

            if event.modifiers() == Qt.KeyboardModifier.ControlModifier:
                self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
                super().mousePressEvent(event)
            else:
                self.map_clicked.emit(scene_pos.x(), scene_pos.y())
                
        elif event.button() == Qt.MouseButton.RightButton:
            self.map_right_clicked.emit(scene_pos.x(), scene_pos.y())

    def mouseReleaseEvent(self, event: QMouseEvent):
        super().mouseReleaseEvent(event)
        self.dragging_point_index = None
        self.setDragMode(QGraphicsView.DragMode.NoDrag)