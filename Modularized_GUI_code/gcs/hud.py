# # cube_visualizer2.py — HUD-style orientation display using pygame
# # Drop-in replacement for the 3-D cube visualizer.
# # Same class name, same constructor, same thread interface — nothing else changes.

# import pygame
# import math
# import queue
# import threading


# # ── Colour palette ────────────────────────────────────────────────────────────
# C = {
#     "bg":          ( 10,  12,  20),   # near-black background
#     "sky":         ( 20,  60, 120),   # artificial horizon sky
#     "ground":      ( 80,  50,  20),   # artificial horizon ground
#     "horizon":     (220, 220, 220),   # horizon line
#     "ladder":      (200, 220, 200),   # pitch ladder lines
#     "ladder_text": (200, 220, 200),
#     "roll_arc":    (255, 200,   0),   # roll scale arc
#     "roll_ptr":    (255, 200,   0),   # roll pointer triangle
#     "wings":       (255, 220,  50),   # fixed aircraft symbol
#     "centre_dot":  (255, 255, 255),
#     "yaw_bar":     ( 30,  30,  50),   # compass bar background
#     "yaw_tick":    (160, 160, 180),
#     "yaw_text":    (200, 220, 255),
#     "yaw_ptr":     (255, 200,   0),
#     "label":       ( 80, 180, 255),   # sidebar label colour
#     "value":       (255, 255, 255),
#     "warn":        (255,  80,  80),
#     "green":       ( 50, 220, 100),
#     "border":      ( 40,  60, 100),
#     "overlay_bg":  (  0,   0,   0, 160),  # semi-transparent
# }


# def _rot(x, y, angle_deg):
#     """Rotate point (x, y) around origin by angle_deg."""
#     a = math.radians(angle_deg)
#     c, s = math.cos(a), math.sin(a)
#     return x * c - y * s, x * s + y * c


# class FlightHUD(threading.Thread):
#     """
#     HUD-style orientation display.

#     Layout
#     ──────
#     ┌─────────────────────────────────────────┐
#     │  [roll °]        PITCH/ROLL HUD         │
#     │                                         │
#     │      pitch ladder + artificial horizon  │
#     │               fixed wings               │
#     │                                         │
#     │  ══════════ compass bar ════════════    │
#     │  Pitch  Roll  Yaw  ← numeric sidebar   │
#     └─────────────────────────────────────────┘

#     Data interface (unchanged from cube version):
#       data_queue.put((roll, pitch, yaw))   — degrees
#     """

#     WIDTH  = 720
#     HEIGHT = 540

#     # HUD circle geometry
#     HUD_CX = 360          # centre x of the attitude indicator
#     HUD_CY = 235          # centre y
#     HUD_R  = 185          # radius of the clipping circle

#     # Pitch ladder: degrees per pixel
#     PX_PER_DEG = 7.0

#     # Roll arc
#     ROLL_R     = 170      # radius of roll scale arc
#     ROLL_TICKS = [        # (angle_from_top°, label)
#         (-60, "60"), (-45, "45"), (-30, "30"), (-20, "20"),
#         (-10, "10"), (0,   "0"),
#         ( 10, "10"), ( 20, "20"), ( 30, "30"), ( 45, "45"),
#         ( 60, "60"),
#     ]

#     # Compass bar
#     COMP_Y  = 455         # y-centre of compass bar
#     COMP_W  = 620         # width
#     COMP_H  = 36
#     COMP_X0 = (WIDTH - COMP_W) // 2

#     def __init__(self, data_queue: queue.Queue):
#         super().__init__(daemon=True)
#         self.data_queue = data_queue
#         self.roll  = 0.0
#         self.pitch = 0.0
#         self.yaw   = 0.0
#         self.running = True

#     # ── Main thread loop ──────────────────────────────────────────────────
#     def run(self):
#         pygame.init()
#         self._screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
#         pygame.display.set_caption("ESP32 — HUD Orientation Display")
#         self._clock = pygame.time.Clock()

#         # Fonts
#         self._font_sm  = pygame.font.SysFont("consolas", 11)
#         self._font_md  = pygame.font.SysFont("consolas", 13)
#         self._font_lg  = pygame.font.SysFont("consolas", 16, bold=True)
#         self._font_xl  = pygame.font.SysFont("consolas", 20, bold=True)

#         # Pre-render static compass labels
#         self._compass_labels = {
#             0: "N", 45: "NE", 90: "E", 135: "SE",
#             180: "S", 225: "SW", 270: "W", 315: "NW",
#         }

#         # Off-screen surface for the HUD circle (with clipping)
#         self._hud_surf = pygame.Surface(
#             (self.WIDTH, self.HEIGHT), pygame.SRCALPHA)

#         while self.running:
#             for ev in pygame.event.get():
#                 if ev.type == pygame.QUIT:
#                     self.running = False

#             # Drain queue — take the latest value only
#             try:
#                 while True:
#                     self.roll, self.pitch, self.yaw = \
#                         self.data_queue.get_nowait()
#             except queue.Empty:
#                 pass

#             self._draw()
#             pygame.display.flip()
#             self._clock.tick(60)

#         pygame.quit()

#     def stop(self):
#         self.running = False

#     # ── Top-level draw ────────────────────────────────────────────────────
#     def _draw(self):
#         scr = self._screen
#         scr.fill(C["bg"])

#         self._draw_attitude()    # artificial horizon inside clipped circle
#         self._draw_roll_scale()  # arc + pointer outside circle top
#         self._draw_fixed_wings() # aircraft symbol
#         self._draw_hud_ring()    # border ring
#         self._draw_compass()     # compass bar at the bottom
#         self._draw_sidebar()     # numeric readouts on the left

#     # ── Artificial horizon ────────────────────────────────────────────────
#     def _draw_attitude(self):
#         """
#         Draw the sky/ground split rotated by roll, shifted by pitch,
#         clipped to the HUD circle.
#         """
#         cx, cy, r = self.HUD_CX, self.HUD_CY, self.HUD_R
#         pitch_px = self.pitch * self.PX_PER_DEG   # positive pitch → horizon moves down

#         surf = pygame.Surface((self.WIDTH, self.HEIGHT))
#         surf.fill(C["bg"])

#         # --- Sky / ground polygon clipped to circle ---
#         # We draw a large rectangle for sky and ground, then rotate around
#         # the HUD centre by the roll angle.

#         # Horizon line passes through (cx, cy - pitch_px) in the un-rotated frame
#         hy = cy - pitch_px

#         # Build a large rect for sky (above horizon) and ground (below)
#         big = r * 3
#         sky_pts    = [(-big, -big), (big, -big),
#                       (big, hy - cy), (-big, hy - cy)]
#         ground_pts = [(-big, hy - cy), (big, hy - cy),
#                       (big, big), (-big, big)]

#         def rotate_pts(pts):
#             return [(cx + rx, cy + ry)
#                     for (rx, ry) in (_rot(x, y, -self.roll)
#                                      for x, y in pts)]

#         sky_world    = rotate_pts(sky_pts)
#         ground_world = rotate_pts(ground_pts)

#         # Draw to surf, then clip to circle
#         pygame.draw.polygon(surf, C["sky"],    sky_world)
#         pygame.draw.polygon(surf, C["ground"], ground_world)

#         # Clip surf to circle and blit onto screen
#         clip_mask = pygame.Surface((self.WIDTH, self.HEIGHT), pygame.SRCALPHA)
#         clip_mask.fill((0, 0, 0, 0))
#         pygame.draw.circle(clip_mask, (255, 255, 255, 255), (cx, cy), r)

#         # Use clip_mask as stencil
#         surf.set_colorkey(C["bg"])
#         clipped = pygame.Surface((self.WIDTH, self.HEIGHT), pygame.SRCALPHA)
#         clipped.blit(surf,       (0, 0))
#         clipped.blit(clip_mask,  (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
#         self._screen.blit(clipped, (0, 0))

#         # --- Horizon line ---
#         hx0, hy0 = cx + _rot(-r, -pitch_px, -self.roll)[0], \
#                    cy + _rot(-r, -pitch_px, -self.roll)[1]
#         hx1, hy1 = cx + _rot( r, -pitch_px, -self.roll)[0], \
#                    cy + _rot( r, -pitch_px, -self.roll)[1]
#         pygame.draw.line(self._screen, C["horizon"],
#                          (int(hx0), int(hy0)), (int(hx1), int(hy1)), 2)

#         # --- Pitch ladder ---
#         self._draw_pitch_ladder()

#     def _draw_pitch_ladder(self):
#         cx, cy, r = self.HUD_CX, self.HUD_CY, self.HUD_R
#         scr = self._screen

#         for deg in range(-30, 35, 5):
#             if deg == 0:
#                 continue
#             y_off  = -(deg - self.pitch) * self.PX_PER_DEG
#             # Rotate for roll
#             lx0, ly0 = _rot(-40, y_off, -self.roll)
#             lx1, ly1 = _rot( 40, y_off, -self.roll)
#             sx0, sy0 = cx + lx0, cy + ly0
#             sx1, sy1 = cx + lx1, cy + ly1

#             # Only draw if inside HUD circle
#             mid_x, mid_y = (sx0 + sx1) / 2, (sy0 + sy1) / 2
#             if math.hypot(mid_x - cx, mid_y - cy) > r - 10:
#                 continue

#             is_ten = (deg % 10 == 0)
#             colour = C["ladder"]
#             width  = 2 if is_ten else 1
#             pygame.draw.line(scr, colour,
#                              (int(sx0), int(sy0)),
#                              (int(sx1), int(sy1)), width)

#             if is_ten:
#                 lbl = self._font_sm.render(f"{deg:+d}", True, C["ladder_text"])
#                 tx, ty = _rot(48, y_off, -self.roll)
#                 scr.blit(lbl, (int(cx + tx) - 4, int(cy + ty) - 7))

#     # ── Roll scale ────────────────────────────────────────────────────────
#     def _draw_roll_scale(self):
#         cx, cy = self.HUD_CX, self.HUD_CY
#         scr    = self._screen
#         R      = self.ROLL_R

#         for angle, label in self.ROLL_TICKS:
#             # angle 0 = top of circle
#             ax, ay = _rot(0, -R, angle)
#             px, py = int(cx + ax), int(cy + ay)

#             # Tick mark
#             inner = 8 if angle in (-30, 0, 30, -60, 60) else 5
#             ix, iy = _rot(0, -(R - inner), angle)
#             pygame.draw.line(scr, C["roll_arc"],
#                              (int(cx + ix), int(cy + iy)), (px, py), 2)

#             # Label for major ticks only
#             if abs(angle) in (30, 60) or angle == 0:
#                 lbl = self._font_sm.render(label, True, C["roll_arc"])
#                 ox, oy = _rot(0, -(R - 22), angle)
#                 scr.blit(lbl, (int(cx + ox) - lbl.get_width() // 2,
#                                int(cy + oy) - lbl.get_height() // 2))

#         # Arc
#         rect = pygame.Rect(cx - R, cy - R, R * 2, R * 2)
#         pygame.draw.arc(scr, C["roll_arc"], rect,
#                         math.radians(90 - 65),
#                         math.radians(90 + 65), 2)

#         # Roll pointer triangle (rotates with roll)
#         tri_tip   = _rot(0, -(R - 2),  self.roll)
#         tri_left  = _rot(-8, -(R - 16), self.roll)
#         tri_right = _rot( 8, -(R - 16), self.roll)
#         pts = [
#             (int(cx + tri_tip[0]),   int(cy + tri_tip[1])),
#             (int(cx + tri_left[0]),  int(cy + tri_left[1])),
#             (int(cx + tri_right[0]), int(cy + tri_right[1])),
#         ]
#         pygame.draw.polygon(scr, C["roll_ptr"], pts)

#         # Roll value text (top-left corner)
#         roll_txt = self._font_lg.render(
#             f"ROLL  {self.roll:+6.1f}°", True, C["roll_ptr"])
#         scr.blit(roll_txt, (14, 14))

#     # ── Fixed aircraft wings ──────────────────────────────────────────────
#     def _draw_fixed_wings(self):
#         cx, cy = self.HUD_CX, self.HUD_CY
#         scr    = self._screen
#         w      = 2   # line thickness

#         # Left wing  ────┤
#         pygame.draw.line(scr, C["wings"], (cx - 90, cy), (cx - 30, cy), w)
#         pygame.draw.line(scr, C["wings"], (cx - 30, cy), (cx - 30, cy + 12), w)
#         pygame.draw.line(scr, C["wings"], (cx - 30, cy + 12), (cx - 10, cy + 12), w)
#         # Right wing  ├────
#         pygame.draw.line(scr, C["wings"], (cx + 90, cy), (cx + 30, cy), w)
#         pygame.draw.line(scr, C["wings"], (cx + 30, cy), (cx + 30, cy + 12), w)
#         pygame.draw.line(scr, C["wings"], (cx + 30, cy + 12), (cx + 10, cy + 12), w)
#         # Centre dot
#         pygame.draw.circle(scr, C["centre_dot"], (cx, cy), 4)

#     # ── HUD ring border ───────────────────────────────────────────────────
#     def _draw_hud_ring(self):
#         pygame.draw.circle(self._screen, C["border"],
#                            (self.HUD_CX, self.HUD_CY),
#                            self.HUD_R, 2)

#     # ── Compass bar ───────────────────────────────────────────────────────
#     def _draw_compass(self):
#         scr = self._screen
#         x0, y0 = self.COMP_X0, self.COMP_Y - self.COMP_H // 2
#         w,  h  = self.COMP_W,  self.COMP_H

#         # Background
#         pygame.draw.rect(scr, C["yaw_bar"],   (x0, y0, w, h))
#         pygame.draw.rect(scr, C["border"],    (x0, y0, w, h), 1)

#         # Ticks: show ±90° of heading centred on current yaw
#         cx_comp = x0 + w // 2
#         deg_per_px = 90.0 / (w // 2)    # how many degrees fit in half the bar

#         for heading in range(0, 360, 5):
#             diff = (heading - self.yaw + 180) % 360 - 180
#             if abs(diff) > 92:
#                 continue
#             px = int(cx_comp + diff / deg_per_px)

#             is_major = (heading % 45 == 0)
#             is_minor = (heading % 15 == 0) and not is_major
#             tick_h   = 14 if is_major else (8 if is_minor else 4)
#             colour   = C["yaw_text"] if is_major else C["yaw_tick"]

#             pygame.draw.line(scr, colour,
#                              (px, y0 + 2),
#                              (px, y0 + 2 + tick_h), 1)

#             if is_major:
#                 label = self._compass_labels.get(heading, str(heading))
#                 lbl   = self._font_sm.render(label, True, C["yaw_text"])
#                 scr.blit(lbl, (px - lbl.get_width() // 2, y0 + 18))

#         # Centre pointer ▼
#         ptr_x = cx_comp
#         ptr_y = y0 - 1
#         pygame.draw.polygon(scr, C["yaw_ptr"], [
#             (ptr_x,     ptr_y),
#             (ptr_x - 7, ptr_y - 10),
#             (ptr_x + 7, ptr_y - 10),
#         ])

#         # Heading value
#         hdg_lbl = self._font_lg.render(
#             f"{int(self.yaw % 360):03d}°", True, C["yaw_ptr"])
#         scr.blit(hdg_lbl, (cx_comp - hdg_lbl.get_width() // 2,
#                            y0 + h + 4))

#     # ── Sidebar numeric readouts ──────────────────────────────────────────
#     def _draw_sidebar(self):
#         scr = self._screen
#         sx  = self.WIDTH - 150
#         sy  = self.HUD_CY - 60

#         entries = [
#             ("PITCH", f"{self.pitch:+.1f}°",
#              C["warn"] if abs(self.pitch) > 25 else C["value"]),
#             ("ROLL",  f"{self.roll:+.1f}°",
#              C["warn"] if abs(self.roll)  > 35 else C["value"]),
#             ("YAW",   f"{self.yaw % 360:.1f}°", C["value"]),
#         ]

#         for i, (label, val, vcol) in enumerate(entries):
#             lbl = self._font_sm.render(label, True, C["label"])
#             scr.blit(lbl, (sx, sy + i * 40))
#             val_surf = self._font_xl.render(val, True, vcol)
#             scr.blit(val_surf, (sx, sy + i * 40 + 14))

#         # Divider line
#         pygame.draw.line(scr, C["border"],
#                          (sx - 10, sy - 10),
#                          (sx - 10, sy + 3 * 40 + 10), 1)

#         # Title
#         title = self._font_md.render("ESP32  HUD", True, C["label"])
#         scr.blit(title, (self.HUD_CX - title.get_width() // 2, 6))


# # Backward-compatibility alias
# CubeVisualizer2 = FlightHUD

# =========================================

# cube_visualizer2.py — HUD-style orientation display using pygame
# Drop-in replacement for the 3-D cube visualizer.
# Same class name, same constructor, same thread interface — nothing else changes.

import pygame
import math
import queue
import threading


# ── Colour palette ────────────────────────────────────────────────────────────
C = {
    "bg":          ( 10,  12,  20),   # near-black background
    "sky":         ( 20,  60, 120),   # artificial horizon sky
    "ground":      ( 80,  50,  20),   # artificial horizon ground
    "horizon":     (220, 220, 220),   # horizon line
    "ladder":      (200, 220, 200),   # pitch ladder lines
    "ladder_text": (200, 220, 200),
    "roll_arc":    (255, 200,   0),   # roll scale arc
    "roll_ptr":    (255, 200,   0),   # roll pointer triangle
    "wings":       (255, 220,  50),   # fixed aircraft symbol
    "centre_dot":  (255, 255, 255),
    "yaw_bar":     ( 30,  30,  50),   # compass bar background
    "yaw_tick":    (160, 160, 180),
    "yaw_text":    (200, 220, 255),
    "yaw_ptr":     (255, 200,   0),
    "label":       ( 80, 180, 255),   # sidebar label colour
    "value":       (255, 255, 255),
    "warn":        (255,  80,  80),
    "green":       ( 50, 220, 100),
    "border":      ( 40,  60, 100),
    "overlay_bg":  (  0,   0,   0, 160),  # semi-transparent
}


def _rot(x, y, angle_deg):
    """Rotate point (x, y) around origin by angle_deg."""
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    return x * c - y * s, x * s + y * c


class FlightHUD(threading.Thread):
    """
    HUD-style orientation display.

    Layout
    ──────
    ┌─────────────────────────────────────────┐
    │  [roll °]        PITCH/ROLL HUD         │
    │                                         │
    │      pitch ladder + artificial horizon  │
    │               fixed wings               │
    │                                         │
    │  ══════════ compass bar ════════════    │
    │  Pitch  Roll  Yaw  ← numeric sidebar   │
    └─────────────────────────────────────────┘

    Data interface (unchanged from cube version):
      data_queue.put((roll, pitch, yaw))   — degrees
    """

    WIDTH  = 720
    HEIGHT = 540

    # HUD circle geometry
    HUD_CX = 360          # centre x of the attitude indicator
    HUD_CY = 235          # centre y
    HUD_R  = 185          # radius of the clipping circle

    # Pitch ladder: degrees per pixel
    PX_PER_DEG = 7.0

    # Roll arc
    ROLL_R     = 170      # radius of roll scale arc
    ROLL_TICKS = [        # (angle_from_top°, label)
        (-60, "60"), (-45, "45"), (-30, "30"), (-20, "20"),
        (-10, "10"), (0,   "0"),
        ( 10, "10"), ( 20, "20"), ( 30, "30"), ( 45, "45"),
        ( 60, "60"),
    ]

    # Compass bar
    COMP_Y  = 455         # y-centre of compass bar
    COMP_W  = 620         # width
    COMP_H  = 36
    COMP_X0 = (WIDTH - COMP_W) // 2

    def __init__(self, data_queue: queue.Queue):
        super().__init__(daemon=True)
        self.data_queue = data_queue
        self.roll  = 0.0
        self.pitch = 0.0
        self.yaw   = 0.0
        self.running = True

    # ── Main thread loop ──────────────────────────────────────────────────
    def run(self):
        pygame.init()
        self._screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        pygame.display.set_caption("ESP32 — HUD Orientation Display")
        self._clock = pygame.time.Clock()

        # Fonts
        self._font_sm  = pygame.font.SysFont("consolas", 11)
        self._font_md  = pygame.font.SysFont("consolas", 13)
        self._font_lg  = pygame.font.SysFont("consolas", 16, bold=True)
        self._font_xl  = pygame.font.SysFont("consolas", 20, bold=True)

        # Pre-render static compass labels
        self._compass_labels = {
            0: "N", 45: "NE", 90: "E", 135: "SE",
            180: "S", 225: "SW", 270: "W", 315: "NW",
        }

        # Off-screen surface for the HUD circle (with clipping)
        self._hud_surf = pygame.Surface(
            (self.WIDTH, self.HEIGHT), pygame.SRCALPHA)

        while self.running:
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.running = False

            # Drain queue — take the latest value only
            try:
                while True:
                    self.pitch, self.roll, self.yaw = \
                        self.data_queue.get_nowait()
            except queue.Empty:
                pass

            self._draw()
            pygame.display.flip()
            self._clock.tick(60)

        pygame.quit()

    def stop(self):
        self.running = False

    # ── Top-level draw ────────────────────────────────────────────────────
    def _draw(self):
        scr = self._screen
        scr.fill(C["bg"])

        self._draw_attitude()    # artificial horizon inside clipped circle
        self._draw_roll_scale()  # arc + pointer outside circle top
        self._draw_fixed_wings() # aircraft symbol
        self._draw_hud_ring()    # border ring
        self._draw_compass()     # compass bar at the bottom
        self._draw_sidebar()     # numeric readouts on the left

    # ── Artificial horizon ────────────────────────────────────────────────
    def _draw_attitude(self):
        """
        Draw the sky/ground split rotated by roll, shifted by pitch,
        clipped to the HUD circle.
        """
        cx, cy, r = self.HUD_CX, self.HUD_CY, self.HUD_R
        pitch_px = self.pitch * self.PX_PER_DEG   # positive pitch → horizon moves down

        surf = pygame.Surface((self.WIDTH, self.HEIGHT))
        surf.fill(C["bg"])

        # --- Sky / ground polygon clipped to circle ---
        # We draw a large rectangle for sky and ground, then rotate around
        # the HUD centre by the roll angle.

        # Horizon line passes through (cx, cy - pitch_px) in the un-rotated frame
        hy = cy - pitch_px

        # Build a large rect for sky (above horizon) and ground (below)
        big = r * 3
        sky_pts    = [(-big, -big), (big, -big),
                      (big, hy - cy), (-big, hy - cy)]
        ground_pts = [(-big, hy - cy), (big, hy - cy),
                      (big, big), (-big, big)]

        def rotate_pts(pts):
            return [(cx + rx, cy + ry)
                    for (rx, ry) in (_rot(x, y, -self.roll)
                                     for x, y in pts)]

        sky_world    = rotate_pts(sky_pts)
        ground_world = rotate_pts(ground_pts)

        # Draw to surf, then clip to circle
        pygame.draw.polygon(surf, C["sky"],    sky_world)
        pygame.draw.polygon(surf, C["ground"], ground_world)

        # Clip surf to circle and blit onto screen
        clip_mask = pygame.Surface((self.WIDTH, self.HEIGHT), pygame.SRCALPHA)
        clip_mask.fill((0, 0, 0, 0))
        pygame.draw.circle(clip_mask, (255, 255, 255, 255), (cx, cy), r)

        # Use clip_mask as stencil
        surf.set_colorkey(C["bg"])
        clipped = pygame.Surface((self.WIDTH, self.HEIGHT), pygame.SRCALPHA)
        clipped.blit(surf,       (0, 0))
        clipped.blit(clip_mask,  (0, 0), special_flags=pygame.BLEND_RGBA_MIN)
        self._screen.blit(clipped, (0, 0))

        # --- Horizon line ---
        hx0, hy0 = cx + _rot(-r, -pitch_px, -self.roll)[0], \
                   cy + _rot(-r, -pitch_px, -self.roll)[1]
        hx1, hy1 = cx + _rot( r, -pitch_px, -self.roll)[0], \
                   cy + _rot( r, -pitch_px, -self.roll)[1]
        pygame.draw.line(self._screen, C["horizon"],
                         (int(hx0), int(hy0)), (int(hx1), int(hy1)), 2)

        # --- Pitch ladder ---
        self._draw_pitch_ladder()

    def _draw_pitch_ladder(self):
        cx, cy, r = self.HUD_CX, self.HUD_CY, self.HUD_R
        scr = self._screen

        for deg in range(-30, 35, 5):
            if deg == 0:
                continue
            y_off  = -(deg - self.pitch) * self.PX_PER_DEG
            # Rotate for roll
            lx0, ly0 = _rot(-40, y_off, -self.roll)
            lx1, ly1 = _rot( 40, y_off, -self.roll)
            sx0, sy0 = cx + lx0, cy + ly0
            sx1, sy1 = cx + lx1, cy + ly1

            # Only draw if inside HUD circle
            mid_x, mid_y = (sx0 + sx1) / 2, (sy0 + sy1) / 2
            if math.hypot(mid_x - cx, mid_y - cy) > r - 10:
                continue

            is_ten = (deg % 10 == 0)
            colour = C["ladder"]
            width  = 2 if is_ten else 1
            pygame.draw.line(scr, colour,
                             (int(sx0), int(sy0)),
                             (int(sx1), int(sy1)), width)

            if is_ten:
                lbl = self._font_sm.render(f"{deg:+d}", True, C["ladder_text"])
                tx, ty = _rot(48, y_off, -self.roll)
                scr.blit(lbl, (int(cx + tx) - 4, int(cy + ty) - 7))

    # ── Roll scale ────────────────────────────────────────────────────────
    def _draw_roll_scale(self):
        cx, cy = self.HUD_CX, self.HUD_CY
        scr    = self._screen
        R      = self.ROLL_R

        for angle, label in self.ROLL_TICKS:
            # angle 0 = top of circle
            ax, ay = _rot(0, -R, angle)
            px, py = int(cx + ax), int(cy + ay)

            # Tick mark
            inner = 8 if angle in (-30, 0, 30, -60, 60) else 5
            ix, iy = _rot(0, -(R - inner), angle)
            pygame.draw.line(scr, C["roll_arc"],
                             (int(cx + ix), int(cy + iy)), (px, py), 2)

            # Label for major ticks only
            if abs(angle) in (30, 60) or angle == 0:
                lbl = self._font_sm.render(label, True, C["roll_arc"])
                ox, oy = _rot(0, -(R - 22), angle)
                scr.blit(lbl, (int(cx + ox) - lbl.get_width() // 2,
                               int(cy + oy) - lbl.get_height() // 2))

        # Arc
        rect = pygame.Rect(cx - R, cy - R, R * 2, R * 2)
        pygame.draw.arc(scr, C["roll_arc"], rect,
                        math.radians(90 - 65),
                        math.radians(90 + 65), 2)

        # Roll pointer triangle (rotates with roll)
        tri_tip   = _rot(0, -(R - 2),  self.roll)
        tri_left  = _rot(-8, -(R - 16), self.roll)
        tri_right = _rot( 8, -(R - 16), self.roll)
        pts = [
            (int(cx + tri_tip[0]),   int(cy + tri_tip[1])),
            (int(cx + tri_left[0]),  int(cy + tri_left[1])),
            (int(cx + tri_right[0]), int(cy + tri_right[1])),
        ]
        pygame.draw.polygon(scr, C["roll_ptr"], pts)

        # Roll value text (top-left corner)
        roll_txt = self._font_lg.render(
            f"ROLL  {self.roll:+6.1f}°", True, C["roll_ptr"])
        scr.blit(roll_txt, (14, 14))

    # ── Fixed aircraft wings ──────────────────────────────────────────────
    def _draw_fixed_wings(self):
        cx, cy = self.HUD_CX, self.HUD_CY
        scr    = self._screen
        w      = 2   # line thickness

        # Left wing  ────┤
        pygame.draw.line(scr, C["wings"], (cx - 90, cy), (cx - 30, cy), w)
        pygame.draw.line(scr, C["wings"], (cx - 30, cy), (cx - 30, cy + 12), w)
        pygame.draw.line(scr, C["wings"], (cx - 30, cy + 12), (cx - 10, cy + 12), w)
        # Right wing  ├────
        pygame.draw.line(scr, C["wings"], (cx + 90, cy), (cx + 30, cy), w)
        pygame.draw.line(scr, C["wings"], (cx + 30, cy), (cx + 30, cy + 12), w)
        pygame.draw.line(scr, C["wings"], (cx + 30, cy + 12), (cx + 10, cy + 12), w)
        # Centre dot
        pygame.draw.circle(scr, C["centre_dot"], (cx, cy), 4)

    # ── HUD ring border ───────────────────────────────────────────────────
    def _draw_hud_ring(self):
        pygame.draw.circle(self._screen, C["border"],
                           (self.HUD_CX, self.HUD_CY),
                           self.HUD_R, 2)

    # ── Compass bar ───────────────────────────────────────────────────────
    def _draw_compass(self):
        scr = self._screen
        x0, y0 = self.COMP_X0, self.COMP_Y - self.COMP_H // 2
        w,  h  = self.COMP_W,  self.COMP_H

        # Background
        pygame.draw.rect(scr, C["yaw_bar"],   (x0, y0, w, h))
        pygame.draw.rect(scr, C["border"],    (x0, y0, w, h), 1)

        # Ticks: show ±90° of heading centred on current yaw
        cx_comp = x0 + w // 2
        deg_per_px = 90.0 / (w // 2)    # how many degrees fit in half the bar

        for heading in range(0, 360, 5):
            diff = (heading - self.yaw + 180) % 360 - 180
            if abs(diff) > 92:
                continue
            px = int(cx_comp + diff / deg_per_px)

            is_major = (heading % 45 == 0)
            is_minor = (heading % 15 == 0) and not is_major
            tick_h   = 14 if is_major else (8 if is_minor else 4)
            colour   = C["yaw_text"] if is_major else C["yaw_tick"]

            pygame.draw.line(scr, colour,
                             (px, y0 + 2),
                             (px, y0 + 2 + tick_h), 1)

            if is_major:
                label = self._compass_labels.get(heading, str(heading))
                lbl   = self._font_sm.render(label, True, C["yaw_text"])
                scr.blit(lbl, (px - lbl.get_width() // 2, y0 + 18))

        # Centre pointer ▼
        ptr_x = cx_comp
        ptr_y = y0 - 1
        pygame.draw.polygon(scr, C["yaw_ptr"], [
            (ptr_x,     ptr_y),
            (ptr_x - 7, ptr_y - 10),
            (ptr_x + 7, ptr_y - 10),
        ])

        # Heading value
        hdg_lbl = self._font_lg.render(
            f"{int(self.yaw % 360):03d}°", True, C["yaw_ptr"])
        scr.blit(hdg_lbl, (cx_comp - hdg_lbl.get_width() // 2,
                           y0 + h + 4))

    # ── Sidebar numeric readouts ──────────────────────────────────────────
    def _draw_sidebar(self):
        scr = self._screen
        sx  = self.WIDTH - 150
        sy  = self.HUD_CY - 60

        entries = [
            ("PITCH", f"{self.pitch:+.1f}°",
             C["warn"] if abs(self.pitch) > 25 else C["value"]),
            ("ROLL",  f"{self.roll:+.1f}°",
             C["warn"] if abs(self.roll)  > 35 else C["value"]),
            ("YAW",   f"{self.yaw % 360:.1f}°", C["value"]),
        ]

        for i, (label, val, vcol) in enumerate(entries):
            lbl = self._font_sm.render(label, True, C["label"])
            scr.blit(lbl, (sx, sy + i * 40))
            val_surf = self._font_xl.render(val, True, vcol)
            scr.blit(val_surf, (sx, sy + i * 40 + 14))

        # Divider line
        pygame.draw.line(scr, C["border"],
                         (sx - 10, sy - 10),
                         (sx - 10, sy + 3 * 40 + 10), 1)

        # Title
        title = self._font_md.render("ESP32  HUD", True, C["label"])
        scr.blit(title, (self.HUD_CX - title.get_width() // 2, 6))


# Backward-compatibility alias
CubeVisualizer2 = FlightHUD
