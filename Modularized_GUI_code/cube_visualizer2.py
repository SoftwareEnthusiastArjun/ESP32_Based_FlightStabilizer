# cube_visualizer2.py
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import threading
import queue

class CubeVisualizer2(threading.Thread):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue  # Shared queue for gx,gy,gz
        self.ax = self.ay = self.az = 0.0
        self.running = True
        self.daemon = True  # Thread closes with main program

    def init_gl(self):
        glViewport(0, 0, 640, 480)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, 640 / 480, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glShadeModel(GL_SMOOTH)
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

    def draw_cube(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -7.0)

        glRotatef(self.ay, 1.0, 0.0, 0.0)   # Pitch
        glRotatef(self.az, 0.0, 1.0, 0.0)   # Yaw
        glRotatef(-self.ax, 0.0, 0.0, 1.0)  # Roll

        glBegin(GL_QUADS)
        glColor3f(0.0, 1.0, 0.0)  # Front
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(1.0, 0.2, 1.0)

        glColor3f(1.0, 0.5, 0.0)  # Back
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(1.0, -0.2, -1.0)

        glColor3f(1.0, 0.0, 0.0)  # Top
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)

        glColor3f(1.0, 1.0, 0.0)  # Bottom
        glVertex3f(1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, -1.0)

        glColor3f(0.0, 0.0, 1.0)  # Left
        glVertex3f(-1.0, 0.2, 1.0)
        glVertex3f(-1.0, 0.2, -1.0)
        glVertex3f(-1.0, -0.2, -1.0)
        glVertex3f(-1.0, -0.2, 1.0)

        glColor3f(1.0, 0.0, 1.0)  # Right
        glVertex3f(1.0, 0.2, -1.0)
        glVertex3f(1.0, 0.2, 1.0)
        glVertex3f(1.0, -0.2, 1.0)
        glVertex3f(1.0, -0.2, -1.0)
        glEnd()

    def run(self):
        pygame.init()
        screen = pygame.display.set_mode((640, 480), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("ESP32 Cube Visualizer")
        clock = pygame.time.Clock()
        self.init_gl()

        while self.running:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False

            # Read from queue if available
            try:
                gx, gy, gz = self.data_queue.get_nowait()
                self.ax, self.ay, self.az = gx, gy, gz
            except queue.Empty:
                pass  # No new data

            self.draw_cube()
            pygame.display.flip()
            clock.tick(60)

        pygame.quit()

    def stop(self):
        self.running = False
