"""
Stellt einen Live-Viewer für die Intel RealSense D405 bereit, der auf dem bestehenden
Produktiv-Controller (D405Controller) aufsetzt. Zeigt ein farbcodiertes Tiefenbild,
die Mittelpunktdistanz sowie die aktuelle FPS an. Beenden mit ESC oder 'q', Screenshot
mit 's'. Dieses Modul trennt klar die Zuständigkeiten gemäß Larmann-Pattern:
Factory/Controller liefern Daten, diese View-Klasse visualisiert sie.
"""

import time
from typing import Optional

import cv2
import numpy as np
from realsense import D405Controller, D405Config


class D405Viewer:
    """
    Visualisiert kontinuierlich den Depth-Stream der D405 in einem OpenCV-Fenster.
    """

    def __init__(self, config: Optional[D405Config] = None, window_name: str = "D405 Depth"):
        """
        Initialisiert Controller, Fenstername und internen Status.
        """
        self.config = config or D405Config()
        self.ctrl = D405Controller(self.config)
        self.window_name = window_name
        self._t_last = None
        self._fps = 0.0

    def _update_fps(self) -> None:
        """
        Aktualisiert die FPS-Schätzung anhand der verstrichenen Zeit seit dem letzten Frame.
        """
        t_now = time.time()
        if self._t_last is not None:
            dt = t_now - self._t_last
            if dt > 0:
                self._fps = 1.0 / dt
        self._t_last = t_now

    def _depth_to_vis(self, depth_frame) -> np.ndarray:
        """
        Wandelt ein Depth-Frame in eine visuelle Darstellung mit Farbcodierung.
        Die Skalierung ist heuristisch und für die D405 im Nahbereich brauchbar.
        """
        depth = np.asanyarray(depth_frame.get_data())
        depth_u = depth_frame.get_units()
        depth_m = depth.astype(np.float32) * np.float32(depth_u)

        vmax = 0.5  # 0.5 m als typische Obergrenze im D405-Nahbereich; bei Bedarf anpassen
        vis_8u = np.clip((depth_m / max(vmax, 1e-6)) * 255.0, 0, 255).astype(np.uint8)
        vis_color = cv2.applyColorMap(vis_8u, cv2.COLORMAP_JET)
        return vis_color

    def run(self) -> None:
        """
        Startet die Pipeline und zeigt kontinuierlich Frames an, bis der Nutzer das Fenster
        mit ESC oder 'q' beendet. Ein Druck auf 's' speichert einen Screenshot.
        """
        self.ctrl.start()
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            while True:
                depth_frame = self.ctrl.grab_depth_frame()
                self._update_fps()

                h = depth_frame.get_height()
                w = depth_frame.get_width()
                center_dist = self.ctrl.measure_distance_at(depth_frame, w // 2, h // 2)

                vis = self._depth_to_vis(depth_frame)
                overlay = vis.copy()
                cv2.putText(
                    overlay,
                    f"Center: {center_dist:.3f} m  |  FPS: {self._fps:4.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow(self.window_name, overlay)

                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
                if key in (ord("s"),):
                    ts = time.strftime("%Y%m%d-%H%M%S")
                    cv2.imwrite(f"depth_{ts}.png", overlay)

        finally:
            cv2.destroyAllWindows()
            self.ctrl.stop()


def main() -> None:
    """
    Startpunkt für den Standalone-Betrieb des Viewers.
    """
    D405Viewer().run()


if __name__ == "__main__":
    main()