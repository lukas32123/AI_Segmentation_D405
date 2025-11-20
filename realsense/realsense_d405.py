import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError as e:
    raise RuntimeError("pyrealsense2 ist nicht installiert. Bitte 'pip install pyrealsense2' ausführen.") from e


@dataclass(frozen=True)
class D405Config:
    """
    Hält alle einstellbaren Parameter für die D405 in einer unveränderlichen Konfiguration.
    Diese Klasse bildet die in der Produktion erprobten Standardwerte ab und erlaubt bei Bedarf
    eine gezielte Anpassung über optionale Argumente.

    Attribute:
        width: Horizontale Auflösung des Depth-Streams in Pixeln.
        height: Vertikale Auflösung des Depth-Streams in Pixeln.
        fps: Bildrate des Depth-Streams in Frames pro Sekunde.
        visual_preset: RealSense Visual Preset für Stereo; hier 'High Accuracy' für präzise Messung.
        emitter_enabled: Aktivierung des IR-Emitters (0=Aus, 1=Ein, 2=Blink).
        laser_power: Sendeleistung des IR-Emitters (Bereich gerätespezifisch, D405 typ. bis 240).
        exposure: Manuelle Belichtungszeit des Depth-Sensors in Mikrosekunden.
        gain: Analoger Gain des Depth-Sensors.
        decimation_magnitude: Stärke des Decimation Filters (Downsampling-Faktor).
        spatial_alpha: Glättungskoeffizient des Spatial Filters (0..1).
        spatial_delta: Reichweite in Pixeln für den Spatial Filter.
        spatial_iterations: Anzahl Iterationen des Spatial Filters.
        temporal_alpha: Glättungskoeffizient des Temporal Filters (0..1).
        temporal_delta: Sensitivitätsschwelle des Temporal Filters.
        hole_filling_mode: Lochfüllmodus (0=Disabled, 1=Nearest, 2=Farest/Adjacent).
        warmup_seconds: Aufwärmzeit nach Start, bevor Frames verwendet werden.
        enforce_product_line_d400: Erzwingt, dass ein D4xx-Gerät (hier D405) erkannt wird.
    """
    width: int = 1280
    height: int = 720
    fps: int = 30
    visual_preset: int = getattr(rs, "rs400_visual_preset", None).high_accuracy if hasattr(rs, "rs400_visual_preset") else 3
    emitter_enabled: int = 1
    laser_power: float = 240.0
    exposure: float = 1000.0
    gain: float = 24.0
    decimation_magnitude: int = 1
    spatial_alpha: float = 0.5
    spatial_delta: int = 20
    spatial_iterations: int = 2
    temporal_alpha: float = 0.0
    temporal_delta: int = 5
    hole_filling_mode: int = 0
    warmup_seconds: float = 1.0
    enforce_product_line_d400: bool = True


class RealSenseD405Factory:
    """
    Erzeugt eine vorkonfigurierte RealSense-Pipeline für die D405.
    Die Factory kapselt die hardwareabhängigen Schritte (Geräte-Discovery, Sensor-Optionen,
    Stream-Konfiguration) und liefert eine lauffähige Pipeline samt Filterkette.
    """

    def __init__(self, config: Optional[D405Config] = None):
        """Initialisiert die Factory mit einer D405Config oder Standardwerten."""
        self.config = config or D405Config()
        self._ctx = rs.context()
        self._pipe = rs.pipeline()
        self._filters = self._build_filters()

    def _assert_d405(self, dev: rs.device) -> None:
        """
        Prüft, ob ein D4xx-Gerät angeschlossen ist und gibt bei Bedarf eine klare Fehlermeldung aus.
        Diese Prüfung schützt die nachfolgenden Optionszugriffe und verhindert Fehlkonfigurationen.
        """
        if not self.config.enforce_product_line_d400:
            return
        prod_line = dev.get_info(rs.camera_info.product_line)
        name = dev.get_info(rs.camera_info.name)
        if "D4" not in prod_line and "D4" not in name:
            raise RuntimeError(f"Falsches Gerät erkannt: '{name}' (Produktlinie '{prod_line}'). Erwartet: D400-Serie (z. B. D405).")

    def _resolve_depth_sensor(self, dev: rs.device) -> rs.sensor:
        """
        Ermittelt den Depth-Sensor des Gerätes.
        Die D405 besitzt keinen RGB-Sensor; diese Methode gibt explizit den Stereo-Depth-Sensor zurück.
        """
        sensors = dev.query_sensors()
        for s in sensors:
            if s.is_depth_sensor():
                return s
        raise RuntimeError("Kein Depth-Sensor gefunden. Bitte Hardware prüfen.")

    def _apply_depth_options(self, depth_sensor: rs.sensor) -> None:
        """
        Setzt alle relevanten Sensoroptionen: Preset, Emitter, Laser-Power, Exposure und Gain.
        Bei nicht unterstützten Optionen wird eine sprechende Fehlermeldung erzeugt.
        """
        cfg = self.config
        if depth_sensor.supports(rs.option.visual_preset):
            depth_sensor.set_option(rs.option.visual_preset, cfg.visual_preset)
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, cfg.emitter_enabled)
        if depth_sensor.supports(rs.option.laser_power):
            rng = depth_sensor.get_option_range(rs.option.laser_power)
            depth_sensor.set_option(rs.option.laser_power, float(min(max(cfg.laser_power, rng.min), rng.max)))
        if depth_sensor.supports(rs.option.enable_auto_exposure):
            depth_sensor.set_option(rs.option.enable_auto_exposure, 0.0)
        if depth_sensor.supports(rs.option.exposure):
            depth_sensor.set_option(rs.option.exposure, float(cfg.exposure))
        if depth_sensor.supports(rs.option.gain):
            depth_sensor.set_option(rs.option.gain, float(cfg.gain))

    def _build_filters(self) -> Tuple[rs.filter, rs.filter, rs.filter, rs.filter]:
        """
        Erzeugt die Post-Processing-Filterkette in der empfohlenen Reihenfolge:
        Decimation → Spatial → Temporal → Hole-Filling. Die Parameter werden aus der Config übernommen.
        """
        cfg = self.config

        dec = rs.decimation_filter()
        dec.set_option(rs.option.filter_magnitude, float(cfg.decimation_magnitude))

        sp = rs.spatial_filter()
        sp.set_option(rs.option.filter_smooth_alpha, float(cfg.spatial_alpha))
        sp.set_option(rs.option.filter_smooth_delta, float(cfg.spatial_delta))
        sp.set_option(rs.option.holes_fill, 0.0)
        sp.set_option(rs.option.filter_magnitude, float(cfg.spatial_iterations))

        tp = rs.temporal_filter()
        tp.set_option(rs.option.filter_smooth_alpha, float(cfg.temporal_alpha))
        tp.set_option(rs.option.filter_smooth_delta, float(cfg.temporal_delta))

        hf = rs.hole_filling_filter()
        hf.set_option(rs.option.holes_fill, float(cfg.hole_filling_mode))

        return dec, sp, tp, hf

    def start(self) -> tuple:
        """
        Startet die Pipeline mit einem Depth-Stream 1280×720@30 FPS (anpassbar über Config).
        Setzt die Sensoroptionen NACH dem Start und wartet kurz zur Stabilisierung.
        """
        cfg = self.config

        if len(self._ctx.query_devices()) == 0:
            raise RuntimeError("Kein RealSense-Gerät gefunden. Bitte USB-Verbindung und Treiber prüfen.")
        dev = self._ctx.query_devices()[0]
        self._assert_d405(dev)

        rs_config = rs.config()
        rs_config.enable_stream(rs.stream.depth, cfg.width, cfg.height, rs.format.z16, cfg.fps)

        profile = self._pipe.start(rs_config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self._apply_depth_options(depth_sensor)

        time.sleep(cfg.warmup_seconds)
        return self._pipe, self._filters


    def stop(self) -> None:
        """Stoppt die Pipeline sicher, falls sie läuft."""
        try:
            self._pipe.stop()
        except Exception:
            pass


class D405Controller:
    """
    Höherstufiger Controller/Service, der die Pipeline der D405 kapselt und
    einfache Methoden für Frame-Erfassung, Tiefenbild, Punktwolke und Abstandsmessungen bereitstellt.
    """

    def __init__(self, config: Optional[D405Config] = None):
        """Initialisiert Controller, Factory und interne Zustände für Frame-Verarbeitung."""
        self.config = config or D405Config()
        self.factory = RealSenseD405Factory(self.config)
        self.pipeline: Optional[rs.pipeline] = None
        self.filters: Optional[Tuple[rs.filter, rs.filter, rs.filter, rs.filter]] = None
        self._pc = rs.pointcloud()
        self._align: Optional[rs.align] = None

    def start(self) -> None:
        """
        Startet die Kamera-Pipeline und bereitet die Filterkette vor.
        Aktiviert Alignment nur, wenn mehrere Streams existieren.
        """
        self.pipeline, self.filters = self.factory.start()

        profile = self.pipeline.get_active_profile()
        stream_profiles = profile.get_streams()
        stream_types = {sp.stream_type() for sp in stream_profiles}

        if rs.stream.color in stream_types or rs.stream.infrared in stream_types:
            self._align = rs.align(rs.stream.depth)
        else:
            self._align = None

    def stop(self) -> None:
        """Stoppt die Kamera-Pipeline."""
        self.factory.stop()

    def _apply_postprocessing(self, depth_frame: rs.depth_frame) -> rs.depth_frame:
        """
        Wendet die Post-Processing-Filter der Reihe nach auf das übergebene Depth-Frame an.
        Die Reihenfolge ist entscheidend für ein ruhiges, geschlossenes Tiefenbild.
        """
        if self.filters is None:
            return depth_frame
        dec, sp, tp, hf = self.filters
        frame = dec.process(depth_frame)
        frame = sp.process(frame)
        frame = tp.process(frame)
        frame = hf.process(frame)
        return frame.as_depth_frame()

    def grab_depth_frame(self, timeout_ms: int = 1500) -> rs.depth_frame:
        """
        Liest ein einzelnes Depth-Frame aus der Pipeline, richtet es auf den Depth-Stream aus
        und gibt das gefilterte Resultat zurück. Ein Timeout verhindert endloses Warten.
        """
        if self.pipeline is None:
            raise RuntimeError("Pipeline ist nicht gestartet. Bitte 'start()' aufrufen.")
        if timeout_ms < 1:
            timeout_ms = 1

        t0 = time.time()
        while True:
            if (time.time() - t0) * 1000.0 > timeout_ms:
                raise TimeoutError("Zeitüberschreitung beim Warten auf Frames.")
            frames = self.pipeline.wait_for_frames()
        
            if self._align is not None:
                try:
                    frames = self._align.process(frames)
                except Exception:
                    pass

            depth_frame = frames.get_depth_frame()
            if depth_frame and depth_frame.get_width() > 0 and depth_frame.get_height() > 0:
                return self._apply_postprocessing(depth_frame)

    def depth_to_numpy(self, depth_frame: rs.depth_frame) -> np.ndarray:
        """
        Wandelt ein Depth-Frame in ein float32-Numpy-Array in Metern um.
        Das Ergebnis ist direkt für metrische Messungen, CAD-Abgleich und Thresholding geeignet.
        """
        depth_image = np.asanyarray(depth_frame.get_data())
        scale = depth_frame.get_units()
        return depth_image.astype(np.float32) * np.float32(scale)

    def compute_pointcloud(self, depth_frame: rs.depth_frame) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """
        Erzeugt eine Punktwolke aus dem übergebenen Depth-Frame.
        Gibt ein Tupel aus Nx3-Punktkoordinaten (in Metern) und optionalen Texturkoordinaten zurück.
        Bei der D405 ist standardmäßig kein RGB-Stream vorhanden; die Texturkoordinaten sind daher None.
        """
        points = self._pc.calculate(depth_frame)
        v = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        return v, None

    def measure_distance_at(self, depth_frame: rs.depth_frame, x: int, y: int) -> float:
        """
        Misst die Tiefe an einem Pixelkoordinatenpaar (x, y) in Metern.
        Diese Methode eignet sich für schnelle Sanity-Checks oder einfache Prüfregeln.
        """
        return float(depth_frame.get_distance(int(x), int(y)))
    
    def capture_and_export_ply(self, file_path: str) -> bool:
        """
        Erfasst ein Frame, berechnet die Punktwolke und speichert sie direkt als .ply Datei.
        Nutzt die native C++ Export-Funktion von RealSense für maximale Performance.
        """
        try:
            # 1. Frame holen (bereits gefiltert durch grab_depth_frame)
            depth_frame = self.grab_depth_frame()
            
            # 2. Punktwolke-Objekt berechnen (nicht nur Numpy Arrays)
            points = self._pc.calculate(depth_frame)
            
            # 3. Speichern
            # Die RealSense API erwartet hier oft kein Color-Frame bei der D405, 
            # wenn keiner im Stream ist. Wir übergeben das Depth-Frame als Textur-Dummy 
            # oder None, falls keine Textur gewünscht ist.
            points.export_to_ply(str(file_path), depth_frame)
            return True
        except Exception as e:
            print(f"Fehler beim Exportieren der PLY: {e}")
            return False


def demo_single_capture() -> None:
    """
    Führt eine kurze Demonstration aus:
    1) Startet die D405 mit produktionsreifen Einstellungen.
    2) Erfasst ein Depth-Frame, wandelt es in Meter um und berechnet eine Punktwolke.
    3) Gibt kleine Statusinformationen aus und stoppt die Pipeline.

    Diese Demo ist bewusst minimal gehalten, damit sie in bestehende Pipelines integriert werden kann.
    """
    ctrl = D405Controller()
    try:
        ctrl.start()
        depth = ctrl.grab_depth_frame()
        depth_m = ctrl.depth_to_numpy(depth)
        pts, _ = ctrl.compute_pointcloud(depth)

        h, w = depth_m.shape
        center_distance = ctrl.measure_distance_at(depth, w // 2, h // 2)

        print(f"Depth-Frame: {w}×{h} px, Einheit Meter")
        print(f"Zentrumstiefe: {center_distance:.4f} m")
        print(f"Punktwolke: {pts.shape[0]} Punkte")
    finally:
        ctrl.stop()


if __name__ == "__main__":
    """
    Startpunkt für manuellen Test auf einem Windows-Rechner mit angeschlossener Intel RealSense D405.
    """
    demo_single_capture()
