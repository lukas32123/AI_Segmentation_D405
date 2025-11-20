"""
Modul für das Laden von Punktwolken- und CAD-Daten.

Enthält die DataProvider-Klasse, die als "Creator" für
open3d-Objekte (PointCloud und TriangleMesh) dient.
"""

import open3d as o3d
from pathlib import Path
import logging
from typing import List, Optional

# Stellen Sie sicher, dass all diese Variablen in Ihrer config.py existieren!
from config import (
    SCAN_DATA_FOLDER, 
    SCAN_FILE_PATTERN, 
    CAD_DATA_FOLDER, 
    DEFAULT_CAD_FILENAME
)

class DataProvider:
    """
    Eine Klasse, die für das Suchen und Laden von 3D-Daten verantwortlich ist.
    
    Agier als "Creator" (Larman-Pattern) für 3D-Geometrieobjekte.
    """

    def __init__(self, 
                 scan_path: Path = SCAN_DATA_FOLDER, 
                 scan_pattern: str = SCAN_FILE_PATTERN,
                 cad_path: Path = CAD_DATA_FOLDER):
        """
        Initialisiert den DataProvider.
        """
        self.scan_path = scan_path
        self.scan_pattern = scan_pattern
        self.cad_path = cad_path
        logging.info("DataProvider initialisiert.")
        logging.info(f" -> Sucht nach Scans: {self.scan_path / self.scan_pattern}")
        logging.info(f" -> Sucht nach CAD: {self.cad_path}")

    def get_scan_file_paths(self) -> List[Path]:
        """
        Sucht im Scan-Ordner nach allen Dateien, die dem Suchmuster entsprechen.
        """
        logging.info("Suche nach Scans...")
        scan_paths = list(self.scan_path.glob(self.scan_pattern))
        
        if not scan_paths:
            logging.warning(f"Keine Scans mit Muster '{self.scan_pattern}' in {self.scan_path} gefunden.")
        else:
            logging.info(f"{len(scan_paths)} Scans gefunden.")
            
        return scan_paths

    def load_point_cloud(self, file_path: Path) -> Optional[o3d.geometry.PointCloud]:
        """
        Lädt eine einzelne Punktwolken-Datei von einem gegebenen Pfad.
        """
        logging.info(f"Lade Punktwolke: {file_path.name}")
        
        if not file_path.exists():
            logging.error(f"Datei existiert nicht: {file_path}")
            return None

        try:
            pcd = o3d.io.read_point_cloud(str(file_path))
            if not pcd.has_points():
                logging.warning(f"Datei geladen, aber keine Punkte gefunden in: {file_path.name}")
                return None
            logging.info(f"Erfolgreich geladen: {len(pcd.points)} Punkte.")
            return pcd
        except Exception as e:
            logging.error(f"Unerwarteter Fehler beim Laden von {file_path.name}: {e}")
            return None

    def load_cad_model(self, filename: str = DEFAULT_CAD_FILENAME) -> Optional[o3d.geometry.TriangleMesh]:
        """
        Lädt das CAD-Referenzmodell.
        
        Wandelt .stp oder .iges automatisch in ein TriangleMesh um.
        
        Args:
            filename (str): Der Dateiname des CAD-Modells.

        Returns:
            o3d.geometry.TriangleMesh: Das geladene Open3D-Mesh-Objekt.
            None: Gibt None bei einem Fehler zurück.
        """
        full_path = self.cad_path / filename
        logging.info(f"Lade CAD-Modell: {full_path.name}")

        if not full_path.exists():
            logging.error(f"CAD-Datei nicht gefunden: {full_path}")
            return None
        
        try:
            # Open3D kann .stp / .iges direkt lesen
            mesh = o3d.io.read_triangle_mesh(str(full_path))
            
            if mesh.is_empty():
                logging.error(f"CAD-Datei geladen, aber Mesh ist leer: {full_path.name}")
                return None

            logging.info("CAD-Modell erfolgreich geladen.")
            return mesh
        except Exception as e:
            logging.error(f"Unerwarteter Fehler beim Laden von {full_path.name}: {e}")
            return None