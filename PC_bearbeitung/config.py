"""
Konfigurationsmodul für das 3D-Inspektionsprojekt.

Dieses Modul zentralisiert alle statischen Pfade und Suchmuster.
Es verwendet den Standort dieser Datei (__file__), um alle anderen
Pfade relativ zu finden.
"""

from pathlib import Path

# --- Sektion 1: Projekt-Struktur ---

# Holt den Pfad der aktuellen Datei (config.py)
# z.B. C:\Users\Lukas\Programme\KI_Segmentierung\PC_bearbeitung\config.py
CONFIG_FILE_PATH = Path(__file__)

# Holt den Ordner, in dem diese Datei liegt
# z.B. C:\Users\Lukas\Programme\KI_Segmentierung\PC_bearbeitung
CONFIG_DIR = CONFIG_FILE_PATH.parent

# Geht EINE Ebene nach oben, um zum Projektstamm zu gelangen
# z.B. C:\Users\Lukas\Programme\KI_Segmentierung
PROJECT_ROOT = CONFIG_DIR.parent


# --- Sektion 2: Pfade zu den Daten ---

# Definiert den Scan-Ordner basierend auf dem Projektstamm
# z.B. C:\Users\Lukas\Programme\KI_Segmentierung\scans
SCAN_DATA_FOLDER = PROJECT_ROOT / "scans"

# Definiert den CAD-Ordner (wir nehmen an, er liegt auch im Stamm)
# z.B. C:\Users\Lukas\Programme\KI_Segmentierung\cad
CAD_DATA_FOLDER = PROJECT_ROOT / "cad"


# --- Sektion 3: Suchmuster ---
SCAN_FILE_PATTERN = "*.ply"
DEFAULT_CAD_FILENAME = "kopfstueck.ply"