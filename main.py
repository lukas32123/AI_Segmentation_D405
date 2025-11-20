import sys
import time
from pathlib import Path

# Importiere unsere Module
from realsense import D405Controller
from scripts.viewer import D405Viewer
from turntable import TurntableController  # <--- NEU

class MainApp:
    """
    Hauptanwendung für Live-View und automatisierten 360°-Scan.
    """

    def run_360_scan(self) -> None:
        """
        Führt einen automatisierten Rundum-Scan durch:
        1. Kamera & Drehteller verbinden
        2. Schleife: Scannen -> Speichern -> Drehen
        """
        # --- KONFIGURATION ---
        step_size = 45  # Schrittweite in Grad
        total_degrees = 360
        
        # Zielordner (Manuell definiert)
        target_directory = Path(r"C:\Users\Lukas\Programme\KI_Segmentierung\scans")
        target_directory.mkdir(parents=True, exist_ok=True)
        
        # Ordner für diesen spezifischen Durchlauf erstellen (Zeitstempel)
        # Damit die 8 Bilder in einem Unterordner landen
        session_name = time.strftime("Session_%Y%m%d_%H%M%S")
        session_dir = target_directory / session_name
        session_dir.mkdir(exist_ok=True)

        # Controller initialisieren
        cam = D405Controller()
        table = TurntableController()

        print(f"\n=== STARTE 360° SCAN (Session: {session_name}) ===")
        
        try:
            # 1. Hardware starten
            cam.start()
            table.connect()
            
            # Kurzes Warten für Kamera-Belichtung
            print("Warte auf Kamera-Initialisierung (2s)...")
            time.sleep(2.0)

            current_angle = 0
            scan_count = 0

            # 2. Die Schleife
            while current_angle < total_degrees:
                print(f"\n--- Schritt {scan_count + 1}: Position {current_angle}° ---")

                # A. Bild aufnehmen
                filename = f"scan_{current_angle:03d}_deg.ply"
                full_path = str(session_dir / filename)
                
                # Warten, bis Wackeln vom Drehen aufgehört hat
                time.sleep(1.0) 
                
                print(f"Erfasse Punktwolke...")
                success = cam.capture_and_export_ply(full_path)
                
                if success:
                    print(f"Gespeichert: {filename}")
                else:
                    print(f"FEHLER beim Speichern von {filename}")

                # B. Weiterdrehen (außer beim letzten Mal, wenn 360 fast erreicht sind)
                # Wir drehen NACH dem Scan für den nächsten Schritt
                if (current_angle + step_size) < total_degrees:
                    table.rotate(step_size)
                
                current_angle += step_size
                scan_count += 1

            print("\n=== 360° SCAN ABGESCHLOSSEN ===")
            print(f"Dateien liegen in: {session_dir}")

        except Exception as e:
            print(f"KRITISCHER FEHLER: {e}")
        finally:
            cam.stop()
            table.disconnect()

    def run_live(self) -> None:
        """Standard-Modus: Einfach nur gucken."""
        print("Starte Live-Viewer...")
        D405Viewer().run()


def main() -> None:
    """
    Logik:
    - Kein Argument -> Live Viewer (Standard)
    - 'scan' -> Automatischer 360° Ablauf
    """
    mode = "live"
    
    # Prüfen ob Argumente da sind
    if len(sys.argv) > 1:
        arg = sys.argv[1].strip().lower()
        if arg in ["scan", "auto", "360"]:
            mode = "scan"
    
    app = MainApp()
    
    if mode == "scan":
        app.run_360_scan()
    else:
        # Standard-Fall
        app.run_live()

if __name__ == "__main__":
    main()