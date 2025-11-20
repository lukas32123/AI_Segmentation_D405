"""
Modul für die interaktive 3D-Visualisierung.

Definiert die Klasse InspectionVisualizer, die als "View" (Ansicht)
in einem MVC-Muster dient. Sie nimmt bereits verarbeitete Daten
entgegen und stellt sie dar.
"""

import open3d as o3d
import logging
from typing import List

class InspectionVisualizer:
    """
    Öffnet ein interaktives Open3D-Fenster zur Anzeige der Inspektionsergebnisse.
    
    Features:
    - Zeigt die final eingefärbten (Rot/Grün) Punktwolken an.
    - Ermöglicht das Ein- und Ausblenden des CAD-Modells per Tastendruck.
    """

    def __init__(self):
        """
        Initialisiert den Visualizer und setzt den Status für die CAD-Sichtbarkeit.
        """
        self.cad_model = None
        self.show_cad = True  # Statusvariable, um die Sichtbarkeit zu verfolgen
        logging.info("InspectionVisualizer initialisiert.")

    def _toggle_cad_visibility(self, vis_control) -> bool:
        """
        Private Callback-Funktion, die bei Tastendruck aufgerufen wird.
        
        Entfernt das CAD-Modell aus der Szene oder fügt es hinzu.
        """
        if self.cad_model is None:
            logging.warning("Kein CAD-Modell zum Umschalten vorhanden.")
            return False

        self.show_cad = not self.show_cad  # Status umkehren
        
        if self.show_cad:
            logging.debug("Füge CAD-Modell zur Szene hinzu.")
            vis_control.add_geometry(self.cad_model, reset_bounding_box=False)
        else:
            logging.debug("Entferne CAD-Modell aus der Szene.")
            vis_control.remove_geometry(self.cad_model, reset_bounding_box=False)
            
        logging.info(f"CAD-Modell Sichtbarkeit: {'AN' if self.show_cad else 'AUS'}")
        
        # Wichtig: Sagen Sie dem Renderer, dass er die Szene aktualisieren soll
        vis_control.update_renderer()
        return True

    def display_results(self, 
                        processed_pcds: List[o3d.geometry.PointCloud], 
                        cad_model: o3d.geometry.TriangleMesh):
        """
        Öffnet das blockierende Visualisierungsfenster und zeigt alle Ergebnisse an.

        Args:
            processed_pcds (List[o3d.geometry.PointCloud]): 
                Eine Liste von Punktwolken, die BEREITS durch
                'analyze_deviation' (Rot/Grün) eingefärbt wurden.
                
            cad_model (o3d.geometry.TriangleMesh): 
                Das Referenz-CAD-Modell, das umgeschaltet werden soll.
        """
        if not processed_pcds:
            logging.warning("Keine verarbeiteten Punktwolken zur Visualisierung übergeben.")
            return

        # Speichere das CAD-Modell für die Callback-Funktion
        # Wir färben es grau ein, damit es sich von der Heatmap abhebt
        self.cad_model = cad_model
        self.cad_model.paint_uniform_color([0.7, 0.7, 0.7]) # Hellgrau

        logging.info("Starte interaktives Visualisierungsfenster...")
        logging.info("--- STEUERUNG ---")
        logging.info(" [V] : CAD-Modell Sichtbarkeit umschalten")
        logging.info(" [Q] : Fenster schließen")
        logging.info("-----------------")

        # Initialisiere den Visualizer mit Key-Callbacks
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="3D-Inspektionsergebnis")

        # Füge alle verarbeiteten (eingefärbten) Punktwolken hinzu
        for pcd in processed_pcds:
            vis.add_geometry(pcd)
        
        # Füge das CAD-Modell initial hinzu (show_cad ist True)
        vis.add_geometry(self.cad_model)

        # Registriere die Tasten-Callbacks
        # ASCII 86 = 'V', ASCII 81 = 'Q'
        vis.register_key_callback(86, self._toggle_cad_visibility) # 'V'
        
        # (Wir brauchen 'Q' nicht, das Schließen des Fensters funktioniert auch so)

        # Starte die Visualisierung (dies blockiert das Skript, bis das Fenster geschlossen wird)
        vis.run()
        vis.destroy_window()
        logging.info("Visualisierungsfenster geschlossen.")