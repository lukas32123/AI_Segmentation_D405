"""
Haupt-Controller-Skript.
Inklusive KI-Segmentierung.
"""
import open3d as o3d
import logging
from data_loader import DataProvider
from processing import PointCloudProcessor
from visualizer import InspectionVisualizer

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

ANALYSIS_TOLERANCE_MM = 0.6 

def main_workflow():
    logging.info("Starte Workflow mit KI-Segmentierung...")

    provider = DataProvider()
    # Hier wird jetzt das KI-Modell im Hintergrund geladen
    processor = PointCloudProcessor(scale_factor=1000.0) 

    cad_model = provider.load_cad_model()
    if cad_model is None: return

    scan_file_paths = provider.get_scan_file_paths()
    if not scan_file_paths: return
    
    final_analyzed_scans = []
    
    for path in scan_file_paths:
        logging.info(f"--- Verarbeite: {path.name} ---")

        raw_pcd = provider.load_point_cloud(path)
        if raw_pcd is None: continue

        scaled_pcd = processor.scale_point_cloud(raw_pcd)

        # 1. AUSRICHTEN (Align)
        # Wir richten ALLES (inkl. Hintergrund) am CAD aus, damit die KI
        # das Bauteil an der erwarteten Stelle findet.
        aligned_pcd = processor.align_to_cad(scaled_pcd, cad_model)
        
        # 2. KI SEGMENTIERUNG
        # Die KI entscheidet jetzt, was Müll ist und was Bauteil
        segmented_pcd = processor.segment_by_ai(aligned_pcd)
        
        if segmented_pcd is None:
            logging.warning(f"KI konnte kein Bauteil in {path.name} finden.")
            continue
            
        # 3. ANALYSE
        analyzed_pcd = processor.analyze_deviation(
            segmented_pcd,
            cad_model,
            analysis_tolerance=ANALYSIS_TOLERANCE_MM
        )
        
        final_analyzed_scans.append(analyzed_pcd)

    if final_analyzed_scans:
        visualizer = InspectionVisualizer()
        visualizer.display_results(final_analyzed_scans, cad_model)
    else:
        logging.info("Keine Ergebnisse.")

if __name__ == "__main__":
    main_workflow()