"""
Modul für die Verarbeitung von Punktwolken.
Inklusive KI-basierter Segmentierung und CAD-Ausrichtung.
"""

import open3d as o3d
import numpy as np
import logging
import torch
from typing import Optional, Tuple

# Importiere unser KI-Modell
from segmentation_model import PointNetSegmentation

class PointCloudProcessor:
    
    # Tuning für die Ausrichtung
    VOXEL_SIZE_GLOBAL_REGISTRATION = 1.0
    
    # Pfad zum trainierten Modell (Sollte idealerweise in config.py stehen)
    MODEL_PATH = "best_segmentation_model.pth"

    def __init__(self, scale_factor: float = 1000.0):
        self.scale_factor = scale_factor
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        logging.info(f"PointCloudProcessor initialisiert. Nutze Device: {self.device}")
        
        # --- KI MODELL LADEN ---
        self.ai_model = PointNetSegmentation(num_classes=2).to(self.device)
        self._load_ai_weights()

    def _load_ai_weights(self):
        """Versucht, trainierte Gewichte zu laden."""
        try:
            self.ai_model.load_state_dict(torch.load(self.MODEL_PATH, map_location=self.device))
            self.ai_model.eval() # Test-Modus (kein Dropout etc.)
            logging.info(f"KI-Modell erfolgreich geladen von {self.MODEL_PATH}")
        except FileNotFoundError:
            logging.warning(f"ACHTUNG: Kein trainiertes Modell unter '{self.MODEL_PATH}' gefunden!")
            logging.warning("Die Segmentierung wird zufällige Ergebnisse liefern, bis trainiert wurde.")

    def scale_point_cloud(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        logging.info(f"Skaliere Punktwolke um Faktor {self.scale_factor}...")
        pcd.scale(self.scale_factor, center=(0, 0, 0))
        return pcd

    def segment_by_ai(self, pcd: o3d.geometry.PointCloud) -> Optional[o3d.geometry.PointCloud]:
        """
        NEU: Segmentiert die Punktwolke mittels PointNet KI.
        Trennt Bauteil (Klasse 1) von Hintergrund (Klasse 0).
        """
        logging.info("Starte KI-Segmentierung (PointNet)...")
        
        if not pcd.has_points():
            return None

        # 1. Daten vorbereiten (Open3D -> PyTorch Tensor)
        # PointNet erwartet Shape [Batch, 3, N]
        points = np.asarray(pcd.points) # Shape (N, 3)
        
        # Transponieren zu (3, N) und Batch Dimension hinzufügen -> (1, 3, N)
        tensor_input = torch.from_numpy(points).float().permute(1, 0).unsqueeze(0)
        tensor_input = tensor_input.to(self.device)

        # 2. Inferenz (Das Modell befragen)
        with torch.no_grad():
            # Output Shape: [1, 2, N] (Log-Wahrscheinlichkeiten)
            pred_log_probs = self.ai_model(tensor_input)
            
            # Die Klasse mit der höheren Wahrscheinlichkeit wählen
            # Shape: [1, N]
            pred_labels = pred_log_probs.max(dim=1)[1] 
            
            # Zurück zu Numpy Array (flach)
            labels_np = pred_labels.cpu().numpy().flatten()

        # 3. Filtern
        # Wir suchen alle Indizes, wo das Label "1" (Bauteil) ist
        target_indices = np.where(labels_np == 1)[0]
        
        if len(target_indices) == 0:
            logging.warning("KI hat keine Bauteil-Punkte gefunden (alles als Hintergrund klassifiziert).")
            # Fallback für Testzwecke: Wenn Modell untrainiert ist, gib alles zurück
            # return pcd 
            return None

        logging.info(f"KI hat {len(target_indices)} Punkte als Bauteil identifiziert.")
        
        # Erstelle neue Wolke nur mit Bauteil-Punkten
        segmented_pcd = pcd.select_by_index(target_indices)
        
        return segmented_pcd

    # --- HIER DRUNTER BLEIBT ALLES GLEICH (FÜR DIE AUSRICHTUNG & ANALYSE) ---
    
    def _preprocess_and_compute_features(self, pcd, voxel_size):
        pcd_down = pcd.voxel_down_sample(voxel_size)
        radius_normal = voxel_size * 2
        pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
        radius_feature = voxel_size * 5
        pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
        return pcd_down, pcd_fpfh

    def _execute_global_registration(self, source_down, target_down, source_features, target_features, voxel_size):
        distance_threshold = voxel_size * 1.5
        result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            source_down, target_down, source_features, target_features, True, distance_threshold,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3, 
            [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9), o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
        return result

    def align_to_cad(self, scan_to_align, cad_model):
        voxel_size = self.VOXEL_SIZE_GLOBAL_REGISTRATION
        scan_down, scan_features = self._preprocess_and_compute_features(scan_to_align, voxel_size)
        cad_pcd_sample = cad_model.sample_points_poisson_disk(100000)
        cad_down, cad_features = self._preprocess_and_compute_features(cad_pcd_sample, voxel_size)
        
        global_result = self._execute_global_registration(scan_down, cad_down, scan_features, cad_features, voxel_size)
        logging.info(f"Globale Registrierung Fitness: {global_result.fitness:.4f}")
        
        if global_result.fitness < 0.01:
            logging.warning("Grobe Ausrichtung fehlgeschlagen.")
            scan_to_align.transform(global_result.transformation)
            return scan_to_align

        scan_to_align.transform(global_result.transformation)
        search_distance_icp = 3.0 
        icp_result = o3d.pipelines.registration.registration_icp(scan_to_align, cad_pcd_sample, search_distance_icp, o3d.pipelines.registration.TransformationEstimationPointToPoint())
        scan_to_align.transform(icp_result.transformation)
        return scan_to_align

    def analyze_deviation(self, segmented_pcd, cad_model, analysis_tolerance):
        logging.info(f"Starte Analyse (Toleranz: {analysis_tolerance}mm)...")
        cad_pcd_sample = cad_model.sample_points_poisson_disk(100000)
        distances = segmented_pcd.compute_point_cloud_distance(cad_pcd_sample)
        distances_np = np.asarray(distances)
        
        max_dist = analysis_tolerance
        normalized_distances = np.clip(distances_np / max_dist, 0.0, 1.0)
        colors = np.zeros((len(normalized_distances), 3))
        colors[:, 0] = normalized_distances 
        colors[:, 1] = 1.0 - normalized_distances
        segmented_pcd.colors = o3d.utility.Vector3dVector(colors)
        return segmented_pcd