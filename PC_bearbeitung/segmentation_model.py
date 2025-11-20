"""
Modul: segmentation_model.py
Definiert die Architektur des neuronalen Netzes (PointNet) für die 
semantische Segmentierung von Punktwolken.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

class PointNetSegmentation(nn.Module):
    def __init__(self, num_classes=2):
        super(PointNetSegmentation, self).__init__()
        
        # --- Feature Extractor (Lernt lokale Geometrie) ---
        # Input: 3 Kanäle (x, y, z). Output: 64 Features
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.bn1 = nn.BatchNorm1d(64)
        
        # 64 -> 128 Features
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.bn2 = nn.BatchNorm1d(128)
        
        # 128 -> 1024 Features (Global Feature Vektor)
        self.conv3 = nn.Conv1d(128, 1024, 1)
        self.bn3 = nn.BatchNorm1d(1024)
        
        # --- Segmentation Head (Entscheidet für jeden Punkt) ---
        # Wir kombinieren lokale Features (64) + globale Features (1024)
        # Input für den Head ist also 1088
        self.conv4 = nn.Conv1d(1088, 512, 1)
        self.bn4 = nn.BatchNorm1d(512)
        
        self.conv5 = nn.Conv1d(512, 256, 1)
        self.bn5 = nn.BatchNorm1d(256)
        
        # Output: Anzahl der Klassen (hier 2: Hintergrund, Bauteil)
        self.conv6 = nn.Conv1d(256, num_classes, 1)

    def forward(self, x):
        # x Shape: [Batch_Size, 3, Num_Points]
        num_points = x.size()[2]
        
        # 1. Lokale Features extrahieren
        x = F.relu(self.bn1(self.conv1(x)))
        local_features = x # Merken wir uns für später (Shape: [B, 64, N])
        
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x))) # Shape: [B, 1024, N]
        
        # 2. Globales Feature (Max Pooling)
        # Wir nehmen den stärksten Wert über alle Punkte hinweg
        global_feature = torch.max(x, 2, keepdim=True)[0] # Shape: [B, 1024, 1]
        
        # 3. Features kombinieren
        # Wir kopieren das globale Feature 1024x und kleben es an jeden Punkt
        global_feature_repeated = global_feature.repeat(1, 1, num_points)
        
        # Zusammenfügen: Lokal (64) + Global (1024) = 1088 Features pro Punkt
        x = torch.cat([local_features, global_feature_repeated], 1)
        
        # 4. Klassifizierung pro Punkt
        x = F.relu(self.bn4(self.conv4(x)))
        x = F.relu(self.bn5(self.conv5(x)))
        
        x = self.conv6(x) # Shape: [B, 2, N]
        
        # Log Softmax für Wahrscheinlichkeiten
        return F.log_softmax(x, dim=1)

if __name__ == "__main__":
    # Kleiner Test, ob das Modell kompiliert
    simulated_input = torch.rand(1, 3, 2000) # 1 Batch, 3 Koordinaten, 2000 Punkte
    model = PointNetSegmentation()
    output = model(simulated_input)
    print("Modell Test erfolgreich. Output Shape:", output.shape) 
    # Erwartet: [1, 2, 2000] -> Für jeden der 2000 Punkte gibt es 2 Wahrscheinlichkeiten