import time

class TurntableController:
    """
    Steuert den Drehteller.
    Aktuell: Simulation (Mock-Objekt) für Tests ohne Hardware.
    Später: Hier kommt der echte serielle Treiber (z.B. PySerial) rein.
    """

    def __init__(self):
        self.current_angle = 0
        print("Initialisiere Drehteller (Simulation)...")

    def connect(self) -> bool:
        """Verbindet mit der Hardware."""
        print("Drehteller verbunden (SIMULIERT).")
        return True

    def rotate(self, degrees: int) -> None:
        """
        Dreht den Teller um X Grad weiter.
        """
        print(f"  >>> Drehteller bewegt sich um {degrees}° ...")
        
        # Simulation der Bewegungszeit (z.B. 2 Sekunden für 45 Grad)
        time.sleep(2.0) 
        
        self.current_angle += degrees
        print(f"  >>> Neue Position: {self.current_angle}°")

    def disconnect(self) -> None:
        """Trennt die Verbindung und gibt Ressourcen frei."""
        print("Drehteller-Verbindung geschlossen.")