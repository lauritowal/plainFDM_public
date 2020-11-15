# Flugdynamisches Modell

# Beschreibung:

detaillierte Beschreibung unter:
https://www.researchgate.net/publication/341353493_Implementation_of_a_software_to_calculate_the_flight_dynamics_of_an_aircraft

Programm zur Simulation des flugdynamischen Verhaltens von Luftfahrzeugen (LFZ).
Die Basis der Berechnung ist die numerischen Lösung von Differentialgleichungen, 
basierend auf den Newtonschen Bewegungsgesetz.

Es ist die Kinetik und Kinematik von 2 LFZ und dem physischen Körper "Ball" implementiert.

Die Parameter für die Kräfte- und Momenteberechnung sind zum einen als DatCom-Dateien (C172, Ball) hinterlegt
und adaptieren den Ansatz von PyFME und zum anderen ist für eine DeHavilland Beaver in Anlehnung an
die matlab-Variante (https://de.mathworks.com/help/aeroblks/fly-the-dehavilland-beaver.html)
eine freie Codierung der Parameter implementiert.

ACHTUNG: aktuell ist die Beaver codiert und intensiv getestet. 
Im Gegensatz zur Matlab-Implementierung sind die Momente des Motors 0.

Die Nutzung der DatCom Dateien erfordert in der Datei Simulation den Import von:

from FDM.physikFDM.aircraft import Aircraft
from FDM.lfz_controlFDM.pid_regler_c172 import PidRegler

und die Instanziierung von Objekten der jeweiligen Klassen. 


# todo:

- Quartenions (Modell funktioniert gut im envelope theta [-30, 30]); phi [-30, 30])
- Motordrehmomente
- Parameter Beaver als DatCom Datei
- Landeklappen
        
