### DrawMeMaybe

DrawMeMaybe ist ein interaktives Projekt, das Künstliche Intelligenz, Computer Vision und Robotik kombiniert, um personalisierte Zeichnungen zu erzeugen.

Eine Kamera nimmt ein Foto einer Person auf. Optional kann die Person zusätzliche Informationen wie Hobbys oder Interessen angeben. Auf Basis dieser Eingaben wird eine stilisierte Karikatur erzeugt, deren Umrisse extrahiert und in Zeichenpfade umgewandelt werden. Ein Roboterarm (UR5e) zeichnet diese Pfade anschließend auf Papier.

Das Projekt untersucht eine durchgängige Pipeline von der Bildeingabe bis zur physikalischen Ausführung durch einen Roboter.

## Projektidee

Ziel von DrawMeMaybe ist es, moderne KI-basierte Bildgenerierung mit klassischer Bildverarbeitung und industrieller Robotik zu verbinden.

Anstatt Bilder nur digital darzustellen, entsteht eine physische Zeichnung, die von einem Roboter erstellt wird. Dadurch wird eine Brücke zwischen digitaler Kreativität und realer Robotik geschlagen.

Systemübersicht

Geplante Pipeline:

Aufnahme eines Bildes mit einer Kamera

(Optional) Generierung oder Stilisierung einer Karikatur mit einem KI-Modell

Extraktion von Kanten und Umrissen

Umwandlung der Umrisse in Pfade und Wegpunkte

Übergabe der Wegpunkte an MoveIt zur Bewegungsplanung

Zeichnen der Pfade durch den Roboterarm
