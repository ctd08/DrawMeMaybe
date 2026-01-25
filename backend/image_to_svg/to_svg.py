import cv2 as cv2
import numpy as np
import json


def contours_to_json(contours, filename="paths.json", scale=0.001):
    paths = []

    for cnt in contours:
        line = []
        for p in cnt:
            x = round(float(p[0][0]) * scale, 4)
            y = round(float(p[0][1]) * scale, 4)
            line.append([x, y])
        paths.append(line)

    data = {
        "paths": paths
    }

    with open(filename, "w") as f:
        json.dump(data, f, indent=2)


def contours_to_svg(contours, filename="out.svg", width=None, height=None):
    # Breite/Höhe optional für viewBox
    if width is None or height is None:
        width = max([p[0][0] for c in contours for p in c]) + 1
        height = max([p[0][1] for c in contours for p in c]) + 1

    with open(filename, "w") as f:
        f.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">\n')
        for cnt in contours:
            # "M x y L x y ..." Path
            d = "M " + " L ".join(f"{p[0][0]} {p[0][1]}" for p in cnt) + " Z"
            f.write(f'<path d="{d}" stroke="black" fill="none"/>\n')
        f.write('</svg>')
        

def to_contours(img, top_n):

    # Pixel in liste umwandeln
    pixels = img.reshape((-1, 1)).astype(np.float32)

    # Kmeans parameter
    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.2) # 0.2 = min änderung nach iteration
    _k, labels, centers = cv2.kmeans(
        pixels,           # Daten
        3,                # Anzahl Cluster (hell, mittel, dunkel)
        None,             # Keine Initiallabels
        criteria,         # Kriterien
        10,               # Anzahl der Initialisierungen (versuche)
        cv2.KMEANS_RANDOM_CENTERS
    )
    # Cluster zurück ins bild
    segmented = centers[labels.flatten()].reshape(img.shape).astype("uint8")

    blurred = cv2.GaussianBlur(segmented, (5, 5), 0) #canny ist sonst empfindlich gegen Bildrauschen und zu hohen Kontrast
    edges = cv2.Canny(blurred, 80, 150)  #thresholds1 = vielleicht Kante, threshold2 = starke Kante




    # Konturen finden
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    top_contours = contours[:top_n]


    # Approximation
    approx_contours = [cv2.approxPolyDP(cnt, epsilon=1.5, closed=True) for cnt in top_contours]
    return approx_contours



def main(args=None):
    img = cv2.imread("backend/image_to_svg/bsp_tuep.png", cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError("Image could not be loaded")
    contours = to_contours(img, 150)         #wieviele Linien es sein sollen
    contours_to_svg(contours, "top150.svg")
    contours_to_json(contours,"paths.json", 0.01)

if __name__ == "__main__":
    main()

