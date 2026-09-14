import RPi.GPIO as GPIO
import cv2, time, numpy as np
from picamera2 import Picamera2

# ============================================================
# --- PARAMÈTRES DU SYSTÈME ---
# ============================================================
T_CYCLE     = 0.1       # Temps entre chaque boucle
RAYON_CIBLE = 50        # Rayon souhaité du cercle
V_BASE      = 80        # Vitesse de base des moteurs (PWM %)
XC_CIBLE    = 160       # Centre horizontal visé (320/2)
K_DIST      = 0.5       # Gain correction distance
K_CENTRE    = 0.25     # Gain correction centrage
R_MIN       = 30        # Rayon minimum toléré
R_MAX       = 70        # Rayon maximum toléré

# ============================================================
# --- CONFIGURATION GPIO POUR LES MOTEURS ET BUZZER ---
# ============================================================
GPIO.setmode(GPIO.BCM)
PIN_G, PIN_D = 13, 19
PIN_BUZZER = 26  # exemple GPIO pour buzzer

GPIO.setup(PIN_G, GPIO.OUT)
GPIO.setup(PIN_D, GPIO.OUT)
GPIO.setup(PIN_BUZZER, GPIO.OUT)

pwmG = GPIO.PWM(PIN_G, 1000)
pwmD = GPIO.PWM(PIN_D, 1000)
pwmG.start(0)
pwmD.start(0)

vG = 10
vD = 10
historique_x = []
historique_r = []

# ============================================================
# --- INITIALISATION DE LA CAMÉRA ---
# ============================================================
try:
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (320, 240)}))
    cam.start()
    print("Caméra OK")
except Exception as e:
    print("Erreur caméra :", e)
    GPIO.cleanup()
    exit()

# ============================================================
# --- BOUCLE PRINCIPALE ---
# ============================================================
try:
    while True:
        # Capture image
        img = cam.capture_array()
        gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 5)

        # Détection de cercle
        cercles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, 1, 50,
            param1=150, param2=50, minRadius=20, maxRadius=150
        )

        buzzer_actif = False

        if cercles is not None:
            cercles = np.uint16(np.around(cercles[0]))
            meilleur = min(cercles, key=lambda c: abs(XC_CIBLE - c[0]))
            x, y, r = meilleur

            # --- Filtrage X ---
            historique_x.append(x)
            if len(historique_x) > 9: historique_x.pop(0)
            moy_x = sum(historique_x)/len(historique_x)
            if abs(x - moy_x) > 10: x = historique_x[-2]

            # --- Filtrage rayon ---
            historique_r.append(r)
            if len(historique_r) > 9: historique_r.pop(0)
            moy_r = sum(historique_r)/len(historique_r)
            if abs(r - moy_r) > 10: r = historique_r[-2]

            # --- Calcul erreurs ---
            err_x = XC_CIBLE - int(x)
            err_r = RAYON_CIBLE - int(r)

            # --- Correction moteurs ---
            vG = vG + K_DIST * err_r
            vD = vD + K_DIST * err_r
            vG = max(0, min(V_BASE, vG))
            vD = max(0, min(V_BASE, vD))

            if err_x > 0:
                vD += K_CENTRE * err_x
                vD = max(0, min(100, vD))
            elif err_x < 0:
                vG += -K_CENTRE * err_x
                vG = max(0, min(100, vG))

            # --- Gestion du buzzer ---
            if r < R_MIN or r > R_MAX:
                GPIO.output(PIN_BUZZER, GPIO.HIGH)
                buzzer_actif = True
            else:
                GPIO.output(PIN_BUZZER, GPIO.LOW)
                buzzer_actif = False

            pwmG.ChangeDutyCycle(vG)
            pwmD.ChangeDutyCycle(vD)

            print(f"R={r} X={x} Y={y} | err_x={err_x} err_r={err_r} | vG={vG:.1f} vD={vD:.1f} | Buzzer={'ON' if buzzer_actif else 'OFF'}")

            # --- Dessin du cercle et infos ---
            cv2.circle(img, (x, y), r, (0, 255, 0), 2)
            cv2.putText(img, f"x={x}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
            cv2.putText(img, f"r={r}", (10, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 1)
            cv2.putText(img, f"Buzzer={'ON' if buzzer_actif else 'OFF'}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255) if buzzer_actif else (0,255,0), 1)

        else:
            print("Aucun cercle détecté")
            pwmG.ChangeDutyCycle(vG)
            pwmD.ChangeDutyCycle(vD)
            GPIO.output(PIN_BUZZER, GPIO.LOW)

        # --- Ligne verticale au centre ---
        cv2.line(img, (XC_CIBLE, 0), (XC_CIBLE, img.shape[0]), (255, 0, 0), 2)

        # --- Affichage ---
        cv2.imshow("Cam", img)
        cv2.imshow("Gray", gray)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(T_CYCLE)

except KeyboardInterrupt:
    pass

finally:
    cam.stop()
    pwmG.stop()
    pwmD.stop()
    GPIO.output(PIN_BUZZER, GPIO.LOW)
    GPIO.cleanup()
    cv2.destroyAllWindows()
    print("Arrêt propre")
