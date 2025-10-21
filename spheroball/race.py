# race.py
# Autonome ronde met Sphero BOLT - parcours met bochten (wijzerszin)
# Platform: Raspberry Pi
# API: spherov2

import time
from spherov2 import scanner
from spherov2.sphero_bolt import SpheroBolt

# === HULPFUNCTIES ===

def calibrate(bolt):
    """
    Manuele kalibratie van de heading.
    Zet de robot aan de startlijn, gericht naar rechts (eerste rechte stuk).
    Heading 0° = richting eerste segment.
    """
    print("\n=== KALIBRATIE ===")
    print("Plaats de BOLT net achter de start/finishlijn.")
    print("Zorg dat de blauwe LED (achterzijde) naar jou wijst.")
    input("Druk op ENTER zodra heading correct is ingesteld...")
    bolt.set_heading(0)
    print("Heading = 0° ingesteld.\n")

def drive_straight(bolt, speed, heading, duration):
    """
    Laat de robot recht rijden voor een bepaalde tijdsduur (s).
    """
    bolt.roll(speed, heading)
    time.sleep(duration)
    bolt.stop()

def smooth_turn(bolt, speed, start_angle, end_angle, step=10, delay=0.1):
    """
    Maakt een vloeiende bocht door heading geleidelijk te veranderen.
    start_angle < end_angle voor rechtsaf
    """
    if start_angle < end_angle:
        angles = range(start_angle, end_angle + 1, step)
    else:
        angles = range(start_angle, end_angle - 1, -step)

    for angle in angles:
        bolt.roll(speed, angle)
        time.sleep(delay)
    bolt.stop()

def drive_race_track(bolt):
    """
    Volledige ronde in wijzerszin met rechte stukken en vloeiende bochten.
    Gebaseerd op parcours met 4 bochten (zoals op foto).
    """
    speed = 80  # experimenteer tussen 60–100
    print("Ronde gestart...")

    # === START ===
    start_time = time.time()

    # 1. Eerste rechte stuk (bovenste zijde)
    drive_straight(bolt, speed, 0, 4.0)  # ca. 4 m

    # 2. Eerste bocht rechtsaf (naar beneden)
    smooth_turn(bolt, speed, 0, 90)

    # 3. Rechte zijde omlaag (rechterzijde)
    drive_straight(bolt, speed, 90, 2.5)

    # 4. Tweede bocht rechtsaf (onderaan)
    smooth_turn(bolt, speed, 90, 180)

    # 5. Onderzijde (naar links)
    drive_straight(bolt, speed, 180, 4.0)

    # 6. Derde bocht rechtsaf (linkerzijde omhoog)
    smooth_turn(bolt, speed, 180, 270)

    # 7. Linkerzijde omhoog
    drive_straight(bolt, speed, 270, 2.5)

    # 8. Laatste bocht rechtsaf (naar startlijn)
    smooth_turn(bolt, speed, 270, 360)  # 360° = 0°

    # 9. Terug over start/finish
    drive_straight(bolt, speed, 0, 0.5)

    # === FINISH ===
    bolt.stop()
    end_time = time.time()
    lap_time = end_time - start_time
    print(f"\n=== RONDE VOLTOOID ===")
    print(f"Tijd: {lap_time:.2f} seconden\n")

# === MAIN ===

def main():
    print("Zoeken naar Sphero BOLT...")
    try:
        bolt = next(scanner.find_bolts())
        calibrate(bolt)
        drive_race_track(bolt)

    except KeyboardInterrupt:
        print("\nHandmatig gestopt — robot veilig stoppen...")
        try:
            bolt.stop()
        except:
            pass
    except Exception as e:
        print(f"⚠️ Fout opgetreden: {e}")
        try:
            bolt.stop()
        except:
            pass

if __name__ == "__main__":
    main()