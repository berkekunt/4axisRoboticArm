import cv2 
import numpy as np
from adafruit_servokit import ServoKit
import time

# Servo kontrolü
kit = ServoKit(channels=16)

# Servo sınırları
servo_config = {
    "0": (0, 180),
    "2": (0, 180),
    "3": (0, 180),
    "4": (0, 180),
}

# Robot pozisyonları
waiting_position = {"0": 0, "2": 120, "3": 0, "4": 100}
turn_to_cubes_position = {"0": 0, "3": 130, "4": 90}
grab_cube_position = {"0": 120, "4": 90, "2": 50}
drop_cube_position_step1 = {"0": 30}
drop_cube_position_step2 = {"3": 0}
drop_cube_position_step3 = {"0": 140, "4": 80}
drop_cube_position_step4 = {"2": 100}

# Renk aralıkları (HSV)
color_ranges = {
    "red": ([0, 100, 100], [10, 255, 255]),
    "yellow": ([20, 100, 100], [30, 255, 255]),
    "blue": ([100, 100, 100], [140, 255, 255])
}

# Servo açısını sınırla
def limit_angle(servo_id, angle):
    min_angle, max_angle = servo_config[str(servo_id)]
    return max(min(angle, max_angle), min_angle)

# Yavaş hareket için fonksiyon
def slow_move_servo(servo_id, start_angle, end_angle, step=2, delay=0.1):
    start_angle = int(limit_angle(servo_id, start_angle))
    end_angle = int(limit_angle(servo_id, end_angle))
    if start_angle < end_angle:
        for angle in range(start_angle, end_angle + 1, step):
            kit.servo[servo_id].angle = angle
            time.sleep(delay)
    else:
        for angle in range(start_angle, end_angle - 1, -step):
            kit.servo[servo_id].angle = angle
            time.sleep(delay)

# Pozisyona kademeli hareket fonksiyonu
def move_to_position(position, delay=0.1, step=2):
    for servo_id, target_angle in position.items():
        current_angle = kit.servo[int(servo_id)].angle or 0
        target_angle = limit_angle(servo_id, target_angle)
        slow_move_servo(int(servo_id), current_angle, target_angle, step, delay)

# Küpleri algılama fonksiyonu
def observe_cubes(cap, desired_color):
    detected_color = None
    cube_center_x = None

    while detected_color != desired_color:
        ret, frame = cap.read()
        if not ret:
            print("Kamera görüntüsü alınamadı!")
            break

        # Görüntüyü HSV renk uzayına çevir
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Her renk için maske oluştur ve kontrol et
        for color_name, (lower, upper) in color_ranges.items():
            lower_bound = np.array(lower, dtype=np.uint8)
            upper_bound = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            # Maske üzerinde belirli bir piksel sayısına ulaşılırsa rengi algıla
            if cv2.countNonZero(mask) > 500:  # Piksel eşik değeri
                detected_color = color_name
                print(f"{color_name} color detected!")
                
                # Tespit edilen renk için konturları bul
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                if contours:
                    largest_contour = max(contours, key=cv2.contourArea)
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:  # Bölge alanı sıfır değilse
                        cube_center_x = int(M["m10"] / M["m00"])  # Küpün merkez X koordinatı
                        cube_center_y = int(M["m01"] / M["m00"])  # Küpün merkez Y koordinatı
                        cv2.circle(frame, (cube_center_x, cube_center_y), 5, (0, 255, 0), -1)  # Merkez noktasını işaretle
                break  # Bir renk tespit edildiyse döngüden çık

        # Kullanıcı istediği renk bulunana kadar görüntüyü göster
        if detected_color is None:
            print("I could not detect the desired color. I continue...")
        else:
            print(f"Detected color {detected_color}")

        cv2.imshow("kamera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Çıkmak için 'q' tuşuna bas
            break

    return detected_color, cube_center_x

# Ana program
if __name__ == "__main__":
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("The camera could not be opened!")
            exit()

        # Kamera boyutlarını ayarla
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # Robotun bekleme konumuna git
        move_to_position(waiting_position)

        # Kullanıcıdan istenen rengi al
        desired_color = input("Which color do you want? (red, yellow, blue): ").strip().lower()

        if desired_color not in color_ranges:
            print("Invalid color selection!")
            exit()

        # Robot küp alanına dön
        move_to_position(turn_to_cubes_position)  # Küp alanına dön
        time.sleep(1)  # Dönüşten sonra bir süre bekle

        # Küp gözlemleme ve algılama
        detected_color, cube_center_x = observe_cubes(cap, desired_color)

        if detected_color == desired_color:
            # Küpün konumunu kontrol et ve ayarla
            if cube_center_x is not None:
                screen_center_x = 320  # Ekranın ortası
                offset = cube_center_x - screen_center_x
                
                # Hangi yönde döneceğini ayarlama
                if offset > 50:  # Sağda
                    move_to_position({"3": 170})  # Sağ döndür
                elif offset < -50:  # Solda
                    move_to_position({"3": 10})  # Sol döndür

                # Küpü al
                move_to_position(grab_cube_position)  
                move_to_position(drop_cube_position_step1)  
                move_to_position(drop_cube_position_step2)
                move_to_position(drop_cube_position_step3)
                move_to_position(drop_cube_position_step4)
                move_to_position(waiting_position)  # Başlangıç konumuna dön
        else:
            print(f"{desired_color} color could not be detected!")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
