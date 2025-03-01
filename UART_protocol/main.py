import serial
import serial.tools.list_ports

BAUDRATE = 115200

ser = None  # Глобальный объект Serial


# Функция для вычисления CRC8
def calculate_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF  # Защита от переполнения
    return crc


# Функция для отправки пакета данных
def send_packet(target, function, data):
    packet = [0xFF, target, function] + list(data)
    crc = calculate_crc8(packet)
    packet.append(crc)
    packet.append(0x0F)

    print(f"Отправка пакета: {packet}")  # Отладка

    ser.write(bytearray(packet))


# Функция для подключения к COM порту
def connect_com_port():
    global ser
    com_ports = list(serial.tools.list_ports.comports())

    if not com_ports:
        print("Нет доступных COM портов.")
        return None

    print("Доступные COM порты:")
    for i, port in enumerate(com_ports):
        print(f"{i + 1}. {port.device}")

    port_choice = int(input("Выберите COM порт (номер): ")) - 1
    if port_choice < 0 or port_choice >= len(com_ports):
        print("Неверный выбор порта.")
        return None

    selected_port = com_ports[port_choice].device

    baudrate = input(f"Введите Baudrate (по умолчанию {BAUDRATE}): ")
    if not baudrate.isdigit():
        baudrate = BAUDRATE
    else:
        baudrate = int(baudrate)

    try:
        ser = serial.Serial(selected_port, baudrate=baudrate, timeout=1)
        print(f"Успешно подключено к {selected_port} с Baudrate {baudrate}")
    except Exception as e:
        print(f"Не удалось подключиться к порту: {e}")
        return None

    return ser


# Функция для отправки данных о LED
def send_led_data():
    led_state = input("Состояние LED? ( 1 / 0 ): ").strip()
    if not led_state.isdigit():
        print("Нужно было ввести число.")
        return
    led_state = int(led_state)
    if not (led_state == 1 or led_state == 0):
        print("Введите 1 или 0.")
        return
    led_state = 0x01 if led_state == 1 else 0x00
    send_packet(0x00, 0x00, [0x00, 0x00, 0x00, led_state])


# Функция для отправки данных о частоте LED
def send_led_frequency():
    frequency_str = input("Введите частоту LED (Hz): ").strip()
    if not frequency_str.isdigit():
        print("Частота должна быть числом.")
        return
    frequency = int(frequency_str)
    send_packet(0x00, 0x01, list(frequency.to_bytes(4, 'big')))


# Функция для отправки данных о гамме LED
def send_led_gamma():
    gamma_str = input("Введите Gamma для LED: ").strip()
    try:
        gamma = int(float(gamma_str) * 10)
    except ValueError:
        print("Неверное значение Gamma.")
        return
    send_packet(0x00, 0x02, list(gamma.to_bytes(4, 'big')))


# Функция для отправки данных об угле сервопривода
def send_servo_angle():
    angle = int(input("Введите угол сервопривода (0-180°): ").strip())
    if angle < 0 or angle > 180:
        print("Угол должен быть в пределах от 0 до 180.")
        return
    send_packet(0x01, 0x00, list(angle.to_bytes(4, 'big')))


# Главная функция для меню
def main():
    global ser
    ser = connect_com_port()
    if not ser:
        return

    while True:
        print("\nМеню:")
        print("1. Включить/выключить LED")
        print("2. Установить частоту LED")
        print("3. Установить гамму LED")
        print("4. Установить угол сервопривода")
        print("5. Выход")

        choice = input("Выберите действие (1-5): ").strip()

        if choice == '1':
            send_led_data()
        elif choice == '2':
            send_led_frequency()
        elif choice == '3':
            send_led_gamma()
        elif choice == '4':
            send_servo_angle()
        elif choice == '5':
            print("Выход из программы.")
            if ser and ser.is_open:
                ser.close()
            break
        else:
            print("Неверный выбор. Попробуйте снова.")


if __name__ == "__main__":
    main()
