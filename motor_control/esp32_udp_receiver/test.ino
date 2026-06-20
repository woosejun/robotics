#define RXD2 16
#define TXD2 17

unsigned long prevTime = 0;

void setup()
{
    Serial.begin(115200);

    Serial2.begin(
        115200,
        SERIAL_8N1,
        RXD2,
        TXD2
    );

    Serial.println("UART TEST START");
}

void loop()
{
    // Jetson으로부터 받은 데이터 출력
    if (Serial2.available())
    {
        String msg =
            Serial2.readStringUntil('\n');

        Serial.print("FROM JETSON: ");
        Serial.println(msg);
    }

    // 1초마다 Jetson으로 TEST 전송
    if (millis() - prevTime > 1000)
    {
        prevTime = millis();

        Serial2.println("HELLO JETSON");

        Serial.println(
            "SEND -> HELLO JETSON"
        );
    }
}