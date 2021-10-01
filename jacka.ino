#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

static RF24 radio(7, 8); // CE, CSN

static unsigned long MANUAL_TIMEOUT = 10000;
static unsigned long DDOS_INTERVAL = 5;

static const int button_0 = A1;
static const int button_1 = A2;
static const int button_2 = A3;
static const int button_3 = A4;

static const int led_g = 3;
static const int led_r = 5;
static const int led_b = 6;

static const int led_jacket = A5;
static const byte address[6] = "00001";

static unsigned long next_ddos_ms = 0;
static unsigned long leave_manual_ms = 0;
static bool slave = true;

static uint8_t device_id = 0;

#pragma pack(push, 1)
typedef struct jack_pack
{
	uint8_t master;
	uint8_t activate;
	uint8_t deactivate;
	uint8_t crc8;
}
jack_pack_t;
#pragma pack(pop)

uint8_t compute_crc8(const uint8_t *data, uint32_t len)
{
	byte crc = 0x00;
	for(uint32_t i = 0; i < len; ++i)
	{
		uint8_t b = *data++;

		for(uint8_t bit = 8; bit; bit--)
		{
			byte tmp = (crc ^ b) & 0x01;
			crc >>= 1;
			if(tmp)
			{
				crc ^= 0x8C;
			}
      		b >>= 1;
		}
    }
	return crc;
}

static void set_rgb(int r, int g, int b)
{
	digitalWrite(led_r, r);
	digitalWrite(led_g, g);
	digitalWrite(led_b, b);
}

static void west_on(void)
{
	set_rgb(255, 0, 0);
	digitalWrite(led_jacket, HIGH);
}

static void west_off(void)
{
	if(slave)
	{
		set_rgb(0, 255, 0);
	}
	else
	{
		set_rgb(0, 0, 255);
	}

	digitalWrite(led_jacket, LOW);
}

static void radio_up(void)
{
	radio.begin();
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(false);

	radio.openWritingPipe(address);
	radio.openReadingPipe(0, address);
	
	Serial.println("Initializaing chip...!");


	for(;;)
	{
		if(radio.isChipConnected())
		{
			break;
		}
	}

	Serial.println("Chip connection established!");
}

static void enter_master(void)
{
	radio.stopListening();
	slave = false;
	west_off();
}

static void enter_slave(void)
{
	set_rgb(0, 255, 0);
	radio.startListening();
	west_off();
}

static bool read_button(int pin)
{
	return (LOW == digitalRead(pin));
}

void setup()
{
	device_id = EEPROM.read(0);

	pinMode(led_jacket, OUTPUT);

	pinMode(led_r, OUTPUT);
	pinMode(led_g, OUTPUT);
	pinMode(led_b, OUTPUT);

	// pinMode(10, OUTPUT); // SJUUUUUKT VIKTIG TA INTE BORT DEN H'R!!!
	pinMode(button_0, INPUT_PULLUP);
	pinMode(button_1, INPUT_PULLUP);
	pinMode(button_2, INPUT_PULLUP);
	pinMode(button_3, INPUT_PULLUP);

	Serial.begin(115200);

	Serial.print("Device ID: ");
	Serial.println(device_id);

	radio_up();
	enter_slave();
	west_off();
}

static void dispatch_packet(const jack_pack_t *p)
{
	if(0 != leave_manual_ms)
	{
		Serial.println("Received packet, but I am manual right now...");
		return;
	}

	if(p->crc8 != compute_crc8((const uint8_t *)p, sizeof(jack_pack_t) - 1))
	{
		Serial.println("Bad CRC! Ignoring packet :(");
		Serial.print(p->master);
		Serial.print(p->activate);
		Serial.println(p->deactivate);
		return;
	}

	if(p->activate & (1 << device_id))
	{
		west_on();
	}
	else if(p->deactivate & (1 << device_id))
	{
		west_off();
	}
}

void loop()
{
	jack_pack_t buf;

	if(read_button(button_0))
	{
		leave_manual_ms = millis() + MANUAL_TIMEOUT;
		west_on();
	}
	else if(0 != leave_manual_ms)
	{
		west_off();
		if(millis() > leave_manual_ms)
		{
			leave_manual_ms = 0;
		}
	}

	if(slave && read_button(button_3))
	{
		enter_master();
	}

	if(slave)
	{
		if(radio.available())
		{
			radio.read(&buf, sizeof(buf));
			dispatch_packet(&buf);
		}
	}
	else
	{
		unsigned long now = millis();

		if(now >= next_ddos_ms)
		{
			next_ddos_ms = now + DDOS_INTERVAL;

			buf.master = (1 << device_id);

			if(read_button(button_1))
			{
				buf.activate = 0xFF;
				buf.deactivate = 0x00;
			}
			else
			{
				buf.activate = 0x00;
				buf.deactivate = 0xFF;
			}

			buf.crc8 = compute_crc8((const uint8_t *)&buf, sizeof(buf) - 1);
			radio.write(&buf, sizeof(buf));

			dispatch_packet(&buf);
		}
	}
}

