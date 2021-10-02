#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>

static RF24 radio(7, 8); // CE, CSN

static unsigned long MANUAL_TIMEOUT = 10000;
static unsigned long DDOS_INTERVAL = 5;
static const uint32_t NUMBER_OF_DEVICES = 5;
static const unsigned int MASTER_BUTTON_TIMEOUT = 3000;

static const int button_0 = A1;
static const int button_1 = A2;
static const int button_2 = A3;
static const int button_3 = A4;

static const int self_dance = button_0;
static const int all_dance = button_1;
static const int master_button = button_2;
static const int shift_button = button_3;

static const int led_r = 5;
static const int led_b = 3;
static const int led_g = 6;

static const int led_jacket = 10;
static const byte address[6] = "00001";

static bool west_is_on = false;

static unsigned long next_ddos_ms = 0;
static unsigned long leave_manual_ms = 0;

static int movie_step = 0;
static int sequence_step = 0;
static int num_activations = 0;
static int sequence_iterations = 0;
static int sequence_delay = 0;
static unsigned long sequence_advance = 0;

static bool slave = true;
static unsigned long master_button_down = 0;

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

#define count_of(x) (sizeof(x) / sizeof(x[0]))

#define light_any(v) { (uint8_t)v, (uint8_t)~(v) }
#define light_one(n) { (1 << n), ~(1 << n) }

typedef struct activations
{
	uint8_t activate;
	uint8_t deactivate;
}
activations_t;

typedef struct sequence
{
	int iterations;
	int step_delay;
	int step_delay_add;

	uint32_t num_activations;
	activations_t activations[64];
}
sequence_t;

static const sequence_t single_person_wave =
{
	20000, 35, 0, NUMBER_OF_DEVICES,
	{ light_one(0), light_one(1), light_one(2), light_one(3), light_one(4) }
};

static const sequence_t *single_wave_movie[] =
{
	&single_person_wave,
	NULL
};

static const sequence_t beer_random_0 = 
{
	7, 35, 0, NUMBER_OF_DEVICES,
	{ light_one(0), light_one(1), light_one(2), light_one(3), light_one(4) }
};

static const sequence_t beer_random_1 = 
{
	7, 35, 50, NUMBER_OF_DEVICES,
	{ light_one(0), light_one(1), light_one(2), light_one(3), light_one(4) }
};

static const sequence_t beer_random_select =
{
	1, 1500, 300, 0x80,
	{ light_one(0), light_one(1), light_one(2), light_one(3), light_one(4) }
};

static const sequence_t *beer_movie[] =
{
	&beer_random_0,
	&beer_random_1,
	&beer_random_select,
	NULL
};

static const sequence_t hyper_wave =
{
	2000, 60, 0, NUMBER_OF_DEVICES,
	{ light_any(0x00), light_any(0x04), light_any(0x0E), light_any(0x1F), light_any(0x0E) }
};

static const sequence_t *hyper_wave_movie[] =
{
	&hyper_wave
};

static int get_activation_steps(const sequence_t *seq)
{
	if(seq->num_activations & 0x80)
	{
		int r = random(NUMBER_OF_DEVICES + 1);
		return r;
	}
	return seq->num_activations;
}

static uint8_t compute_crc8(const uint8_t *data, uint32_t len)
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

static bool read_button(int pin)
{
	return (LOW == digitalRead(pin));
}

static bool set_rgb(int r, int g, int b)
{
	digitalWrite(led_r, r);
	digitalWrite(led_g, g);
	digitalWrite(led_b, b);

	return true;
}

static void update_rgb(void)
{
	if(!read_button(button_0)
	&& !read_button(button_1)
	&& !read_button(button_2)
	&& !read_button(button_3))
	{
		if(!west_is_on)
		{
			digitalWrite(led_r, 0);
			digitalWrite(led_g, 0);
			digitalWrite(led_b, 0);
		}
	}
}

static void led_show_master()
{
	if(slave)
	{
		set_rgb(255, 0, 0);
	}
	else
	{
		set_rgb(0, 0, 255);
	}
}

static void west_on(void)
{
	if(!west_is_on)
	{
		Serial.println("West OFF => ON");
	}

	led_show_master();

	digitalWrite(led_jacket, HIGH);
	west_is_on = true;
}

static void west_off(void)
{
	if(west_is_on)
	{
		Serial.println("West ON => OFF");
	}

	set_rgb(0, 0, 0);
	digitalWrite(led_jacket, LOW);
	west_is_on = false;
}

static void radio_up(void)
{
	radio.begin();
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(false);

	radio.openWritingPipe(address);
	radio.openReadingPipe(0, address);

	Serial.print("PA before we set it: ");
	Serial.println(radio.getPALevel());

	radio.setPALevel(RF24_PA_MAX);

	Serial.print("PA after we set it: ");
	Serial.println(radio.getPALevel());

	Serial.println("Initializaing chip...!");

	for(;;)
	{
		if(radio.isChipConnected())
		{
			break;
		}
	}

	Serial.println("Chip is happy! Fanfare.");

	set_rgb(255, 0, 0);
	delay(500);
	set_rgb(0, 255, 0);
	delay(500);
	set_rgb(0, 0, 255);
	delay(500);
	set_rgb(255, 255, 255);
	delay(500);
	set_rgb(0, 0, 0);

	Serial.println("Chip connection established!");
}

static void enter_master(void)
{
	slave = false;
	radio.stopListening();
}

static void enter_slave(void)
{
	slave = true;
	radio.startListening();
}

static void toggle_master(void)
{
	if(slave)
	{
		enter_master();
	}
	else
	{
		enter_slave();
	}
}

void setup()
{
	int seed;
	device_id = EEPROM.read(0);	

	pinMode(led_jacket, OUTPUT);

	pinMode(led_r, OUTPUT);
	pinMode(led_g, OUTPUT);
	pinMode(led_b, OUTPUT);

	pinMode(10, OUTPUT); // SJUUUUUKT VIKTIG TA INTE BORT DEN H'R!!!

	pinMode(button_0, INPUT_PULLUP);
	pinMode(button_1, INPUT_PULLUP);
	pinMode(button_2, INPUT_PULLUP);
	pinMode(button_3, INPUT_PULLUP);

	Serial.begin(115200);

	seed = analogRead(A0);
	Serial.print("Seed: ");
	Serial.println(seed);

	randomSeed(seed);

	Serial.print("Device ID: ");
	Serial.println(device_id);

	radio_up();
	enter_slave();
	west_off();
}

static void handle_sequences(jack_pack_t *buf)
{
	const sequence_t **mov;
	const sequence_t *seq;

	if(read_button(button_0))
	{
		mov = beer_movie;
	}
	else if(read_button(button_1))
	{
		mov = single_wave_movie;
	}
	else if(read_button(button_2))
	{
		mov = hyper_wave_movie;
	}
	else
	{
		return;
	}

	seq = mov[movie_step];

	if(NULL == seq)
	{
		if(beer_movie == mov)
		{
			if(millis() >= sequence_advance)
			{
				sequence_advance = millis() + 100;
				if(sequence_step & 1)
				{
					buf->activate = 1 << (num_activations - 1);
					buf->deactivate = ~(1 << (num_activations - 1));
				}
				else
				{
					buf->activate = 0x00;
					buf->deactivate = 0xFF;
				}
				++sequence_step;
			}
		}

		return;
	}

	if(0 == sequence_advance)
	{
		sequence_step = 0;
		num_activations = get_activation_steps(seq);
		sequence_iterations = 0;
		sequence_delay = seq->step_delay;
		sequence_advance = millis() + sequence_delay;

		return;
	}

	if(sequence_iterations < seq->iterations)
	{
		buf->activate = seq->activations[sequence_step].activate;
		buf->deactivate = seq->activations[sequence_step].deactivate;

		if(millis() > sequence_advance)
		{
			if(++sequence_step >= num_activations)
			{
				sequence_step = 0;
				sequence_delay += seq->step_delay_add;
				sequence_iterations += 1;
			}

			sequence_advance = millis() + sequence_delay;
		}
	}
	else
	{
		++movie_step;
		sequence_advance = 0;
	}
}

static void handle_master_button(void)
{
	if(0 == master_button_down
	&& read_button(master_button))
	{
		Serial.print("Start master toggle... (3 sec)");
		Serial.println(slave);
		master_button_down = millis() + MASTER_BUTTON_TIMEOUT;
	}

	if(!read_button(master_button))
	{
		if(master_button_down)
		{
			Serial.println("Master button released.");
			master_button_down = 0;
		}
		return;
	}

	Serial.println("Showing master/slave state LED");
	led_show_master();

	if(master_button_down
	&& millis() >= master_button_down)
	{
		Serial.println("Master button held, toggling master state...");
		Serial.print("Previous state: ");
		Serial.println(slave);

		toggle_master();
		master_button_down = 0;
	}
}

static void dispatch_packet(const jack_pack_t *p)
{
	if(slave && (0 != leave_manual_ms))
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

	while(!radio.isChipConnected())
	{
		Serial.println("Connection with RF24 lost...");

		set_rgb(0xFF, 0xA0, 0);
		delay(150);
		set_rgb(0, 0, 0);
		delay(150);
	}

	update_rgb();

	if(!read_button(shift_button))
	{
		movie_step = 0;

		if(read_button(self_dance))
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
	
		handle_master_button();
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
			buf.activate = 0;
			buf.deactivate = 0;

			if(read_button(shift_button))
			{
				handle_sequences(&buf);
			}
			else
			{
				sequence_advance = 0;
				if(read_button(all_dance))
				{
					buf.activate = 0xFF;
					buf.deactivate = 0x00;
				}
				else
				{
					buf.activate = 0x00;
					buf.deactivate = 0xFF;
				}
			}

			buf.crc8 = compute_crc8((const uint8_t *)&buf, sizeof(buf) - 1);
			dispatch_packet(&buf);

			radio.write(&buf, sizeof(buf));
		}
	}
}

