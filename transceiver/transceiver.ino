#include <Key.h>
#include <Keypad.h>
#include <TimerOne.h>
#include <LiquidCrystal.h>

#define SEND_PIN 3
#define RECV_PIN 2

constexpr int rs = A5, en = A4, d4 = A3, d5 = A2, d6 = A1, d7 = A0;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

constexpr char keymap[] = {
  'a', 'b', 'c', 'A',
  'd', 'e', 'f', 'J',
  'g', 'h', 'i', 'S',
  'C', ' ', 'B', 'N',
};
constexpr byte rowpins[] = {12, 11, 10, 9};
constexpr byte colpins[] = { 8,  7,  6, 5}; 
Keypad keypad((char *)keymap, (byte *)rowpins, (byte *)colpins, 4, 4);

constexpr unsigned BITRATE = 5000;
constexpr uint32_t OVERSAM = 8;
constexpr uint32_t PERIOD = 1e6 / (OVERSAM * BITRATE);

constexpr unsigned BITS_PER_SYM = 10;
constexpr unsigned SYM_MASK = (1 << BITS_PER_SYM) - 1;
constexpr uint16_t SFD = 0b0101111100;
constexpr uint16_t EFD = (~SFD) & SYM_MASK;

constexpr unsigned PAYLOAD_SIZE = 255;
static uint8_t send_buffer[PAYLOAD_SIZE + 1] = {};
static uint8_t recv_buffer[PAYLOAD_SIZE + 1] = {};

static volatile enum {NSND, SEND, DSND} send_state = NSND;
static volatile enum {NRCV, RECV, BRKN, DRCV} recv_state = NRCV;
static volatile uint16_t send_total = 0;
static volatile uint16_t recv_total = 0;
static volatile uint16_t symbol = 0;

void setup() {
  pinMode(SEND_PIN, OUTPUT);
  Timer1.initialize(PERIOD);
  Timer1.attachInterrupt(full_duplex);
  
  if (digitalRead(13)) {
    Serial.begin(115200);
    Serial.setTimeout(100);
    Serial.print("READY\n");
    while (1) serial_loop();
  } else {
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print(">               ");
    lcd.setCursor(0, 1);
    lcd.print("<               ");
    while (1) lcd_keypad_loop();
  }
}

void loop() {}

void serial_loop() {
  switch (send_state) {
    case NSND: {
      if (!Serial.available()) break;
      send_total = Serial.readBytes(send_buffer, PAYLOAD_SIZE);
      send_buffer[send_total] = 0;
      send_state = SEND;
    } break;

    case DSND: {
      send_total = 0;
      Serial.write("SENT\n");
      send_state = NSND;
    } break;
  }

  switch (recv_state) {
    case DRCV: {
      Serial.write("RECV\n");
      Serial.write(recv_total);
      for (int i = 0; i < recv_total; i ++)
        Serial.write(recv_buffer[i]);
      recv_state = NRCV;
    } break;

    case BRKN: {
      Serial.write("BRKN\n");
      recv_state = NRCV;
    } break;
  }  
}

void lcd_keypad_loop() {
  static char offset = 0;
  static bool caps = false;
  
  switch (send_state) {
    case NSND: {
      char a = keypad.getKey();
      switch (a) {
        case 0: break;
        case 'N': send_total += 1; send_state = SEND; break;
        case 'B': {
          if (send_total <= 0) break;
          if (send_total <= 15) {
            lcd.setCursor(send_total, 0);
            lcd.print(' ');
          } else {
            lcd.setCursor(1, 0);
            lcd.print((char*)(send_buffer + send_total - 16));
          }
          send_buffer[send_total --] = 0;
        } break;
        case 'C' : caps = !caps; break;
        case 'A' :
        case 'J' :
        case 'S' : offset = a - 'A'; break;
        default: {
          if (send_total >= PAYLOAD_SIZE) break;
          a = a == ' ' ? a : (a + offset) ^ (caps << 5);
          send_buffer[send_total ++] = a;
          send_buffer[send_total] = 0;
          if (send_total <= 15) {
            lcd.setCursor(send_total, 0);
            lcd.print(a);
          } else {
            lcd.setCursor(1, 0);
            lcd.print((char*)(send_buffer + send_total - 15));
          }
        } break;
      }
    } break;

    case DSND: {
      send_total = 0;
      lcd.setCursor(0, 0);
      lcd.print(">               ");
      send_state = NSND;
    } break;
  }

  switch (recv_state) {
    case DRCV: {
      lcd.setCursor(0, 1);
      lcd.print("<               ");
      lcd.setCursor(1, 1);
      lcd.print((char*)recv_buffer);
      recv_state = NRCV;
    } break;

    case BRKN: {
      lcd.setCursor(0, 1);
      lcd.print("x               ");
      recv_state = NRCV;
    } break;
  }
}

//////////////////////////////////////////////////
// Full Duplex Communication
//////////////////////////////////////////////////

void full_duplex() {
  static unsigned ticks = 0;

  if (++ticks == OVERSAM) {
    ticks = 0;
    if (send_state == SEND)
      send();
    else
      PORTD ^= 1 << SEND_PIN;
  }

  if (sample()) {
    if (recv_state == RECV)
      recv();
    else if (recv_state == NSND && symbol == SFD)
      recv_state = RECV;
  }
}

void send() {
  static unsigned sent_bits = BITS_PER_SYM;
  static unsigned sent_syms = 0;
  static uint16_t curr_sym;
  static uint8_t rd = 1;
  static bool first = true;
  static bool last, done;

  if (sent_bits == BITS_PER_SYM) {
    if (done) {
      rd = 1;
      first = true;
      last = false;
      done = false;
      sent_syms = 0;
      send_state = DSND;
      return;
    }
    
    sent_bits = 0;
    if (first) {
      curr_sym = SFD;
      first = false;
    } else if (last) {
      curr_sym = EFD;
      done = true;
    } else {
      curr_sym = enc8b10b(send_buffer[sent_syms++], &rd);
      last = sent_syms >= send_total;
    }
  }

  send_bit(curr_sym & 1);
  curr_sym >>= 1;
  sent_bits ++;
}

void recv() {
  static unsigned recv_bits, recv_syms;

  constexpr auto recv_end = [](unsigned s) {
    recv_bits = 0;
    recv_syms = 0;
    recv_state = s;
  };

  if (symbol == EFD) {
    recv_total = recv_syms;
    recv_end(DRCV);
    return;
  }
  
  if (recv_syms > PAYLOAD_SIZE) {
    recv_end(BRKN);
    return;
  }

  if (++recv_bits == BITS_PER_SYM) {
    recv_bits = 0;
    recv_buffer[recv_syms] = dec8b10b(symbol);
    recv_buffer[++recv_syms] = 0;
  }
}

//////////////////////////////////////////////////
// Sample using SPLL
//////////////////////////////////////////////////

static bool sample() {   
  constexpr uint16_t NCO_BIAS  = (1l << 16) / OVERSAM;  
  constexpr int32_t PHASE_TARGET = 90 * (1l << 16) / 360;

  constexpr int16_t Kp = 6; // bit shift division
  constexpr int16_t Ki = 9; // bit shift division

  int32_t error;
  static uint8_t prev;
  static uint32_t integrator;
  static struct {
    uint16_t phase = 0;
    int16_t  word  = 0;
    uint8_t  out   = 0;
    uint8_t  last  = 0;
  } nco;
  
  static unsigned no_sam_change;
  static unsigned no_nco_change;

  constexpr auto reset = []() {
    no_sam_change = 0;
    no_nco_change = 0;
    integrator = 0;
    memset(&nco, 0, sizeof(nco));
  };

  uint8_t curr = recv_bit();
  if (curr ^ prev) {
    prev = curr;
    no_sam_change = 0;
    
    error = PHASE_TARGET - nco.phase;
    integrator += error; 
    nco.word = (error >> Kp) + (integrator >> Ki);
  } else if (++no_sam_change > OVERSAM * 100) {
    reset();
    return false;
  }

  nco.phase += nco.word;
  nco.phase += NCO_BIAS;
  nco.out = (nco.phase & (1 << 15)) != 0;

  bool rising_edge = !nco.last && nco.out; 
  nco.last = nco.out;

  if (rising_edge) {
    uint16_t msb = curr << (BITS_PER_SYM - 1);
    symbol >>= 1;
    symbol |= msb;
    symbol &= SYM_MASK;
    no_nco_change = 0;
  } else if (++no_nco_change > OVERSAM * 100) {
    reset();
    return false;
  }

  return rising_edge;
}

//////////////////////////////////////////////////
// Helper
//////////////////////////////////////////////////

static inline void send_bit(uint16_t bit) {
  bit ? PORTD |= (1 << SEND_PIN) : PORTD &= ~(1 << SEND_PIN);
}

static inline uint8_t recv_bit() {
  return (PIND & (1 << RECV_PIN)) != 0;
}

//////////////////////////////////////////////////
// 8b/10b encoder
//////////////////////////////////////////////////

static const uint8_t enclut5b6b[] = {
  0b111001, 0b000110, 0b101110, 0b010001, 0b101101, 0b010010, 0b100011, 0b100011,
  0b101011, 0b010100, 0b100101, 0b100101, 0b100110, 0b100110, 0b000111, 0b111000,
  0b100111, 0b011000, 0b101001, 0b101001, 0b101010, 0b101010, 0b001011, 0b001011,
  0b101100, 0b101100, 0b001101, 0b001101, 0b001110, 0b001110, 0b111010, 0b000101,
  0b110110, 0b001001, 0b110001, 0b110001, 0b110010, 0b110010, 0b010011, 0b010011,
  0b110100, 0b110100, 0b010101, 0b010101, 0b010110, 0b010110, 0b010111, 0b101000,
  0b110011, 0b001100, 0b011001, 0b011001, 0b011010, 0b011010, 0b011011, 0b100100,
  0b011100, 0b011100, 0b011101, 0b100010, 0b011110, 0b100001, 0b110101, 0b001010,
};

static const uint8_t enclut3b4b[] = {
  0b1101, 0b0010, 0b1001, 0b1001, 0b1010, 0b1010, 0b0011, 0b1100,
  0b1011, 0b0100, 0b0101, 0b0101, 0b0110, 0b0110, 0b0111, 0b1000, 0b1110, 0b0001,
};

static uint16_t enc8b10b(uint8_t data_in, uint8_t *r) {
  uint8_t d = *r;

  uint8_t x = (data_in >> 0) & 0b11111;
  uint8_t ex = enclut5b6b[(x << 1) + d];

  uint8_t a = ex;
  a = (a & 0x55) + ((a >> 1) & 0x55);
  a = (a & 0x33) + ((a >> 2) & 0x33);
  a = (a & 0x0F) + ((a >> 4) & 0x0F);
  d += a - 3;

  uint8_t y = (data_in >> 5) & 0b111;
  uint8_t c = y == 7;
  uint8_t i = (y << 1) + d;
  i += (y == 7 && d == 0 && (ex >> 4) == 3) << 1;
  i += (y == 7 && d == 1 && (ex >> 4) == 0) << 1;
  uint8_t ey = enclut3b4b[i];

  uint8_t b = ey & 0b1111;
  b = (b & 0x5) + ((b >> 1) & 0x5);
  b = (b & 0x3) + ((b >> 2) & 0x3);

  uint8_t o = (a + b) != 5;
  *r = (*r + o) & 1;

  return ey << 6 | ex;
}

//////////////////////////////////////////////////
// 8b/10b decoder
//////////////////////////////////////////////////

static const uint8_t declut5b6b[] = {
  0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b01111, 0b00000, 0b00111,
  0b11111, 0b10000, 0b11111, 0b01011, 0b11000, 0b01101, 0b01110, 0b11111,
  0b11111, 0b00001, 0b00010, 0b10011, 0b00100, 0b10101, 0b10110, 0b10111,
  0b01000, 0b11001, 0b11010, 0b11011, 0b11100, 0b11101, 0b11110, 0b11111,
  0b11111, 0b11110, 0b11101, 0b00011, 0b11011, 0b00101, 0b00110, 0b01000,
  0b10111, 0b01001, 0b01010, 0b00100, 0b01100, 0b00010, 0b00001, 0b11111,
  0b11111, 0b10001, 0b10010, 0b11000, 0b10100, 0b11111, 0b10000, 0b11111,
  0b00111, 0b00000, 0b01111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111,
};

static const uint8_t declut3b4b[] = {
  0b111, 0b111, 0b000, 0b011, 0b100, 0b101, 0b110, 0b111,
  0b111, 0b001, 0b010, 0b100, 0b011, 0b000, 0b111, 0b111,
};

uint8_t dec8b10b(uint16_t data_in) {
    uint8_t l6 = (data_in >> 0) & 0b111111;
    uint8_t h4 = (data_in >> 6) & 0b1111;
    uint8_t l5 = declut5b6b[l6];
    uint8_t h3 = declut3b4b[h4];
    return (h3 << 5) | l5;
}
