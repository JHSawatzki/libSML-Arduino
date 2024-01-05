// Copyright 2023 Risto Koiva
//
// This file is part of libSML-Arduino.
//
// libSML is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// libSML is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with libSML.  If not, see <http://www.gnu.org/licenses/>.

#include <Arduino.h>
#include <sml_file.h>

// Adjust serial ports below according to your board - this example targets w/o mods e.g. Teensy 4.0

#define LOGSERIAL        Serial
#define LOGBAUD          115200

#define METERSERIAL      Serial1
#define METERBAUD        9600

// SML constants
const byte START_SEQUENCE[] = {0x1B, 0x1B, 0x1B, 0x1B, 0x01, 0x01, 0x01, 0x01};
const byte END_SEQUENCE[] = {0x1B, 0x1B, 0x1B, 0x1B, 0x1A};
const size_t BUFFER_SIZE = 3840; // Max datagram duration 400ms at 9600 Baud
const uint8_t READ_TIMEOUT = 5;  // in seconds

// States
enum State
{
    INIT,
    WAIT_FOR_START_SEQUENCE,
    READ_MESSAGE,
    PROCESS_MESSAGE,
    READ_CHECKSUM
};

struct MeterData
{
  double dMeterCurrW = 0;
  uint64_t ui64_lastMeasurementTime = 0;
  volatile bool bUnread = false;
};

uint64_t millis64()
{
    volatile static uint32_t low32, high32;
    uint32_t new_low32 = millis();
    if (new_low32 < low32)
        high32++;
    low32 = new_low32;
    return (uint64_t)high32 << 32 | low32;
}

class Meter
{
public:
    Meter()
    {
      this->set_state(WAIT_FOR_START_SEQUENCE);
    }

    void setup()
    {
      METERSERIAL.begin(METERBAUD);
    }

    void loop()
    {
      if (this->state != INIT)
      {
        if ((millis64() - this->last_state_reset) > (READ_TIMEOUT * 1000))
        {
          this->set_state(WAIT_FOR_START_SEQUENCE); // Reset
        }
        switch (this->state)
        {
          case WAIT_FOR_START_SEQUENCE: this->wait_for_start_sequence(); break;
          case READ_MESSAGE:            this->read_message(); break;
          case READ_CHECKSUM:           this->read_checksum(); break;
          case PROCESS_MESSAGE:         this->process_message(); break;
          default: break;
        }
      }
    }

    MeterData getdata()
	  {
	    return this->meterData;
	  }

    void clearUnreadFlag()
    {
      this->meterData.bUnread = false;
    }

private:
    byte buffer[BUFFER_SIZE];
    size_t position = 0;
    uint64_t last_state_reset = 0;
    uint8_t bytes_until_checksum = 0;
    State state = INIT;
    MeterData meterData;

    // Set state
    void set_state(State new_state)
    {
        if (new_state == WAIT_FOR_START_SEQUENCE)
        {
            this->last_state_reset = millis64();
            this->position = 0;
        }
        else if (new_state == READ_CHECKSUM)
        {
            this->bytes_until_checksum = 3;
        }
        this->state = new_state;
    }

    // Wait for the start_sequence to appear
    void wait_for_start_sequence()
    {
        while (METERSERIAL.available())
        {
            this->buffer[this->position] = METERSERIAL.read();
            //yield();

            this->position = (this->buffer[this->position] == START_SEQUENCE[this->position]) ? (this->position + 1) : 0;
            if (this->position == sizeof(START_SEQUENCE))
            {
              // Start sequence has been found
              this->set_state(READ_MESSAGE);
              return;
            }
        }
    }

    // Read the rest of the message
    void read_message()
    {
        while (METERSERIAL.available())
        {
            // Check whether the buffer is still big enough to hold the number of fill bytes (1 byte) and the checksum (2 bytes)
            if ((this->position + 3) == BUFFER_SIZE)
            {
                this->set_state(WAIT_FOR_START_SEQUENCE); // Buffer will overflow, starting over
                return;
            }
            this->buffer[this->position++] = METERSERIAL.read();

            // Check for end sequence
            int last_index_of_end_seq = sizeof(END_SEQUENCE) - 1;
            for (int i = 0; i <= last_index_of_end_seq; i++)
            {
                if (END_SEQUENCE[last_index_of_end_seq - i] != this->buffer[this->position - (i + 1)])
                {
                    break;
                }
                if (i == last_index_of_end_seq)
                {
                    this->set_state(READ_CHECKSUM);
                    return;
                }
            }
        }
    }

    // Read the number of fillbytes and the checksum
    void read_checksum()
    {
        while (this->bytes_until_checksum > 0 && METERSERIAL.available())
        {
            this->buffer[this->position++] = METERSERIAL.read();
            this->bytes_until_checksum--;
        }

        if (this->bytes_until_checksum == 0)
        {
            this->set_state(PROCESS_MESSAGE);
        }
    }

    void GetSumActivePower(sml_file *file)
    {
      for (int i = 0; i < file->messages_len; i++)
      {
        sml_message *message = file->messages[i];
        if (*message->message_body->tag == SML_MESSAGE_GET_LIST_RESPONSE)
        {
          sml_list *entry;
          sml_get_list_response *body;
          body = (sml_get_list_response *)message->message_body->data;
          for (entry = body->val_list; entry != NULL; entry = entry->next)
          {
            if (!entry->value)
            { // do not crash on null value
              continue;
            }

            // Look for instantaneous power 1-0:16.7.0/255
            if (entry->obj_name->str[0] == 1 &&
  					    entry->obj_name->str[1] == 0 &&
  					    entry->obj_name->str[2] == 16 &&
  					    entry->obj_name->str[3] == 7 &&
  					    entry->obj_name->str[4] == 0 &&
  					    entry->obj_name->str[5] == 255)
  			    {
              if (((entry->value->type & SML_TYPE_FIELD) == SML_TYPE_INTEGER) ||
                  ((entry->value->type & SML_TYPE_FIELD) == SML_TYPE_UNSIGNED))
              {
                double dValue = sml_value_to_double(entry->value);
                int32_t i32Scaler = (entry->scaler) ? *entry->scaler : 0;
                int32_t i32Prec = -i32Scaler;
                if (i32Prec < 0)
                  i32Prec = 0;
                this->meterData.dMeterCurrW = dValue * pow(10, i32Scaler);
                this->meterData.ui64_lastMeasurementTime = millis64();
                this->meterData.bUnread = true;
              }
            }
          }
        }
      }
    }    

    void process_message()
    {
      sml_file *file = sml_file_parse(this->buffer + 8, this->position - 16);
      this->GetSumActivePower(file);
      sml_file_free(file);
      this->set_state(WAIT_FOR_START_SEQUENCE); // Start over
    }
};

Meter my_meter;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  LOGSERIAL.begin(LOGBAUD);
  METERSERIAL.begin(METERBAUD);
}

void loop() {
  static uint64_t ui64LastErrorTime = 0;  
  my_meter.loop();
  yield();

  if (millis64() < (my_meter.getdata().ui64_lastMeasurementTime + (READ_TIMEOUT * 1000)))
  {
    if (my_meter.getdata().bUnread)
    {
      LOGSERIAL.print("instantaneous power: ");
      LOGSERIAL.print(my_meter.getdata().dMeterCurrW);
      LOGSERIAL.println(" Watt");
      my_meter.clearUnreadFlag();
    }
  }
  else
  {
    // Timeout getting meter data

    // Limit outputting to every 5 seconds only
    if (millis64() > (ui64LastErrorTime + (READ_TIMEOUT * 1000)))
    {
      ui64LastErrorTime = millis64();
      LOGSERIAL.println("Timeout getting SML data!");
    }
  }
}
