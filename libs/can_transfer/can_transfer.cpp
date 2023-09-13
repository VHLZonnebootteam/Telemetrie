#include "can_transfer.h"

/*
 * de begin functie initialized het object.
 * als deze functie een tweede keer gecalled word word die call genegeerd
 */

void Can_Transfer::begin(Stream *stream, Stream *debug_out /*= NULL*/)
{
    if(!did_init)
    {
        if (debug_out)
        {
            has_debug = true;
            debug_stream = debug_out;
        }
        else
        {
            has_debug = false;
        }
        _stream = stream;
        did_init = true;
    }

}

bool Can_Transfer::is_available()
{
    return frame_queue.count() > 0;
}

void Can_Transfer::send_frame(Can_Frame frame)
{
    _stream->write(HEADER_1);
    _stream->write(HEADER_2);
    _stream->write(frame.id);
    _stream->write(frame.id << 8);
    _stream->write(frame.dlc);
    for( int i=0; i < 8; i++ )
    {
        _stream->write(frame.data[i]);
    }
    _stream->write(CLOSING_BYTE);
}


// todo is naam index nodig?
// is data length niet genoeg
void Can_Transfer::update()
{
    // deze methode leest de bytes 1 voor 1 uit de serial buffer.
    while (_stream->available() > 3)   // er is een rede voor de >3 want de andere lib deed het.
    {

        if (transfer_phase ==  READING_HEADER1)
        {
            uint8_t b = _stream->read();
            if((b) == HEADER_1)
            {
                // als header byte gevonden next phase
                println_string_debug(String("header1"));
                transfer_phase = READING_HEADER2;
            }
            else
            {
                // als header bytes niet gevonden trash buffer todat je 'm wel vind
                println_string_debug(String((char)b));
                transfer_phase = TRANSFER_FAILED;
                debug.trashed_bytes++;
            }
        }
        else if (transfer_phase ==  READING_HEADER2)
        {
            if(_stream->read() == HEADER_2)
            {
                // als header byte gevonden next phase
                println_string_debug(String("header2"));
                transfer_phase = READING_MESSAGE;
            }
            else
            {
                // als header bytes niet gevonden trash buffer todat je 'm wel vind
                transfer_phase = TRANSFER_FAILED;
                debug.trashed_bytes + 2;
            }
        }
        else if (transfer_phase ==  READING_MESSAGE)
        {
            if (recieved.frame_read_idx < CAN_LEN_IDX )
            {
                // reading id
                recieved.id[recieved.frame_read_idx] = _stream->read();
            }
            else if (recieved.frame_read_idx == CAN_LEN_IDX)
            {
                // reading len
                recieved.dlc = _stream->read();
            }
            else if (recieved.frame_read_idx > CAN_LEN_IDX && recieved.frame_read_idx  < CAN_FRAME_LEN)
            {
                //reading data
                recieved.data[recieved.data_read_idx] = _stream->read();
                recieved.data_read_idx++;
            }
            else
            {
                // looks for the closing byte and if found creates struct
                if (_stream->read() == CLOSING_BYTE)
                {
                    // creates struct
                    uint16_t id;
                    id = recieved.id[0] | (uint16_t)recieved.id[1]<< 8;
                    Can_Frame ret = {id, recieved.dlc};
                    memcpy(&ret.data, &recieved.data, sizeof(ret.data));
                    if(!(frame_queue.count() < QUEUE_LEN))	// if full
                    {
                        frame_queue.pop();
                    }
                    frame_queue.push(ret);
                }
                else
                {
                    transfer_phase = TRANSFER_FAILED;
                    // transfer failed
                    //todo
                }
                //reset
                recieved.frame_read_idx = 0;
                recieved.data_read_idx = 0;
                transfer_phase = READING_HEADER1;
                return;
            }
            recieved.frame_read_idx++;


        }
        else if (transfer_phase ==  TRANSFER_FAILED)
        {
            // als de transfer mislukt
            println_string_debug(String("failed"));
            debug.failed_transfers++;
            recieved.frame_read_idx = 0;
            recieved.dlc = 0;
            recieved.data_read_idx = 0;
            transfer_phase = READING_HEADER1;
        }
    }
}

Can_Frame Can_Transfer::get_next()
{
    return frame_queue.pop();
}

/*
 * debug print functies.
 */
void Can_Transfer::println_string_debug(String str)
{
    if (debug_enabled)
    {
        str += "\n";
        uint16_t b_length = str.length() +1;
        char buf[b_length];
        str.toCharArray(buf, b_length);
        print_debug_buffer(buf);
    }


}

void Can_Transfer::println_int_debug(String name, int val)
{
    if (debug_enabled)
    {
        int b_size = name.length() + 10;
        char buffer[b_size];
        name += ":%d\n";
        sprintf(buffer, str_2_char(name), val);
        print_debug_buffer(buffer);

    }
}

void Can_Transfer::print_debug_buffer(char buffer[])
{
    if (debug_enabled)
    {
        int i = 0;
        char c = buffer[4];
        while(buffer[i] > 0)
        {
            debug_stream->write(buffer[i]);
            i++;
        }
    }
}

/*
 * convert aruduino string naar char*
 * ik weet niet zeker of dit segfaults gaat veroorzaken
 */
char* Can_Transfer::str_2_char(String str)
{
    if(str.length()!=0)
    {
        char *p = const_cast<char*>(str.c_str());
        return p;
    }
    return NULL;
}



