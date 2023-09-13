
#ifndef CAN_TRANSFER_H
#define CAN_TRANSFER_H


//make it a little prettier on the front end.
#define details(name) (byte*)&name,sizeof(name)

//Not neccessary, but just in case.
#if ARDUINO > 22
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include "Stream.h"
//#include "Queue.h" // is taken from https://github.com/sdesalas/Arduino-Queue.h





template<class T>
class Queue
{
private:
    int _front, _back, _count;
    T *_data;
    int _maxitems;
public:
    Queue(int maxitems = 256)
    {
        _front = 0;
        _back = 0;
        _count = 0;
        _maxitems = maxitems;
        _data = new T[maxitems + 1];
    }
    ~Queue()
    {
        delete[] _data;
    }
    inline int count();
    inline int front();
    inline int back();
    void push(const T &item);
    T peek();
    T pop();
    void clear();
};

template<class T>
inline int Queue<T>::count()
{
    return _count;
}

template<class T>
inline int Queue<T>::front()
{
    return _front;
}

template<class T>
inline int Queue<T>::back()
{
    return _back;
}

template<class T>
void Queue<T>::push(const T &item)
{
    if(_count < _maxitems)   // Drops out when full
    {
        _data[_back++]=item;
        ++_count;
        // Check wrap around
        if (_back > _maxitems)
            _back -= (_maxitems + 1);
    }
}

template<class T>
T Queue<T>::pop()
{
    if(_count <= 0) return T(); // Returns empty
    else
    {
        T result = _data[_front];
        _front++;
        --_count;
        // Check wrap around
        if (_front > _maxitems)
            _front -= (_maxitems + 1);
        return result;
    }
}

template<class T>
T Queue<T>::peek()
{
    if(_count <= 0) return T(); // Returns empty
    else return _data[_front];
}

template<class T>
void Queue<T>::clear()
{
    _front = _back;
    _count = 0;
}


const int CAN_FRAME_LEN = 11;
const int CAN_LEN_IDX = 2;

// buffer_size bepaald de maximale lengte van commandos
const uint8_t QUEUE_LEN = 50;
// dit zijn de header bytes. elk bericht moet met deze bytes beginnen
// deze codes zijn niet gebruikt in ascii
const uint8_t HEADER_1 = 140;
const uint8_t HEADER_2 = 156;
const uint8_t CLOSING_BYTE = 188;

const bool debug_enabled = false;

enum Transfer_Phase
{
    TRANSFER_FAILED,
    READING_HEADER1,
    READING_HEADER2,
    READING_MESSAGE,
};

struct Can_Frame
{
    uint16_t id;
    uint8_t dlc;// data len
    uint8_t data[8];
};

class Can_Transfer
{
public:
    Can_Transfer() {}
    void begin(Stream *stream, Stream *debug_out = NULL);

    bool is_available();

    Can_Frame get_next();

    void send_frame(Can_Frame frame);

    // leest de serial uit en update de waardes
    void update();

    struct
    {
        int failed_transfers = 0;
        int trashed_bytes = 0;
        int wrong_type = 0;
    } debug;

    void print_debug();
private:
    Stream *_stream;
    Stream *debug_stream;
    bool has_debug;
    bool did_init;
    Queue<Can_Frame> frame_queue = Queue<Can_Frame>(QUEUE_LEN);

    Transfer_Phase transfer_phase = READING_HEADER1;

    char* str_2_char(String str);
    void println_int_debug(String name, int val);
    void println_string_debug(String str);
    void print_debug_buffer(char buffer[]);

    struct
    {
        bool reading_message = false;
        uint8_t frame_read_idx;
        uint8_t id[2];
        uint8_t dlc;
        uint8_t data_read_idx = 0;
        uint8_t data[8];
    } recieved;

};


#endif
