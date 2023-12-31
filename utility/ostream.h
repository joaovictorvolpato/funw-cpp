#pragma once

// EPOS OStream Interface

#include <stdio.h>
#include <pthread.h>


extern "C" {
    void _print_preamble();
    void _print(const char * s);
    void _print_trailler(bool error);
}

class OStream
{
public:
    struct Begl {};
    struct Endl {};
    struct Hex {};
    struct Dec {};
    struct Oct {};
    struct Bin {};
    struct Err {};

private:
	static pthread_mutex_t MutexHandle;
	static bool MutexInitialized;


public:
	OStream() : _base(10), _error(false)
	{
		if (!MutexInitialized)
		{
			pthread_mutexattr_t mutexAttr;
			pthread_mutexattr_init(&mutexAttr);
			pthread_mutexattr_settype(&mutexAttr, PTHREAD_MUTEX_RECURSIVE);
			pthread_mutex_init(&MutexHandle, &mutexAttr);
			pthread_mutexattr_destroy(&mutexAttr);
			MutexInitialized = true;
		}
	}


    OStream & operator<<(const Hex & hex) {
        _base = 16;
        return *this;
    }
    OStream & operator<<(const Dec & dec) {
        _base = 10;
        return *this;
    }
    OStream & operator<<(const Oct & oct) {
        _base = 8;
        return *this;
    }
    OStream & operator<<(const Bin & bin) {
        _base = 2;
        return *this;
    }

    OStream & operator<<(const Err & err)
    {
        _error = true;
        return *this;
    }

    OStream & operator<<(char c) {
        char buf[2];
        buf[0] = c;
        buf[1] = '\0';
        print(buf);
        return *this;
    }
    OStream & operator<<(unsigned char c) {
        return operator<<(static_cast<unsigned int>(c));
    }

    OStream & operator<<(int i) {
        char buf[64];
        buf[itoa(i, buf)] = '\0';
        print(buf);
        return *this;
    }
    OStream & operator<<(short s) {
        return operator<<(static_cast<int>(s));
    }
    OStream & operator<<(long l) {
        return operator<<(static_cast<int>(l));
    }

    OStream & operator<<(unsigned int u) {
        char buf[64];
        buf[utoa(u, buf)] = '\0';
        print(buf);
        return *this;
    }
    OStream & operator<<(unsigned short s) {
        return operator<<(static_cast<unsigned int>(s));
    }
    OStream & operator<<(unsigned long l) {
        return operator<<(static_cast<unsigned int>(l));
    }

    OStream & operator<<(long long int u) {
        char buf[64];
        buf[llitoa(u, buf)] = '\0';
        print(buf);
        return *this;
    }

    OStream & operator<<(unsigned long long int u) {
        char buf[64];
        buf[llutoa(u, buf)] = '\0';
        print(buf);
        return *this;
    }

    OStream & operator<<(const void * p) {
        char buf[64];
        buf[ptoa(p, buf)] = '\0';
        print(buf);
        return *this;
    }

    OStream & operator<<(const char * s) {
        print(s);
        return *this;
    }

    OStream & operator<<(float f) {
        if(f < 0.0001f && f > -0.0001f){
            (*this) << "0.0000";
            return *this;
        }
        if (f < 0) {
            (*this) << "-";
            f *= -1;
        }

        long long b = (long long) f;
        (*this) << b << ".";
        long long m = (long long) ((f - b)  * 10000);
        (*this) << m;
        return *this;
    }

    OStream & operator<<(double d) {
        return operator<<(static_cast<float>(d));
    }

private:
    void print(const char * s)
	{
		pthread_mutex_lock(&MutexHandle); 
		printf("%s", s);
		pthread_mutex_unlock(&MutexHandle);
	}

    int itoa(int v, char * s);
    int utoa(unsigned int v, char * s, unsigned int i = 0);
    int llitoa(long long int v, char * s);
    int llutoa(unsigned long long int v, char * s, unsigned int i = 0);
    int ptoa(const void * p, char * s);

private:
    int _base;
    volatile bool _error;

    static const char _digits[];
};

constexpr OStream::Begl begl;
constexpr OStream::Endl endl;
constexpr OStream::Hex hex;
constexpr OStream::Dec dec;
constexpr OStream::Oct oct;
constexpr OStream::Bin bin;

extern OStream kout, kerr;
