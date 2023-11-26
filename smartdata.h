
#pragma once
// EPOS SmartData Declarations
//
// Smart Data encapsulates Transducers (i.e. sensors and actuators), local or remote, and bridges them with Network
// Transducers must be Observed objects, must implement either sense() or actuate(), and must define UNIT, NUM, and UNCERTAINTY.

#include "meta.h"
#include "system/types.h"
#include "utility/geometry.h"
#include "utility/observer.h"
#include "utility/predictor.h"
#include <tuple>
#include "system/types.h"

#define __UTIL

// Scale for Geographic Space used by communication protocols (applications always get CM_32)
namespace Space_Time {
    enum Scale {
        _NULL    = 0,
        CMx50_8 = 1,
        CM_16   = 2,
        CM_32   = 3
    };

    template<Scale S>
    struct Select_Scale {
        using Number          = typename SWITCH<S, CASE<_NULL,  Int8, CASE<CMx50_8,  Int8, CASE<CM_16,  Int16, CASE<CM_32,  Int32>>>>>::Result;
        using Unsigned_Number = typename SWITCH<S, CASE<_NULL, UInt8, CASE<CMx50_8, UInt8, CASE<CM_16, UInt16, CASE<CM_32, UInt32>>>>>::Result;

        static const unsigned int PADDING = (S == CMx50_8) ? 8 : (S == CM_16) ? 16 : 0;
    };

    // Scalable Geographic Space (expressed in m from the center of the Earth; compressible by communication protocols; see https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system)
    template<Scale S>
    class _Space: public Point<typename Select_Scale<S>::Number, 3>, private Padding<Select_Scale<S>::PADDING>
    {
    public:
        using Number = typename Select_Scale<S>::Number;
        using Unsigned_Number = typename Select_Scale<S>::Unsigned_Number;

        static const Number UNKNOWN = 1 << (sizeof(Number) * 8 - 1);

    public:
        _Space() {}
        _Space(const Number & x, const Number & y, const Number & z): Point<Number, 3>(x, y, z) {}
        _Space(const Point<Number, 3> & p): Point<Number, 3>(p.x(), p.y(), p.z()) {}

        operator _Space<_NULL>() const;
        operator _Space<CMx50_8>() const;
        operator _Space<CM_16>() const;
        operator _Space<CM_32>() const;
    } __attribute__((packed));

    template<> class _Space<_NULL>
    {
    public:
        using Number = typename Select_Scale<_NULL>::Number;
        using Distance = typename Select_Scale<_NULL>::Unsigned_Number;

        static const Number UNKNOWN = 1 << (sizeof(Number) * 8 - 1);

    public:
        _Space() {}
        _Space(const Number & x, const Number & y, const Number & z) {}
        _Space(const Point<Number, 3> & p) {}

        Number x() const { return 0; }
        Number y() const { return 0; }
        Number z() const { return 0; }

        operator _Space<CMx50_8>() const { return _Space<CMx50_8>(0, 0, 0); };
        operator _Space<CM_16>()   const { return _Space<CM_16>(0, 0, 0); };
        operator _Space<CM_32>()   const { return _Space<CM_32>(0, 0, 0); };

        bool operator==(const _Space<_NULL> & s) const { return true; }
        bool operator!=(const _Space<_NULL> & s) const { return false; }

        Distance operator+(const _Space<_NULL> & s) const { return 0; }
        Distance operator-(const _Space<_NULL> & s) const { return 0; }
        _Space<_NULL> & operator+=(const _Space<_NULL> & p) { return *this; }
        _Space<_NULL> & operator-=(const _Space<_NULL> & p) { return *this; }

        static _Space<_NULL> trilaterate(const _Space<_NULL> & s1, const Distance & d1,  const _Space<_NULL> & s2, const Distance & d2, const _Space<_NULL> & s3, const Distance & d3) { return _Space<_NULL>(0, 0, 0); }

        friend OStream & operator<<(OStream & os, const _Space<_NULL> & s) { return os; }

    } __attribute__((packed));

    template<> inline _Space<CMx50_8>::operator _Space<_NULL>()    const { return _Space<_NULL>   (Point<Number, 3>::x() * 50, Point<Number, 3>::y() * 50, Point<Number, 3>::z() * 50); }
    template<> inline _Space<CMx50_8>::operator _Space<CM_16>()   const { return _Space<CM_16>  (Point<Number, 3>::x() * 50, Point<Number, 3>::y() * 50, Point<Number, 3>::z() * 50); }
    template<> inline _Space<CMx50_8>::operator _Space<CM_32>()   const { return _Space<CM_32>  (Point<Number, 3>::x() * 50, Point<Number, 3>::y() * 50, Point<Number, 3>::z() * 50); }

    template<> inline _Space<CM_16>::operator   _Space<_NULL>()    const { return _Space<_NULL>   (Point<Number, 3>::x(),      Point<Number, 3>::y(),      Point<Number, 3>::z()); }
    template<> inline _Space<CM_16>::operator   _Space<CMx50_8>() const { return _Space<CMx50_8>(Point<Number, 3>::x() / 50, Point<Number, 3>::y() / 50, Point<Number, 3>::z() / 50); }
    template<> inline _Space<CM_16>::operator   _Space<CM_32>()   const { return _Space<CM_32>  (Point<Number, 3>::x(),      Point<Number, 3>::y(),      Point<Number, 3>::z()); }

    template<> inline _Space<CM_32>::operator   _Space<_NULL>()    const { return _Space<_NULL>   (Point<Number, 3>::x(),      Point<Number, 3>::y(),      Point<Number, 3>::z()); }
    template<> inline _Space<CM_32>::operator   _Space<CMx50_8>() const { return _Space<CMx50_8>(Point<Number, 3>::x() / 50, Point<Number, 3>::y() / 50, Point<Number, 3>::z() / 50); }
    template<> inline _Space<CM_32>::operator   _Space<CM_16>()   const { return _Space<CM_16>  (Point<Number, 3>::x(),      Point<Number, 3>::y(),      Point<Number, 3>::z()); }

    // Time (expressed in us)
    class Time
    {
    public:
        typedef Int64 Type;

    public:
        Time() {};
        Time(const Infinity &): _time(INFINITE) {};
        Time(const Type & time): _time(time) {};

        void operator=(const Type & time) { _time = time; }
        void operator=(const Type & time) volatile { _time = time; }

        operator Type() { return _time; }
        operator Type() const { return _time; }
        operator Type() volatile { return _time; }

    private:
        Type _time;
    } __attribute__((packed));

    class Short_Time
    {
    public:
        typedef Int32 Type;

    public:
        Short_Time() {};
        Short_Time(const Infinity &): _time(INFINITE) {};
        Short_Time(const Type & time): _time(time) {};

        void operator=(const Type & time) { _time = time; }
        void operator=(const Type & time) volatile { _time = time; }

        operator Type() { return _time; }
        operator Type() const { return _time; }
        operator Type() volatile { return _time; }

    private:
        Type _time;
    } __attribute__((packed));
    typedef Short_Time Time_Offset;

    struct Time_Interval
    {
        Time_Interval(const Time & begin, const Time & end): t0(begin), t1(end) {}

        bool contains(const Time & t) const { return (t >= t0) && (t <= t1); }

        Time t0;
        Time t1;
    } __attribute__((packed));

    template<Scale S>
    struct _Spacetime
    {
        using Space = _Space<S>;
        using Number = typename Space::Number;

        _Spacetime() {}
        _Spacetime(const Number & x, const Number & y, const Number & z, const Time & _t): space(x, y, z), time(_t) {}
        _Spacetime(const Space & s, const Time & t): space(s), time(t) {}

        _Spacetime & operator=(const Space & s) { space = s; return *this; }
        _Spacetime & operator=(const Time & t) { time = t; return *this; }

        _Spacetime & operator+(const Space & s) { space += s; return *this; }
        _Spacetime & operator+(const Time & t) { time = time + t; return *this; }

        operator Space() const { return const_cast<const Space &>(space); }
        operator Time() const { return const_cast<const Time &>(time); }

        friend OStream & operator<<(OStream & os, const _Spacetime & st) {
            os << "{" << st.space << ",t=" << st.time << "}";
            return os;
        }

        Space space;
        Time time;
    } __attribute__((packed));

    template<Scale S>
    class _Region: public Sphere<typename Select_Scale<S>::Number, typename Select_Scale<S>::Unsigned_Number>, public Time_Interval // no need for padding: 3x8+8 | 3x16+16 | 3x32+32
    {
    public:
        using _Sphere = Sphere<typename Select_Scale<S>::Number, typename Select_Scale<S>::Unsigned_Number>;
        using typename _Sphere::Number;
        using typename _Sphere::Center;
        using typename _Sphere::Radius;

        _Region() {}
        _Region(const Number & x, const Number & y, const Number & z, const Radius & r, const Time & t0, const Time & t1)
                : _Sphere(Center(x, y, z), r), Time_Interval(t0, t1) {}
        _Region(const Center & c, const Radius & r, const Time & t0, const Time & t1)
                : _Sphere(c, r), Time_Interval(t0, t1) {}
        _Region(const _Spacetime<S> & st, const Radius & r, const Time & t1)
                : _Sphere(Space(st), r), Time_Interval(st, t1) {}

        operator _Spacetime<S>() const { return _Spacetime<S>(this->center(), t0); }

        bool operator==(const _Region & r) const { return !memcmp(this, &r, sizeof(_Region)); }
        bool operator!=(const _Region & r) const { return !(*this == r); }

        bool contains(const _Spacetime<S> & st) const {
            return Time_Interval::contains(st.time) && _Sphere::contains(st.space);
        }
        bool contains(const Center & c, const Time & t) const {
            return Time_Interval::contains(t) && _Sphere::contains(c);
        }

        friend OStream & operator<<(OStream & os, const _Region & r) {
            os << "{" << reinterpret_cast<const _Sphere &>(r) << ",t0=" << r.t0 << ",t1=" << r.t1 << "}";
            return os;
        }
    } __attribute__((packed));

    template<> class _Region<_NULL> : public Time_Interval // no need for padding: 3x8+8 | 3x16+16 | 3x32+32
    {
    public:
        using Number = typename Select_Scale<_NULL>::Number;
        using Radius = typename Select_Scale<_NULL>::Unsigned_Number;
        using Center = Point<Select_Scale<_NULL>::Number, 3>;

        _Region() : Time_Interval(0, 0) {}
        _Region(const Number & x, const Number & y, const Number & z, const Radius & r, const Time & t0, const Time & t1)
                : Time_Interval(t0, t1) {}
        _Region(const Center & c, const Radius & r, const Time & t0, const Time & t1)
                : Time_Interval(t0, t1) {}
        _Region(const _Spacetime<_NULL> & st, const Radius & r, const Time & t1)
                : Time_Interval(st, t1) {}

        operator _Spacetime<_NULL>() const { return _Spacetime<_NULL>(Center(0, 0, 0), t0); }

        bool operator==(const _Region & r) const { return r.t0 == t0 && r.t1 == t1; }
        bool operator!=(const _Region & r) const { return !(*this == r); }

        bool contains(const _Spacetime<_NULL> & st) const {
            return Time_Interval::contains(st.time);
        }
        bool contains(const Center & c, const Time & t) const {
            return Time_Interval::contains(t);
        }

        Radius radius() const { return 0; }
        Center center() const { return Center(0, 0, 0); }

        friend OStream & operator<<(OStream & os, const _Region & r) {
            os << "{t0=" << r.t0 << ",t1=" << r.t1 << "}";
            return os;
        }
    } __attribute__((packed));
};

// SmartData basic definitions used also by the associated communication protocols
class SmartData {
protected:
    static const unsigned int PAN = 10; // Nodes
    static const unsigned int LAN = 100; // Nodes
    static const unsigned int WAN = 10000; // Nodes
    //static const unsigned int NODES = Traits<Build>::NODES;
    //static const Space_Time::Scale SCALE = Traits<Network>::routing ? ((Traits<Build>::NODES <= PAN) ? Space_Time::CMx50_8 : (Traits<Build>::NODES <= WAN) ? Space_Time::CM_16 : Space_Time::CM_32) : Space_Time::_NULL;

public:
    using Time = Space_Time::Time;
    using Short_Time = Space_Time::Short_Time;
    using Time_Interval = Space_Time::Time_Interval;
    using Time_Offset = Space_Time::Time_Offset;
    using Scale = Space_Time::Scale;

    template<UInt32 LEN>
    struct Digital {
        Digital() {}

        Digital(unsigned int v) { memset(_data, v, LEN); }

        template<typename T>
        void operator+=(const T data) {
            for (unsigned int i = 0; i < ((sizeof(T) > LEN) ? LEN : sizeof(T)); i++)
                _data[i] += data[i];
        }

        unsigned char &operator[](const unsigned int i) { return _data[i]; }

        const unsigned char &operator[](const unsigned int i) const { return _data[i]; }

        operator const unsigned char *() const { return _data; }

        operator unsigned char *() { return _data; }

        unsigned char _data[LEN];
    };

    // SI Unit defining the SmartData semantics (inspired by IEEE 1451 TEDs)
    class Unit {
    public:
        // Formats
        // Bit       31                                 16                                     0
        //         +--+----------------------------------+-------------------------------------+
        // Digital |0 | type                             | len                                 |
        //         +--+----------------------------------+-------------------------------------+

        // Bit       31   29   27     24     21     18     15     12      9      6      3      0
        //         +--+----+----+------+------+------+------+------+------+------+------+------+
        // SI      |1 |NUM |MOD |sr+4  |rad+4 |m+4   |kg+4  |s+4   |A+4   |K+4   |mol+4 |cd+4  |
        //         +--+----+----+------+------+------+------+------+------+------+------+------+
        // Bits     1   2    2     3      3      3      3      3      3      3      3      3

        // Valid values for field SI
        enum : UInt32 {
            DIGITAL = 0U
                    << 31, // The Unit is plain digital data. Subsequent 15 bits designate the data type. Lower 16 bits are application-specific, usually a device selector.
            SI = 1U << 31, // The Unit is SI. Remaining bits are interpreted as specified here.
            SID = SI
        };

        // Valid values for field NUM
        enum : UInt32 {
            I32 =
            0 << 29, // Value is an integral int stored in the 32 last significant bits of a 32-bit big-endian integer.
            I64 =
            1 << 29, // Value is an integral int stored in the 64 last significant bits of a 64-bit big-endian integer.
            F32 = 2 << 29, // Value is a real int stored as an IEEE 754 binary32 big-endian floating point.
            D64 =
            3 << 29, // Value is a real int stored as an IEEE 754 binary64 big-endian double precision floating point.
            NUM = D64      // AND mask to select NUM bits
        };

        // Valid values for field MOD
        enum : UInt32 {
            DIR = 0
                    << 27, // Unit is described by the product of SI base units raised to the powers recorded in the remaining fields.
            DIV = 1
                    << 27, // Unit is U/U, where U is described by the product SI base units raised to the powers recorded in the remaining fields.
            LOG = 2
                    << 27, // Unit is log_e(U), where U is described by the product of SI base units raised to the powers recorded in the remaining fields.
            LOG_DIV = 3
                    << 27, // Unit is log_e(U/U), where U is described by the product of SI base units raised to the powers recorded in the remaining fields.
            MOD = LOG_DIV  // AND mask to select MOD bits
        };

        // Masks to select the SI units
        enum : UInt32 {
            SR = 7 << 24,
            RAD = 7 << 21,
            M = 7 << 18,
            KG = 7 << 15,
            S = 7 << 12,
            A = 7 << 9,
            K = 7 << 6,
            MOL = 7 << 3,
            CD = 7 << 0
        };

        // Mask to select field LEN of digital data
        enum : UInt32 {
            LEN = (1 << 16) - 1
        };

        // Helper to create digital units
        template<UInt32 _TYPE, UInt32 _SUBTYPE, UInt32 _LEN>
        class Digital_Unit {
        public:
            enum : UInt32 {
                UNIT = DIGITAL | _TYPE << 24 | _SUBTYPE << 16 | _LEN << 0
            };

        private:
            // Compile-time verifications
            static const typename IF<(_TYPE & (~((1 << 23) - 1))), void, bool>::Result Invalid_TYPE = false;
            static const typename IF<(_SUBTYPE & (~((1 << 15) - 1))), void, bool>::Result Invalid_SUBTYPE = false;
            static const typename IF<(_LEN & (~LEN)), void, bool>::Result Invalid_LEN = false;
        };

        // Helper to create SI units
        template<int _MOD, int _SR, int _RAD, int _M, int _KG, int _S, int _A, int _K, int _MOL, int _CD>
        class SI_Unit {
        public:
            //                     SI  |  MOD  |        sr       |         rad      |        m       |        kg       |        s       |        A      |        K      |       mol       |     cd
            enum : UInt32 {
                UNIT =
                SI | _MOD | (4 + _SR) << 24 | (4 + _RAD) << 21 | (4 + _M) << 18 | (4 + _KG) << 15 | (4 + _S) << 12 |
                (4 + _A) << 9 | (4 + _K) << 6 | (4 + _MOL) << 3 | (4 + _CD)
            };

        private:
            // Compile-time verifications
            static const typename IF<(_MOD & (~MOD)), void, bool>::Result Invalid_MOD = false;
            static const typename IF<((_SR + 4) & (~7u)), void, bool>::Result Invalid_SR = false;
            static const typename IF<((_RAD + 4) & (~7u)), void, bool>::Result Invalid_RAD = false;
            static const typename IF<((_M + 4) & (~7u)), void, bool>::Result Invalid_M = false;
            static const typename IF<((_KG + 4) & (~7u)), void, bool>::Result Invalid_KG = false;
            static const typename IF<((_S + 4) & (~7u)), void, bool>::Result Invalid_S = false;
            static const typename IF<((_A + 4) & (~7u)), void, bool>::Result Invalid_A = false;
            static const typename IF<((_K + 4) & (~7u)), void, bool>::Result Invalid_K = false;
            static const typename IF<((_MOL + 4) & (~7u)), void, bool>::Result Invalid_MOL = false;
            static const typename IF<((_CD + 4) & (~7u)), void, bool>::Result Invalid_CD = false;
        };

        // Typical SI Quantities
        enum Quantity : UInt32 {
            //                                mod,     sr,    rad,      m,     kg,      s,      A,      K,    mol,     cd            unit
            Acceleration = SI_Unit<DIR, +0, +0, +1, +0, -2, +0, +0, +0, +0>::UNIT, // m/s2
            Angle = SI_Unit<DIR, +0, +1, +0, +0, +0, +0, +0, +0, +0>::UNIT, // rad
            Amount_of_Substance = SI_Unit<DIR, +0, +0, +0, +0, +0, +0, +0, +1, +0>::UNIT, // mol
            Angular_Velocity = SI_Unit<DIR, +0, +1, +0, +0, -1, +0, +0, +0, +0>::UNIT, // rad/s
            Area = SI_Unit<DIR, +0, +0, +2, +0, +0, +0, +0, +0, +0>::UNIT, // m2
            Current = SI_Unit<DIR, +0, +0, +0, +0, +0, +1, +0, +0, +0>::UNIT, // Ampere
            Electric_Current = Current,
            Force = SI_Unit<DIR, +0, +0, +1, +1, -2, +0, +0, +0, +0>::UNIT, // Newton
            Humidity = SI_Unit<DIR, +0, +0, -3, +1, +0, +0, +0, +0, +0>::UNIT, // kg/m3
            Length = SI_Unit<DIR, +0, +0, +1, +0, +0, +0, +0, +0, +0>::UNIT, // m
            Luminous_Intensity = SI_Unit<DIR, +0, +0, +0, +0, +0, +0, +0, +0, +1>::UNIT, // cd
            Mass = SI_Unit<DIR, +0, +0, +0, +1, +0, +0, +0, +0, +0>::UNIT, // kg
            Power = SI_Unit<DIR, +0, +0, +2, +1, -3, +0, +0, +0, +0>::UNIT, // Watt
            Pressure = SI_Unit<DIR, +0, +0, -1, +1, -2, +0, +0, +0, +0>::UNIT, // Pascal
            Velocity = SI_Unit<DIR, +0, +0, +1, +0, -1, +0, +0, +0, +0>::UNIT, // m/s
            Sound_Intensity = SI_Unit<DIR, +0, +0, +0, +1, -3, +0, +0, +0, +0>::UNIT, // W/m2
            Temperature = SI_Unit<DIR, +0, +0, +0, +0, +0, +0, +1, +0, +0>::UNIT, // Kelvin
            Time = SI_Unit<DIR, +0, +0, +0, +0, +1, +0, +0, +0, +0>::UNIT, // s
            Speed = Velocity,
            Volume = SI_Unit<DIR, +0, +0, +3, +0, +0, +0, +0, +0, +0>::UNIT, // m3
            Voltage = SI_Unit<DIR, +0, +0, +2, +1, -3, -1, +0, +0, +0>::UNIT, // Volt
            Water_Flow = SI_Unit<DIR, +0, +0, +3, +0, -1, +0, +0, +0, +0>::UNIT, // m3/s
            Frequency = SI_Unit<DIR, +0, +0, +0, +0, -1, +0, +0, +0, +0>::UNIT, // Hz

            Ratio = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, -4>::UNIT, // not an SI unit
            Percent = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, -3>::UNIT, // not an SI unit, a ratio < 1
            PPM = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, -2>::UNIT, // not an SI unit, a ratio in parts per million
            PPB = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, -1>::UNIT, // not an SI unit, a ratio in parts per billion
            Relative_Humidity = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, +0>::UNIT, // not an SI unit, a percentage representing the partial pressure of water vapor in the mixture to the equilibrium vapor pressure of water over a flat surface of pure water at a given temperature
            Power_Factor = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, +1>::UNIT, // not an SI unit, a ratio of the real power absorbed by the load to the apparent power flowing in the circuit; a dimensionless number in [-1,1]
            Counter = SI_Unit<LOG_DIV, -4, -4, -4, -4, -4, -4, -4, -4, +2>::UNIT, // not an SI unit, the current value of an external counter
            Antigravity = SI_Unit<LOG_DIV, +3, +3, +3, +3, +3, +3, +3, +3, +3>::UNIT  // for Dummy_Transducer :-)
        };

        // Digital data types
        enum Digital_Data : UInt32 {
            //                                         type, subtype,   length
            // Switches
            Direction = Digital_Unit<0, 1, 1>::UNIT,
            Switch = Digital_Unit<0, 0, 1>::UNIT,
            On_Off = Switch,

            // RFIDs and SmartCartds
            RFID32 = Digital_Unit<1, 0, 4>::UNIT,

            // Audio and Video (from RTP)                                       A/V         Hz        Ch       Ref
            PCMU = Digital_Unit<2, 0, 0>::UNIT, // A         8000        1       [RFC3551]
            GSM = Digital_Unit<2, 3, 0>::UNIT, // A         8000        1       [RFC3551]
            G723 = Digital_Unit<2, 4, 0>::UNIT, // A         8000        1       [Vineet_Kumar][RFC3551]
            DVI4_8 = Digital_Unit<2, 5, 0>::UNIT, // A         8000        1       [RFC3551]
            DVI4_16 = Digital_Unit<2, 6, 0>::UNIT, // A        16000        1       [RFC3551]
            LPC = Digital_Unit<2, 7, 0>::UNIT, // A         8000        1       [RFC3551]
            PCMA = Digital_Unit<2, 8, 0>::UNIT, // A         8000        1       [RFC3551]
            G722 = Digital_Unit<2, 9, 0>::UNIT, // A         8000        1       [RFC3551]
            L16_2 = Digital_Unit<2, 10, 0>::UNIT, // A        44100        2       [RFC3551]
            L16_1 = Digital_Unit<2, 11, 0>::UNIT, // A        44100        1       [RFC3551]
            QCELP = Digital_Unit<2, 12, 0>::UNIT, // A         8000        1       [RFC3551]
            CN = Digital_Unit<2, 13, 0>::UNIT, // A         8000        1       [RFC3389]
            MPA = Digital_Unit<2, 14, 0>::UNIT, // A        90000                [RFC3551][RFC2250]
            G728 = Digital_Unit<2, 15, 0>::UNIT, // A         8000        1       [RFC3551]
            DVI4_11 = Digital_Unit<2, 16, 0>::UNIT, // A        11025        1       [Joseph_Di_Pol]
            DVI4_22 = Digital_Unit<2, 17, 0>::UNIT, // A        22050        1       [Joseph_Di_Pol]
            G729 = Digital_Unit<2, 18, 0>::UNIT, // A         8000        1       [RFC3551]
            CelB = Digital_Unit<2, 25, 0>::UNIT, // V        90000                [RFC2029]
            JPEG = Digital_Unit<2, 26, 0>::UNIT, // V        90000                [RFC2435]
            nv = Digital_Unit<2, 28, 0>::UNIT, // V        90000                [RFC3551]
            H261 = Digital_Unit<2, 31, 0>::UNIT, // V        90000                [RFC4587]
            MPV = Digital_Unit<2, 32, 0>::UNIT, // V        90000                [RFC2250]
            MP2T = Digital_Unit<2, 33, 0>::UNIT, // AV       90000                [RFC2250]
            H263 = Digital_Unit<2, 34, 0>::UNIT, // V        90000                [Chunrong_Zhu]
            H264 = Digital_Unit<2, 35, 0>::UNIT, // V        90000                [Chunrong_Zhu]
            MPEG2TS = Digital_Unit<2, 36, 0>::UNIT, // V        90000                [Chunrong_Zhu]

            //AV_IMAGE                = Digital_Unit<       2,      36, Traits<AV_AMPERA_Project>::CAMERA_Pixels>::UNIT,
            //CARLA_IMAGE             = JPEG,
            //CLOUD_POINTS_LIDAR      = Digital_Unit<       5,       0, Traits<AV_AMPERA_Project>::LIDAR_Points>::UNIT,
            //CLOUD_POINTS_RADAR      = Digital_Unit<       5,       0, Traits<AV_AMPERA_Project>::RADAR_Points>::UNIT,

            MOTION_VECTOR_GLOBAL = Digital_Unit<((1 << 30) >> 24), 0, 0>::UNIT,
            MOTION_VECTOR_LOCAL = Digital_Unit<((1 << 30) >> 24) | 1, 0, 0>::UNIT,
            AV_DYNAMICS_STATE = Digital_Unit<((1 << 30) >> 24) | 2, 0, 0>::UNIT,
            RSS = Digital_Unit<7, 0, 1>::UNIT,
            // experimenting -- Deprecated (do not use)
            GPS3I = Digital_Unit<3, 0, 3 * 4>::UNIT,
            Dynamics = Digital_Unit<3, 1, 11 * 4 +
                                          1>::UNIT,       // 1 vehicle dynamics (x, y, z, yaw, speed, steer, is-vehicle, bounding box(x,y,z),id)
            Dynamics_Array = Digital_Unit<3, 2, 1 + (11 * 4 + 1) *
                                                    150>::UNIT,  // 1 char for counter of vehicles + up to 150 vehicle dynamics (x, y, z, yaw, speed, steer, is-vehicle, bounding box(x,y,z),id)
            ROUTE_POINTS = Digital_Unit<3, 2, 3 * 4 * 100>::UNIT
        };

        // SI Factors
        typedef char Factor;
        enum {
            // Name           Code         Symbol    Factor
            ATTO = (8 - 8), //     a       0.000000000000000001
            FEMTO = (8 - 7), //     f       0.000000000000001
            PICO = (8 - 6), //     p       0.000000000001
            NANO = (8 - 5), //     n       0.000000001
            MICRO = (8 - 4), //     μ       0.000001
            MILI = (8 - 3), //     m       0.001
            CENTI = (8 - 2), //     c       0.01
            DECI = (8 - 1), //     d       0.1
            NONE = (8), //     -       1
            DECA = (8 + 1), //     da      10
            HECTO = (8 + 2), //     h       100
            KILO = (8 + 3), //     k       1000
            MEGA = (8 + 4), //     M       1000000
            GIGA = (8 + 5), //     G       1000000000
            TERA = (8 + 6), //     T       1000000000000
            PETA = (8 + 7)  //     P       1000000000000000
        };

        template<UInt32 UNIT>
        struct Get {
            static const size_t UNKNOWN = 1;
            static const size_t DIGITAL_LEN = (UNIT & SID) == SI ? 0 :
                                              (UNIT >> 16) == (MOTION_VECTOR_GLOBAL >> 16) ? (UNIT & LEN) *
                                                                                             ((3 * 8 + 4 * 4 + 3 * 4) +
                                                                                              2 * 4 + 8) : (
                                                      (UNIT >> 16) == (MOTION_VECTOR_LOCAL >> 16) ? (UNIT & LEN) *
                                                                                                    ((2 * 4 + 4 * 4 +
                                                                                                      3 * 4) + 2 * 4 +
                                                                                                     8) : (
                                                              (UNIT >> 16) == (AV_DYNAMICS_STATE >> 16) ? (UNIT & LEN) *
                                                                                                          (3 * 8 +
                                                                                                           6 * 4 + 1) :
                                                              // To be filled when used: UNIT & LEN can yield a config, therefore, extend the if to include all configurations as the one below:
                                                              (UNIT >> 16) == (PCMU >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (GSM >> 16) ? ((UNIT & LEN) == 1 ? UNKNOWN
                                                                                                               : UNIT &
                                                                                                                 LEN) :
                                                              (UNIT >> 16) == (G723 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (DVI4_8 >> 16) ? ((UNIT & LEN) == 1
                                                                                                ? UNKNOWN : UNIT & LEN)
                                                                                             :
                                                              (UNIT >> 16) == (DVI4_16 >> 16) ? ((UNIT & LEN) == 1
                                                                                                 ? UNKNOWN : UNIT & LEN)
                                                                                              :
                                                              (UNIT >> 16) == (LPC >> 16) ? ((UNIT & LEN) == 1 ? UNKNOWN
                                                                                                               : UNIT &
                                                                                                                 LEN) :
                                                              (UNIT >> 16) == (PCMA >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (G722 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (L16_2 >> 16) ? ((UNIT & LEN) == 1
                                                                                               ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (L16_1 >> 16) ? ((UNIT & LEN) == 1
                                                                                               ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (QCELP >> 16) ? ((UNIT & LEN) == 1
                                                                                               ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (CN >> 16) ? ((UNIT & LEN) == 1 ? UNKNOWN
                                                                                                              : UNIT &
                                                                                                                LEN) :
                                                              (UNIT >> 16) == (MPA >> 16) ? ((UNIT & LEN) == 1 ? UNKNOWN
                                                                                                               : UNIT &
                                                                                                                 LEN) :
                                                              (UNIT >> 16) == (G728 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (DVI4_11 >> 16) ? ((UNIT & LEN) == 1
                                                                                                 ? UNKNOWN : UNIT & LEN)
                                                                                              :
                                                              (UNIT >> 16) == (DVI4_22 >> 16) ? ((UNIT & LEN) == 1
                                                                                                 ? UNKNOWN : UNIT & LEN)
                                                                                              :
                                                              (UNIT >> 16) == (G729 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (CelB >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (JPEG >> 16) ? ((UNIT & LEN) == 1 ? 61440
                                                                                                                : UNIT &
                                                                                                                  LEN) :
                                                              (UNIT >> 16) == (nv >> 16) ? ((UNIT & LEN) == 1 ? UNKNOWN
                                                                                                              : UNIT &
                                                                                                                LEN) :
                                                              (UNIT >> 16) == (H261 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (MPV >> 16) ? ((UNIT & LEN) == 1 ? UNKNOWN
                                                                                                               : UNIT &
                                                                                                                 LEN) :
                                                              (UNIT >> 16) == (MP2T >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (H263 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (H264 >> 16) ? ((UNIT & LEN) == 1
                                                                                              ? UNKNOWN : UNIT & LEN) :
                                                              (UNIT >> 16) == (MPEG2TS >> 16) ? ((UNIT & LEN) == 1
                                                                                                 ? UNKNOWN : UNIT & LEN)
                                                                                              :
                                                              UNIT & LEN
                                                      ));

            typedef typename IF<((UNIT & SID) == SI) && ((UNIT & NUM) == I32), Int32,
                    typename IF<((UNIT & SID) == SI) && ((UNIT & NUM) == I64), Int64,
                            typename IF<((UNIT & SID) == SI) && ((UNIT & NUM) == F32), Float32,
                                    typename IF<((UNIT & SID) == SI) && ((UNIT & NUM) == D64), Float64,
                                            typename IF<((UNIT & SID) == DIGITAL), struct Digital<DIGITAL_LEN>,
                                                    void>::Result>::Result>::Result>::Result>::Result Type;
        };

        template<typename T>
        struct GET;

        template<UInt32 U>
        struct Wrap {
            enum : UInt32 {
                UNIT = U
            };
        };

    public:
        Unit() : _unit(0) {}

        Unit(UInt32 u) { _unit = u; }

        operator UInt32() const { return _unit; }

        //retirar daqui
        size_t value_size() const {
            return (_unit & SI) && ((_unit & NUM) == I32) ? sizeof(Int32)
                                                          : (_unit & SI) && ((_unit & NUM) == I64) ? sizeof(Int64)
                                                                                                   : (_unit & SI) &&
                                                                                                     ((_unit & NUM) ==
                                                                                                      F32)
                                                                                                     ? sizeof(Float32)
                                                                                                     : (_unit & SI) &&
                                                                                                       ((_unit & NUM) ==
                                                                                                        D64)
                                                                                                       ? sizeof(Float64)
                                                                                                       : !(_unit & SI)
                                                                                                         ? digital_value_size(
                                                            _unit) : 0;
        }

        static size_t digital_value_size(unsigned int unit) {
            if ((unit & SI))
                return 0;

            switch (unit >> 16) {
                case MOTION_VECTOR_GLOBAL >> 16: {
                    // # elements * (3 D64 Loc + F32 speed + F32 Heading + F32 YawR. + F32 Accel + 3 F32 Dim)
                    // plus ID, Class, and Confidence (to be removed after modifications in UNIT)
                    return (unit & LEN) * ((3 * 8 + 4 * 4 + 3 * 4) + 2 * 4 + 8);
                }
                case MOTION_VECTOR_LOCAL >> 16: {
                    // # elements * (2 F32 Loc + F32 speed + F32 Heading + F32 YawR. + F32 Accel + 3 F32 Dim)
                    // plus ID, Class, and Confidence (to be removed after modifications in UNIT)
                    return (unit & LEN) * ((2 * 4 + 4 * 4 + 3 * 4) + 2 * 4 + 8);
                }
                case AV_DYNAMICS_STATE >> 16: {
                    // # elements * (3 D64 Loc + F32 speed + F32 Heading + F32 YawR. + F32 Accel + F32 Orientation + F32 Steering Angle + On_Off Parking)
                    return (unit & LEN) * (3 * 8 + 6 * 4 + 1);
                }
                case JPEG >> 16:
                    return (unit & LEN) == 1 ? 61440 : unit & LEN;
                case PCMU >> 16: // TODO
                case GSM >> 16:
                case G723 >> 16:
                case DVI4_8 >> 16:
                case DVI4_16 >> 16:
                case LPC >> 16:
                case PCMA >> 16:
                case G722 >> 16:
                case L16_2 >> 16:
                case L16_1 >> 16:
                case QCELP >> 16:
                case CN >> 16:
                case MPA >> 16:
                case G728 >> 16:
                case DVI4_11 >> 16:
                case DVI4_22 >> 16:
                case G729 >> 16:
                case CelB >> 16:
                case nv >> 16:
                case H261 >> 16:
                case MPV >> 16:
                case MP2T >> 16:
                case H263 >> 16:
                case MPEG2TS >> 16:
                    return ((unit & LEN) == 1 ? 1 : unit & LEN);
                default:
                    return unit & LEN;
            }
        }

        int sr() const { return ((_unit & SR) >> 24) - 4; }

        int rad() const { return ((_unit & RAD) >> 21) - 4; }

        int m() const { return ((_unit & M) >> 18) - 4; }

        int kg() const { return ((_unit & KG) >> 15) - 4; }

        int s() const { return ((_unit & S) >> 12) - 4; }

        int a() const { return ((_unit & A) >> 9) - 4; }

        int k() const { return ((_unit & K) >> 6) - 4; }

        int mol() const { return ((_unit & MOL) >> 3) - 4; }

        int cd() const { return ((_unit & CD) >> 0) - 4; }

        friend OStream &operator<<(OStream &os, const Unit &u) {
            if (u & SI) {
                os << "{SI";
                switch (u & MOD) {
                    case DIR:
                        break;
                    case DIV:
                        os << "[U/U]";
                        break;
                    case LOG:
                        os << "[log(U)]";
                        break;
                    case LOG_DIV:
                        os << "[log(U/U)]";
                };
                switch (u & NUM) {
                    case I32:
                        os << ":I32";
                        break;
                    case I64:
                        os << ":I64";
                        break;
                    case F32:
                        os << ":F32";
                        break;
                    case D64:
                        os << ":D64";
                }
                os << ':';
                if (u.sr())
                    os << "sr^" << u.sr() << '.';
                if (u.rad())
                    os << "rad^" << u.rad() << '.';
                if (u.m())
                    os << "m^" << u.m() << '.';
                if (u.kg())
                    os << "kg^" << u.kg() << '.';
                if (u.s())
                    os << "s^" << u.s() << '.';
                if (u.a())
                    os << "A^" << u.a() << '.';
                if (u.k())
                    os << "K^" << u.k() << '.';
                if (u.mol())
                    os << "mol^" << u.mol() << '.';
                if (u.cd())
                    os << "cdr^" << u.cd() << '.';
                os << '\b';
            } else
                os << "{D:" << "l=" << (u & LEN);
            os << "}";
            return os;
        }

    private:
        UInt32 _unit;
    } __attribute__((packed));

    // Numeric value (either integer32, integer64, float32, double64 according to Unit::NUM)
    template<UInt32 UNIT>
    class Value {
        typedef typename Unit::Get<UNIT>::Type Type;

    public:
        Value() {}

        Value(const Type &v) : _value(v) {}

        operator Type &() { return _value; }

    private:
        Type _value;
    } __attribute__((packed));
};


