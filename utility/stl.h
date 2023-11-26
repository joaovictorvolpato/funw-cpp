#pragma once

#include <system/types.h>
#include <utility/debug.h>
#include <utility/buffer.h>

// static const bool TOLERATES_REPLACE = true;
class Sample {
friend class Until_Operator;
friend class Until_Operator_Time_Sensitive;
public:
    Sample(Microsecond timestamp=-1, bool value=false) : _timestamp(timestamp), _value(value) {}
    Microsecond timestamp() { return _timestamp; }
    bool value () { return _value; }

    friend bool operator!(const Sample & s) { 
        return !(s._value);
    }

private:
    Microsecond _timestamp;
    bool _value;
};

class Until_Operator {
public:
    Until_Operator(Microsecond begin, Microsecond end, Microsecond period) : _begin(ticks(begin, period)), _end(ticks(end, period)) {
        db<SmartData>(TRC) << "begin:" << _begin << ", end:" << _end << endl;
        _historical_buffer_left  = new Dynamic_Circular_Buffer<Sample>(_end+1);
        _historical_buffer_right = new Dynamic_Circular_Buffer<Sample>(_end+1);
        _out = Sample(-1, false);
        assert(_begin < _end);
        for (unsigned int i = 0; i <= _end; i++) {
            _historical_buffer_left->insert(Sample(-1, 1));
            _historical_buffer_right->insert(Sample(-1, 1));
        }
    }

    Sample update(Sample const & new_left, Sample const & new_right) {
        bool outv = 0;
        _historical_buffer_left->insert(new_left);
        _historical_buffer_right->insert(new_right);

        Sample left(-1, 0);
        Sample right(-1, 0);

        for (unsigned int i = _begin; i <= _end; i++) {
            right._value = (*_historical_buffer_right)[i].value();
            left._value  = 1;
            for (unsigned int j = 0; j <= i; j++) {
                // left._value = min(left.value(), _historical_buffer_left[j].value());
                left._value = left.value() && (*_historical_buffer_left)[j].value();
            }
            // out = max(out, min(left.value(), right.value()));
            outv = outv || (left.value() && right.value());
        }
        _out = Sample(right.timestamp(), outv);
        return _out;
    }

    Sample out() { return _out; }

private:
    inline unsigned int ticks(Microsecond time, Microsecond period) {
        return time / period;
    }

protected:
    unsigned int _begin;
    unsigned int _end;
    Sample _out;
    Dynamic_Circular_Buffer<Sample> *_historical_buffer_left;
    Dynamic_Circular_Buffer<Sample> *_historical_buffer_right;
};

class Eventually_Operator : private Until_Operator {
public:
    Eventually_Operator(Microsecond begin, Microsecond end, Microsecond period) : Until_Operator(begin, end, period) {
        db<SmartData>(TRC) << "begin:" << _begin << ", end:" << _end << endl;
    }

    Sample update(Sample & new_right) {
        return Until_Operator::update(Sample(new_right.timestamp(), 1), new_right);
    }
    using Until_Operator::out;
};

class Until_Operator_Time_Sensitive {
public:
    Until_Operator_Time_Sensitive(Microsecond begin, Microsecond end, Microsecond period) : _begin(ticks(begin, period)), _end(ticks(end, period)), _period(period) {
        db<SmartData>(TRC) <<"Until_Operator_Time_Sensitive(): begin:" << _begin << ", end:" << _end << endl;
        if (_end <= _begin) {
            db<SmartData>(ERR) << "Timing error: end (" << _end << ") <= begin (" << _begin << ")" << endl;
            throw 1;
        }
        _historical_buffer_left  = new Dynamic_Circular_Buffer<Sample>(_end+1);
        _historical_buffer_right = new Dynamic_Circular_Buffer<Sample>(_end+1);
        Sample s_left(-1, 1);
        Sample s_right(-1, 0);
        for (unsigned int i = 0; i <= _end; i++) {
            _historical_buffer_left->insert(s_left);
            _historical_buffer_right->insert(s_right);
        }
        _start_time = 0;//now(); // just for tests which are using 0 as a base for time
        _out = Sample(_start_time, 0);
    }

    Sample update_left(Sample new_sample, bool run_update=false) {
        db<SmartData>(TRC) <<"Until_Operator_Time_Sensitive():update_left:" << new_sample.value() << ",t:" << new_sample.timestamp() << endl;
        if ( (*_historical_buffer_left)[_end].timestamp() == -1) {
            _historical_buffer_left->insert(new_sample);
            db<SmartData>(INF) <<"-1" << endl;
        } else {
            unsigned int index_new  = period_index(new_sample.timestamp());
            unsigned int index_last = period_index((*_historical_buffer_left)[_end].timestamp());
            unsigned int index_right = 0;
            if ((*_historical_buffer_right)[_end].timestamp() != -1) { // if left has at least one sample, calculate distance to right
                index_right = period_index((*_historical_buffer_right)[_end].timestamp());
            }
            db<SmartData>(INF) << "\tIndexes_left: new=" << index_new << ",last=" << index_last << ",right=" << index_right << endl;
            if (index_right < index_new) {                           // if right is missing samples, add as many missing samples as needed
                if (index_new - index_right > _end) {
                    for (unsigned int i = index_new-_end; i <= index_new; i++)
                        _historical_buffer_right->insert(Sample(_start_time + i*_period, 0));
                } else {
                    for (unsigned int i = index_right+1; i <= index_new; i++)
                        _historical_buffer_right->insert(Sample(_start_time + i*_period, 0));
                }
            }

            if (index_new == index_last) { // if there is already a sample in this period, merge (possibly added by a previous update in right)
                //if (TOLERATES_REPLACE)
                (*_historical_buffer_left)[_end]._value = new_sample.value() || (*_historical_buffer_left)[_end]._value;
                //else
                //    _historical_buffer_left[_end]._value = new_sample.value() && _historical_buffer_left[_end]._value;
            } else if (index_new < index_last) { // if this data is outdated, discard ( only one level of delay is tolerated!!! )
                return _out;
            } else if (index_new >= 2*index_last) { // if this data is two or more samples ahead of last sample, add missing sample
                // if index_last is 0 and index_new is 1 (2*0<1), this loop will be skipped
                // if index_last is 1 and index_new is 2 (2*1==2), this loop will be skipped
                // for other cases, this loop fixes 2 or more samples behind... one sample behind is the regular behavior and will only add, skipping the for.
                for (unsigned int i = index_last+1; i < index_new; i++)
                // 0+1 < 1 (skip), 0+1 < 2 (add 1 missing), 0+1 < 3 (add 2 missing)
                // 1+1 < 1 (skip), 1+1 < 2 (skip)         , 1+1 < 3 (add 1 missing), 1+1 < 4 (add 2 missing)
                // 2+1 < 2 (skip), 2+1 < 3 (skip)         , 2+1 < 4 (add 1 missing), 2+1 < 5 (add 2 missing)
                    _historical_buffer_right->insert(Sample(_start_time + i*_period, 0));
                _historical_buffer_left->insert(new_sample);
            } else {
                _historical_buffer_left->insert(new_sample); // regular "new" period insert
            }
        }

        if (new_sample.timestamp() > _out.timestamp())
            _out._timestamp = new_sample.timestamp();

        if (run_update)
            update();
        return _out;
    }

    Sample update_right(Sample new_sample, bool run_update=true) {
        db<SmartData>(TRC) <<"Until_Operator_Time_Sensitive():update_right:" << new_sample.value() << ",t:" << new_sample.timestamp() << endl;
        _historical_buffer_right->insert(new_sample);
        if ((*_historical_buffer_right)[_end].timestamp() == -1) {
            _historical_buffer_right->insert(new_sample);
        } else {
            unsigned int index_new  = period_index(new_sample.timestamp());
            unsigned int index_last = period_index((*_historical_buffer_right)[_end].timestamp());
            unsigned int index_left = 0;
            if ((*_historical_buffer_left)[_end].timestamp() != -1) { // if left has at least one sample, calculate distance to right
                index_left = period_index((*_historical_buffer_left)[_end].timestamp());
            }
            // db<SmartData>(TRC) << "\tIndexes_Right: left-ts=" <<"new=" << index_new << ",last=" << index_last << ",left=" << index_left << endl;
            if (index_left < index_new) {                           // if left is missing samples, add as many missing samples as needed
                if (index_new - index_left > _end) {
                    for (unsigned int i = index_new-_end; i <= index_new; i++)
                        _historical_buffer_left->insert(Sample(_start_time + i*_period, 0));
                } else {
                    for (unsigned int i = index_left+1; i <= index_new; i++)
                        _historical_buffer_left->insert(Sample(_start_time + i*_period, 0));
                }
            }

            if (index_new == index_last) { // if there is already a sample in this period, merge (possibly added by a previous updated in left)
                //if (TOLERATES_REPLACE)
                (*_historical_buffer_right)[_end]._value = new_sample.value() || (*_historical_buffer_right)[_end]._value;
                // else
                //    _historical_buffer_right[_end]._value = new_sample.value() && _historical_buffer_right[_end]._value;
            } else if (index_new < index_last) { // if this data is outdated, discard
                return _out;
            } else if (index_new >= 2*index_last) { // if this data is two or more samples ahead of last sample, add missing sample
                // if index_last is 0 and index_new is 1 (2*0<1), this loop will be skipped
                // if index_last is 1 and index_new is 2 (2*1==2), this loop will be skipped
                // for other cases, this loop fixes 2 or more samples behind... one sample behind is the regular behavior and will only add, skipping the for.
                for (unsigned int i = index_last+1; i < index_new; i++)
                // 0+1 < 1 (skip), 0+1 < 2 (add 1 missing), 0+1 < 3 (add 2 missing)
                // 1+1 < 1 (skip), 1+1 < 2 (skip)         , 1+1 < 3 (add 1 missing), 1+1 < 4 (add 2 missing)
                // 2+1 < 2 (skip), 2+1 < 3 (skip)         , 2+1 < 4 (add 1 missing), 2+1 < 5 (add 2 missing)
                    _historical_buffer_right->insert(Sample(_start_time + i*_period, 0));
                _historical_buffer_right->insert(new_sample);
            } else {
                _historical_buffer_right->insert(new_sample); // regular "new" period insert
            }
        }
        if (new_sample.timestamp() > _out.timestamp())
            _out._timestamp = new_sample.timestamp();

        if (run_update)
            update();
        return _out;
    }

    Sample out() { return _out; }

private:
    inline unsigned int ticks(Microsecond time, Microsecond period) {
        return time / period;
    }

    inline unsigned int period_index(Microsecond timestamp) {
        return (timestamp - _start_time)/_period;
    }

    void update() {
        /*
        * no need to handle right being one period ahead than left, and vice-versa
        * synchronization is done on previous method (update_right or update_left)
        */
        bool outv = 0;
        Sample left(-1, 0);
        Sample right(-1, 0);
        for (unsigned int i = _begin; i <= _end; i++) {
            right._value = (*_historical_buffer_right)[i].value();
            // db<SmartData>(TRC) << "\t\t i=" << i << ",r_value=" << right.value() << ",l_value=" << _historical_buffer_left[i].value() << endl;
            left._value  = 1;
            for (unsigned int j = 0; j <= i; j++) {
                // left._value = min(left.value(), _historical_buffer_left[j].value());
                left._value = left.value() && (*_historical_buffer_left)[j].value();
            }
            // out = max(out, min(left.value(), right.value()));
            outv = outv || (left.value() && right.value());
        }
        _out._value = outv;
    }

protected:
    unsigned int _begin;
    unsigned int _end;
    Microsecond _period;
    Microsecond _start_time;
    Sample _out;
    Dynamic_Circular_Buffer<Sample> *_historical_buffer_left;
    Dynamic_Circular_Buffer<Sample> *_historical_buffer_right;
};

class Eventually_Operator_Time_Sensitive : private Until_Operator_Time_Sensitive {
public:
    Eventually_Operator_Time_Sensitive(Microsecond begin, Microsecond end, Microsecond period) : Until_Operator_Time_Sensitive(begin, end, period) {
        db<SmartData>(TRC) << "begin:" << _begin << ", end:" << _end << endl;
    }

    Sample update(Sample new_right, bool run_update=true) {
        update_left(Sample(new_right.timestamp(), 1), false);
        return update_right(new_right, run_update);
    }

    using Until_Operator_Time_Sensitive::out;
};