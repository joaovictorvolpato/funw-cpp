
#pragma once

#include "meta.h"
#include <iostream>

class Test1{
public:
    Test1() { std::cout << "Test1::Test1(constructor)" << std::endl; }
    ~Test1() { std::cout << "Test1::Test1(destructor)"<< std::endl; }
};

class Test2{
public:
    Test2() { std::cout << "Test2::Test2(constructor)" << std::endl; }
    ~Test2() { std::cout << "Test2::Test2(destructor)"<< std::endl; }
};

class Test3{
public:
    Test3() { std::cout << "Test3::Test3(constructor)" << std::endl; }
    ~Test3() { std::cout << "Test3::Test3(destructor)"<< std::endl; }
};

class Test5{
public:
    Test5() { std::cout << "Test5::Test5(constructor)" << std::endl; }
    ~Test5() { std::cout << "Test5::Test5(destructor)"<< std::endl; }
};


typedef LIST<Test1, Test2, Test3, Test5> TestList;


class Test4: public TestList::Recur{
public:
    Test4() { std::cout << "Test4::Test4(constructor)" << std::endl; }
    ~Test4() { std::cout << "Test4::Test4(destructor)"<< std::endl; }
};

typedef float Float32;
typedef double Float64;
typedef unsigned int UInt32;


template<UInt32 Unit>
struct MultiUnitSmartData_element {
public:
    unsigned int _unit = Unit*2;
    int a;
    //overload the << operator for the MultiUnitSmartData_element class
    //std::ostream operator << (std::ostream & os) {
    //    os << "MultiUnitSmartData_element<" << Unit << ">::_unit = " << _unit << std::endl;
    //}

};


template <UInt32... Units>
struct MultiUnitSmartData : public LIST<MultiUnitSmartData_element<Units>...>::Recur{
public:
    //This takes a buffer and a list of values and serializes them into the buffer, the const Tn & ... an means that it can take both references to rvalues and lvalues
    void serialize(char* buffer) {
        // Force a compilation error in case out is called with too many arguments
        //typename IF<(SIZEOF<Tn ...>::Result <= SIZEOF<Units ...>::Result(), int, void>::Result index = 0;
        SERIALIZE(buffer, 0, getElement<Units>()...);
    }

    template<typename ... Tn>
    void deserialize(char* buffer, Tn && ... an) {
        // Force a compilation error in case out is called with too many arguments
        //typename IF<(SIZEOF<Tn ...>::Result <= MAX_PARAMETERS_SIZE), int, void>::Result index = 0;
        DESERIALIZE(buffer, 0, an ...);
    }

    template<UInt32 Unit>
    static unsigned int getValue(MultiUnitSmartData<Units...> & msd) {
        return msd.MultiUnitSmartData_element<Unit>::_unit;
    }

    template<UInt32 Unit>
    void setValue(unsigned int value) {
        //this->MultiUnitSmartData_element<Unit>::_unit = value;
        this->MultiUnitSmartData_element<Unit>::a = value;
    }

    template <UInt32 Unit>
    MultiUnitSmartData_element<Unit>& getElement(){
        return *this;
    }

    struct print{
        template<int i>
        struct Code {
            static void exec(MultiUnitSmartData<Units...> & msd)
            {std::cout << "MultiUniSmartdata" << msd.template getValue<LISTV<Units ...>::template Get<i>()> << std::endl;}
        };
    };

    template<int from, class Compare, int to, int by, class Statement>
    struct EFOR{
        template<class MSD>
        static void exec(MSD && msd)
        {
            IF<Compare::template Code<from,to>::Result,
                    typename Statement::template Code<from>,STOP
            >::Result::exec(msd);
            IF<Compare::template Code<from,to>::Result,
                    EFOR<from+by,Compare,to,by,Statement>,STOP
            >::Result::exec(msd);
        }
    };

    void printfunc(MultiUnitSmartData<Units...> & msd){
        //std::cout << "Test" << static_cast<unsigned int>(msd.getValue<LISTV<Units ...>::template Get<2>()>(msd)) << std::endl;
        EFOR<0, LessThan, LISTV<Units ...>::Length, +1, print>::template exec(msd);
    }
};

