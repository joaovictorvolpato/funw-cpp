
#include "inheritance_test.h"

int main()
{
    unsigned char * buffer[100];

    MultiUnitSmartData<1, 2, 3> multiUnitSmartData;

    multiUnitSmartData.setValue<1>(10);
    multiUnitSmartData.setValue<2>(20);
    multiUnitSmartData.setValue<3>(30);

    multiUnitSmartData.setValue<3>(300);
    multiUnitSmartData.setValue<1>(100);
    multiUnitSmartData.setValue<2>(200);

    multiUnitSmartData.printfunc(multiUnitSmartData);


    MultiUnitSmartData_element<1> multiUnitSmartData_element10;
    MultiUnitSmartData_element<2> multiUnitSmartData_element20;
    MultiUnitSmartData_element<3> multiUnitSmartData_element30;

    multiUnitSmartData.serialize(reinterpret_cast<char *>(buffer));

    std::cout << "multiUnitSmartData getElement<1>() = " << multiUnitSmartData.getElement<3>().a << std::endl;
    std::cout << "multiUnitSmartData getElement<2>() = " << multiUnitSmartData.getElement<1>().a << std::endl;
    std::cout << "multiUnitSmartData getElement<3>() = " << multiUnitSmartData.getElement<2>().a << std::endl;

    //std::cout << "Test" << multiUnitSmartData.getValue<LISTV<1,2,3>::Get<2>()>() << std::endl;

    multiUnitSmartData.deserialize(reinterpret_cast<char *>(buffer), multiUnitSmartData_element20, multiUnitSmartData_element10, multiUnitSmartData_element30);


    std::cout << "multiUnitSmartData_element10.a = " << multiUnitSmartData_element10.a << std::endl;
    std::cout << "multiUnitSmartData_element20.a = " << multiUnitSmartData_element20.a << std::endl;
    std::cout << "multiUnitSmartData_element30.a = " << multiUnitSmartData_element30.a << std::endl;


    return 0;
}