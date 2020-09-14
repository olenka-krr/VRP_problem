#ifndef VOGTEST_H
#define VOGTEST_H
#include <string>

class vehicle{
private:
public:
    std::string id;
    double baseX, baseY;
    int capacity;
    int distance_price;

public:
    vehicle();
    virtual ~vehicle();

};

class shipment{
private:
public:
    std::string id;
    double pickupX, pickupY;
    double deliveryX, deliveryY;
    int capacityDemand;
public:
    shipment();
    virtual ~shipment();

};

#endif // VOGTEST_H
