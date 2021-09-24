#include "geometry.h"
#include "utils.h"

class FoodItem{
	protected:
		bool dynamics;
};

class FoodItem : public FoodItem, public Circle{
    public:
        FoodItem(float xc, float yc, float radius);
        std::string tostring();
};
