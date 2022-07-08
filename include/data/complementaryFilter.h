// complementary filters to make our more reliable. 


class ComplementaryFilter {
    public:
        ComplementaryFilter(float k);     // constructor method k = current value coefficient
        float calculate(float f);         // compute method
    
    private:
        float coeff  = 0.0;     // coefficient
        float oldVal = 0.0;     // old value 
};

