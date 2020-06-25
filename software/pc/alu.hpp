#include "type.hpp"

#if nALU > 0

class CALU
{
private:
    versat_t ina = 0, inb = 0, out = 0;
    versat_t output_buff[ALU_LAT]; //output FIFO
public:
    int versat_base, alu_base;
    int opa = 0, opb = 0, fns = 0;

    //Default constructor
    CALU();
    CALU(int versat_base, int i);

    //start run
    void start_run();

    //update output buffer, write results to databus
    void update();
    versat_t output();

    void setOpA(int opa);
    void setOpB(int opb);
    void setFNS(int fns);

    string info();
}; //end class CALU

extern versat_t global_databus[(nSTAGE + 1) * N];
extern int sALU[nALULITE];
#endif