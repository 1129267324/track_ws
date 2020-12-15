#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1
#include<boost/date_time/posix_time/posix_time.hpp>
using namespace boost;
using namespace std;

class PID
{
  public:

  #define AUTOMATIC	1
  #define MANUAL	0
  #define DIRECT  0
  #define REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1
 
	 PID(){};
	void Init(double*, double*, double*, double, double, double, int, int);                                        
	void Init(double*, double*, double*, double, double, double, int);           
  //commonly used functions **************************************************************************
  PID(double*, double*, double*, double, double, double, int, int);    
  PID(double*, double*, double*,  double, double, double, int);        
  void SetMode(int Mode);             
  bool Compute();                
  void SetOutputLimits(double, double); 
  //available but not commonly used functions ********************************************************
  void SetTunings(double, double,  double);         	                                     
  void SetTunings(double, double,  double, int);         	  
	void SetControllerDirection(int);	 
  void SetSampleTime(int);       							  

  private:
	void Initialize();
		
	double kp;    
  double ki;   
  double kd;     

	int controllerDirection;
	int pOn;
  double *myInput;       
  double *myOutput;      
  double *mySetpoint;     
	unsigned long lastTime;
	double outputSum, lastInput;
	unsigned long SampleTime;
	double outMin, outMax;
	bool inAuto, pOnE;

public://****************************************************************
bool Compute3();
void InitLastInput(double initLastInput);
protected:
int GetTickCount() const;
};
#endif

