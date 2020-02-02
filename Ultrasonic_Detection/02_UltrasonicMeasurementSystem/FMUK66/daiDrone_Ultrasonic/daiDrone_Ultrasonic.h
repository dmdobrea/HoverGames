#include <time.h>
#include <signal.h>
#include <semaphore.h>

extern "C" __EXPORT int daiDrone_Ultrasonic_main (int argc, char *argv[]);

static void timer_expiration_US (int signo, siginfo_t *info, void *ucontext);
static void usageUs(void);

union I2C_data {
   unsigned short int dataInt  [4];
   unsigned char      dataChar [8];
};
