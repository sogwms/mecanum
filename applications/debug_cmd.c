#include <rtthread.h>
#include <chassis.h>
#include <ano.h>

extern chassis_t chas;

#define ENC_FL  chas->c_wheels[0]->w_encoder
#define ENC_FR  chas->c_wheels[1]->w_encoder
#define ENC_BL  chas->c_wheels[2]->w_encoder
#define ENC_BR  chas->c_wheels[3]->w_encoder

static void debug(int argc, char *argv[])
{
    if (argc < 2)
    {
        return;
    }

    if (!rt_strcmp(argv[1], "read_wheels_pluse"))
    {
        rt_kprintf("pluse: %5d %5d %5d %5d\n",
            encoder_read(ENC_FL),
            encoder_read(ENC_FR),
            encoder_read(ENC_BL),
            encoder_read(ENC_BR));
    }

    if (argc < 3)
    {
        return;
    }
    if (!rt_strcmp(argv[1], "ano_init"))
    {
        ano_init(argv[2]);
    }
}
MSH_CMD_EXPORT(debug, test);
