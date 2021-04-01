
#define LED_PORT GPIOA
#define LED_OUT_PORT GPIOB

#define LED_a LL_GPIO_PIN_1
#define LED_b LL_GPIO_PIN_2
#define LED_c LL_GPIO_PIN_3
#define LED_d LL_GPIO_PIN_4
#define LED_e LL_GPIO_PIN_9
#define LED_f LL_GPIO_PIN_6
#define LED_g LL_GPIO_PIN_10
#define LED_dp LL_GPIO_PIN_8

#define LED_DIG_1 LL_GPIO_PIN_9
#define LED_DIG_2 LL_GPIO_PIN_10
#define LED_DIG_3 LL_GPIO_PIN_12
#define LED_DIG_4 LL_GPIO_PIN_13

static void gpio_config_leds() {
    LL_GPIO_SetPinMode(LED_PORT, LED_a, LL_GPIO_MODE_OUTPUT);//a
    LL_GPIO_SetPinMode(LED_PORT, LED_b, LL_GPIO_MODE_OUTPUT);//b
    LL_GPIO_SetPinMode(LED_PORT, LED_c, LL_GPIO_MODE_OUTPUT);//c
    LL_GPIO_SetPinMode(LED_PORT, LED_d, LL_GPIO_MODE_OUTPUT);//d
    LL_GPIO_SetPinMode(LED_PORT, LED_e, LL_GPIO_MODE_OUTPUT);//e
    LL_GPIO_SetPinMode(LED_PORT, LED_f, LL_GPIO_MODE_OUTPUT);//f
    LL_GPIO_SetPinMode(LED_PORT, LED_g, LL_GPIO_MODE_OUTPUT);//g
    LL_GPIO_SetPinMode(LED_PORT, LED_dp, LL_GPIO_MODE_OUTPUT);//dp

    LL_GPIO_SetPinMode(LED_OUT_PORT, LED_DIG_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_OUT_PORT, LED_DIG_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_OUT_PORT, LED_DIG_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(LED_OUT_PORT, LED_DIG_4, LL_GPIO_MODE_OUTPUT);

    //set opendrain to minus
    LL_GPIO_SetPinOutputType(LED_OUT_PORT, LED_DIG_1, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinOutputType(LED_OUT_PORT, LED_DIG_2, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinOutputType(LED_OUT_PORT, LED_DIG_3, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinOutputType(LED_OUT_PORT, LED_DIG_4, LL_GPIO_OUTPUT_OPENDRAIN);
}

static void ind_dig(uint32_t dig) {
    static uint32_t mask = LED_a | LED_b | LED_c | LED_d | LED_e | LED_f | LED_g | LED_dp;
   
    static uint32_t digits[] = {
        LED_a | LED_b | LED_c | LED_d | LED_e | LED_f, //0
        LED_b | LED_c, //1
        LED_a | LED_b | LED_d | LED_e | LED_g, //2
        LED_a | LED_b | LED_d | LED_g | LED_c, //3
        LED_b | LED_c | LED_f | LED_g, //4
        LED_a | LED_c | LED_d | LED_f | LED_g, //5
        LED_a | LED_c | LED_d | LED_e | LED_f | LED_g,//6
        LED_a | LED_b | LED_c , //7
        LED_a | LED_b | LED_c | LED_d | LED_e | LED_f | LED_g,//8
        LED_a | LED_b | LED_c | LED_d | LED_f | LED_g, //9
    };

    const uint32_t size_digs = sizeof(digits) / sizeof(uint32_t);

    uint32_t state = (LL_GPIO_ReadOutputPort(LED_PORT) & ~mask) | digits[dig % size_digs];
    
    LL_GPIO_WriteOutputPort(LED_PORT, state);
}

static void off_dp() {
    uint32_t mask = LED_a | LED_b | LED_c | LED_d | LED_e | LED_f | LED_g | LED_dp;
    LL_GPIO_WriteOutputPort(LED_PORT, (LL_GPIO_ReadOutputPort(LED_PORT) & ~mask) |
     0 );    
}

static void on_dp() {
    uint32_t mask = LED_a | LED_b | LED_c | LED_d | LED_e | LED_f | LED_g | LED_dp;
    LL_GPIO_WriteOutputPort(LED_PORT, (LL_GPIO_ReadOutputPort(LED_PORT) & ~mask) |
    LED_dp);    
}











