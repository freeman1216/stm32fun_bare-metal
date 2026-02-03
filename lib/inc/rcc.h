/** @file    rcc.h
 *  @brief   Reset and Clock Control (RCC) header file.
 */

#ifndef LIB_INC_RCC_H_
#define LIB_INC_RCC_H_

struct rcc {
    union {
        uint32_t reg;      /**< Clock control register */
        struct {
            uint32_t HSION : 1;      /**< Internal high-speed clock enable */
            uint32_t HSIRDY : 1;     /**< Internal high-speed clock ready flag */
            uint32_t Reserved1 : 1;  /**< Reserved */
            uint32_t HSITRIM : 5;    /**< Internal high-speed clock trimming */
            uint32_t HSICAL : 8;     /**< Internal high-speed clock calibration */
            uint32_t HSEON : 1;      /**< External high-speed clock enable */
            uint32_t HSERDY : 1;     /**< External high-speed clock ready flag */
            uint32_t HSEBYP : 1;     /**< External high-speed clock bypass */
            uint32_t CSSON : 1;      /**< Clock security system enable */
            uint32_t Reserved2 : 4;  /**< Reserved */
            uint32_t PLLON : 1;      /**< PLL enable */
            uint32_t PLLRDY : 1;     /**< PLL clock ready flag */
            uint32_t PLLI2SON : 1;   /**< PLLI2S enable */
            uint32_t PLLI2SRDY : 1;  /**< PLLI2S clock ready flag */   
            uint32_t PLLSAION : 1;   /**< PLLSAI enable */
            uint32_t PLLSAIRDY : 1;  /**< PLLSAI clock ready flag */
            uint32_t Reserved3 : 2;  /**< Reserved */
        } fields;
    } CR; /**< Clock control register */
    union {
        uint32_t reg;      /**< PLL configuration register */
        struct {
            uint32_t PLLM : 6;       /**< Division factor for the main PLL and audio PLL input clock */
            uint32_t PLLN : 9;       /**< Main PLL multiplication factor for VCO */
            uint32_t Reserved1 : 1;  /**< Reserved */
            uint32_t PLLP : 2;       /**< Main PLL division factor for main system clock */
            uint32_t Reserved2 : 4;  /**< Reserved */
            uint32_t PLLQ : 4;       /**< Main PLL division factor for USB OTG FS, SDIO and RNG clocks */
            uint32_t PLLR : 3;       /**< Main PLL division factor for SAI1 and SAI2 clocks */
            uint32_t Reserved3 : 3;  /**< Reserved */
        } fields;
    } PLLCFGR; /**< PLL configuration register */
    union {
        uint32_t reg;      /**< Clock configuration register */
        struct {
            uint32_t SW : 2;         /**< System clock switch */
            uint32_t SWS : 2;        /**< System clock switch status */
            uint32_t HPRE : 4;       /**< AHB prescaler */
            uint32_t PPRE1 : 3;      /**< APB1 low-speed prescaler (APB1) */
            uint32_t PPRE2 : 3;      /**< APB2 high-speed prescaler (APB2) */
            uint32_t RTCPRE : 5;     /**< HSE division factor for RTC clock */
            uint32_t MCO1 : 2;       /**< Microcontroller clock output 1 */
            uint32_t I2SSRC : 1;     /**< I2S clock source */
            uint32_t MCO1PRE : 3;    /**< MCO1 prescaler */
            uint32_t MCO2PRE : 3;    /**< MCO2 prescaler */
            uint32_t MCO2 : 2;       /**< Microcontroller clock output 2 */
        } fields;
    } CFGR; /**< Clock configuration register */
    union {
        uint32_t reg;      /**< Clock interrupt register */
        struct {
            uint32_t LSIRDYF : 1;    /**< LSI ready interrupt flag */
            uint32_t LSERDYF : 1;    /**< LSE ready interrupt flag */
            uint32_t HSIRDYF : 1;    /**< HSI ready interrupt flag */
            uint32_t HSERDYF : 1;    /**< HSE ready interrupt flag */
            uint32_t PLLRDYF : 1;    /**< PLL ready interrupt flag */
            uint32_t PLLI2SRDYF : 1; /**< PLLI2S ready interrupt flag */
            uint32_t PLLSAIRDYF : 1; /**< PLLSAI ready interrupt flag */
            uint32_t CSSF : 1;       /**< Clock security system interrupt flag */
            uint32_t LSIRDYIE : 1;   /**< LSI ready interrupt enable */
            uint32_t LSERDYIE : 1;   /**< LSE ready interrupt enable */
            uint32_t HSIRDYIE : 1;   /**< HSI ready interrupt enable */
            uint32_t HSERDYIE : 1;   /**< HSE ready interrupt enable */
            uint32_t PLLRDYIE : 1;   /**< PLL ready interrupt enable */
            uint32_t PLLI2SRDYIE : 1;/**< PLLI2S ready interrupt enable */
            uint32_t PLLSAIRDYIE : 1;/**< PLLSAI ready interrupt enable */
            uint32_t Reserved1 : 1;  /**< Reserved */
            uint32_t LSIRDYC : 1;    /**< LSI ready interrupt clear */
            uint32_t LSERDYC : 1;    /**< LSE ready interrupt clear */
            uint32_t HSIRDYC : 1;    /**< HSI ready interrupt clear */
            uint32_t HSERDYC : 1;    /**< HSE ready interrupt clear */
            uint32_t PLLRDYC : 1;    /**< PLL ready interrupt clear */
            uint32_t PLLI2SRDYC : 1; /**< PLLI2S ready interrupt clear */
            uint32_t PLLSAIRDYC : 1; /**< PLLSAI ready interrupt clear */  
            uint32_t Reserved2 : 8;  /**< Reserved */
        } fields;
    } CIR;
    union {
        uint32_t reg;
    }
};

#define RCC ((volatile struct rcc *)0x40023800)

#endif /* LIB_INC_RCC_H_ */
// vim: set et sw=4 ts=4: